#!/usr/bin/env python3
"""
Ground-station side of the mosaic mission.

Listens for ``drone_control_msgs/CapturedFrame`` from the drone.  For every
frame it immediately writes the photo to disk and refreshes a ``drone_stitch``
manifest, so the session folder is always a valid, partially filled stitcher
input.  Stitching then happens through one of two backends:

* ``webapp`` - every frame is forwarded, as it arrives, to the stitcher's
  local web app (``POST /api/sessions/<id>/frames``).  The web app shows the
  session filling up (n/9), stitches automatically when the ninth frame is
  in, and the browser dashboard displays the map.  This node polls the
  session, downloads the finished map and republishes it for RViz.
* ``cli`` - the inherited stitcher is run directly in its own virtualenv
  (``python -m drone_stitch stitch``) once all frames are on disk.

``auto`` (default) uses ``webapp`` when ``GET /api/health`` answers at the
start of a session and ``cli`` otherwise; an upload failure mid-session also
falls back to ``cli``, so a map is produced either way.  The ROS Python
environment never imports OpenCV - the stitcher has its own virtualenv.

Session folder (this node's copy; the web app keeps its own under its
``runs/web/sessions/<id>``)::

    <output_root>/<session_id>/
        images/capture_01_id5.png ... capture_09_id7.png
        manifest.json          drone_stitch input (rewritten after every frame)
        frames.json            raw per-frame metadata as received
        stitch_01/             map.png (+ overview.png, transforms.json,
                               diagnostics.json, ... when available)
        stitch_result.json     backend, status, timings

Manifest conventions (see the stitcher's README): image 5 is the origin,
local X = image right, local Y = image top; ``ned_m``/``yaw_rad``/``agl_m``
per image come straight from the frame; ``grid.heading_rad`` is left out so
the stitcher derives it from image 5's yaw plus ``image_top_yaw_offset_rad``,
exactly how the navigator laid out the grid.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from drone_control.image_io import image_to_pil, png_to_image
from drone_control_msgs.msg import CapturedFrame
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
import requests
from sensor_msgs.msg import CameraInfo, Image

# drone_stitch.geometry.CAPTURE_ORDER - the only order the stitcher accepts.
CAPTURE_ORDER = (5, 4, 1, 2, 3, 6, 9, 8, 7)
# Files the web app's job directory may contain next to map.png.
_RESULT_FILES = ('map.png', 'overview.png', 'valid_mask.png', 'coverage.png',
                 'transforms.json', 'diagnostics.json')


@dataclass
class Session:
    """Everything received for one mission run."""

    session_id: str
    directory: str
    total: int
    image_size: Tuple[int, int] = (0, 0)
    grid_spacing: Tuple[float, float] = (0.0, 0.0)
    frames: Dict[int, dict] = field(default_factory=dict)   # sequence -> raw frame record
    stitch_runs: int = 0
    stitching: bool = False
    backend: str = ''            # 'webapp' or 'cli' once decided
    webapp_job_id: str = ''

    @property
    def complete(self) -> bool:
        return self.total > 0 and all(seq in self.frames for seq in range(1, self.total + 1))

    def missing(self) -> List[int]:
        return [seq for seq in range(1, self.total + 1) if seq not in self.frames]


def _run_dir(session: Session) -> str:
    """Next unused stitch output directory (the stitcher insists on a new/empty one)."""
    while True:
        session.stitch_runs += 1
        candidate = os.path.join(session.directory, f'stitch_{session.stitch_runs:02d}')
        if not os.path.exists(candidate):
            return candidate


class GroundStation(Node):
    """Collect frames, maintain the stitcher manifest, stitch when complete."""

    def __init__(self) -> None:
        super().__init__('ground_station')
        self._declare_parameters()
        self._read_parameters()

        # Must mirror the navigator's frame publisher so late joiners get
        # every frame of the session.
        frame_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=9,
        )
        self.create_subscription(CapturedFrame, self._frame_topic, self._on_frame, frame_qos)

        # Latched so RViz shows the last map whenever it (re)connects.
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_pub = self.create_publisher(Image, self._map_topic, result_qos)
        self._overview_pub = self.create_publisher(Image, self._overview_topic, result_qos)

        self._sessions: Dict[str, Session] = {}
        self._results: 'queue.Queue[Tuple[Session, dict]]' = queue.Queue()
        self.create_timer(0.5, self._publish_results)

        # Real camera: the driver's CameraInfo (from a checkerboard calibration)
        # replaces the ideal-pinhole model derived from camera_hfov_rad.
        self._camera_info: Optional[CameraInfo] = None
        if self._camera_info_topic:
            self.create_subscription(CameraInfo, self._camera_info_topic, self._on_camera_info,
                                     qos_profile_sensor_data)

        if self._stitch_backend != 'webapp' and not os.path.isfile(self._stitch_python):
            self.get_logger().warn(
                f'stitcher interpreter not found: {self._stitch_python}. Frames will still be '
                f'saved; create the venv as described in {self._stitch_project_dir}/README.md '
                f'(python3 scripts/setup_env.py) to enable CLI stitching')
        self.get_logger().info(
            f'listening on {self._frame_topic}; sessions -> {self._output_root}; '
            f'backend={self._stitch_backend} webapp={self._webapp_url} '
            f'cli={self._stitch_python} (auto_stitch={self._auto_stitch})')

    # ------------------------------------------------------------------
    # parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('frame_topic', '/survey/frames')
        self.declare_parameter('output_root', '~/drone_survey_sessions')
        self.declare_parameter('stitch_backend', 'auto')
        self.declare_parameter('webapp_url', 'http://127.0.0.1:8000')
        self.declare_parameter('webapp_timeout_s', 15.0)
        self.declare_parameter('stitch_project_dir', '~/drone_ws/src/drone-stitching')
        self.declare_parameter('stitch_python', '')
        self.declare_parameter('stitch_config', '')
        self.declare_parameter('stitch_timeout_s', 600.0)
        self.declare_parameter('auto_stitch', True)
        self.declare_parameter('camera_hfov_rad', 1.2)
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('image_top_yaw_offset_rad', 0.0)
        self.declare_parameter('fallback_agl_m', 5.0)
        self.declare_parameter('map_topic', '/survey/mosaic/map')
        self.declare_parameter('overview_topic', '/survey/mosaic/overview')

    def _read_parameters(self) -> None:
        self._frame_topic = str(self.get_parameter('frame_topic').value)
        self._output_root = os.path.expanduser(str(self.get_parameter('output_root').value))
        os.makedirs(self._output_root, exist_ok=True)

        self._stitch_backend = str(self.get_parameter('stitch_backend').value).strip().lower()
        if self._stitch_backend not in ('auto', 'webapp', 'cli'):
            raise ValueError('"stitch_backend" must be "auto", "webapp" or "cli"')
        self._webapp_url = str(self.get_parameter('webapp_url').value).rstrip('/')
        self._webapp_timeout = float(self.get_parameter('webapp_timeout_s').value)

        self._stitch_project_dir = os.path.expanduser(
            str(self.get_parameter('stitch_project_dir').value))
        python = str(self.get_parameter('stitch_python').value).strip()
        self._stitch_python = os.path.expanduser(python) if python else os.path.join(
            self._stitch_project_dir, '.venv', 'bin', 'python')
        config = str(self.get_parameter('stitch_config').value).strip()
        self._stitch_config = os.path.expanduser(config) if config else os.path.join(
            get_package_share_directory('drone_control'), 'config', 'stitch_config.json')
        self._stitch_timeout = float(self.get_parameter('stitch_timeout_s').value)
        self._auto_stitch = bool(self.get_parameter('auto_stitch').value)

        self._hfov = float(self.get_parameter('camera_hfov_rad').value)
        if not 0.0 < self._hfov < math.pi:
            raise ValueError('"camera_hfov_rad" must be in (0, pi)')
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value).strip()
        self._yaw_offset = float(self.get_parameter('image_top_yaw_offset_rad').value)
        self._fallback_agl = float(self.get_parameter('fallback_agl_m').value)
        if self._fallback_agl <= 0.0:
            raise ValueError('"fallback_agl_m" must be > 0')
        self._map_topic = str(self.get_parameter('map_topic').value)
        self._overview_topic = str(self.get_parameter('overview_topic').value)

    # ------------------------------------------------------------------
    # frames in
    # ------------------------------------------------------------------
    def _on_frame(self, msg: CapturedFrame) -> None:
        session = self._sessions.get(msg.session_id)
        if session is None:
            directory = os.path.join(self._output_root, msg.session_id or 'session_unnamed')
            os.makedirs(os.path.join(directory, 'images'), exist_ok=True)
            session = Session(msg.session_id, directory, int(msg.total))
            session.backend = self._choose_backend(session)
            self._sessions[msg.session_id] = session
            self.get_logger().info(
                f'new session {msg.session_id}: expecting {msg.total} frames -> {directory} '
                f'(backend: {session.backend})')

        if msg.sequence in session.frames:
            self.get_logger().warn(
                f'{msg.session_id}: frame {msg.sequence} received again; keeping the first')
            return

        try:
            pil = image_to_pil(msg.image)
        except ValueError as error:
            self.get_logger().error(f'{msg.session_id}: frame {msg.sequence} dropped: {error}')
            return
        filename = f'images/capture_{msg.sequence:02d}_id{msg.grid_id}.png'
        pil.save(os.path.join(session.directory, filename))

        if session.image_size == (0, 0):
            session.image_size = pil.size
        elif pil.size != session.image_size:
            self.get_logger().warn(
                f'{msg.session_id}: frame {msg.sequence} is {pil.size}, '
                f'session started with {session.image_size}')
        if msg.grid_spacing_m[0] > 0.0 and msg.grid_spacing_m[1] > 0.0:
            session.grid_spacing = (float(msg.grid_spacing_m[0]), float(msg.grid_spacing_m[1]))

        agl, agl_source = self._plausible_agl(msg)
        record = {
            'sequence': int(msg.sequence),
            'grid_id': int(msg.grid_id),
            'file': filename,
            'timestamp_s': float(msg.timestamp_s),
            'ned_m': [float(v) for v in msg.ned_m],
            'yaw_rad': float(msg.yaw_rad),
            'agl_m': agl,
            'agl_source': agl_source,
            'agl_reported_m': float(msg.agl_m) if msg.agl_valid else None,
            'grid_local_xy_m': [float(v) for v in msg.grid_local_xy_m],
            'received_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'image': {'width': msg.image.width, 'height': msg.image.height,
                      'encoding': msg.image.encoding},
        }
        session.frames[int(msg.sequence)] = record
        self._write_manifest(session)
        self.get_logger().info(
            f'{msg.session_id}: frame {msg.sequence}/{session.total} (cell {msg.grid_id}) saved '
            f'N={msg.ned_m[0]:.2f} E={msg.ned_m[1]:.2f} yaw={math.degrees(msg.yaw_rad):.1f}deg '
            f'agl={msg.agl_m:.2f}m' + ('' if msg.agl_valid else ' (invalid)'))

        if session.backend == 'webapp':
            self._upload_frame(session, record)

        if session.complete:
            if self._auto_stitch:
                self.start_stitch(session)
        elif msg.sequence == session.total:
            self.get_logger().error(
                f'{msg.session_id}: last frame arrived but frames {session.missing()} are '
                f'missing; not stitching')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0 and msg.k[4] > 0.0 and msg.width > 0 and msg.height > 0:
            if self._camera_info is None:
                self.get_logger().info(
                    f'camera calibration received on {self._camera_info_topic}: '
                    f'{msg.width}x{msg.height} fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                    f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} D={list(msg.d)}')
            self._camera_info = msg

    def _plausible_agl(self, msg: CapturedFrame) -> Tuple[Optional[float], str]:
        """
        Height above ground to put in the manifest, and where it came from.

        The frame's range-finder AGL scales that image's footprint, so one bad
        reading (a sensor seeing part of the airframe, a dropout) shrinks the
        image 20x and the stitch fails with "implausible footprint scale".
        Over the flat SITL ground the altitude above the takeoff pad (-z) is
        the same quantity, so a reading that disagrees with it by more than
        2x is replaced by -z and reported.  Real terrain can legitimately
        differ from -z; widen or drop this check there.
        """
        altitude = -float(msg.ned_m[2])
        reported = float(msg.agl_m) if msg.agl_valid else None
        if reported is not None and reported > 0.0 and (
                altitude <= 0.0 or 0.5 <= reported / altitude <= 2.0):
            return reported, msg.agl_source
        if altitude > 0.0:
            self.get_logger().warn(
                f'{msg.session_id}: frame {msg.sequence} range AGL '
                f'{"invalid" if reported is None else f"{reported:.2f} m"} is implausible '
                f'against altitude {altitude:.2f} m above the pad; using the altitude')
            return altitude, 'altitude_above_origin'
        return None, 'unknown'

    # ------------------------------------------------------------------
    # manifest
    # ------------------------------------------------------------------
    def _camera_block(self, session: Session, reference_agl: float) -> dict:
        """
        Camera block of the manifest: footprint, reference AGL, intrinsics.

        With a CameraInfo (real camera, calibrated) its K and distortion are
        used; otherwise an ideal pinhole from camera_hfov_rad and the image
        size (the Gazebo camera).  Footprint = [W*h/fx, H*h/fy] either way.
        """
        width, height = session.image_size
        info = self._camera_info
        if info is not None and (info.width, info.height) == (width, height):
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            distortion = [float(v) for v in info.d]
            if len(distortion) not in (4, 5, 8, 12, 14):
                distortion = (distortion + [0.0] * 5)[:5]
            return {
                'footprint_m': [width * reference_agl / fx, height * reference_agl / fy],
                'reference_agl_m': reference_agl,
                'image_top_yaw_offset_rad': self._yaw_offset,
                'calibration': {
                    'image_size': [width, height],
                    'K': [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                    'distortion': distortion,
                },
                'scale_note': (f'Calibrated camera from {self._camera_info_topic} '
                               f'(distortion_model {info.distortion_model or "plumb_bob"}). '
                               f'reference_agl_m is the range-finder height of cell 5 '
                               f'(or the configured fallback).'),
            }
        if info is not None:
            self.get_logger().warn(
                f'CameraInfo is {info.width}x{info.height} but frames are {width}x{height}; '
                f'using the hfov model instead', throttle_duration_sec=30.0)
        fx = (width / 2.0) / math.tan(self._hfov / 2.0)
        return {
            # Ground length along image u (width) and v (height) at reference_agl:
            # [width*h/fx, height*h/fy] with fy == fx (square pixels).
            'footprint_m': [width * reference_agl / fx, height * reference_agl / fx],
            'reference_agl_m': reference_agl,
            'image_top_yaw_offset_rad': self._yaw_offset,
            'calibration': {
                'image_size': [width, height],
                'K': [[fx, 0.0, (width - 1) / 2.0],
                      [0.0, fx, (height - 1) / 2.0],
                      [0.0, 0.0, 1.0]],
                'distortion': [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            'scale_note': (f'Ideal pinhole from the Gazebo camera SDF: horizontal_fov='
                           f'{self._hfov:.4f} rad, no distortion. reference_agl_m is the '
                           f'range-finder height of cell 5 (or the configured fallback).'),
        }

    def _reference_agl(self, session: Session) -> Tuple[float, str]:
        reference = next((f for f in session.frames.values()
                          if f['grid_id'] == 5 and f['agl_m']), None)
        if reference:
            return reference['agl_m'], 'cell5_frame'
        return self._fallback_agl, 'fallback_agl_m'

    def _grid_block(self, session: Session) -> dict:
        spacing = list(session.grid_spacing) if session.grid_spacing[0] > 0.0 else [2.5, 5.0]
        return {'spacing_m': spacing}

    @staticmethod
    def _image_entry(frame: dict) -> dict:
        entry = {
            'id': frame['grid_id'],
            'file': frame['file'],
            'timestamp_s': frame['timestamp_s'],
            'ned_m': frame['ned_m'],
            'yaw_rad': frame['yaw_rad'],
        }
        if frame['agl_m']:
            entry['agl_m'] = frame['agl_m']
        return entry

    def _write_manifest(self, session: Session) -> None:
        ordered = [session.frames[seq] for seq in sorted(session.frames)]
        reference_agl, reference_source = self._reference_agl(session)
        manifest = {
            'version': 1,
            'dataset_kind': 'real',
            'ned_frame_id': session.session_id,
            'camera': self._camera_block(session, reference_agl),
            'grid': self._grid_block(session),
            'images': [self._image_entry(frame) for frame in ordered],
            'ground_station': {
                'frames_received': len(ordered),
                'frames_expected': session.total,
                'expected_capture_order': list(CAPTURE_ORDER),
                'reference_agl_source': reference_source,
                'backend': session.backend,
            },
        }
        with open(os.path.join(session.directory, 'manifest.json'), 'w') as handle:
            json.dump(manifest, handle, indent=2)
        with open(os.path.join(session.directory, 'frames.json'), 'w') as handle:
            json.dump(ordered, handle, indent=2)

    # ------------------------------------------------------------------
    # web app backend
    # ------------------------------------------------------------------
    def _choose_backend(self, session: Session) -> str:
        if self._stitch_backend == 'cli':
            return 'cli'
        try:
            reply = requests.get(f'{self._webapp_url}/api/health', timeout=3.0)
            reply.raise_for_status()
            if reply.json().get('status') == 'ok':
                return 'webapp'
            reason = f'unexpected health reply {reply.text[:80]!r}'
        except (requests.RequestException, ValueError) as error:
            reason = str(error)
        if self._stitch_backend == 'webapp':
            self.get_logger().error(
                f'{session.session_id}: web app at {self._webapp_url} not reachable ({reason}); '
                f'frames will be saved but nothing will be stitched')
            return 'webapp'
        self.get_logger().warn(
            f'{session.session_id}: web app at {self._webapp_url} not reachable ({reason}); '
            f'falling back to the CLI stitcher')
        return 'cli'

    def _upload_frame(self, session: Session, frame: dict) -> None:
        """Forward one frame to the web app's session API; fall back to CLI on failure."""
        reference_agl, _ = self._reference_agl(session)
        metadata = {
            'sequence': frame['sequence'],
            'grid_id': frame['grid_id'],
            'total': session.total,
            'timestamp_s': frame['timestamp_s'],
            'ned_m': frame['ned_m'],
            'yaw_rad': frame['yaw_rad'],
            'agl_m': frame['agl_m'],
            'agl_source': frame['agl_source'],
            'camera': self._camera_block(session, reference_agl),
            'grid': self._grid_block(session),
            'ned_frame_id': session.session_id,
        }
        path = os.path.join(session.directory, frame['file'])
        try:
            with open(path, 'rb') as handle:
                reply = requests.post(
                    f'{self._webapp_url}/api/sessions/{session.session_id}/frames',
                    files={'image': (os.path.basename(path), handle, 'image/png')},
                    data={'metadata': json.dumps(metadata)},
                    timeout=self._webapp_timeout)
            reply.raise_for_status()
            status = reply.json()
        except (requests.RequestException, ValueError, OSError) as error:
            detail = ''
            response = getattr(error, 'response', None)
            if response is not None:
                detail = f' ({response.status_code}: {response.text[:200]})'
            self.get_logger().error(
                f'{session.session_id}: upload of frame {frame["sequence"]} to the web app '
                f'failed{detail}: {error}; switching this session to the CLI stitcher')
            session.backend = 'cli'
            return
        self.get_logger().info(
            f'{session.session_id}: web app has {status.get("received")}/{status.get("total")} '
            f'frames (status {status.get("status")})')
        if status.get('job_id'):
            session.webapp_job_id = str(status['job_id'])

    def _stitch_via_webapp(self, session: Session) -> dict:
        """Wait for the web app's job for this session, then fetch its results."""
        run_dir = _run_dir(session)
        os.makedirs(run_dir, exist_ok=True)
        result: dict = {'session_id': session.session_id, 'backend': 'webapp',
                        'run_dir': run_dir, 'webapp_url': self._webapp_url,
                        'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
        deadline = time.monotonic() + self._stitch_timeout
        status: dict = {}
        try:
            while time.monotonic() < deadline:
                reply = requests.get(f'{self._webapp_url}/api/sessions/{session.session_id}',
                                     timeout=self._webapp_timeout)
                reply.raise_for_status()
                status = reply.json()
                if status.get('status') in ('completed', 'failed'):
                    break
                time.sleep(1.0)
            else:
                raise TimeoutError(f'web app did not finish within {self._stitch_timeout:.0f}s '
                                   f'(last status {status.get("status")!r})')
            result['job_id'] = status.get('job_id')
            result['webapp_status'] = status
            if status.get('status') != 'completed':
                raise RuntimeError(status.get('error') or 'web app reported failure')
            reply = requests.get(f'{self._webapp_url}{status["map_url"]}',
                                 timeout=self._webapp_timeout)
            reply.raise_for_status()
            with open(os.path.join(run_dir, 'map.png'), 'wb') as handle:
                handle.write(reply.content)
            # The web app runs on this PC in the normal setup: copy the rest of
            # its output (overview, transforms, diagnostics) when it is readable.
            output_dir = status.get('output_dir')
            if output_dir and os.path.isdir(output_dir):
                for name in _RESULT_FILES:
                    source = os.path.join(output_dir, name)
                    if os.path.isfile(source):
                        shutil.copyfile(source, os.path.join(run_dir, name))
            result['status'] = 'success'
        except (requests.RequestException, ValueError, OSError, RuntimeError,
                TimeoutError) as error:
            result.update(status='failed', error=f'{type(error).__name__}: {error}')
        return result

    # ------------------------------------------------------------------
    # stitching
    # ------------------------------------------------------------------
    def start_stitch(self, session: Session) -> Optional[threading.Thread]:
        """Produce the map for this session in a worker thread (web app or CLI)."""
        if session.stitching:
            self.get_logger().warn(f'{session.session_id}: a stitch is already running')
            return None
        if session.backend != 'webapp' and not os.path.isfile(self._stitch_python):
            self.get_logger().error(
                f'{session.session_id}: all frames saved but the stitcher venv is missing '
                f'({self._stitch_python}); stitch by hand once it exists')
            return None
        session.stitching = True
        thread = threading.Thread(target=self._stitch, args=(session,), daemon=True)
        thread.start()
        return thread

    def _stitch(self, session: Session) -> None:
        started = time.monotonic()
        if session.backend == 'webapp':
            self.get_logger().info(
                f'{session.session_id}: all frames at the web app; waiting for its map')
            result = self._stitch_via_webapp(session)
        else:
            result = self._stitch_via_cli(session)
        result['elapsed_s'] = time.monotonic() - started
        self._summarise_diagnostics(result)
        with open(os.path.join(session.directory, 'stitch_result.json'), 'w') as handle:
            json.dump(result, handle, indent=2)
        session.stitching = False
        self._results.put((session, result))

    def _stitch_via_cli(self, session: Session) -> dict:
        run_dir = _run_dir(session)
        manifest = os.path.join(session.directory, 'manifest.json')
        command = [self._stitch_python, '-I', '-m', 'drone_stitch', 'stitch', manifest,
                   '--output', run_dir, '--config', self._stitch_config]
        # The stitcher must see only its own venv: drop the ROS PYTHONPATH.
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONHOME', None)
        self.get_logger().info(f'{session.session_id}: stitching (CLI) -> {run_dir}')
        result: dict = {'session_id': session.session_id, 'backend': 'cli', 'run_dir': run_dir,
                        'command': command,
                        'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
        try:
            completed = subprocess.run(command, capture_output=True, text=True,
                                       timeout=self._stitch_timeout, env=env,
                                       cwd=self._stitch_project_dir, check=False)
            result.update(returncode=completed.returncode,
                          stdout=completed.stdout[-4000:], stderr=completed.stderr[-4000:])
            result['status'] = 'success' if (completed.returncode == 0 and os.path.isfile(
                os.path.join(run_dir, 'map.png'))) else 'failed'
        except (OSError, subprocess.TimeoutExpired) as error:
            result.update(status='failed', error=f'{type(error).__name__}: {error}')
        return result

    @staticmethod
    def _summarise_diagnostics(result: dict) -> None:
        diagnostics_path = os.path.join(result['run_dir'], 'diagnostics.json')
        if not os.path.isfile(diagnostics_path):
            return
        try:
            with open(diagnostics_path) as handle:
                diagnostics = json.load(handle)
            result['diagnostics'] = {
                key: diagnostics.get(key) for key in
                ('status', 'error', 'map_size_pixels', 'valid_pixel_fraction',
                 'elapsed_seconds', 'warnings')}
        except (OSError, ValueError):
            pass

    def _publish_results(self) -> None:
        """Report finished stitches and publish their images (executor thread)."""
        while True:
            try:
                session, result = self._results.get_nowait()
            except queue.Empty:
                return
            if result['status'] != 'success':
                reason = result.get('error') or f'see {result["run_dir"]}/diagnostics.json'
                self.get_logger().error(
                    f'{session.session_id}: stitch FAILED via {result["backend"]} ({reason}): '
                    f'{(result.get("stderr") or "").strip()[-300:]}')
                continue
            run_dir = result['run_dir']
            diag = result.get('diagnostics') or {}
            self.get_logger().info(
                f'{session.session_id}: stitch OK via {result["backend"]} in '
                f'{result["elapsed_s"]:.1f}s -> {run_dir}/map.png '
                f'{diag.get("map_size_pixels") or ""}')
            stamp = self.get_clock().now().to_msg()
            for publisher, name in ((self._map_pub, 'map.png'),
                                    (self._overview_pub, 'overview.png')):
                path = os.path.join(run_dir, name)
                if not os.path.isfile(path):
                    continue
                try:
                    publisher.publish(png_to_image(path, session.session_id, stamp))
                except (OSError, ValueError) as error:
                    self.get_logger().warn(f'could not publish {path}: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GroundStation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - surface config errors to the user
        print(f'ground_station failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
