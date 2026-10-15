#!/usr/bin/env python3
"""
Ground-station side of the mosaic mission.

Listens for ``drone_control_msgs/CapturedFrame`` from the drone.  For every
frame it immediately writes the photo to disk and refreshes a ``drone_stitch``
manifest, so the session folder is always a valid, partially filled stitcher
input.  When the last frame of a session arrives it runs the inherited
stitcher (``src/drone-stitching``) in that project's own virtualenv - the ROS
Python environment never imports OpenCV - and publishes the resulting map and
overview as ``sensor_msgs/Image`` for RViz.

Session folder::

    <output_root>/<session_id>/
        images/capture_01_id5.png ... capture_09_id7.png
        manifest.json          drone_stitch input (rewritten after every frame)
        frames.json            raw per-frame metadata as received
        stitch_01/             one stitcher run: map.png, overview.png,
                               transforms.json, diagnostics.json, ...
        stitch_result.json     status of the last run

Re-run a stitch by hand (same thing this node does)::

    <stitch_project_dir>/.venv/bin/python -m drone_stitch stitch
        <session>/manifest.json --output <session>/stitch_02 --config <config>

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
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from drone_control.image_io import image_to_pil, png_to_image
from drone_control_msgs.msg import CapturedFrame
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# drone_stitch.geometry.CAPTURE_ORDER - the only order the stitcher accepts.
CAPTURE_ORDER = (5, 4, 1, 2, 3, 6, 9, 8, 7)


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

        if not os.path.isfile(self._stitch_python):
            self.get_logger().warn(
                f'stitcher interpreter not found: {self._stitch_python}. Frames will still be '
                f'saved; create the venv as described in {self._stitch_project_dir}/README.md '
                f'(python3 scripts/setup_env.py) to enable stitching')
        self.get_logger().info(
            f'listening on {self._frame_topic}; sessions -> {self._output_root}; '
            f'stitcher: {self._stitch_python} (auto_stitch={self._auto_stitch})')

    # ------------------------------------------------------------------
    # parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter('frame_topic', '/survey/frames')
        self.declare_parameter('output_root', '~/drone_survey_sessions')
        self.declare_parameter('stitch_project_dir', '~/drone_ws/src/drone-stitching')
        self.declare_parameter('stitch_python', '')
        self.declare_parameter('stitch_config', '')
        self.declare_parameter('stitch_timeout_s', 600.0)
        self.declare_parameter('auto_stitch', True)
        self.declare_parameter('camera_hfov_rad', 1.2)
        self.declare_parameter('image_top_yaw_offset_rad', 0.0)
        self.declare_parameter('fallback_agl_m', 5.0)
        self.declare_parameter('map_topic', '/survey/mosaic/map')
        self.declare_parameter('overview_topic', '/survey/mosaic/overview')

    def _read_parameters(self) -> None:
        self._frame_topic = str(self.get_parameter('frame_topic').value)
        self._output_root = os.path.expanduser(str(self.get_parameter('output_root').value))
        os.makedirs(self._output_root, exist_ok=True)

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
            self._sessions[msg.session_id] = session
            self.get_logger().info(
                f'new session {msg.session_id}: expecting {msg.total} frames -> {directory}')

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
        session.frames[int(msg.sequence)] = {
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
        self._write_manifest(session)
        self.get_logger().info(
            f'{msg.session_id}: frame {msg.sequence}/{session.total} (cell {msg.grid_id}) saved '
            f'N={msg.ned_m[0]:.2f} E={msg.ned_m[1]:.2f} yaw={math.degrees(msg.yaw_rad):.1f}deg '
            f'agl={msg.agl_m:.2f}m' + ('' if msg.agl_valid else ' (invalid)'))

        if session.complete:
            if self._auto_stitch:
                self.start_stitch(session)
        elif msg.sequence == session.total:
            self.get_logger().error(
                f'{msg.session_id}: last frame arrived but frames {session.missing()} are '
                f'missing; not stitching')

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
        """Footprint + intrinsics of an ideal pinhole camera from hfov and image size."""
        width, height = session.image_size
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

    def _write_manifest(self, session: Session) -> None:
        ordered = [session.frames[seq] for seq in sorted(session.frames)]
        reference = next((f for f in ordered if f['grid_id'] == 5 and f['agl_m']), None)
        reference_agl = reference['agl_m'] if reference else self._fallback_agl
        images = []
        for frame in ordered:
            entry = {
                'id': frame['grid_id'],
                'file': frame['file'],
                'timestamp_s': frame['timestamp_s'],
                'ned_m': frame['ned_m'],
                'yaw_rad': frame['yaw_rad'],
            }
            if frame['agl_m']:
                entry['agl_m'] = frame['agl_m']
            images.append(entry)
        spacing = list(session.grid_spacing) if session.grid_spacing[0] > 0.0 else [2.5, 5.0]
        manifest = {
            'version': 1,
            'dataset_kind': 'real',
            'ned_frame_id': session.session_id,
            'camera': self._camera_block(session, reference_agl),
            'grid': {'spacing_m': spacing},
            'images': images,
            'ground_station': {
                'frames_received': len(ordered),
                'frames_expected': session.total,
                'expected_capture_order': list(CAPTURE_ORDER),
                'reference_agl_source': 'cell5_frame' if reference else 'fallback_agl_m',
            },
        }
        with open(os.path.join(session.directory, 'manifest.json'), 'w') as handle:
            json.dump(manifest, handle, indent=2)
        with open(os.path.join(session.directory, 'frames.json'), 'w') as handle:
            json.dump(ordered, handle, indent=2)

    # ------------------------------------------------------------------
    # stitching
    # ------------------------------------------------------------------
    def start_stitch(self, session: Session) -> Optional[threading.Thread]:
        """Run the stitcher for this session in a worker thread."""
        if session.stitching:
            self.get_logger().warn(f'{session.session_id}: a stitch is already running')
            return None
        if not os.path.isfile(self._stitch_python):
            self.get_logger().error(
                f'{session.session_id}: all frames saved but the stitcher venv is missing '
                f'({self._stitch_python}); stitch by hand once it exists')
            return None
        session.stitching = True
        thread = threading.Thread(target=self._stitch, args=(session,), daemon=True)
        thread.start()
        return thread

    def _stitch(self, session: Session) -> None:
        run_dir = _run_dir(session)
        manifest = os.path.join(session.directory, 'manifest.json')
        command = [self._stitch_python, '-m', 'drone_stitch', 'stitch', manifest,
                   '--output', run_dir, '--config', self._stitch_config]
        # The stitcher must see only its own venv: drop the ROS PYTHONPATH.
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        self.get_logger().info(f'{session.session_id}: stitching -> {run_dir}')
        started = time.monotonic()
        result: dict = {'session_id': session.session_id, 'run_dir': run_dir,
                        'command': command, 'started_utc': time.strftime(
                            '%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
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
        result['elapsed_s'] = time.monotonic() - started

        diagnostics_path = os.path.join(run_dir, 'diagnostics.json')
        if os.path.isfile(diagnostics_path):
            try:
                with open(diagnostics_path) as handle:
                    diagnostics = json.load(handle)
                result['diagnostics'] = {
                    key: diagnostics.get(key) for key in
                    ('status', 'error', 'map_size_pixels', 'valid_pixel_fraction',
                     'elapsed_seconds', 'warnings')}
            except (OSError, ValueError):
                pass
        with open(os.path.join(session.directory, 'stitch_result.json'), 'w') as handle:
            json.dump(result, handle, indent=2)
        session.stitching = False
        self._results.put((session, result))

    def _publish_results(self) -> None:
        """Report finished stitches and publish their images (executor thread)."""
        while True:
            try:
                session, result = self._results.get_nowait()
            except queue.Empty:
                return
            if result['status'] != 'success':
                self.get_logger().error(
                    f'{session.session_id}: stitch FAILED ({result.get("error") or "see "}'
                    f'{result["run_dir"]}/diagnostics.json): '
                    f'{(result.get("stderr") or "").strip()[-300:]}')
                continue
            run_dir = result['run_dir']
            diag = result.get('diagnostics') or {}
            self.get_logger().info(
                f'{session.session_id}: stitch OK in {result["elapsed_s"]:.1f}s -> '
                f'{run_dir}/map.png {diag.get("map_size_pixels")} px, '
                f'valid {100.0 * (diag.get("valid_pixel_fraction") or 0.0):.0f}%')
            stamp = self.get_clock().now().to_msg()
            for publisher, name in ((self._map_pub, 'map.png'),
                                    (self._overview_pub, 'overview.png')):
                path = os.path.join(run_dir, name)
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
