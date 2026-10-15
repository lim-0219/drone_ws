#!/usr/bin/env python3
"""
PX4 offboard waypoint navigation and survey-and-mark mission node.

Flies a list of waypoints (loaded from YAML, or a single target given as
parameters) in PX4 offboard mode and then, depending on ``final_action``:

* ``hover``  - holds a hover at the last waypoint,
* ``land``   - lands,
* ``survey`` - hovers, takes one photo with the downward camera, then holds
  position while drawing the three laser beams and their ground triangle as
  RViz markers (the beams are geometry only; nothing is simulated),
* ``mosaic`` - treats the target as cell 5 of a 3x3 grid, visits the nine
  cells in the order ``5 4 1 2 3 6 9 8 7`` the stitcher expects, photographs
  each one and streams every photo to the ground station
  (:mod:`drone_control.ground_station`) as a ``CapturedFrame`` the moment it
  is taken.  The ground station builds the ``drone_stitch`` manifest and runs
  the stitcher once all nine frames are in.

Coordinate convention
---------------------
PX4 works in a local NED frame: x = North, y = East, z = Down.  "Down" means a
point 5 m above the origin has ``z = -5.0``.  Writing negative numbers for
"up" is a classic source of mistakes, so the YAML file (and the ``target_*``
parameters) take a positive altitude and this node flips the sign exactly
once, in :func:`_parse_waypoint` / :meth:`WaypointNavigator._read_parameters`::

    z_ned = -altitude

The local origin is the EKF origin, which in SITL is where the vehicle sat
when the estimator initialised - i.e. the takeoff pad.

Yaw in the YAML file is in degrees by default (``yaw_in_degrees`` parameter),
following the NED convention: 0 = North, +90 = East, 180 = South, -90 = West.
It is converted to radians and wrapped into [-pi, pi] for PX4.

RViz markers and the ``map -> base_link`` transform are published in ENU
(x = East, y = North, z = Up); see :mod:`drone_control.laser_triangle`.

Topic names
-----------
PX4 main appends a message-version suffix (``_v4``, ``_v1``) to the DDS topic
name of every message that declares a non-zero ``MESSAGE_VERSION``.  The
suffix is added at *runtime* by the uXRCE-DDS client, so it does not show up
in the generated ``dds_topics.h`` or in the strings of the ``px4`` binary -
the only reliable check is ``ros2 topic list | grep fmu`` against a running
bridge.  The ``_TOPIC_*`` constants below hold the verified names.

Range finder path
-----------------
Gazebo ``gpu_lidar`` (1 beam) -> ros_gz bridge -> ``sensor_msgs/LaserScan``
-> this node -> ``px4_msgs/DistanceSensor`` on ``/fmu/in/distance_sensor``
-> EKF2 range-finder height aiding -> ``vehicle_local_position.dist_bottom``.
EKF2 picks the first ``distance_sensor`` instance whose
``orientation == ROTATION_DOWNWARD_FACING`` (EKF2.cpp, UpdateRangeSample),
so that field is what makes the measurement count.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
import json
import math
import os
import time
from typing import Callable, Deque, Dict, List, Optional, Tuple

from drone_control.image_io import image_to_pil
from drone_control.laser_triangle import (
    Beam,
    EmitterLayout,
    ground_beams,
    ned_to_enu,
    triangle_side,
)
from drone_control_msgs.msg import CapturedFrame
from geometry_msgs.msg import Point, TransformStamped
from px4_msgs.msg import (
    DistanceSensor,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml

# Topic names *relative* to the PX4 namespace.  Keeping them relative is what
# lets a second vehicle be driven by the same node under a different
# `px4_namespace` (e.g. '/px4_1') without touching the code.
#
# The `_vN` suffix is PX4's message version: the uXRCE-DDS client appends
# `_v<MESSAGE_VERSION>` at runtime for every message that declares a non-zero
# MESSAGE_VERSION, and leaves the name bare otherwise.  Verified against the
# live bridge with `ros2 topic list | grep fmu`:
#   VehicleLocalPosition MESSAGE_VERSION = 1 -> vehicle_local_position_v1
#   VehicleStatus        MESSAGE_VERSION = 4 -> vehicle_status_v4
#   TrajectorySetpoint / VehicleCommand  = 0 -> no suffix
#   OffboardControlMode  (no constant)       -> no suffix
#   DistanceSensor       (no constant)       -> no suffix
# Re-check with `ros2 topic list` after a PX4 update; a bumped version renames
# the topic and the subscription silently goes quiet.
_TOPIC_OFFBOARD_CONTROL_MODE = 'fmu/in/offboard_control_mode'
_TOPIC_TRAJECTORY_SETPOINT = 'fmu/in/trajectory_setpoint'
_TOPIC_VEHICLE_COMMAND = 'fmu/in/vehicle_command'
_TOPIC_DISTANCE_SENSOR = 'fmu/in/distance_sensor'
_TOPIC_VEHICLE_LOCAL_POSITION = 'fmu/out/vehicle_local_position_v1'
_TOPIC_VEHICLE_STATUS = 'fmu/out/vehicle_status_v4'

# PX4 custom main mode used by VEHICLE_CMD_DO_SET_MODE.  param1 = 1 selects
# "custom mode", param2 = 6 selects PX4_CUSTOM_MAIN_MODE_OFFBOARD.
_PX4_BASE_MODE_CUSTOM = 1.0
_PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

# Only the modes this node can end up in; anything else is logged numerically.
_NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'AUTO_LOITER',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'OFFBOARD',
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 'AUTO_LAND',
}

# DistanceSensor bookkeeping.  EKF2 ignores device_id; it only has to be
# stable and non-zero so the reading is attributable in logs.  The variance is
# the SDF ray resolution (0.01 m) squared.
_RANGE_DEVICE_ID = 0x52414E47  # 'RANG'
_RANGE_VARIANCE = 0.01 * 0.01

# Capture order of the 3x3 mosaic grid - the order drone_stitch requires
# (drone_stitch.geometry.CAPTURE_ORDER).  Cell layout, X right / Y up:
#     7 8 9
#     4 5 6
#     1 2 3
MOSAIC_CAPTURE_ORDER = (5, 4, 1, 2, 3, 6, 9, 8, 7)


class NavState(Enum):
    """
    States of the navigation state machine.

    Transitions (all of them, in one place):

    IDLE       -> TAKEOFF   : offboard stream warmed up AND nav_state == OFFBOARD
                              AND arming_state == ARMED
    TAKEOFF    -> NAVIGATE  : vertical climb target held within tolerance for
                              `reach_hold_time` seconds
    NAVIGATE   -> NAVIGATE  : current waypoint held within tolerance for
                              `reach_hold_time` seconds, waypoints remain
    NAVIGATE   -> CAPTURE   : a waypoint flagged `capture` reached (the survey
                              target, or every cell of the mosaic grid)
    CAPTURE    -> NAVIGATE  : photo saved (or `capture_timeout` elapsed, with a
                              warning) AND waypoints remain
    CAPTURE    -> <final>   : photo saved / timed out AND it was the last
                              waypoint - see "final" below
    NAVIGATE   -> <final>   : last waypoint reached and it needs no capture
    <final>                 : HOLD       if final_action == 'hover'
                              LANDING    if final_action == 'land'
                              LASER_HOLD if final_action == 'survey'
                              HOLD or LANDING per `after_mosaic` if 'mosaic'
    LANDING    -> LANDED    : arming_state == DISARMED
    HOLD, LASER_HOLD, LANDED: terminal
    """

    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    NAVIGATE = 'NAVIGATE'
    HOLD = 'HOLD'
    CAPTURE = 'CAPTURE'
    LASER_HOLD = 'LASER_HOLD'
    LANDING = 'LANDING'
    LANDED = 'LANDED'


@dataclass(frozen=True)
class Waypoint:
    """A single target expressed in the PX4 local NED frame."""

    x: float
    y: float
    z: float
    yaw: float
    name: str
    capture: bool = False                       # take a photo here
    grid_id: int = 0                            # 3x3 mosaic cell (1..9), 0 = none
    grid_xy: Tuple[float, float] = (0.0, 0.0)   # cell position relative to cell 5, metres

    @property
    def altitude(self) -> float:
        """Height above the local origin, in metres (positive up)."""
        return -self.z

    def describe(self) -> str:
        return (
            f'{self.name} (N={self.x:.2f} E={self.y:.2f} '
            f'alt={self.altitude:.2f}m yaw={math.degrees(self.yaw):.1f}deg)'
        )


def _wrap_pi(angle: float) -> float:
    """Wrap an angle in radians into [-pi, pi], which is what PX4 expects."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _parse_waypoint(index: int, entry: dict, yaw_in_degrees: bool) -> Waypoint:
    """
    Turn one YAML entry into a NED :class:`Waypoint`.

    This is the *only* place where the altitude sign is flipped for YAML
    input: the file carries a positive, human-friendly altitude and PX4 wants
    NED "down".
    """
    if not isinstance(entry, dict):
        raise ValueError(f'waypoint #{index} must be a mapping, got {type(entry).__name__}')

    missing = [key for key in ('x', 'y') if key not in entry]
    if missing:
        raise ValueError(f'waypoint #{index} is missing required key(s): {", ".join(missing)}')

    if 'altitude' in entry:
        altitude = float(entry['altitude'])
    elif 'z' in entry:
        # Accepted as an alias so that an x/y/z/yaw-style file still loads, but
        # it is read as a positive-up altitude, NOT as a raw NED z.
        altitude = float(entry['z'])
    else:
        raise ValueError(f'waypoint #{index} needs an "altitude" (or "z") key')

    yaw_raw = float(entry.get('yaw', 0.0))
    yaw = _wrap_pi(math.radians(yaw_raw) if yaw_in_degrees else yaw_raw)

    return Waypoint(
        x=float(entry['x']),
        y=float(entry['y']),
        z=-altitude,  # positive-up altitude -> NED down
        yaw=yaw,
        name=str(entry.get('name', f'wp{index}')),
    )


def load_waypoints(path: str, yaw_in_degrees: bool) -> List[Waypoint]:
    """Load and validate the waypoint list from a YAML file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f'waypoint file not found: {path}')

    with open(path, 'r') as handle:
        document = yaml.safe_load(handle)

    if not isinstance(document, dict) or 'waypoints' not in document:
        raise ValueError(f'{path}: expected a top-level "waypoints:" list')

    entries = document['waypoints']
    if not isinstance(entries, list) or not entries:
        raise ValueError(f'{path}: "waypoints" must be a non-empty list')

    return [_parse_waypoint(i, entry, yaw_in_degrees) for i, entry in enumerate(entries)]


def grid_cell(grid_id: int) -> Tuple[int, int]:
    """Return the (column, row) of a 3x3 cell, cell 5 at (0, 0) (drone_stitch GRID)."""
    return (grid_id - 1) % 3 - 1, (grid_id - 1) // 3 - 1


def camera_footprint(agl: float, hfov: float, width: int, height: int) -> Tuple[float, float]:
    """
    Compute the ground area seen by a nadir pinhole camera (along image u, along v).

    u = 2 * agl * tan(hfov / 2); square pixels give v = u * height / width.
    """
    along_u = 2.0 * agl * math.tan(hfov / 2.0)
    return along_u, along_u * height / width


def mosaic_waypoints(centre: Waypoint, spacing: Tuple[float, float],
                     heading: float) -> List[Waypoint]:
    """
    Build the nine capture waypoints of the mosaic in drone_stitch's capture order.

    The grid lives in the stitcher's local frame: cell 5 is the origin,
    X points along the image's right edge, Y along its top.  With the camera
    mounted so that image top = body forward (image_top_yaw_offset_rad = 0)
    the grid heading equals the vehicle yaw, and a cell at (X, Y) sits at::

        dN = -sin(heading) * X + cos(heading) * Y
        dE =  cos(heading) * X + sin(heading) * Y

    which is the inverse of drone_stitch.geometry.ned_to_local_matrix, so the
    stitcher's metadata-based initial placement reproduces exactly this grid.
    Every waypoint keeps the centre altitude and the same yaw.
    """
    s, c = math.sin(heading), math.cos(heading)
    waypoints = []
    for grid_id in MOSAIC_CAPTURE_ORDER:
        col, row = grid_cell(grid_id)
        gx, gy = col * spacing[0], row * spacing[1]
        waypoints.append(Waypoint(
            x=centre.x - s * gx + c * gy,
            y=centre.y + c * gx + s * gy,
            z=centre.z,
            yaw=heading,
            name=f'grid{grid_id}',
            capture=True,
            grid_id=grid_id,
            grid_xy=(gx, gy),
        ))
    return waypoints


class WaypointNavigator(Node):
    """Offboard waypoint navigation and survey state machine."""

    def __init__(self) -> None:
        super().__init__('waypoint_navigator')

        self._declare_parameters()
        self._read_parameters()

        # QoS used by the uXRCE-DDS bridge for every /fmu topic (matches the
        # working px4_ros_com example).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, self._topic(_TOPIC_OFFBOARD_CONTROL_MODE), qos)
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self._topic(_TOPIC_TRAJECTORY_SETPOINT), qos)
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, self._topic(_TOPIC_VEHICLE_COMMAND), qos)
        self._distance_sensor_pub = self.create_publisher(
            DistanceSensor, self._topic(_TOPIC_DISTANCE_SENSOR), qos)
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        # Frames go to the ground station reliably and are kept (transient
        # local, depth = frames per mission) so a ground station that starts
        # late, or restarts, still receives every photo of the session.
        self._frame_pub = self.create_publisher(
            CapturedFrame, self._frame_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST,
                       depth=max(1, self._capture_total)))

        self.create_subscription(
            VehicleLocalPosition, self._topic(_TOPIC_VEHICLE_LOCAL_POSITION),
            self._on_local_position, qos)
        self.create_subscription(
            VehicleStatus, self._topic(_TOPIC_VEHICLE_STATUS), self._on_vehicle_status, qos)
        # Sensor-data QoS (best effort) is compatible with both the reliable
        # and the best-effort profiles the ros_gz bridge may publish with.
        self.create_subscription(
            LaserScan, self._range_topic, self._on_range_scan, qos_profile_sensor_data)
        # RELIABLE on purpose: a 1280x960 rgb8 frame is 3.7 MB (~2,600 UDP
        # fragments).  With the Linux default net.core.rmem_max the receive
        # buffer overflows and a best-effort reader drops the whole frame
        # ('one image, then nothing').  The bridge publishes reliable, so a
        # reliable reader gets the missing fragments resent instead.
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, self._camera_topic, self._on_camera_image, camera_qos)

        self._local_position: Optional[VehicleLocalPosition] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._latest_image: Optional[Image] = None
        self._latest_image_time: Optional[float] = None
        self._images_received = 0
        self._image_times: Deque[float] = deque(maxlen=10)
        self._raw_range: Optional[float] = None
        self._raw_range_time: Optional[float] = None
        self._range_messages_published = 0

        self._state = NavState.IDLE
        self._warmup_ticks = 0
        self._last_command_tick = 0
        self._command_attempts = 0
        self._inside_tolerance_since: Optional[float] = None
        self._waypoint_index = 0
        self._takeoff_target: Optional[Waypoint] = None
        self._land_command_sent = False
        self._capture_started: Optional[float] = None
        self._photo_path: Optional[str] = None
        self._captures_done = 0

        self._handlers: Dict[NavState, Callable[[], None]] = {
            NavState.IDLE: self._run_idle,
            NavState.TAKEOFF: self._run_takeoff,
            NavState.NAVIGATE: self._run_navigate,
            NavState.HOLD: self._run_hold,
            NavState.CAPTURE: self._run_capture,
            NavState.LASER_HOLD: self._run_laser_hold,
            NavState.LANDING: self._run_landing,
            NavState.LANDED: self._run_landed,
        }

        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoint(s) from {self._waypoint_source}')
        for waypoint in self._waypoints:
            self.get_logger().info(f'  - {waypoint.describe()}')
        self.get_logger().info(
            f'namespace="{self._namespace or "<none>"}" rate={1.0 / self._timer_period:.1f}Hz '
            f'tolerance={self._tolerance}m hold={self._reach_hold_time}s '
            f'final_action={self._final_action}')
        if self._final_action == 'survey':
            self.get_logger().info(
                f'survey: camera={self._camera_topic} range={self._range_topic} '
                f'photos -> {self._photo_dir} '
                f'laser half-angle={math.degrees(self._laser_layout.half_angle):.1f}deg '
                f'markers -> {self._marker_topic} ({self._marker_frame})')
        if self._final_action == 'mosaic':
            self.get_logger().info(
                f'mosaic: session={self._session_id} {self._capture_total} cells, '
                f'spacing X={self._mosaic_spacing[0]:.2f}m Y={self._mosaic_spacing[1]:.2f}m, '
                f'frames -> {self._frame_topic}, after_mosaic={self._after_mosaic}')

        self._timer = self.create_timer(self._timer_period, self._on_timer)

    # ------------------------------------------------------------------
    # parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # navigation
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('position_tolerance', 0.3)
        self.declare_parameter('reach_hold_time', 1.0)
        self.declare_parameter('final_action', 'hover')
        self.declare_parameter('setpoint_warmup_count', 20)
        self.declare_parameter('yaw_in_degrees', True)
        # survey target (used when final_action == 'survey' and target_altitude > 0;
        # otherwise the survey happens at the last YAML waypoint)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        # sensors bridged from Gazebo
        self.declare_parameter('camera_topic', '/survey/camera/image')
        self.declare_parameter('range_topic', '/survey/range/scan')
        self.declare_parameter('range_sensor_down_offset', 0.05)
        # mosaic (final_action == 'mosaic'): 3x3 grid around the target
        self.declare_parameter('mosaic_overlap', 0.5)
        self.declare_parameter('mosaic_spacing_x', 0.0)
        self.declare_parameter('mosaic_spacing_y', 0.0)
        self.declare_parameter('camera_hfov_rad', 1.2)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 960)
        self.declare_parameter('after_mosaic', 'hover')
        self.declare_parameter('frame_topic', '/survey/frames')
        self.declare_parameter('session_id', '')
        # photo capture
        self.declare_parameter('photo_dir', '~/drone_survey_photos')
        self.declare_parameter('capture_settle_time', 2.0)
        self.declare_parameter('capture_timeout', 30.0)
        # laser triangle
        self.declare_parameter('laser_half_angle_deg', 15.0)
        self.declare_parameter('laser_mount_radius', 0.08)
        self.declare_parameter('laser_mount_down', 0.03)
        self.declare_parameter('marker_topic', '/survey/laser_markers')
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

    def _read_parameters(self) -> None:
        raw_namespace = str(self.get_parameter('px4_namespace').value).strip().rstrip('/')
        if raw_namespace and not raw_namespace.startswith('/'):
            raw_namespace = '/' + raw_namespace
        self._namespace = raw_namespace

        self._timer_period = float(self.get_parameter('timer_period').value)
        if self._timer_period <= 0.0:
            raise ValueError('parameter "timer_period" must be > 0')

        self._tolerance = float(self.get_parameter('position_tolerance').value)
        if self._tolerance <= 0.0:
            raise ValueError('parameter "position_tolerance" must be > 0')

        self._reach_hold_time = float(self.get_parameter('reach_hold_time').value)
        if self._reach_hold_time < 0.0:
            raise ValueError('parameter "reach_hold_time" must be >= 0')

        self._final_action = str(self.get_parameter('final_action').value).strip().lower()
        if self._final_action not in ('hover', 'land', 'survey', 'mosaic'):
            raise ValueError(
                'parameter "final_action" must be "hover", "land", "survey" or "mosaic"')
        self._after_mosaic = str(self.get_parameter('after_mosaic').value).strip().lower()
        if self._after_mosaic not in ('hover', 'land'):
            raise ValueError('parameter "after_mosaic" must be "hover" or "land"')

        self._warmup_required = int(self.get_parameter('setpoint_warmup_count').value)
        if self._warmup_required < 1:
            raise ValueError('parameter "setpoint_warmup_count" must be >= 1')

        yaw_in_degrees = bool(self.get_parameter('yaw_in_degrees').value)

        # Where the vehicle goes: a single parameter-given target for the survey
        # mission, otherwise the YAML list.  target_altitude is positive-up and
        # negated here, exactly like the YAML altitude in _parse_waypoint().
        target_altitude = float(self.get_parameter('target_altitude').value)
        waypoint_file = str(self.get_parameter('waypoint_file').value)
        self._mosaic_spacing: Tuple[float, float] = (0.0, 0.0)
        if self._final_action in ('survey', 'mosaic') and target_altitude > 0.0:
            target_yaw = float(self.get_parameter('target_yaw').value)
            target = Waypoint(
                x=float(self.get_parameter('target_x').value),
                y=float(self.get_parameter('target_y').value),
                z=-target_altitude,  # positive-up altitude -> NED down
                yaw=_wrap_pi(math.radians(target_yaw) if yaw_in_degrees else target_yaw),
                name='survey_target',
                capture=True,
                grid_id=5,
            )
            if self._final_action == 'survey':
                self._waypoints = [target]
                self._waypoint_source = 'target_x/target_y/target_altitude parameters'
            else:
                self._mosaic_spacing = self._mosaic_grid_spacing(target_altitude)
                self._waypoints = mosaic_waypoints(target, self._mosaic_spacing, target.yaw)
                self._waypoint_source = '3x3 mosaic grid around target_x/target_y/target_altitude'
        elif self._final_action == 'mosaic':
            raise ValueError('final_action "mosaic" needs "target_altitude" > 0 with '
                             '"target_x"/"target_y" (the grid is built around that point)')
        elif waypoint_file:
            self._waypoints = load_waypoints(waypoint_file, yaw_in_degrees)
            self._waypoint_source = waypoint_file
            if self._final_action == 'survey':
                # Survey at the last YAML waypoint.
                self._waypoints[-1] = replace(self._waypoints[-1], capture=True, grid_id=5)
        else:
            raise ValueError(
                'set parameter "waypoint_file" (path to the waypoint YAML), or for the survey '
                'mission set "target_altitude" > 0 together with "target_x"/"target_y"')
        self._capture_total = sum(1 for wp in self._waypoints if wp.capture)

        self._frame_topic = str(self.get_parameter('frame_topic').value)
        self._session_id = str(self.get_parameter('session_id').value).strip() or \
            time.strftime('session_%Y%m%dT%H%M%SZ', time.gmtime())

        self._camera_topic = str(self.get_parameter('camera_topic').value)
        self._range_topic = str(self.get_parameter('range_topic').value)
        self._range_down_offset = float(self.get_parameter('range_sensor_down_offset').value)

        self._photo_dir = os.path.expanduser(str(self.get_parameter('photo_dir').value))
        self._capture_settle_time = float(self.get_parameter('capture_settle_time').value)
        self._capture_timeout = float(self.get_parameter('capture_timeout').value)
        if self._capture_settle_time < 0.0 or self._capture_timeout <= 0.0:
            raise ValueError('"capture_settle_time" must be >= 0 and "capture_timeout" > 0')
        if self._final_action == 'survey':
            os.makedirs(self._photo_dir, exist_ok=True)

        half_angle_deg = float(self.get_parameter('laser_half_angle_deg').value)
        if not 0.0 <= half_angle_deg < 90.0:
            raise ValueError('"laser_half_angle_deg" must be in [0, 90)')
        self._laser_layout = EmitterLayout(
            half_angle=math.radians(half_angle_deg),
            mount_radius=float(self.get_parameter('laser_mount_radius').value),
            mount_down=float(self.get_parameter('laser_mount_down').value),
        )
        self._marker_topic = str(self.get_parameter('marker_topic').value)
        self._marker_frame = str(self.get_parameter('marker_frame').value)
        self._body_frame = str(self.get_parameter('body_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

    def _mosaic_grid_spacing(self, altitude: float) -> Tuple[float, float]:
        """Grid step: explicit `mosaic_spacing_x/y`, else footprint * (1 - overlap)."""
        override = (float(self.get_parameter('mosaic_spacing_x').value),
                    float(self.get_parameter('mosaic_spacing_y').value))
        if all(v > 0.0 for v in override):
            return override
        overlap = float(self.get_parameter('mosaic_overlap').value)
        if not 0.0 <= overlap < 1.0:
            raise ValueError('"mosaic_overlap" must be in [0, 1)')
        hfov = float(self.get_parameter('camera_hfov_rad').value)
        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        if not 0.0 < hfov < math.pi or width <= 0 or height <= 0:
            raise ValueError('"camera_hfov_rad" must be in (0, pi) and image size positive')
        along_u, along_v = camera_footprint(altitude, hfov, width, height)
        return along_u * (1.0 - overlap), along_v * (1.0 - overlap)

    def _topic(self, relative_name: str) -> str:
        """Prefix a relative PX4 topic with the configured namespace."""
        return f'{self._namespace}/{relative_name}'

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        previous = self._vehicle_status
        self._vehicle_status = msg

        if previous is None or previous.nav_state != msg.nav_state:
            name = _NAV_STATE_NAMES.get(msg.nav_state, f'nav_state {msg.nav_state}')
            self.get_logger().info(f'PX4 mode -> {name}')
        if previous is None or previous.arming_state != msg.arming_state:
            label = {
                VehicleStatus.ARMING_STATE_DISARMED: 'DISARMED',
                VehicleStatus.ARMING_STATE_ARMED: 'ARMED',
            }.get(msg.arming_state, str(msg.arming_state))
            self.get_logger().info(f'PX4 arming_state -> {label}')

    def _on_camera_image(self, msg: Image) -> None:
        self._latest_image = msg
        self._latest_image_time = self._now_seconds()
        self._image_times.append(self._latest_image_time)
        self._images_received += 1
        if self._images_received == 1:
            self.get_logger().info(
                f'Camera online: first image on {self._camera_topic} '
                f'({msg.width}x{msg.height} {msg.encoding})')

    def _camera_status(self) -> str:
        """One-line camera diagnostic for the capture logs."""
        if self._images_received == 0:
            return (f'no image has ever arrived on {self._camera_topic} - the ros_gz bridge is '
                    f'not running, or it cannot discover Gazebo (GZ_IP / GZ_PARTITION mismatch)')
        age = self._now_seconds() - (self._latest_image_time or 0.0)
        rate = ''
        if len(self._image_times) >= 2:
            span = self._image_times[-1] - self._image_times[0]
            if span > 0.0:
                rate = f', ~{(len(self._image_times) - 1) / span:.1f} Hz'
        hint = ''
        if age > 5.0:
            hint = (' - frames are being lost between the bridge and this node; large images '
                    'need a bigger UDP receive buffer (net.core.rmem_max), see README')
        return f'{self._images_received} images received, last one {age:.1f}s ago{rate}{hint}'

    def _on_range_scan(self, msg: LaserScan) -> None:
        """Forward the single-beam Gazebo range reading to PX4 as a DistanceSensor."""
        if not msg.ranges:
            return
        reading = float(msg.ranges[0])
        if not math.isfinite(reading) or not (msg.range_min <= reading <= msg.range_max):
            # No return within the sensor's limits (e.g. nothing under the
            # beam).  Publishing nothing lets EKF2's range timeout handle it.
            self.get_logger().warn(
                f'range reading {reading} outside [{msg.range_min}, {msg.range_max}] m; skipped',
                throttle_duration_sec=5.0)
            return

        self._raw_range = reading
        self._raw_range_time = self._now_seconds()

        out = DistanceSensor()
        out.timestamp = self._timestamp()
        out.device_id = _RANGE_DEVICE_ID
        out.min_distance = float(msg.range_min)
        out.max_distance = float(msg.range_max)
        out.current_distance = reading
        out.variance = _RANGE_VARIANCE
        out.signal_quality = 100
        out.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
        out.h_fov = 0.0
        out.v_fov = 0.0
        # EKF2 only fuses instances that declare themselves downward facing.
        out.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING
        out.mode = DistanceSensor.MODE_ENABLED
        self._distance_sensor_pub.publish(out)

        self._range_messages_published += 1
        if self._range_messages_published == 1:
            self.get_logger().info(
                f'Range finder online: {reading:.2f} m, forwarding to '
                f'{self._topic(_TOPIC_DISTANCE_SENSOR)}')

    def _is_offboard(self) -> bool:
        status = self._vehicle_status
        return status is not None and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _is_armed(self) -> bool:
        status = self._vehicle_status
        return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    # ------------------------------------------------------------------
    # publishing helpers
    # ------------------------------------------------------------------
    def _timestamp(self) -> int:
        """PX4 timestamps are microseconds since boot."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_offboard_control_mode(self) -> None:
        """Publish the offboard 'heartbeat'; PX4 exits offboard without it."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp()
        self._offboard_control_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self, target: Waypoint) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(target.x), float(target.y), float(target.z)]
        msg.yaw = float(target.yaw)
        msg.timestamp = self._timestamp()
        self._trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp()
        self._vehicle_command_pub.publish(msg)

    def _engage_offboard_mode(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=_PX4_BASE_MODE_CUSTOM,
            param2=_PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )

    def _arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _land(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Sent LAND command')

    def _publish_body_transform(self) -> None:
        """Broadcast map -> base_link (ENU) so RViz can place the vehicle."""
        if self._tf_broadcaster is None:
            return
        position = self._current_position()
        if position is None:
            return
        east, north, up = ned_to_enu((position.x, position.y, position.z))
        # NED heading is clockwise from North; ENU yaw is counter-clockwise from East.
        yaw_enu = math.pi / 2.0 - position.yaw

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._marker_frame
        transform.child_frame_id = self._body_frame
        transform.transform.translation.x = east
        transform.transform.translation.y = north
        transform.transform.translation.z = up
        transform.transform.rotation.z = math.sin(yaw_enu / 2.0)
        transform.transform.rotation.w = math.cos(yaw_enu / 2.0)
        self._tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------------------
    # survey helpers
    # ------------------------------------------------------------------
    def _current_agl(self) -> Optional[Tuple[float, str]]:
        """
        Height of the body origin above the ground, and where it came from.

        Preferred: the EKF's range-aided ``dist_bottom`` (already compensated
        for sensor offset and vehicle tilt).  Fallback: the raw range reading
        plus the sensor's mounting offset.  Never barometric / GPS altitude -
        see laser_triangle.py for why.
        """
        position = self._local_position
        if position is not None and position.dist_bottom_valid:
            return float(position.dist_bottom), 'dist_bottom'
        if self._raw_range is not None and self._raw_range_time is not None \
                and self._now_seconds() - self._raw_range_time < 0.5:
            return self._raw_range + self._range_down_offset, 'raw range'
        return None

    def _save_photo(self, image: Image, pose: Waypoint, target: Waypoint) -> str:
        """Write the image as PNG plus a JSON sidecar; return the PNG path."""
        pil = image_to_pil(image)
        stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        base = os.path.join(
            self._photo_dir,
            f'{self._session_id}_{stamp}_seq{self._captures_done + 1:02d}_id{target.grid_id}'
            f'_N{pose.x:+.2f}_E{pose.y:+.2f}_alt{pose.altitude:.2f}')
        png_path = base + '.png'
        pil.save(png_path)

        agl = self._current_agl()
        metadata = {
            'capture_time_utc': stamp,
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'image': {
                'stamp_sec': image.header.stamp.sec,
                'stamp_nanosec': image.header.stamp.nanosec,
                'frame_id': image.header.frame_id,
                'encoding': image.encoding,
                'width': image.width,
                'height': image.height,
                'topic': self._camera_topic,
            },
            'pose_ned': {
                'x_north_m': pose.x,
                'y_east_m': pose.y,
                'z_down_m': pose.z,
                'altitude_above_origin_m': pose.altitude,
                'heading_rad': pose.yaw,
                'heading_deg': math.degrees(pose.yaw),
            },
            'height_above_ground_m': None if agl is None else agl[0],
            'height_above_ground_source': None if agl is None else agl[1],
            'session_id': self._session_id,
            'sequence': self._captures_done + 1,
            'grid_id': target.grid_id,
            'grid_local_xy_m': list(target.grid_xy),
            'target': target.describe(),
            'state': self._state.value,
        }
        with open(base + '.json', 'w') as handle:
            json.dump(metadata, handle, indent=2)
        return png_path

    def _publish_frame(self, image: Image, pose: Waypoint, target: Waypoint) -> None:
        """Hand the photo and its pose to the ground station."""
        agl = self._current_agl()
        msg = CapturedFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._session_id
        msg.session_id = self._session_id
        msg.sequence = self._captures_done
        msg.grid_id = target.grid_id
        msg.total = self._capture_total
        msg.timestamp_s = self._now_seconds()
        msg.ned_m = [float(pose.x), float(pose.y), float(pose.z)]
        msg.yaw_rad = float(pose.yaw)
        msg.agl_valid = agl is not None
        msg.agl_m = float(agl[0]) if agl is not None else 0.0
        msg.agl_source = agl[1] if agl is not None else ''
        msg.grid_local_xy_m = [float(target.grid_xy[0]), float(target.grid_xy[1])]
        msg.grid_spacing_m = [float(self._mosaic_spacing[0]), float(self._mosaic_spacing[1])]
        msg.image = image
        self._frame_pub.publish(msg)
        self.get_logger().info(
            f'Frame {msg.sequence}/{msg.total} (cell {msg.grid_id}) sent to {self._frame_topic}')

    def _publish_laser_markers(self, beams: List[Beam], agl: float, source: str) -> None:
        """Draw the three beams, their ground spots and the ground triangle."""
        stamp = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

        def marker(marker_id: int, marker_type: int, r: float, g: float, b: float,
                   width: float) -> Marker:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._marker_frame
            m.ns = 'laser'
            m.id = marker_id
            m.type = marker_type
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.scale.y = width
            m.scale.z = width
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            m.lifetime = lifetime
            return m

        def point(ned: Tuple[float, float, float]) -> Point:
            east, north, up = ned_to_enu(ned)
            return Point(x=east, y=north, z=up)

        beams_marker = marker(0, Marker.LINE_LIST, 1.0, 0.1, 0.1, 0.03)
        spots_marker = marker(1, Marker.SPHERE_LIST, 1.0, 0.2, 0.2, 0.15)
        outline_marker = marker(2, Marker.LINE_STRIP, 1.0, 0.8, 0.1, 0.05)
        for beam in beams:
            beams_marker.points.append(point(beam.emitter_ned))
            beams_marker.points.append(point(beam.ground_ned))
            spots_marker.points.append(point(beam.ground_ned))
            outline_marker.points.append(point(beam.ground_ned))
        outline_marker.points.append(point(beams[0].ground_ned))  # close the triangle

        centroid = tuple(sum(b.ground_ned[i] for b in beams) / 3.0 for i in range(3))
        label = marker(3, Marker.TEXT_VIEW_FACING, 1.0, 1.0, 1.0, 0.4)
        label.pose.position = point((centroid[0], centroid[1], centroid[2] - 0.3))
        label.text = (f'AGL {agl:.2f} m ({source})  '
                      f'side {triangle_side(agl, self._laser_layout):.2f} m')

        self._marker_pub.publish(
            MarkerArray(markers=[beams_marker, spots_marker, outline_marker, label]))

    # ------------------------------------------------------------------
    # state machine
    # ------------------------------------------------------------------
    def _transition(self, new_state: NavState, reason: str) -> None:
        self.get_logger().info(f'STATE {self._state.value} -> {new_state.value} ({reason})')
        self._state = new_state
        self._inside_tolerance_since = None

    def _on_timer(self) -> None:
        self._publish_body_transform()

        # Once the land command is out, PX4 owns the vehicle in AUTO_LAND.
        # Publishing the offboard heartbeat then would fight that mode, so
        # these two states are handled before the heartbeat is sent.
        if self._state in (NavState.LANDING, NavState.LANDED):
            self._handlers[self._state]()
            return

        self._publish_offboard_control_mode()
        self._handlers[self._state]()

    def _current_position(self) -> Optional[Waypoint]:
        """Return the current position as a Waypoint, or None if not yet valid."""
        position = self._local_position
        if position is None or not (position.xy_valid and position.z_valid):
            return None
        return Waypoint(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=float(position.heading),
            name='current',
        )

    def _distance_to(self, target: Waypoint) -> Optional[float]:
        current = self._current_position()
        if current is None:
            return None
        return math.dist((current.x, current.y, current.z), (target.x, target.y, target.z))

    def _target_reached(self, target: Waypoint) -> bool:
        """Report whether `target` has been held within tolerance long enough."""
        distance = self._distance_to(target)
        if distance is None:
            self._inside_tolerance_since = None
            return False

        now = self._now_seconds()
        if distance > self._tolerance:
            self._inside_tolerance_since = None
            return False

        if self._inside_tolerance_since is None:
            self._inside_tolerance_since = now
        return (now - self._inside_tolerance_since) >= self._reach_hold_time

    def _run_idle(self) -> None:
        """Stream setpoints, then request OFFBOARD and ARM (in that order)."""
        # Hold station at the current position so that engaging offboard does
        # not command a jump; fall back to the origin until the EKF is ready.
        seed = self._current_position() or Waypoint(0.0, 0.0, 0.0, 0.0, 'origin')
        self._publish_trajectory_setpoint(seed)

        self._warmup_ticks += 1
        if self._warmup_ticks < self._warmup_required:
            return

        if self._is_offboard() and self._is_armed():
            first = self._waypoints[0]
            start = self._current_position() or seed
            # Vertical climb first: keep the current x/y, take the altitude and
            # yaw from the first waypoint, then move horizontally in NAVIGATE.
            self._takeoff_target = Waypoint(
                x=start.x, y=start.y, z=first.z, yaw=first.yaw, name='takeoff')
            self._transition(NavState.TAKEOFF, 'offboard engaged and armed')
            self.get_logger().info(f'Climbing to {self._takeoff_target.describe()}')
            return

        # First attempt fires as soon as the warmup stream is done; after that
        # re-send roughly once a second until PX4 accepts both commands.
        retry_ticks = max(1, round(1.0 / self._timer_period))
        if self._command_attempts and self._warmup_ticks - self._last_command_tick < retry_ticks:
            return
        self._last_command_tick = self._warmup_ticks
        self._engage_offboard_mode()
        self._arm()

        self._command_attempts += 1
        if self._command_attempts == 1:
            self.get_logger().info(
                f'Streamed {self._warmup_ticks} setpoints; requesting OFFBOARD mode and ARM')
        elif self._vehicle_status is None:
            self.get_logger().warn(
                f'No {self._topic(_TOPIC_VEHICLE_STATUS)} received after '
                f'{self._command_attempts} attempts - is the uXRCE-DDS agent running and '
                f'connected to PX4?', throttle_duration_sec=5.0)
        else:
            self.get_logger().warn(
                f'PX4 has not accepted OFFBOARD+ARM yet (attempt {self._command_attempts}); '
                f'retrying', throttle_duration_sec=5.0)

    def _run_takeoff(self) -> None:
        target = self._takeoff_target
        assert target is not None  # set on the IDLE -> TAKEOFF transition
        self._publish_trajectory_setpoint(target)

        if not self._check_still_offboard():
            return

        if self._target_reached(target):
            self._transition(NavState.NAVIGATE, f'reached takeoff altitude {target.altitude:.2f}m')
            self.get_logger().info(f'Heading to {self._waypoints[0].describe()}')

    def _run_navigate(self) -> None:
        target = self._waypoints[self._waypoint_index]
        self._publish_trajectory_setpoint(target)

        if not self._check_still_offboard():
            return

        if not self._target_reached(target):
            return

        self.get_logger().info(
            f'Reached waypoint {self._waypoint_index + 1}/{len(self._waypoints)}: '
            f'{target.describe()}')

        if target.capture:
            self._capture_started = self._now_seconds()
            self._transition(NavState.CAPTURE, f'photo requested at {target.name}')
            return

        self._advance_or_finish()

    def _advance_or_finish(self) -> None:
        """Head for the next waypoint, or run the final action after the last one."""
        if self._waypoint_index + 1 < len(self._waypoints):
            self._waypoint_index += 1
            self._inside_tolerance_since = None
            nxt = self._waypoints[self._waypoint_index]
            self.get_logger().info(f'Heading to {nxt.describe()}')
            if self._state is not NavState.NAVIGATE:
                self._transition(NavState.NAVIGATE, 'next waypoint')
            return
        self._finish_mission()

    def _finish_mission(self) -> None:
        """Dispatch on final_action once every waypoint (and photo) is done."""
        if self._final_action == 'land':
            self._transition(NavState.LANDING, 'last waypoint reached, final_action=land')
        elif self._final_action == 'survey':
            self._transition(NavState.LASER_HOLD, 'survey photo done, final_action=survey')
        elif self._final_action == 'mosaic':
            reason = (f'{self._captures_done}/{self._capture_total} frames captured, '
                      f'after_mosaic={self._after_mosaic}')
            if self._after_mosaic == 'land':
                self._transition(NavState.LANDING, reason)
            else:
                self._transition(NavState.HOLD, reason)
        else:
            self._transition(NavState.HOLD, 'last waypoint reached, final_action=hover')

    def _run_hold(self) -> None:
        """Hover at the last waypoint indefinitely."""
        target = self._waypoints[-1]
        self._publish_trajectory_setpoint(target)
        self._check_still_offboard()
        self.get_logger().info(
            f'Holding hover at {target.describe()}', throttle_duration_sec=10.0)

    def _run_capture(self) -> None:
        """Hover at the current waypoint, settle, save one frame and send it onwards."""
        target = self._waypoints[self._waypoint_index]
        self._publish_trajectory_setpoint(target)
        self._check_still_offboard()

        assert self._capture_started is not None  # set on the NAVIGATE -> CAPTURE transition
        now = self._now_seconds()
        elapsed = now - self._capture_started
        if elapsed < self._capture_settle_time:
            return

        # The photo must show the vehicle *at the waypoint*: accept the first
        # frame received after the settle period, however slowly the
        # (possibly software-rendered) camera delivers frames.
        ready_since = self._capture_started + self._capture_settle_time
        if self._latest_image is not None and self._latest_image_time is not None \
                and self._latest_image_time >= ready_since:
            pose = self._current_position() or target
            try:
                self._photo_path = self._save_photo(self._latest_image, pose, target)
            except (OSError, ValueError) as error:
                self.get_logger().error(f'Failed to save photo at {target.name}: {error}')
                self._advance_or_finish()
                return
            self._captures_done += 1
            self.get_logger().info(
                f'Photo {self._captures_done}/{self._capture_total} saved: {self._photo_path}')
            self._publish_frame(self._latest_image, pose, target)
            self._advance_or_finish()
            return

        if elapsed >= self._capture_timeout:
            self.get_logger().warn(
                f'No image taken at {target.name} within {self._capture_timeout:.0f}s '
                f'({self._camera_status()}); continuing without a photo')
            self._advance_or_finish()
            return

        self.get_logger().warn(
            f'Waiting for a camera frame taken after settling: {self._camera_status()}',
            throttle_duration_sec=5.0)

    def _run_laser_hold(self) -> None:
        """Hover at the target and draw the laser triangle on the ground."""
        target = self._waypoints[-1]
        self._publish_trajectory_setpoint(target)
        self._check_still_offboard()

        position = self._current_position()
        agl = self._current_agl()
        if position is None or agl is None:
            self.get_logger().warn(
                'Laser triangle needs a valid position and a range-finder height '
                '(dist_bottom_valid or a fresh range reading); nothing drawn',
                throttle_duration_sec=5.0)
            return

        height, source = agl
        try:
            beams = ground_beams((position.x, position.y, position.z), position.yaw,
                                 height, self._laser_layout)
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)
            return

        self._publish_laser_markers(beams, height, source)
        self.get_logger().info(
            f'Laser hold: AGL {height:.2f} m ({source}), '
            f'triangle side {triangle_side(height, self._laser_layout):.2f} m',
            throttle_duration_sec=5.0)

    def _run_landing(self) -> None:
        if not self._land_command_sent:
            self._land()
            self._land_command_sent = True

        if self._vehicle_status is not None and not self._is_armed():
            self._transition(NavState.LANDED, 'vehicle disarmed after landing')

    def _run_landed(self) -> None:
        self.get_logger().info('Landed and disarmed', once=True)

    def _check_still_offboard(self) -> bool:
        """Warn if PX4 left offboard mode (failsafe, RC override, ...)."""
        if self._is_offboard():
            return True
        self.get_logger().warn(
            'PX4 is not in OFFBOARD mode; still streaming setpoints',
            throttle_duration_sec=5.0)
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - surface config errors to the user
        print(f'waypoint_navigator failed to start: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
