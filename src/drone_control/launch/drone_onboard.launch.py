r"""
Onboard launch for the REAL vehicle (runs on the Raspberry Pi 5).

Starts the mission node and the survey camera driver (Arducam B0353 through
Picamera2).  The laser/servo controller is NOT started unless ``laser:=true``
(mid-term presentation runs fly without the payload moving).  PX4 talks to
ROS through the uXRCE-DDS agent on the TELEM2 serial link, started separately:

    MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

The ground station and the stitcher web app run on the ground PC.  Default
mission: a 20 x 10 m mosaic around (target_x, target_y) at the altitude the
B0353's footprint needs for that area (6.25 m with the stock 3.6 mm lens,
see ``ros2 run drone_control mosaic_plan``)::

    ros2 launch drone_control drone_onboard.launch.py \\
        target_x:=5.0 target_y:=0.0 \\
        camera_info_url:=~/camera_calibration/b0353.yaml camera_hfov_rad:=1.3495

After a checkerboard calibration set ``camera_hfov_rad := 2*atan(W/(2*fx))``;
the navigator warns at start-up if the published CameraInfo disagrees.

Lasers: ``laser:=true laser_hardware_enable:=true laser_backend:=rpi`` and,
for the servo mechanism, ``laser_servo_enable:=true``.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'drone_control'
CAMERA_TOPIC = '/survey/camera/image'
CAMERA_INFO_TOPIC = '/survey/camera/camera_info'
FRAME_TOPIC = '/survey/frames'

# Arducam B0353: AR0234 1920x1200 @ 3.0 um behind a 3.6 mm M12 lens ->
# fx = 1200 px, hfov = 2*atan(960/1200) = 1.3495 rad (77.3 deg).  Conservative
# (the brochure's 90 deg would only add overlap); replace after calibration.
B0353_WIDTH = 1920
B0353_HEIGHT = 1200
B0353_HFOV_RAD = 2.0 * math.atan(B0353_WIDTH * 3.0e-3 / 2.0 / 3.6)

# Mission requirement: the stitched image covers 20 m along image right (X,
# body right) and 10 m along image top (Y, body forward).  At 50 % overlap
# along X the B0353 needs a 10 m wide footprint -> 6.25 m AGL.
MOSAIC_EXTENT_X = 20.0
MOSAIC_EXTENT_Y = 10.0
DEFAULT_ALTITUDE = MOSAIC_EXTENT_X / (3.0 - 2.0 * 0.5) / (2.0 * math.tan(B0353_HFOV_RAD / 2.0))


def _typed(name: str, value_type: type) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument('mission', default_value='mosaic',
                              description='final_action: mosaic, survey, hover or land.'),
        DeclareLaunchArgument('target_x', default_value='5.0',
                              description='Cell-5 North coordinate, metres (NED x).'),
        DeclareLaunchArgument('target_y', default_value='0.0',
                              description='Cell-5 East coordinate, metres (NED y).'),
        DeclareLaunchArgument('target_altitude', default_value=f'{DEFAULT_ALTITUDE:.2f}',
                              description='Flight altitude above the takeoff point, metres. '
                                          '0 = derive from mosaic_extent_x and mosaic_overlap.'),
        DeclareLaunchArgument('target_yaw', default_value='0.0',
                              description='Heading held over the grid (deg, 0 = North). The '
                                          '20 m side of the mosaic runs along body right.'),
        DeclareLaunchArgument('mosaic_extent_x', default_value=f'{MOSAIC_EXTENT_X:g}',
                              description='Stitched size along image right, metres '
                                          '(0 = legacy footprint*(1-overlap) step).'),
        DeclareLaunchArgument('mosaic_extent_y', default_value=f'{MOSAIC_EXTENT_Y:g}',
                              description='Stitched size along image top, metres.'),
        DeclareLaunchArgument('mosaic_overlap', default_value='0.5',
                              description='Overlap along X used when target_altitude is 0.'),
        DeclareLaunchArgument('mosaic_min_overlap', default_value='0.3',
                              description='Refuse to fly a grid with less overlap than this.'),
        DeclareLaunchArgument('after_mosaic', default_value='land'),
        DeclareLaunchArgument('camera_hfov_rad', default_value=f'{B0353_HFOV_RAD:.4f}',
                              description='2*atan(W/(2*fx)); B0353 nominal 1.3495. Sets the '
                                          'footprint and therefore the grid step.'),
        DeclareLaunchArgument('position_tolerance', default_value='0.5'),
        DeclareLaunchArgument('reach_hold_time', default_value='2.0'),
        DeclareLaunchArgument('capture_settle_time', default_value='3.0'),
        DeclareLaunchArgument('capture_timeout', default_value='20.0',
                              description='Seconds to wait for a frame per attempt.'),
        DeclareLaunchArgument('capture_attempts', default_value='3',
                              description='Attempts per cell before the mission HOLDs there.'),
        DeclareLaunchArgument('photo_dir', default_value='~/drone_survey_photos'),
        # camera (survey_camera node)
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the survey_camera node here.'),
        DeclareLaunchArgument('camera_backend', default_value='picamera2',
                              description='"picamera2" (B0353 on the Pi) or "v4l2" (USB cam).'),
        DeclareLaunchArgument('camera_num', default_value='0',
                              description='Pi 5 connector: 0 = CAM/DISP 0, 1 = CAM/DISP 1.'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0',
                              description='v4l2 backend only.'),
        DeclareLaunchArgument('camera_width', default_value=str(B0353_WIDTH)),
        DeclareLaunchArgument('camera_height', default_value=str(B0353_HEIGHT)),
        DeclareLaunchArgument('camera_fps', default_value='3',
                              description='6.9 MB per frame on the local DDS; 3 Hz is plenty '
                                          'for one photo per cell.'),
        DeclareLaunchArgument('camera_rotate_deg', default_value='0',
                              description='0/90/180/270 so that image top = body forward.'),
        DeclareLaunchArgument('camera_exposure_us', default_value='0',
                              description='Fixed exposure in microseconds (0 = auto). Try '
                                          '2000-4000 in daylight to freeze motion.'),
        DeclareLaunchArgument('camera_analogue_gain', default_value='1.0'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='camera_calibration YAML; publishes CameraInfo.'),
        # laser / servo payload (off by default)
        DeclareLaunchArgument('laser', default_value='false',
                              description='Start laser_control at all. Default false so a '
                                          'mission run never touches the payload.'),
        DeclareLaunchArgument('laser_backend', default_value='mock',
                              description='"mock" (log only) or "rpi" (gpiozero BCM pins).'),
        DeclareLaunchArgument('laser_hardware_enable', default_value='false'),
        DeclareLaunchArgument('laser_servo_enable', default_value='false',
                              description='Let laser_control move the servos (DO_SET_ACTUATOR).'),
        DeclareLaunchArgument('laser_pattern', default_value='triangle'),
    ]

    navigator = Node(
        package=PACKAGE_NAME, executable='waypoint_navigator', name='waypoint_navigator',
        output='screen', emulate_tty=True,
        parameters=[{
            'final_action': _typed('mission', str),
            'target_x': _typed('target_x', float),
            'target_y': _typed('target_y', float),
            'target_altitude': _typed('target_altitude', float),
            'target_yaw': _typed('target_yaw', float),
            'yaw_in_degrees': True,
            'mosaic_extent_x': _typed('mosaic_extent_x', float),
            'mosaic_extent_y': _typed('mosaic_extent_y', float),
            'mosaic_overlap': _typed('mosaic_overlap', float),
            'mosaic_min_overlap': _typed('mosaic_min_overlap', float),
            'after_mosaic': _typed('after_mosaic', str),
            'camera_hfov_rad': _typed('camera_hfov_rad', float),
            'image_width': _typed('camera_width', int),
            'image_height': _typed('camera_height', int),
            'position_tolerance': _typed('position_tolerance', float),
            'reach_hold_time': _typed('reach_hold_time', float),
            'capture_settle_time': _typed('capture_settle_time', float),
            'capture_timeout': _typed('capture_timeout', float),
            'capture_attempts': _typed('capture_attempts', int),
            'camera_topic': CAMERA_TOPIC,
            'camera_info_topic': CAMERA_INFO_TOPIC,
            'range_topic': '/survey/range/scan',   # unused: the TF-Luna is on the Pixhawk
            'photo_dir': _typed('photo_dir', str),
            'frame_topic': FRAME_TOPIC,
            'laser_pattern': _typed('laser_pattern', str),
        }],
    )

    camera = Node(
        package=PACKAGE_NAME, executable='survey_camera', name='survey_camera',
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{
            'backend': _typed('camera_backend', str),
            'camera_num': _typed('camera_num', int),
            'device': _typed('camera_device', str),
            'width': _typed('camera_width', int),
            'height': _typed('camera_height', int),
            'fps': _typed('camera_fps', int),
            'rotate_deg': _typed('camera_rotate_deg', int),
            'exposure_us': _typed('camera_exposure_us', int),
            'analogue_gain': _typed('camera_analogue_gain', float),
            'image_topic': CAMERA_TOPIC,
            'camera_info_topic': CAMERA_INFO_TOPIC,
            'camera_info_url': _typed('camera_info_url', str),
        }],
    )

    laser = Node(
        package=PACKAGE_NAME, executable='laser_control', name='laser_control',
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('laser')),
        parameters=[{
            'backend': _typed('laser_backend', str),
            'hardware_enable': _typed('laser_hardware_enable', bool),
            'servo_enable': _typed('laser_servo_enable', bool),
        }],
    )

    return LaunchDescription(arguments + [navigator, camera, laser])
