"""
Onboard launch for the REAL vehicle (runs on the Jetson).

Starts the mission node and the laser/servo controller; the camera driver is
started here too unless `camera:=false` (e.g. when a different driver is
used). PX4 talks to ROS through the uXRCE-DDS agent on the TELEM2 serial link,
started separately:

    MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

The ground station and the stitcher web app run on the ground PC. Example::

    ros2 launch drone_control drone_onboard.launch.py \
        target_x:=5.0 target_y:=0.0 target_altitude:=5.0 \
        camera_info_url:=~/camera_calibration/imx219.yaml

Lasers stay off until `laser_hardware_enable:=true laser_backend:=jetson`.
"""

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


def _typed(name: str, value_type: type) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument('mission', default_value='mosaic',
                              description='final_action: mosaic, survey, hover or land.'),
        DeclareLaunchArgument('target_x', default_value='5.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_altitude', default_value='5.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('mosaic_overlap', default_value='0.5'),
        DeclareLaunchArgument('after_mosaic', default_value='land'),
        DeclareLaunchArgument('camera_hfov_rad', default_value='1.2',
                              description='2*atan(W/(2*fx)) from the calibration; '
                                          'sets the grid step.'),
        DeclareLaunchArgument('position_tolerance', default_value='0.5'),
        DeclareLaunchArgument('reach_hold_time', default_value='2.0'),
        DeclareLaunchArgument('capture_settle_time', default_value='3.0'),
        DeclareLaunchArgument('photo_dir', default_value='~/drone_survey_photos'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_sensor_id', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='5'),
        DeclareLaunchArgument('camera_flip', default_value='0'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('laser_backend', default_value='mock',
                              description='"mock" (log only) or "jetson" (Jetson.GPIO pins).'),
        DeclareLaunchArgument('laser_hardware_enable', default_value='false'),
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
            'mosaic_overlap': _typed('mosaic_overlap', float),
            'after_mosaic': _typed('after_mosaic', str),
            'camera_hfov_rad': _typed('camera_hfov_rad', float),
            'image_width': _typed('camera_width', int),
            'image_height': _typed('camera_height', int),
            'position_tolerance': _typed('position_tolerance', float),
            'reach_hold_time': _typed('reach_hold_time', float),
            'capture_settle_time': _typed('capture_settle_time', float),
            'camera_topic': CAMERA_TOPIC,
            'range_topic': '/survey/range/scan',   # unused when the TF-Luna is on the Pixhawk
            'photo_dir': _typed('photo_dir', str),
            'frame_topic': FRAME_TOPIC,
            'laser_pattern': _typed('laser_pattern', str),
        }],
    )

    camera = Node(
        package=PACKAGE_NAME, executable='csi_camera', name='csi_camera',
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{
            'sensor_id': _typed('camera_sensor_id', int),
            'width': _typed('camera_width', int),
            'height': _typed('camera_height', int),
            'fps': _typed('camera_fps', int),
            'flip_method': _typed('camera_flip', int),
            'image_topic': CAMERA_TOPIC,
            'camera_info_topic': CAMERA_INFO_TOPIC,
            'camera_info_url': _typed('camera_info_url', str),
        }],
    )

    laser = Node(
        package=PACKAGE_NAME, executable='laser_control', name='laser_control',
        output='screen', emulate_tty=True,
        parameters=[{
            'backend': _typed('laser_backend', str),
            'hardware_enable': _typed('laser_hardware_enable', bool),
        }],
    )

    return LaunchDescription(arguments + [navigator, camera, laser])
