"""
Launch the 9-frame mosaic mission: Gazebo bridge + navigator + ground station + RViz.

Requires PX4 SITL running the x500_survey model (`make px4_sitl gz_x500_survey`),
the uXRCE-DDS agent, and the stitcher venv (`src/drone-stitching/.venv`).  Example::

    ros2 launch drone_control mosaic_mission.launch.py \
        target_x:=5.0 target_y:=0.0 target_altitude:=5.0 mosaic_overlap:=0.5

The ground station can also run on another machine: start it there with
`ros2 run drone_control ground_station` (same ROS_DOMAIN_ID) and pass
`ground_station:=false` here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'drone_control'

# Pinned in the x500_survey model SDF; the camera's horizontal FOV too (1.2 rad).
GZ_CAMERA_TOPIC = '/survey/camera/image'
GZ_RANGE_TOPIC = '/survey/range/scan'
CAMERA_HFOV_RAD = 1.2
FRAME_TOPIC = '/survey/frames'


def _typed(name: str, value_type: type) -> ParameterValue:
    """Launch arguments arrive as strings; declare the type the node expects."""
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    default_rviz = os.path.join(share, 'config', 'mosaic.rviz')
    default_stitch_config = os.path.join(share, 'config', 'stitch_config.json')

    arguments = [
        # mission target = grid cell 5 (NED position, positive-up altitude)
        DeclareLaunchArgument('target_x', default_value='5.0',
                              description='Cell-5 North coordinate in metres (NED x).'),
        DeclareLaunchArgument('target_y', default_value='0.0',
                              description='Cell-5 East coordinate in metres (NED y).'),
        DeclareLaunchArgument('target_altitude', default_value='5.0',
                              description='Flight altitude above the takeoff point, metres.'),
        DeclareLaunchArgument('target_yaw', default_value='0.0',
                              description='Heading held for the whole grid (deg, NED, 0=N). '
                                          'The grid +Y axis points this way.'),
        DeclareLaunchArgument('mosaic_overlap', default_value='0.5',
                              description='Fraction of the camera footprint shared by '
                                          'neighbouring cells; grid step = '
                                          'footprint * (1 - overlap).'),
        DeclareLaunchArgument('mosaic_spacing_x', default_value='0.0',
                              description='Explicit grid step along image right, metres '
                                          '(0 = derive from footprint and overlap).'),
        DeclareLaunchArgument('mosaic_spacing_y', default_value='0.0',
                              description='Explicit grid step along image top, metres.'),
        DeclareLaunchArgument('after_mosaic', default_value='hover',
                              description='"hover" or "land" after the ninth photo.'),
        DeclareLaunchArgument('session_id', default_value='',
                              description='Session name (empty = session_<UTC time>).'),
        # navigation
        DeclareLaunchArgument('px4_namespace', default_value=''),
        DeclareLaunchArgument('position_tolerance', default_value='0.3'),
        DeclareLaunchArgument('reach_hold_time', default_value='1.0'),
        DeclareLaunchArgument('capture_settle_time', default_value='2.0'),
        DeclareLaunchArgument('photo_dir', default_value='~/drone_survey_photos',
                              description='Drone-side copy of every photo (PNG + JSON).'),
        # ground station
        DeclareLaunchArgument('ground_station', default_value='true',
                              description='Run the ground station here (false if it runs on '
                                          'another machine).'),
        DeclareLaunchArgument('output_root', default_value='~/drone_survey_sessions',
                              description='Ground-station session folders.'),
        DeclareLaunchArgument('stitch_project_dir',
                              default_value='~/drone_ws/src/drone-stitching',
                              description='The drone-stitching checkout (its .venv is used).'),
        DeclareLaunchArgument('stitch_config', default_value=default_stitch_config),
        DeclareLaunchArgument('auto_stitch', default_value='true'),
        DeclareLaunchArgument('stitch_backend', default_value='auto',
                              description='"auto" (web app if reachable, else CLI), '
                                          '"webapp" or "cli".'),
        DeclareLaunchArgument('webapp_url', default_value='http://127.0.0.1:8000',
                              description='The stitcher web app (scripts/start_webapp.py).'),
        # processes
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('gz_ip', default_value='127.0.0.1',
                              description='GZ_IP for the bridge; PX4 starts Gazebo with '
                                          '127.0.0.1 and gz-transport only discovers peers '
                                          'on the same interface.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='survey_gz_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        additional_env={'GZ_IP': LaunchConfiguration('gz_ip')},
        arguments=[
            f'{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{GZ_RANGE_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    navigator = Node(
        package=PACKAGE_NAME,
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'final_action': 'mosaic',
            'target_x': _typed('target_x', float),
            'target_y': _typed('target_y', float),
            'target_altitude': _typed('target_altitude', float),
            'target_yaw': _typed('target_yaw', float),
            'yaw_in_degrees': True,
            'mosaic_overlap': _typed('mosaic_overlap', float),
            'mosaic_spacing_x': _typed('mosaic_spacing_x', float),
            'mosaic_spacing_y': _typed('mosaic_spacing_y', float),
            'camera_hfov_rad': CAMERA_HFOV_RAD,
            'after_mosaic': _typed('after_mosaic', str),
            'session_id': _typed('session_id', str),
            'frame_topic': FRAME_TOPIC,
            'px4_namespace': _typed('px4_namespace', str),
            'position_tolerance': _typed('position_tolerance', float),
            'reach_hold_time': _typed('reach_hold_time', float),
            'capture_settle_time': _typed('capture_settle_time', float),
            'camera_topic': GZ_CAMERA_TOPIC,
            'range_topic': GZ_RANGE_TOPIC,
            'photo_dir': _typed('photo_dir', str),
        }],
    )

    ground_station = Node(
        package=PACKAGE_NAME,
        executable='ground_station',
        name='ground_station',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('ground_station')),
        parameters=[{
            'frame_topic': FRAME_TOPIC,
            'output_root': _typed('output_root', str),
            'stitch_project_dir': _typed('stitch_project_dir', str),
            'stitch_config': _typed('stitch_config', str),
            'auto_stitch': _typed('auto_stitch', bool),
            'stitch_backend': _typed('stitch_backend', str),
            'webapp_url': _typed('webapp_url', str),
            'camera_hfov_rad': CAMERA_HFOV_RAD,
            'fallback_agl_m': _typed('target_altitude', float),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(arguments + [bridge, navigator, ground_station, rviz])
