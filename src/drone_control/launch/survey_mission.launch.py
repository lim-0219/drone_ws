"""
Launch the survey-and-mark mission: Gazebo bridge + navigator + RViz.

Requires PX4 SITL running the x500_survey model (`make px4_sitl gz_x500_survey`)
and the uXRCE-DDS agent.  Example::

    ros2 launch drone_control survey_mission.launch.py \
        target_x:=5.0 target_y:=3.0 target_altitude:=6.0

Set `rviz:=false` or `bridge:=false` to skip those processes.
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

# Gazebo topics are pinned in the model SDF (<topic> on each sensor), so they
# do not depend on the world or on the spawned model's instance name.
GZ_CAMERA_TOPIC = '/survey/camera/image'
GZ_RANGE_TOPIC = '/survey/range/scan'


def _typed(name: str, value_type: type) -> ParameterValue:
    """Launch arguments arrive as strings; declare the type the node expects."""
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    default_rviz = os.path.join(share, 'config', 'survey.rviz')

    arguments = [
        # mission target (NED position, positive-up altitude - negated in the node)
        DeclareLaunchArgument('target_x', default_value='5.0',
                              description='Target North coordinate in metres (NED x).'),
        DeclareLaunchArgument('target_y', default_value='0.0',
                              description='Target East coordinate in metres (NED y).'),
        DeclareLaunchArgument('target_altitude', default_value='5.0',
                              description='Hover altitude above the takeoff point, metres, > 0.'),
        DeclareLaunchArgument('target_yaw', default_value='0.0',
                              description='Heading at the target (degrees, NED, 0 = North).'),
        # navigation
        DeclareLaunchArgument('arm_approved', default_value='true',
                              description='ARM gate; true in SITL. false = wait for '
                                          'ros2 param set /waypoint_navigator arm_approved true'),
        DeclareLaunchArgument('px4_namespace', default_value='',
                              description='PX4 topic prefix, e.g. "/px4_1". Empty = default.'),
        DeclareLaunchArgument('timer_period', default_value='0.1',
                              description='Setpoint period in seconds (0.1 = 10 Hz).'),
        DeclareLaunchArgument('position_tolerance', default_value='0.3',
                              description='Arrival radius in metres.'),
        DeclareLaunchArgument('reach_hold_time', default_value='1.0',
                              description='Seconds inside the tolerance before "reached".'),
        DeclareLaunchArgument('setpoint_warmup_count', default_value='20',
                              description='Setpoints streamed before requesting OFFBOARD + ARM.'),
        # survey
        DeclareLaunchArgument('photo_dir', default_value='~/drone_survey_photos',
                              description='Directory for the captured PNG + JSON metadata.'),
        DeclareLaunchArgument('capture_settle_time', default_value='2.0',
                              description='Seconds to hover at the target before the photo.'),
        DeclareLaunchArgument('laser_half_angle_deg', default_value='15.0',
                              description='Tilt of each laser beam from straight down, degrees.'),
        DeclareLaunchArgument('range_sensor_down_offset', default_value='0.05',
                              description='Range sensor height below the body origin, metres '
                                          '(matches the SDF and EKF2_RNG_POS_Z).'),
        # processes
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Start the ros_gz parameter_bridge for camera + range.'),
        DeclareLaunchArgument('gz_ip', default_value='127.0.0.1',
                              description="GZ_IP for the bridge. PX4's make target starts Gazebo "
                                          'with GZ_IP=127.0.0.1 and gz-transport only discovers '
                                          'peers on the same interface.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz with the survey configuration.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz configuration file.'),
    ]

    # Gazebo -> ROS only ("["), one bridge process for both sensors.
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
            'final_action': 'survey',
            'arm_approved': _typed('arm_approved', bool),
            'target_x': _typed('target_x', float),
            'target_y': _typed('target_y', float),
            'target_altitude': _typed('target_altitude', float),
            'target_yaw': _typed('target_yaw', float),
            'yaw_in_degrees': True,
            'px4_namespace': _typed('px4_namespace', str),
            'timer_period': _typed('timer_period', float),
            'position_tolerance': _typed('position_tolerance', float),
            'reach_hold_time': _typed('reach_hold_time', float),
            'setpoint_warmup_count': _typed('setpoint_warmup_count', int),
            'camera_topic': GZ_CAMERA_TOPIC,
            'range_topic': GZ_RANGE_TOPIC,
            'range_sensor_down_offset': _typed('range_sensor_down_offset', float),
            'photo_dir': _typed('photo_dir', str),
            'capture_settle_time': _typed('capture_settle_time', float),
            'laser_half_angle_deg': _typed('laser_half_angle_deg', float),
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

    return LaunchDescription(arguments + [bridge, navigator, rviz])
