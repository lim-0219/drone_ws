"""
Launch the PX4 offboard waypoint navigator.

Every node parameter is exposed as a launch argument, e.g.::

    ros2 launch drone_control waypoint_nav.launch.py \
        waypoint_file:=/abs/path/to/my_waypoints.yaml \
        final_action:=land position_tolerance:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'drone_control'


def _typed(name: str, value_type: type) -> ParameterValue:
    """
    Launch arguments arrive as strings; declare the type the node expects.

    Without this every parameter would be passed as a string and the node's
    double/int/bool declarations would reject it at startup.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    default_waypoints = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'square_waypoints.yaml')

    arguments = [
        DeclareLaunchArgument(
            'waypoint_file', default_value=default_waypoints,
            description='Absolute path to the waypoint YAML file.'),
        DeclareLaunchArgument('arm_approved', default_value='true',
                              description='ARM gate; true in SITL. false = wait for '
                                          'ros2 param set /waypoint_navigator arm_approved true'),
        DeclareLaunchArgument(
            'px4_namespace', default_value='',
            description='Topic prefix for the vehicle, e.g. "/px4_1". '
                        'Empty means the default single-vehicle PX4 topics.'),
        DeclareLaunchArgument(
            'timer_period', default_value='0.1',
            description='Offboard setpoint publishing period in seconds (0.1 = 10 Hz).'),
        DeclareLaunchArgument(
            'position_tolerance', default_value='0.3',
            description='Radius in metres within which a waypoint counts as reached.'),
        DeclareLaunchArgument(
            'reach_hold_time', default_value='1.0',
            description='Seconds the vehicle must stay inside the tolerance sphere.'),
        DeclareLaunchArgument(
            'final_action', default_value='hover',
            description='What to do after the last waypoint: "hover" or "land".'),
        DeclareLaunchArgument(
            'setpoint_warmup_count', default_value='20',
            description='Setpoints streamed before requesting OFFBOARD and ARM.'),
        DeclareLaunchArgument(
            'yaw_in_degrees', default_value='true',
            description='Whether the YAML yaw values are degrees (true) or radians (false).'),
        DeclareLaunchArgument(
            'node_namespace', default_value='',
            description='ROS namespace for the node itself (does not affect PX4 topics).'),
    ]

    navigator = Node(
        package=PACKAGE_NAME,
        executable='waypoint_navigator',
        name='waypoint_navigator',
        namespace=LaunchConfiguration('node_namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'waypoint_file': _typed('waypoint_file', str),
            'px4_namespace': _typed('px4_namespace', str),
            'timer_period': _typed('timer_period', float),
            'position_tolerance': _typed('position_tolerance', float),
            'reach_hold_time': _typed('reach_hold_time', float),
            'final_action': _typed('final_action', str),
            'arm_approved': _typed('arm_approved', bool),
            'setpoint_warmup_count': _typed('setpoint_warmup_count', int),
            'yaw_in_degrees': _typed('yaw_in_degrees', bool),
        }],
    )

    return LaunchDescription(arguments + [navigator])
