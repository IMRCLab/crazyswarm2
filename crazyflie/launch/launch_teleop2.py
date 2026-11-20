"""
Fly two Crazyflies manually, each with its own joystick.

This reuses launch.py (server, motion capture, rviz) with its single global
teleop disabled, and starts one teleop node and one joy node per Crazyflie.
Each pair runs in the namespace of its Crazyflie (e.g. /cf231), so the
relative topic and service names of the teleop node (joy, takeoff, land,
arm, emergency, cmd_full_state, ...) resolve directly to that Crazyflie.

Example:
    ros2 launch crazyflie launch_teleop2.py cf_1:=cf231 cf_2:=cf5

Make sure that both Crazyflies are enabled in crazyflies.yaml and that the
mode in the teleop yaml file(s) is set to the desired manual flight mode.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def teleop_nodes(context):
    nodes = []
    for i in [1, 2]:
        cf_name = LaunchConfiguration(f'cf_{i}').perform(context)
        teleop_yaml = LaunchConfiguration(f'teleop_yaml_file_{i}').perform(context)
        device_id = int(LaunchConfiguration(f'joy_device_id_{i}').perform(context))

        # The yaml file is keyed by the node name (/teleop); since the node runs
        # in a namespace, we pass the parameters directly instead.
        with open(teleop_yaml, 'r') as f:
            teleop_params = yaml.safe_load(f)['/teleop']['ros__parameters']

        nodes += [
            Node(
                package='crazyflie',
                executable='teleop',
                name='teleop',
                namespace=cf_name,
                output='screen',
                parameters=[teleop_params],
            ),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                namespace=cf_name,
                parameters=[{'device_id': device_id}],
            ),
        ]
    return nodes


def generate_launch_description():
    default_teleop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')

    launch_py = os.path.join(
        get_package_share_directory('crazyflie'),
        'launch',
        'launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('cf_1', default_value='cf231',
                              description='Name of the Crazyflie controlled by joystick 1'),
        DeclareLaunchArgument('cf_2', default_value='cf5',
                              description='Name of the Crazyflie controlled by joystick 2'),
        DeclareLaunchArgument('teleop_yaml_file_1', default_value=default_teleop_yaml_path),
        DeclareLaunchArgument('teleop_yaml_file_2', default_value=default_teleop_yaml_path),
        DeclareLaunchArgument('joy_device_id_1', default_value='0'),
        DeclareLaunchArgument('joy_device_id_2', default_value='1'),
        # All other arguments of launch.py (backend, mocap, rviz, ...) can be
        # passed on the command line and are forwarded automatically.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_py),
            launch_arguments=[('teleop', 'False')],
        ),
        OpaqueFunction(function=teleop_nodes),
    ])
