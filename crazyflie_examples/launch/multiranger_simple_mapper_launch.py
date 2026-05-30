import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    keyboard_velmux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie_examples'),
                'launch/keyboard_velmux_launch.py',
            )
        ),
    )

    crazyflie_name = '/cf231'

    return LaunchDescription(
        [
            keyboard_velmux,
            Node(
                package='crazyflie_examples',
                executable='simple_mapper_multiranger',
                name='simple_mapper_multiranger',
                output='screen',
                parameters=[{'robot_prefix': crazyflie_name}],
            ),
        ]
    )
