import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'launch/keyboard_velmux_launch.py',
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch/online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(
                        get_package_share_directory('crazyflie_examples'),
                        'config/slam_params.yaml',
                    ),
                    'use_sim_time': 'False',
                }.items(),
            ),
        ]
    )
