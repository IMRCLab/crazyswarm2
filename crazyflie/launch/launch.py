import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def parse_yaml(context):
    # Load the crazyflies YAML file
    crazyflies_yaml_file = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml_file, 'r') as file:
        crazyflies = yaml.safe_load(file)

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robots'] = crazyflies['robots']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['robot_types'] = crazyflies['robot_types']
    server_yaml_content["/crazyflie_server"]["ros__parameters"]['all'] = crazyflies['all']

    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_yaml_content["/crazyflie_server"]["ros__parameters"]["robot_description"] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_content["/motion_capture_tracking"]["ros__parameters"]["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_content["/motion_capture_tracking"]["ros__parameters"]["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    # copy relevent settings to server params
    server_yaml_content["/crazyflie_server"]["ros__parameters"]["poses_qos_deadline"] = motion_capture_content[
        "/motion_capture_tracking"]["ros__parameters"]["topics"]["poses"]["qos"]["deadline"]

    # Save server and mocap in temp file such that nodes can read it out later
    with open('tmp_server.yaml', 'w') as outfile:
        yaml.dump(server_yaml_content, outfile, default_flow_style=False, sort_keys=False)

    with open('tmp_motion_capture.yaml', 'w') as outfile:
        yaml.dump(motion_capture_content, outfile, default_flow_style=False, sort_keys=False)


def generate_launch_description():
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    telop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
        
    return LaunchDescription([
        DeclareLaunchArgument('crazyflies_yaml_file', 
                              default_value=default_crazyflies_yaml_path),
        OpaqueFunction(function=parse_yaml),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('mocap', default_value='True'),
        DeclareLaunchArgument('server_yaml_file', default_value=''),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        DeclareLaunchArgument('mocap_yaml_file', default_value=''),
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' != 'sim' and '", LaunchConfiguration('mocap'), "' == 'True'"])),
            name='motion_capture_tracking',
            output='screen',
            parameters= [PythonExpression(["'tmp_motion_capture.yaml' if '", LaunchConfiguration('mocap_yaml_file'), "' == '' else '", LaunchConfiguration('mocap_yaml_file'), "'"])],
        ),
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('arm', 'all/arm'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
                # uncomment to manually control (and update teleop.yaml)
                # ('cmd_vel_legacy', 'cf6/cmd_vel_legacy'),
                # ('cmd_full_state', 'cf6/cmd_full_state'),
                # ('notify_setpoints_stop', 'cf6/notify_setpoints_stop'),
            ],
            parameters= [PythonExpression(["'" + telop_yaml_path +"' if '", LaunchConfiguration('teleop_yaml_file'), "' == '' else '", LaunchConfiguration('teleop_yaml_file'), "'"])],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node' # by default id=0
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= [PythonExpression(["'tmp_server.yaml' if '", LaunchConfiguration('server_yaml_file'), "' == '' else '", LaunchConfiguration('server_yaml_file'), "'"])],
        ),
        Node(
            condition=LaunchConfigurationEquals('rviz', 'True'),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('crazyflie'), 'config', 'config.rviz')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('gui', 'True'),
            package='crazyflie',
            namespace='',
            executable='gui.py',
            name='gui',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            package='crazyflie_examples',
            executable='obstacle_publisher_from_cf',
            name='obstacle_publisher_from_cf'
        ),
    ])
