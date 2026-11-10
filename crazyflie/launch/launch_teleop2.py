import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def parse_yaml(context):
    # Load the crazyflies YAML file
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)
    # store the fileversion
    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie_description'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = LaunchConfiguration('motion_capture_yaml_file').perform(context)
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture_content['/motion_capture_tracking']['ros__parameters']
    motion_capture_params['rigid_bodies'] = dict()
    for key, value in crazyflies['robots'].items():
        type = crazyflies['robot_types'][value['type']]
        if value['enabled'] and \
            ((fileversion == 1 and type['motion_capture']['enabled']) or \
            ((fileversion >= 2 and type['motion_capture']['tracking'] == "librigidbodytracker"))):
            motion_capture_params['rigid_bodies'][key] =  {
                    'initial_position': value['initial_position'],
                    'marker': type['motion_capture']['marker'],
                    'dynamics': type['motion_capture']['dynamics'],
                }

    # copy relevent settings to server params
    server_params[1]['poses_qos_deadline'] = motion_capture_params['topics']['poses']['qos']['deadline']
    
    return [
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' != 'sim' and '", LaunchConfiguration('mocap'), "' == 'True'"])),
            name='motion_capture_tracking',
            output='screen',
            parameters= [motion_capture_params],
        ),
        Node(
            package='crazyflie_server_cflib',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
        ),
        Node(
            package='crazyflie_server_cpp',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= server_params,
        )]

def launch_joysticks(context):
    if LaunchConfiguration('teleop').perform(context) != 'True':
        return []

    teleop_yaml = LaunchConfiguration('teleop_yaml_file').perform(context)
    if not teleop_yaml:
        teleop_yaml = os.path.join(
            get_package_share_directory('crazyflie'), 'config', 'teleop.yaml')
    with open(teleop_yaml, 'r') as file:
        teleop_params = yaml.safe_load(file)['/teleop']['ros__parameters']

    with open(LaunchConfiguration('crazyflies_yaml_file').perform(context), 'r') as file:
        crazyflies = yaml.safe_load(file)

    nodes = []
    for device_id, robot in enumerate(('cfblack', 'cfblue')):
        config = crazyflies['robots'][robot]
        if not config['enabled']:
            raise ValueError(f'{robot} must be enabled in crazyflies_yaml_file')
        x, y, z = config['initial_position']
        # Load the /teleop parameters as a dictionary so they also apply to
        # namespaced nodes. Each drone starts with its own position target.
        nodes.append(Node(
            package='crazyflie',
            executable='teleop',
            namespace=robot,
            name='teleop',
            parameters=[teleop_params, {
                'initial_position.x': float(x),
                'initial_position.y': float(y),
                'initial_position.z': float(z),
            }],
        ))
        # Relative command, service, and joy names resolve inside the same
        # drone namespace, keeping both control paths independent.
        nodes.append(Node(
            package='joy',
            executable='joy_node',
            namespace=robot,
            name='joy_node',
            parameters=[{'device_id': device_id}],
        ))
    return nodes


def generate_launch_description():
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    default_motion_capture_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('crazyflies_yaml_file', 
                              default_value=default_crazyflies_yaml_path),
        DeclareLaunchArgument('motion_capture_yaml_file', 
                              default_value=default_motion_capture_yaml_path),
        DeclareLaunchArgument('rviz_config_file', 
                              default_value=default_rviz_config_path),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('teleop', default_value='True'),
        DeclareLaunchArgument('mocap', default_value='True'),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        OpaqueFunction(function=parse_yaml),
        OpaqueFunction(function=launch_joysticks),
        Node(
            condition=LaunchConfigurationEquals('rviz', 'True'),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
    ])
