"""Launch file for mas_offboard — spawns one OffboardControl node per vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_config(context):
    config_file = LaunchConfiguration('config_file').perform(context)

    if os.path.isabs(config_file):
        config_path = config_file
    else:
        pkg_share = get_package_share_directory('mas_offboard')
        config_path = os.path.join(pkg_share, config_file)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f'Vehicle config not found: {config_path}')

    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context):
    config = _load_config(context)
    vehicle_filter = LaunchConfiguration('vehicle_filter').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    vehicles = config['vehicles']
    if vehicle_filter:
        vehicles = [v for v in vehicles if v['namespace'] == vehicle_filter]

    nodes = []

    for vehicle in vehicles:
        ns = vehicle['namespace']
        pos = vehicle['position']

        node = Node(
            package='mas_offboard',
            executable='offboard_control',
            name='offboard_control',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'vehicle_name': ns,
                'update_rate': vehicle.get('update_rate', 100.0),
                'target_system': vehicle.get('target_system', 1),
                'position.x': pos['x'],
                'position.y': pos['y'],
                'position.z': pos['z'],
                'position.yaw_deg': pos['yaw_deg'],
                'takeoff_speed': vehicle.get('takeoff_speed', 3.0),
                'use_sim_time': use_sim_time,
            }],
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/vehicles.yaml',
            description='Path to the vehicle configuration file',
        ),
        DeclareLaunchArgument(
            'vehicle_filter',
            default_value='',
            description='If set, only launch for this vehicle namespace (e.g. px4_1)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (sim) time when true, wall time otherwise',
        ),
        OpaqueFunction(function=launch_setup),
    ])
