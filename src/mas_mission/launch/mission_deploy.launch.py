"""Multi-agent launch file for mas_mission.

Launches one mission_node per vehicle, each in its own namespace.
Reads vehicle roster from vehicles.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    initial_state = LaunchConfiguration('initial_state').perform(context)
    vehicle_filter = LaunchConfiguration('vehicle_filter').perform(context)

    if os.path.isabs(config_file):
        config_path = config_file
    else:
        pkg_share = get_package_share_directory('mas_mission')
        config_path = os.path.join(pkg_share, config_file)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f'Vehicle config not found: {config_path}')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    vehicles = config['vehicles']
    if vehicle_filter:
        vehicles = [v for v in vehicles if v['namespace'] == vehicle_filter]

    nodes = []

    for vehicle in vehicles:
        ns = vehicle['namespace']

        node = Node(
            package='mas_mission',
            executable='mission_node',
            name='mission_node',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'initial_state': int(initial_state),
                'heartbeat_rate_hz': 1.0,
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
            'initial_state',
            default_value='0',
            description='Initial mission state (0=IDLE, 1=TRACKING, 2=MISSION)',
        ),
        DeclareLaunchArgument(
            'vehicle_filter',
            default_value='',
            description='If set, only launch for this vehicle namespace (e.g. px4_1)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
