"""Multi-agent launch file for mas_policy.

Launches one policy_node per vehicle, each in its own namespace.
Each node gets its vehicle_name and the list of peer_names computed
from the vehicles.yaml roster.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _resolve_path(path: str, package: str) -> str:
    """Resolve a config path — absolute or relative to package share."""
    if os.path.isabs(path):
        return path
    return os.path.join(get_package_share_directory(package), path)


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    vehicles_file = LaunchConfiguration('vehicles_file').perform(context)
    checkpoint_path = LaunchConfiguration('checkpoint_path').perform(context)
    agent_id = LaunchConfiguration('agent_id').perform(context)
    dry_run = LaunchConfiguration('dry_run').perform(context)
    use_mission_gate = LaunchConfiguration('use_mission_gate').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Resolve paths
    config_path = _resolve_path(config_file, 'mas_policy')
    vehicles_path = _resolve_path(vehicles_file, 'mas_policy')

    if not os.path.exists(vehicles_path):
        raise FileNotFoundError(f"Vehicles config not found: {vehicles_path}")

    with open(vehicles_path, 'r') as f:
        vehicles_config = yaml.safe_load(f)

    vehicle_filter = LaunchConfiguration('vehicle_filter').perform(context)

    vehicles = vehicles_config['vehicles']
    if vehicle_filter:
        vehicles = [v for v in vehicles if v['namespace'] == vehicle_filter]

    all_namespaces = [v['namespace'] for v in vehicles_config['vehicles']]
    nodes = []

    for vehicle in vehicles:
        namespace = vehicle['namespace']
        peer_names = [ns for ns in all_namespaces if ns != namespace]

        # agent_id: per-vehicle from vehicles.yaml, or launch arg, or empty
        veh_agent_id = vehicle.get('agent_id', agent_id)

        # When use_mission_gate is true, remap policy outputs to policy/* prefixed
        # topics so mas_mission can gate them before forwarding downstream.
        remappings = []
        if use_mission_gate == 'true':
            remappings = [
                ('cmd_vel', 'policy/cmd_vel'),
                ('gimbal_cmd_los_rate', 'policy/gimbal_cmd_los_rate'),
                ('zoom_rate_cmd', 'policy/zoom_rate_cmd'),
            ]

        node = Node(
            package='mas_policy',
            executable='policy_node',
            name='policy_node',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[
                config_path,
                {
                    'vehicle_name': namespace,
                    'peer_names': peer_names,
                    'checkpoint_path': checkpoint_path,
                    'agent_id': veh_agent_id,
                    'dry_run': dry_run == 'true',
                    'use_sim_time': use_sim_time == 'true',
                },
            ],
            remappings=remappings,
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('mas_policy')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'policy_deploy.yaml'),
            description='Path to shared policy config YAML',
        ),
        DeclareLaunchArgument(
            'vehicles_file',
            default_value=os.path.join(pkg_share, 'config', 'vehicles.yaml'),
            description='Path to vehicles roster YAML',
        ),
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value='',
            description='Path to SKRL .pt checkpoint file',
        ),
        DeclareLaunchArgument(
            'agent_id',
            default_value='',
            description='Agent key in checkpoint for scaler loading (e.g., drone_0). '
                        'Can also be set per-vehicle in vehicles.yaml.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, log observations/actions without publishing',
        ),
        DeclareLaunchArgument(
            'use_mission_gate',
            default_value='true',
            description='If true, remap outputs to policy/* for mas_mission gating',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, use /clock sim time for timers (required for sim)',
        ),
        DeclareLaunchArgument(
            'vehicle_filter',
            default_value='',
            description='If set, only launch for this vehicle namespace (e.g. px4_1)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
