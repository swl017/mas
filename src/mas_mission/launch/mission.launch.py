from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns', default_value='/',
        description='Vehicle namespace',
    )
    initial_state_arg = DeclareLaunchArgument(
        'initial_state', default_value='0',
        description='Initial mission state (0=IDLE, 1=TRACKING, 2=MISSION)',
    )
    heartbeat_rate_arg = DeclareLaunchArgument(
        'heartbeat_rate_hz', default_value='1.0',
        description='Mission state heartbeat publish rate (Hz)',
    )

    mission_node = Node(
        package='mas_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        namespace=LaunchConfiguration('ns'),
        parameters=[
            {'initial_state': LaunchConfiguration('initial_state')},
            {'heartbeat_rate_hz': LaunchConfiguration('heartbeat_rate_hz')},
        ],
        remappings=[
            # Tracking source: point_to_region publishes to gimbal_command_los_world_deg
            ('tracking/gimbal_cmd_los_world_deg', 'tracking/gimbal_command_los_world_deg'),
        ],
    )

    return LaunchDescription([
        ns_arg,
        initial_state_arg,
        heartbeat_rate_arg,
        mission_node,
    ])
