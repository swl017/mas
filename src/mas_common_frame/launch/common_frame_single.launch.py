from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='/',
        description='Namespace for the node'
    )
    return LaunchDescription([
        ns_arg,
        Node(
            package='mas_common_frame',
            executable='common_frame_node_single',
            name='common_frame_node_single',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[
                {'common_frame_origin': [
                    38.736832, #36.3740841,
                    -9.137977, #127.3660736,
                    143.8, #100.0,
                ]},
            ]
        )
    ])
