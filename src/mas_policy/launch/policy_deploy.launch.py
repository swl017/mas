"""Launch file for mas_policy deployment node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mas_policy')
    default_config = os.path.join(pkg_share, 'config', 'policy_deploy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to policy deployment config YAML',
        ),
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value='',
            description='Path to SKRL .pt checkpoint file',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, log observations/actions without publishing',
        ),

        Node(
            package='mas_policy',
            executable='policy_node',
            name='mas_policy_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                    'dry_run': LaunchConfiguration('dry_run'),
                },
            ],
        ),
    ])
