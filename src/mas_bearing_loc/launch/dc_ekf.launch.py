from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    veh = LaunchConfiguration('vehicle')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='px4_1',
                              description='Per-vehicle namespace (no leading slash).'),
        DeclareLaunchArgument('target_class_name', default_value=''),
        DeclareLaunchArgument('init_range_guess', default_value='15.0'),
        DeclareLaunchArgument('sigma_pix', default_value='3.0'),
        DeclareLaunchArgument('sigma_target_acc', default_value='0.5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='25.0'),

        Node(
            package='mas_bearing_loc',
            executable='dc_ekf_node',
            name='dc_ekf_node',
            namespace=veh,
            output='screen',
            parameters=[{
                'target_class_name': LaunchConfiguration('target_class_name'),
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': LaunchConfiguration('sigma_pix'),
                'sigma_target_acc': LaunchConfiguration('sigma_target_acc'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'use_sim_time': True,
            }],
        ),
    ])
