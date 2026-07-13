"""OLD ticket-004 estimator parameters reconstructed from runtime launch files.

World-frame CV-EKF: init=50, sigma_pix=1, sigma_target_acc=5.
Direct Projection EKF: init=30, sigma_pix=5, sigma_target_acc=0.05.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    veh = LaunchConfiguration('vehicle')
    common = {
        'target_class_name': LaunchConfiguration('target_class_name'),
        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='px4_1'),
        DeclareLaunchArgument('target_class_name', default_value='drone'),
        DeclareLaunchArgument('init_range_guess', default_value='50.0'),
        DeclareLaunchArgument('direct_init_range_guess', default_value='30.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='25.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='mas_bearing_loc', executable='simple_ekf_node',
            name='simple_ekf_node', namespace=veh, output='screen',
            parameters=[{**common,
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': 1.0, 'sigma_target_acc': 5.0,
                'publish_prefix': 'simple_loc'}]),
        Node(
            package='mas_bearing_loc', executable='direct_projection_ekf_node',
            name='direct_projection_ekf_node', namespace=veh, output='screen',
            parameters=[{**common,
                'init_range_guess': LaunchConfiguration('direct_init_range_guess'),
                'sigma_pix': 5.0, 'sigma_target_acc': 0.05,
                'publish_prefix': 'direct_loc', 'use_obs_accel': True}]),
    ])
