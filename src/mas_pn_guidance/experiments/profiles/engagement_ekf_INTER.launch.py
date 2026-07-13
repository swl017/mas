"""INTERMEDIATE (smoothing) estimator arms — ticket-007 closed-loop comparison.
Between OLD and TUNED, biased toward smoothness (looser R for SimpleEKF, lower sta for
DirectProjection): SimpleEKF sigma_pix=0.5/sta=3; DirectProjection init=50/sta=1/floor=0.03."""
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
        DeclareLaunchArgument('direct_init_range_guess', default_value='50.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='25.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='mas_bearing_loc', executable='simple_ekf_node',
            name='simple_ekf_node', namespace=veh, output='screen',
            parameters=[{**common,
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': 0.5, 'sigma_target_acc': 3.0,
                'publish_prefix': 'simple_loc'}]),
        Node(
            package='mas_bearing_loc', executable='direct_projection_ekf_node',
            name='direct_projection_ekf_node', namespace=veh, output='screen',
            parameters=[{**common,
                'init_range_guess': LaunchConfiguration('direct_init_range_guess'),
                'sigma_pix': 5.0, 'sigma_target_acc': 1.0,
                'sigma_bearing_floor': 0.03, 'reject_mahalanobis': 0.0,
                'publish_prefix': 'direct_loc', 'use_obs_accel': True}]),
    ])
