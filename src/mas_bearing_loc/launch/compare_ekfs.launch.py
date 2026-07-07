"""Launch the bearing-only EKF variants side-by-side for comparison.

All nodes consume the same input topics but publish under different
prefixes within the vehicle namespace:
    {veh}/bearing_loc/...   (DC-EKF, paper-faithful, delay-compensated, 18D)
    {veh}/vanilla_loc/...   (vanilla 18D EKF, no delay compensation)
    {veh}/simple_loc/...    (6D absolute-target CV-EKF, pinhole update)
    {veh}/direct_loc/...    (6D relative-state EKF, direct unit-bearing update)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    veh = LaunchConfiguration('vehicle')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='px4_1'),
        DeclareLaunchArgument('target_class_name', default_value='drone'),
        DeclareLaunchArgument('init_range_guess', default_value='15.0'),
        DeclareLaunchArgument('sigma_pix', default_value='3.0'),
        DeclareLaunchArgument('sigma_target_acc', default_value='0.5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='25.0'),

        # DC-EKF (delay-compensated)
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
                'publish_prefix': 'bearing_loc',
                'disable_delay_compensation': False,
                'override_attitude_from_odom': True,
                'use_sim_time': True,
            }],
        ),
        # Vanilla EKF (no delay compensation; 18D state, IBVS dynamics)
        Node(
            package='mas_bearing_loc',
            executable='dc_ekf_node',
            name='vanilla_ekf_node',
            namespace=veh,
            output='screen',
            parameters=[{
                'target_class_name': LaunchConfiguration('target_class_name'),
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': LaunchConfiguration('sigma_pix'),
                'sigma_target_acc': LaunchConfiguration('sigma_target_acc'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'publish_prefix': 'vanilla_loc',
                'disable_delay_compensation': True,
                'override_attitude_from_odom': True,
                'use_sim_time': True,
            }],
        ),
        # Simple 6D EKF — no IMU, no IBVS state, no quaternion state, no delay-comp
        Node(
            package='mas_bearing_loc',
            executable='simple_ekf_node',
            name='simple_ekf_node',
            namespace=veh,
            output='screen',
            parameters=[{
                'target_class_name': LaunchConfiguration('target_class_name'),
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': LaunchConfiguration('sigma_pix'),
                'sigma_target_acc': LaunchConfiguration('sigma_target_acc'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'publish_prefix': 'simple_loc',
                'use_sim_time': True,
            }],
        ),
        # Direct-projection 6D EKF — relative state, observer accel as control,
        # direct unit-bearing tangent-plane update (no in-state image feature)
        Node(
            package='mas_bearing_loc',
            executable='direct_projection_ekf_node',
            name='direct_projection_ekf_node',
            namespace=veh,
            output='screen',
            parameters=[{
                'target_class_name': LaunchConfiguration('target_class_name'),
                'init_range_guess': LaunchConfiguration('init_range_guess'),
                'sigma_pix': LaunchConfiguration('sigma_pix'),
                'sigma_target_acc': LaunchConfiguration('sigma_target_acc'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'publish_prefix': 'direct_loc',
                'use_obs_accel': True,
                'use_sim_time': True,
            }],
        ),
    ])
