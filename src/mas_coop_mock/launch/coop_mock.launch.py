"""ticket 019 mock-cooperative belief pipeline (interceptor side).

Launches the velocity smoother that turns the multi-observer fused
`chosen_target_pose` into the PN cooperative drop-in contract
(`coop_loc/target_pose` + `coop_loc/target_twist`), and — optionally — the
peer-communication (AoI) delay stage. The multi-observer fusion itself
(`mas_multiview` triangulation + `mas_tracker` sort3d) is launched separately
(see sim_interceptor.tmuxp.yaml) so its cross-package args stay with the existing
launches.

    ros2 launch mas_coop_mock coop_mock.launch.py namespace:=px4_1 \
        alpha:=0.6 beta:=0.15 enable_delay:=false tau_s:=0.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    ust = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='px4_1'),
        DeclareLaunchArgument('alpha', default_value='0.6'),
        DeclareLaunchArgument('beta', default_value='0.15'),
        DeclareLaunchArgument('coop_prefix', default_value='coop_loc'),
        DeclareLaunchArgument('in_topic', default_value='chosen_target_pose'),
        DeclareLaunchArgument('enable_delay', default_value='false'),
        DeclareLaunchArgument('tau_s', default_value='0.0'),
        DeclareLaunchArgument('delay_in', default_value='/px4_3/target_rays_w_raw'),
        DeclareLaunchArgument('delay_out', default_value='/px4_3/target_rays_w'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        GroupAction([
            PushRosNamespace(ns),
            Node(package='mas_coop_mock', executable='cv_smoother', name='cv_smoother',
                 output='screen',
                 parameters=[{
                     'alpha': LaunchConfiguration('alpha'),
                     'beta': LaunchConfiguration('beta'),
                     'coop_prefix': LaunchConfiguration('coop_prefix'),
                     'in_topic': LaunchConfiguration('in_topic'),
                     'use_sim_time': ust,
                 }]),
        ]),
        Node(package='mas_coop_mock', executable='ray_delay', name='ray_delay',
             output='screen',
             condition=IfCondition(LaunchConfiguration('enable_delay')),
             parameters=[{
                 'in_topic': LaunchConfiguration('delay_in'),
                 'out_topic': LaunchConfiguration('delay_out'),
                 'tau_s': LaunchConfiguration('tau_s'),
                 'use_sim_time': ust,
             }]),
    ])
