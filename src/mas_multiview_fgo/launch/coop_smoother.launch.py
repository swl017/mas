"""Ticket 024 S2 — cooperative GTSAM smoother arm for the interception harness.

Drop-in alternative to the ticket-019 `cv_smoother` cooperative arm: instead of
`sort3d chosen_target_pose -> alpha-beta`, run the measurement-time-aware GTSAM smoother
(`coop_smoother_node`) directly on the interceptor's raw ego pixels + the peer's transmitted
bearing ray, publishing the SAME `coop_loc/target_pose + target_twist` contract pn consumes.

Roles (config/roles.yaml `coop_1obs`): interceptor = px4_1 (ego pixel), observer = px4_2 (peer
bearing = /px4_2/target_rays_w, published by the interceptor's triangulation_node), target = px4_3.

Fair peer-only latency (ticket 020 axis): the peer ray always routes through `ray_delay` (tau_s=0 =
passthrough); the fresh ego pixel is never staled. Sweep `tau_s` for the AoI x-axis.

    ros2 launch mas_multiview_fgo coop_smoother.launch.py \
        interceptor_ns:=px4_1 observer_ns:=px4_2 tau_s:=0.10 gimbal_angle_order:=zxy
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    # Resolve launch args to plain strings so list params are genuine string arrays.
    g = lambda k: LaunchConfiguration(k).perform(context)
    interc = g('interceptor_ns').strip('/')
    obs = g('observer_ns').strip('/')
    ust = g('use_sim_time').lower() == 'true'

    peer_raw = f'/{obs}/target_rays_w'
    peer_delayed = f'/{obs}/target_rays_w_fgo'
    ego = lambda s: f'/{interc}/{s}'

    ray_delay = Node(
        package='mas_coop_mock', executable='ray_delay', name='ray_delay_fgo',
        namespace=f'/{interc}', output='screen',   # -> /{interc}/ray_delay_fgo for live-param set
        parameters=[{
            'in_topic': peer_raw,
            'out_topic': peer_delayed,
            'latency_s': float(g('tau_s')),
            'latency_jitter_s': float(g('latency_jitter_s')),
            'drop_p': float(g('drop_p')),
            'use_sim_time': ust,
        }])

    smoother = Node(
        package='mas_multiview_fgo', executable='coop_smoother_node', name='coop_smoother',
        namespace=f'/{interc}', output='screen',
        parameters=[{
            'ego_detection_topic': ego('yolo_result_vision'),
            'ego_camera_info_topic': ego('camera/color/camera_info'),
            'ego_odom_topic': ego('common_frame/odom'),   # vehicle world pose (camera pose = pose x gimbal)
            'ego_gimbal_topic': ego('gimbal_state_rpy_deg'),
            'ego_zoom_topic': ego('camera/zoom_level'),
            'gimbal_angle_order': g('gimbal_angle_order'),
            'peer_ray_topics': [peer_delayed],   # genuine string array (plain python list)
            'coop_prefix': g('coop_prefix'),
            'bearing_sigma_deg': float(g('bearing_sigma_deg')),
            'sigma_psi_deg': float(g('sigma_psi_deg')),
            'pixel_sigma_px': float(g('pixel_sigma_px')),
            'window_s': float(g('window_s')),
            'q_c': float(g('q_c')),
            'use_robust': g('use_robust').lower() == 'true',
            'use_sim_time': ust,
        }])

    return [ray_delay, smoother]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interceptor_ns', default_value='px4_1'),
        DeclareLaunchArgument('observer_ns', default_value='px4_2'),
        DeclareLaunchArgument('coop_prefix', default_value='coop_loc'),
        DeclareLaunchArgument('gimbal_angle_order', default_value='zxy'),  # sim uses zxy
        DeclareLaunchArgument('tau_s', default_value='0.0'),               # fair peer-only AoI axis
        DeclareLaunchArgument('latency_jitter_s', default_value='0.0'),
        DeclareLaunchArgument('drop_p', default_value='0.0'),
        DeclareLaunchArgument('bearing_sigma_deg', default_value='0.5'),
        DeclareLaunchArgument('sigma_psi_deg', default_value='0.0'),       # Q10 Tier-1 (bias bound)
        DeclareLaunchArgument('pixel_sigma_px', default_value='2.0'),
        DeclareLaunchArgument('window_s', default_value='0.6'),
        DeclareLaunchArgument('q_c', default_value='4.0'),
        DeclareLaunchArgument('use_robust', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
