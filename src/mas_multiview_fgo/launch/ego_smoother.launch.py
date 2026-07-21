"""RAL ticket 028 — EGO-ONLY FGO arm (backend-matched control for the conversion figure).

Runs `coop_smoother_node` with NO peer ray topics: the graph carries only the ego pixel
factor + CV motion chain over the fixed-lag window. Publishes under `ego_fgo_loc/*`
(never `coop_loc/*` — an ego-only estimate must not be labeled cooperative). PN consumes
it by pointing its cooperative estimate source at this prefix.

Parity defaults (RAL 028 fairness protocol):
  - init_range_m = 50.0  — the deployed BO-EKF floor's `init_range_guess`
    (engagement_ekf_OLD profile), NOT the 024 default 30.0;
  - gate_min_peer = 0    — ego-only windows are the point of this arm; the cov-trace
    gate (and every other gate) stays live as the honesty backstop;
  - everything else = the 024 S7 deployed config (sigma 120 px, w=1.2 s, q_c=4,
    vel_cov_inflation=20, gate on, warm start, fixedlag backend).

    ros2 launch mas_multiview_fgo ego_smoother.launch.py interceptor_ns:=px4_1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    g = lambda k: LaunchConfiguration(k).perform(context)
    interc = g('interceptor_ns').strip('/')
    ust = g('use_sim_time').lower() == 'true'
    ego = lambda s: f'/{interc}/{s}'

    smoother = Node(
        package='mas_multiview_fgo', executable='coop_smoother_node', name='ego_smoother',
        namespace=f'/{interc}', output='screen',
        parameters=[{
            'ego_detection_topic': ego('yolo_result_vision'),
            'ego_camera_info_topic': ego('camera/color/camera_info'),
            'ego_odom_topic': ego('common_frame/odom'),
            'ego_gimbal_topic': ego('gimbal_state_rpy_deg'),
            'ego_zoom_topic': ego('camera/zoom_level'),
            'gimbal_angle_order': g('gimbal_angle_order'),
            # EGO-ONLY: peer_ray_topics is deliberately OMITTED — the node's declared
            # default is the empty string-array (launch_ros cannot type a literal []).
            'coop_prefix': g('prefix'),
            'pixel_sigma_px': float(g('pixel_sigma_px')),
            'window_s': float(g('window_s')),
            'q_c': float(g('q_c')),
            'backend': g('backend'),
            'fl_reset_period_s': float(g('fl_reset_period_s')),
            'target_class': g('target_class'),
            'min_det_score': float(g('min_det_score')),
            'use_robust_ego': g('use_robust_ego').lower() == 'true',
            'vel_cov_inflation': float(g('vel_cov_inflation')),
            'gate_enabled': g('gate_enabled').lower() == 'true',
            'gate_min_peer': 0,                  # RAL 028: ego-only windows are the arm
            'init_range_m': float(g('init_range_m')),
            'use_sim_time': ust,
        }])

    return [smoother]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interceptor_ns', default_value='px4_1'),
        DeclareLaunchArgument('prefix', default_value='ego_fgo_loc'),
        DeclareLaunchArgument('gimbal_angle_order', default_value='zxy'),
        DeclareLaunchArgument('pixel_sigma_px', default_value='120.0'),   # 024 S4 measured
        DeclareLaunchArgument('window_s', default_value='1.2'),
        DeclareLaunchArgument('q_c', default_value='4.0'),
        DeclareLaunchArgument('backend', default_value='fixedlag'),       # 024 S7 deployment backend
        DeclareLaunchArgument('fl_reset_period_s', default_value='0.0'),
        DeclareLaunchArgument('target_class', default_value='drone'),
        DeclareLaunchArgument('min_det_score', default_value='0.25'),
        DeclareLaunchArgument('use_robust_ego', default_value='false'),
        DeclareLaunchArgument('vel_cov_inflation', default_value='20.0'),
        DeclareLaunchArgument('gate_enabled', default_value='true'),
        # Prior parity with the deployed BO-EKF floor (engagement_ekf_OLD init_range_guess).
        DeclareLaunchArgument('init_range_m', default_value='50.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
