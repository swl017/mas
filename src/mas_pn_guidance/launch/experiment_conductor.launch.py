"""Launch the Ticket 004 experiment conductor for one sim boot.

Both the interceptor and target stacks must already be up and engaged-idle
(mission_state IDLE). Each boot is one PX4 SITL session (boot-variable EKF
attitude bias = the experimental block); pass a fresh boot_id per session.

    ros2 launch mas_pn_guidance experiment_conductor.launch.py boot_id:=A
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        ('boot_id', 'A'),
        ('interceptor_ns', 'px4_1'),
        ('target_ns', 'px4_2'),
        ('intercept_radius_m', '0.5'),    # primary success radius (matches ticket 003)
        ('engage_timeout_s', '45.0'),    # SIM seconds (dynamic_mild intercepts ~10 s sim)
        ('wall_timeout_s', '400.0'),     # wall-clock safety cap
        ('repeats', '1'),
        ('seed', '0'),            # 0 = nondeterministic shuffle
        # Matrix dims — comma-separated; empty = use all.
        ('estimators', ''),
        ('regimes', ''),           # = target conditions; empty = all of the mode
        ('geometries', ''),
        # Target conditions: 'named' (4 regimes) or 'capability_grid' (fwd × a_lat).
        ('target_condition_mode', 'named'),
        ('target_forward_speeds', '4.5,6.0,7.0,8.0'),
        ('target_lateral_accels', '1.5,3.0,4.5,7.1'),
        ('target_frequency_hz', '0.25'),
        ('record', 'true'),
        ('bag_script', '/home/usrg/mas/bag/rosbag_record.sh'),  # full -a recorder; override to rosbag_record_reduced.sh for the ~13-topic set
        ('dry_run', 'false'),
        ('use_sim_time', 'true'),
    ]
    params = {a: LaunchConfiguration(a) for a, _ in args}
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value=d) for a, d in args] + [
            Node(
                package='mas_pn_guidance',
                executable='experiment_conductor',
                name='experiment_conductor',
                output='screen',
                parameters=[params],
            ),
        ])
