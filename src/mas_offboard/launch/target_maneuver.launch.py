"""Launch the target-maneuver node for a regime (Ticket 004, Slice 3).

  ros2 launch mas_offboard target_maneuver.launch.py vehicle_filter:=px4_2 \
      regime:=dynamic_mild forward_heading_deg:=0.0 use_sim_time:=true

Regime presets mirror the point-mass scenario_grid AGILITY_LEVELS
(target_speed_mps, sinusoid_amplitude_m, sinusoid_frequency_hz).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (target_speed_mps, sinusoid_amplitude_m, sinusoid_frequency_hz)
REGIMES = {
    "static_wind":  (0.0, 1.0, 0.20),
    "dynamic_mild": (6.0, 1.0, 0.20),
    "dynamic_paper": (7.0, 1.5, 0.25),
    "dynamic_hard": (8.0, 2.0, 0.30),
}


def launch_setup(context):
    ns = LaunchConfiguration("vehicle_filter").perform(context)
    regime = LaunchConfiguration("regime").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    heading = float(LaunchConfiguration("forward_heading_deg").perform(context))
    if regime not in REGIMES:
        raise ValueError(f"unknown regime '{regime}' (have: {sorted(REGIMES)})")
    speed, amp, freq = REGIMES[regime]
    return [Node(
        package="mas_offboard",
        executable="target_maneuver_node",
        name="target_maneuver_node",
        namespace=ns,
        output="screen",
        parameters=[{
            "regime": regime,
            "target_speed_mps": speed,
            "sinusoid_amplitude_m": amp,
            "sinusoid_frequency_hz": freq,
            "forward_heading_deg": heading,
            "use_sim_time": use_sim_time,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_filter", default_value="px4_2"),
        DeclareLaunchArgument("regime", default_value="dynamic_mild"),
        DeclareLaunchArgument("forward_heading_deg", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
