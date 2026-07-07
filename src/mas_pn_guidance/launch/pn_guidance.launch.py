"""Launch the PN guidance node under a vehicle namespace.

  ros2 launch mas_pn_guidance pn_guidance.launch.py ns:=/px4_1 \
      estimate_source:=oracle use_sim_time:=true

estimate_source ∈ {oracle, simple_ekf, direct_projection}. Params default from
config/pn_guidance.yaml (point-mass parity); CLI args override.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns = LaunchConfiguration("ns")
    cfg = os.path.join(
        get_package_share_directory("mas_pn_guidance"), "config", "pn_guidance.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="/px4_1"),
        DeclareLaunchArgument("estimate_source", default_value="oracle"),
        DeclareLaunchArgument("v_max", default_value="9.0"),
        DeclareLaunchArgument("a_max", default_value="6.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="mas_pn_guidance",
            executable="pn_guidance_node",
            name="pn_guidance_node",
            namespace=ns,
            output="screen",
            parameters=[
                cfg,
                {
                    "estimate_source": LaunchConfiguration("estimate_source"),
                    "v_max": ParameterValue(LaunchConfiguration("v_max"), value_type=float),
                    "a_max": ParameterValue(LaunchConfiguration("a_max"), value_type=float),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
