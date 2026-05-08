"""Launch the mavros_replicator for a single vehicle.

Usage:
    ros2 launch mavros_replicator replicator.launch.py robot_name:=px4_1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name", default_value="px4_1",
            description="Robot namespace prefix (matches PX4 UXRCE_DDS_PTCFG)."
        ),
        Node(
            package="mavros_replicator",
            executable="mavros_replicator",
            name="mavros_replicator",
            output="screen",
            parameters=[{
                "robot_name": robot_name,
            }],
        ),
    ])
