"""Launch the raw world-LOS publisher under a vehicle namespace (ticket 012).

Publishes `/{vehicle}/bearing_raw/los` — the range-free image-feature bearing that
`mas_pn_guidance` `guidance_mode:=raw_ibvs` servos.

  ros2 launch mas_bearing_loc raw_los.launch.py vehicle:=px4_1 use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    veh = LaunchConfiguration('vehicle')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='px4_1',
                              description='Per-vehicle namespace (no leading slash).'),
        DeclareLaunchArgument('target_class_name', default_value='drone'),
        DeclareLaunchArgument('min_confidence', default_value='0.25'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='mas_bearing_loc',
            executable='raw_los_node',
            name='raw_los_node',
            namespace=veh,
            output='screen',
            parameters=[{
                'target_class_name': LaunchConfiguration('target_class_name'),
                'min_confidence': ParameterValue(
                    LaunchConfiguration('min_confidence'), value_type=float),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
