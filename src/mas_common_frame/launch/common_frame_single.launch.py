from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='/',
        description='Namespace for the node'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (sim) time when true, wall time otherwise',
    )
    return LaunchDescription([
        ns_arg,
        use_sim_time_arg,
        Node(
            package='mas_common_frame',
            executable='common_frame_node_single',
            name='common_frame_node_single',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[
                # Altitude is AMSL (meters above mean sea level), same convention
                # as MAVROS HOME_POSITION.geo.altitude and PX4
                # VehicleLocalPosition.ref_alt. Pegasus's default sim world
                # origin is AMSL 90.0 — see PegasusSimulator/extensions/
                # pegasus.simulator/config/configs.yaml:global_coordinates.
                {'common_frame_origin': [
                    38.736832, #36.3740841,
                    -9.137977, #127.3660736,
                    90.0,      #100.0,
                ]},
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ]
        )
    ])
