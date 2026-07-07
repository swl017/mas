from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mas_common_frame',
            executable='common_frame_node',
            name='common_frame_node',
            output='screen',
            parameters=[
                {'vehicle_name_prefix': 'px4_'},
                {'num_vehicles': 6},
                # Altitude is AMSL, same convention as MAVROS HOME_POSITION.
                # geo.altitude / PX4 VehicleLocalPosition.ref_alt. Pegasus's
                # default sim world origin is AMSL 90.0 (configs.yaml).
                {'common_frame_origin': [
                    38.7368498, #37.7749,
                    -9.1379544, #-122.4194,
                    90.0,       #0.0
                    ]},
            ]
        )
    ])