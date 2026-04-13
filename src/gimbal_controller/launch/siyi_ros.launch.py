from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='/',
        description='Namespace for the node'
    )
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='192.168.144.26',
        description='IP address of the SIYI camera'
    )
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='37260',
        description='Port number of the SIYI camera'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Rate at which to publish gimbal angles (Hz)'
    )
    yaw_direction_arg = DeclareLaunchArgument(
        'yaw_direction',
        default_value='1.0',
        description='Yaw sign multiplier (1.0: left=positive)'
    )
    pitch_direction_arg = DeclareLaunchArgument(
        'pitch_direction',
        default_value='-1.0',
        description='Pitch sign multiplier (-1.0: down=positive after negation)'
    )
    enable_aircraft_attitude_arg = DeclareLaunchArgument(
        'enable_aircraft_attitude',
        default_value='true',
        description='Enable aircraft attitude injection to gimbal (0x22 from IMU, 0x3E from GPS)'
    )
    siyi_ros_node = Node(
        package='gimbal_controller',
        executable='siyi_ros_node',
        name='siyi_ros_node',
        output='screen',
        namespace=LaunchConfiguration('ns'),
        parameters=[
            {'server_ip': LaunchConfiguration('server_ip')},
            {'server_port': LaunchConfiguration('server_port')},
            {'publish_rate_hz': LaunchConfiguration('publish_rate')},
            {'yaw_direction': LaunchConfiguration('yaw_direction')},
            {'pitch_direction': LaunchConfiguration('pitch_direction')},
            {'enable_aircraft_attitude': LaunchConfiguration('enable_aircraft_attitude')},
        ],
        remappings=[
            # Derived joint angles (0x0D yaw=encoder, pitch/roll=heading-aircraft) are primary.
            # Raw 0x0D heading-frame angles available as secondary.
            ('siyi_gimbal_angles/encoder_rpy_deg', 'gimbal_state_rpy_deg'),
            ('siyi_gimbal_angles/state_rpy_deg', 'gimbal_imu_rpy_deg'),
            ('siyi_gimbal_angles/command_rpy_deg', 'gimbal_command_los_world_deg'),
        ]
    )
    return LaunchDescription([
        ns_arg,
        server_ip_arg,
        server_port_arg,
        publish_rate_arg,
        yaw_direction_arg,
        pitch_direction_arg,
        enable_aircraft_attitude_arg,
        siyi_ros_node
    ])
