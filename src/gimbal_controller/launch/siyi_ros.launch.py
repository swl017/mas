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
        default_value='25.0',
        description='Rate at which to publish gimbal angles'
    )
    yaw_direction_arg = DeclareLaunchArgument(
        'yaw_direction',
        default_value='1.0',
        description='Direction of yaw rotation (1.0 for clockwise, -1.0 for counter-clockwise)'
    )
    pitch_direction_arg = DeclareLaunchArgument(
        'pitch_direction',
        default_value='-1.0',
        description='Direction of pitch rotation (1.0 for up, -1.0 for down)'
    )
    enable_encoder_stream_arg = DeclareLaunchArgument(
        'enable_encoder_stream',
        default_value='true',
        description='Enable magnetic encoder angle streaming (0x26)'
    )
    enable_aircraft_attitude_arg = DeclareLaunchArgument(
        'enable_aircraft_attitude',
        default_value='true',
        description='Enable aircraft attitude injection to gimbal (0x22)'
    )
    encoder_stream_freq_arg = DeclareLaunchArgument(
        'encoder_stream_freq',
        default_value='50',
        description='Encoder angle stream frequency in Hz'
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
            {'enable_encoder_stream': LaunchConfiguration('enable_encoder_stream')},
            {'enable_aircraft_attitude': LaunchConfiguration('enable_aircraft_attitude')},
            {'encoder_stream_freq': LaunchConfiguration('encoder_stream_freq')},
        ],
        remappings=[
            # Encoder angles (0x26, body-frame joint angles) are the primary gimbal state.
            # IMU-stabilized angles (0x0D, world-frame) available as secondary.
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
        enable_encoder_stream_arg,
        enable_aircraft_attitude_arg,
        encoder_stream_freq_arg,
        siyi_ros_node
    ])