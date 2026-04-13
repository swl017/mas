from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    ns_arg = DeclareLaunchArgument(
        'ns', default_value='/',
        description='Namespace for the node'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='common_frame',
        description='Frame ID for the triangulated points'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='3.0',
        description='Rate at which to publish triangulated points'
    )

    num_camera_arg = DeclareLaunchArgument(
        'num_camera',
        default_value='2',
        description='Number of cameras to use for triangulation'
    )

    camera_name_prefix_arg = DeclareLaunchArgument(
        'camera_name_prefix',
        default_value='/px4_',
        description='Prefix for camera names'
    )

    detection_topic_suffix_arg = DeclareLaunchArgument(
        'detection_topic_suffix',
        default_value='yolo_result_vision',
        description='Suffix for detection topics from each camera'
    )

    camera_info_topic_suffix_arg = DeclareLaunchArgument(
        'camera_info_topic_suffix',
        default_value='camera/color/camera_info',
        description='Suffix for camera info topics from each camera'
    )

    camera_pose_topic_suffix_arg = DeclareLaunchArgument(
        'camera_pose_topic_suffix',
        default_value='camera_pose',
        description='Suffix for camera pose topics from each camera'
    )

    camera_odom_topic_suffix_arg = DeclareLaunchArgument(
        'camera_odom_topic_suffix',
        default_value='common_frame/odom',
        description='Suffix for camera odom topics from each camera'
    )

    gimbal_topic_suffix_arg = DeclareLaunchArgument(
        'gimbal_topic_suffix',
        default_value='gimbal_state_rpy_deg',
        description='Suffix for gimbal angle topics from each camera'
    )

    gimbal_angle_order = DeclareLaunchArgument(
        'gimbal_angle_order',
        default_value='zyx',
        description='Order of gimbal angles (e.g., zyx, zxy)'
    )

    max_solve_time_arg = DeclareLaunchArgument(
        'max_solve_time',
        default_value='0.1',
        description='Maximum time to solve triangulation'
    )
    max_reprojection_error_arg = DeclareLaunchArgument(
        'max_reprojection_error',
        default_value='10000.0',
        description='Minimum time to solve triangulation'
    )

    # Create node
    triangulation_node = Node(
        namespace=LaunchConfiguration('ns'),
        package='mas_multiview',
        executable='triangulation_node',
        name='triangulation_node',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'num_camera': LaunchConfiguration('num_camera'),
            'camera_name_prefix': LaunchConfiguration('camera_name_prefix'),
            'detection_topic_suffix': LaunchConfiguration('detection_topic_suffix'),
            'camera_info_topic_suffix': LaunchConfiguration('camera_info_topic_suffix'),
            'camera_pose_topic_suffix': LaunchConfiguration('camera_pose_topic_suffix'),
            'camera_odom_topic_suffix': LaunchConfiguration('camera_odom_topic_suffix'),
            'gimbal_topic_suffix': LaunchConfiguration('gimbal_topic_suffix'),
            'gimbal_angle_order': LaunchConfiguration('gimbal_angle_order'),
            'max_solve_time': LaunchConfiguration('max_solve_time'),
            'max_reprojection_error': LaunchConfiguration('max_reprojection_error'),
        }],
        remappings=[
            # ('triangulated_points', '/triangulated_points'),
        ]
    )

    # Create launch description
    return LaunchDescription([
        ns_arg,
        frame_id_arg,
        publish_rate_arg,
        num_camera_arg,
        camera_name_prefix_arg,
        detection_topic_suffix_arg,
        camera_info_topic_suffix_arg,
        camera_pose_topic_suffix_arg,
        camera_odom_topic_suffix_arg,
        gimbal_topic_suffix_arg,
        gimbal_angle_order,
        max_solve_time_arg,
        max_reprojection_error_arg,
        triangulation_node
    ])