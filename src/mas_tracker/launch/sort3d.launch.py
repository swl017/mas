from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_sort3d(context):
    ns = LaunchConfiguration('ns').perform(context)
    num_cameras = int(LaunchConfiguration('num_cameras').perform(context))
    camera_name_prefix = LaunchConfiguration('camera_name_prefix').perform(context)
    self_camera_index = int(LaunchConfiguration('self_camera_index').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # Build per-camera ray topic remappings
    remappings = [
        ('input_detections/triangulated_points', 'triangulated_points'),
    ]
    for i in range(1, num_cameras + 1):
        remappings.append(
            (f'target_rays_w_{i}', f'{camera_name_prefix}{i}/target_rays_w')
        )

    return [Node(
        namespace=ns,
        package='mas_tracker',
        executable='mas_tracker_node',
        name='sort3d',
        output='screen',
        remappings=remappings,
        parameters=[{
            'association_distance_threshold': 15.0,
            'min_tracker_hits_for_valid': LaunchConfiguration('min_tracker_hits_for_valid'),
            'max_track_age': 20,
            'number_of_object_classes': LaunchConfiguration('number_of_object_classes'),
            'num_cameras': num_cameras,
            'camera_name_prefix': camera_name_prefix,
            'self_camera_index': self_camera_index,
            'use_sim_time': use_sim_time,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='/',
                              description='Namespace for the node'),
        DeclareLaunchArgument('number_of_object_classes', default_value='1',
                              description='Number of object classes to track'),
        DeclareLaunchArgument('min_tracker_hits_for_valid', default_value='5',
                              description='Minimum tracker hits for a valid detection'),
        DeclareLaunchArgument('num_cameras', default_value='3',
                              description='Number of cameras in the multiview system'),
        DeclareLaunchArgument('camera_name_prefix', default_value='/px4_',
                              description='Prefix for per-camera topics (e.g. /px4_)'),
        DeclareLaunchArgument('self_camera_index', default_value='1',
                              description='1-indexed camera index for this drone'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (sim) time when true, wall time otherwise'),
        OpaqueFunction(function=_launch_sort3d),
    ])
