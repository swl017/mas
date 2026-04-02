"""Launch file for mas_operator.

Reads vehicle list and parameters from operator.yaml,
launches a single operator_node that monitors all vehicles.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)

    if os.path.isabs(config_file):
        config_path = config_file
    else:
        pkg_share = get_package_share_directory('mas_operator')
        config_path = os.path.join(pkg_share, config_file)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f'Operator config not found: {config_path}')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    vehicles = config['vehicles']

    node = Node(
        package='mas_operator',
        executable='operator_node',
        name='operator_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicles': vehicles,
            'aoi_warn_ms': config.get('aoi_warn_ms', 500.0),
            'aoi_critical_ms': config.get('aoi_critical_ms', 2000.0),
            'cov_warn_threshold': config.get('cov_warn_threshold', 5.0),
            'safety_distance_m': config.get('safety_distance_m', 9.5),
            'tri_timeout_s': config.get('tri_timeout_s', 1.0),
            'status_rate_hz': config.get('status_rate_hz', 2.0),
            'num_object_classes': config.get('num_object_classes', 1),
        }],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/operator.yaml',
            description='Path to the operator configuration file',
        ),
        OpaqueFunction(function=launch_setup),
    ])
