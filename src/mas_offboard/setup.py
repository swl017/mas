from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mas_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Multi-agent offboard drone control for PX4 via MAVROS',
    license='MIT',

    entry_points={
        'console_scripts': [
            'offboard_control = mas_offboard.offboard_control:main',
            'target_maneuver_node = mas_offboard.target_maneuver_node:main',
            'auto_arm = mas_offboard.auto_arm_node:main',
        ],
    },
)
