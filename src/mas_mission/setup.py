from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mas_mission'

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
    description='Mission state management and command routing for multi-agent systems',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = mas_mission.mission_node:main',
        ],
    },
)
