from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mas_policy'

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
    description='Policy deployment node for iris_ma6 trained MARL policies',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'policy_node = mas_policy.policy_node:main',
        ],
    },
)
