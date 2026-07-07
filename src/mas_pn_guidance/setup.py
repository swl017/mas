from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mas_pn_guidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seungwook Lee',
    maintainer_email='seungwook1024@gmail.com',
    description='PN interception guidance + engagement harness (RA-L ticket 004)',
    license='MIT',

    entry_points={
        'console_scripts': [
            # Slice 1: role/namespace inspection helper + PX4 limit matching.
            'roles = mas_pn_guidance.roles:main',
            'set_px4_limits = mas_pn_guidance.set_px4_limits:main',
            'gimbal_gt_tracker = mas_pn_guidance.gimbal_gt_tracker:main',
            'pn_guidance_node = mas_pn_guidance.pn_guidance_node:main',   # Slice 2
            # Slice 5: condition-matrix experiment conductor.
            'experiment_conductor = mas_pn_guidance.experiment_conductor:main',
        ],
    },
)
