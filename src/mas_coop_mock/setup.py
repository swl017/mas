import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mas_coop_mock'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seungwook Lee',
    maintainer_email='seungwook1024@gmail.com',
    description='Mock cooperative-sensing closed loop for the PN harness (RA-L ticket 019)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ticket 019 mock-cooperative nodes (see r_research / i_design)
            'cv_smoother = mas_coop_mock.cv_smoother_node:main',
            'viewing_offset = mas_coop_mock.viewing_offset_node:main',
            'ray_delay = mas_coop_mock.ray_delay_node:main',
            'peer_ray = mas_coop_mock.peer_ray_node:main',
        ],
    },
)
