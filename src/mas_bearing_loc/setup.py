from setuptools import setup
from glob import glob

package_name = 'mas_bearing_loc'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='usrg',
    maintainer_email='seungwook1024@gmail.com',
    description='Monocular bearing-only target localization via delay-compensated EKF '
                '(reproduction of Liu et al. 2026, arXiv 2606.10639).',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dc_ekf_node = mas_bearing_loc.dc_ekf_node:main',
            'simple_ekf_node = mas_bearing_loc.simple_ekf_node:main',
            'direct_projection_ekf_node = mas_bearing_loc.direct_projection_ekf_node:main',
            'raw_los_node = mas_bearing_loc.raw_los_node:main',
            'bearing_residual_monitor = mas_bearing_loc.bearing_residual_monitor:main',
        ],
    },
)
