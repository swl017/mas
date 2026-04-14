from setuptools import setup, find_packages
from glob import glob

package_name = 'gimbal_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/usb_cam_config', glob('usb_cam_config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
        ('share/' + package_name, ['README_calibration.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usrg',
    maintainer_email='seungwook1024@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_to_region_node = gimbal_controller.point_to_region_node:main',
            'siyi_ros_node = gimbal_controller.siyi_ros_node:main',
            'gimbal_los_tracker_node = gimbal_controller.gimbal_los_tracker_node:main',
            'gimbal_calibration = gimbal_controller.gimbal_calibration:main',
        ],
    },
)
