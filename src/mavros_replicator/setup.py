from glob import glob
from setuptools import setup

package_name = "mavros_replicator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="usrg",
    maintainer_email="seungwook1024@gmail.com",
    description="PX4 px4_msgs (NED/FRD) → MAVROS-shaped ROS topics (ENU/FLU) translator.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mavros_replicator = mavros_replicator.replicator_node:main",
        ],
    },
)
