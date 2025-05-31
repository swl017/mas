sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-geographic-msgs
cd /opt/ros/${ROS_DISTRO}/lib/mavros
sudo bash install_geographiclib_datasets.sh