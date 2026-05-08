mkdir -p ~/px4_msgs_ws/src
cd ~/px4_msgs_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
cd ~/px4_msgs_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=OFF # Save time by skipping tests