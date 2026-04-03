ros2 bag record -a -o bag_$(date +%Y%m%d_%H%M%S) \
  --qos-profile-overrides-path /home/usrg/mas/bag/rosbag_qos_overrides.yaml \
  --exclude '/px4_.*/image_raw|/px4_.*/camera/color/image_raw'