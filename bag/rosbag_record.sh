#!/bin/bash
# Record all MAS topics except raw images.
# Uses QoS overrides for BEST_EFFORT topics (rosbag defaults to RELIABLE).
#
# Usage:
#   bash bag/rosbag_record.sh
#   bash bag/rosbag_record.sh my_experiment   # custom bag name suffix

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUFFIX="${1:-}"
BAG_NAME="bag_$(date +%Y%m%d_%H%M%S)${SUFFIX:+_${SUFFIX}}"

ros2 bag record -a -o "${SCRIPT_DIR}/${BAG_NAME}" \
  --qos-profile-overrides-path "${SCRIPT_DIR}/rosbag_qos_overrides.yaml" \
  --exclude '/px4_.*/image_raw|/px4_.*/camera/color/image_raw' \
  --use-sim-time
