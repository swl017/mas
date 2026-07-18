#!/bin/bash
# Reduced-topic engagement recorder (ticket 004 E2+).
# Records ONLY the ~13 topics analyze_bags.py / analyze_target_tracking.py need,
# instead of `-a`. Far less data/heap pressure than the full recorder — the fix
# for the recurring librosbag2_transport.so segfaults seen in the killed E1 run.
# Same interface as rosbag_record.sh: $1 = bag-name suffix; SIGINT finalizes.
#
#   bash bag/rosbag_record_reduced.sh eng_E2_00_simple_ekf_..._crossing
#
# Namespaces are env-parameterized (ticket 019 M4). Defaults reproduce the
# historical px4_1-interceptor / px4_2-target layout EXACTLY, so existing callers
# and analysis scripts are unaffected:
#   INT_NS  interceptor namespace                (default px4_1)
#   TGT_NS  target namespace — its truth odom is the CPA reference (default px4_2)
#   OBS_NS  cooperative-observer namespace       (default empty)
# When OBS_NS is set (a cooperative run, e.g. ticket 019 coop_1obs: INT=px4_1,
# TGT=px4_3, OBS=px4_2) the recorder ALSO logs the observer's truth odom (to
# reconstruct the observer parallax geometry) and the fused cooperative belief
# (coop_loc pose/twist), so CPA, parallax, and belief health are all
# independently reconstructable from the bag — the gap that made ticket 019 S6
# QA read "0 conditions".

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUFFIX="${1:-}"
BAG_NAME="bag_$(date +%Y%m%d_%H%M%S)${SUFFIX:+_${SUFFIX}}"

INT_NS="${INT_NS:-px4_1}"
TGT_NS="${TGT_NS:-px4_2}"
OBS_NS="${OBS_NS:-}"

# Diagnostics topics (see analyze_bags.py read()): interceptor + target truth
# odom, mission state x2, PN closing/saturation diag, gimbal+zoom+camera_info+yolo
# (bearing-noise bridge), and both ego EKF arms' target pose/twist estimates.
TOPICS=(
  "/${INT_NS}/common_frame/odom"
  "/${TGT_NS}/common_frame/odom"
  "/${INT_NS}/mission_state"
  "/${TGT_NS}/mission_state"
  "/${INT_NS}/pn/diagnostics"
  "/${INT_NS}/pn/fim_diagnostics"
  "/${INT_NS}/gimbal_state_rpy_deg"
  "/${INT_NS}/camera/zoom_level"
  "/${INT_NS}/camera/color/camera_info"
  "/${INT_NS}/yolo_result_vision"
  "/${INT_NS}/simple_loc/target_pose"
  "/${INT_NS}/simple_loc/target_twist"
  "/${INT_NS}/direct_loc/target_pose"
  "/${INT_NS}/direct_loc/target_twist"
)

# Cooperative run: add the observer truth odom (parallax geometry) + the fused
# cooperative belief that PN actually consumes (coop_loc). Harmless to include
# even if a topic never publishes (ros2 bag records zero messages).
#
# Ticket 024 S3 track B (rev1 §8/§11.1): also log the PEER RAY factor inputs the
# smoother/baseline actually consume — the exact-interface audit gap the earlier
# reduced bags left (they recorded odom+coop_loc but NOT the rays). Both routings:
#   mock GT peer   peer_ray_node -> target_rays_w_raw -> ray_delay -> target_rays_w
#   real-cam peer  triangulation -> target_rays_w      -> ray_delay_fgo -> target_rays_w_fgo
# plus the live smoother solver_diagnostics (Float64MultiArray). The ego pixel-factor
# inputs (yolo_result_vision, camera_info, gimbal, zoom, odom) are already recorded
# above, so a re-record captures the FULL smoother input for offline replay.
if [ -n "${OBS_NS}" ]; then
  TOPICS+=(
    "/${OBS_NS}/common_frame/odom"
    "/${INT_NS}/coop_loc/target_pose"
    "/${INT_NS}/coop_loc/target_twist"
    "/${INT_NS}/coop_loc/solver_diagnostics"
    "/${OBS_NS}/target_rays_w_raw"
    "/${OBS_NS}/target_rays_w"
    "/${OBS_NS}/target_rays_w_fgo"
  )
fi

ros2 bag record -o "${SCRIPT_DIR}/${BAG_NAME}" \
  --qos-profile-overrides-path "${SCRIPT_DIR}/rosbag_qos_overrides.yaml" \
  --use-sim-time \
  "${TOPICS[@]}"
