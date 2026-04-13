# mas_tracker

## Purpose
3D multi-object tracking using SORT algorithm (Hungarian assignment + Kalman filtering). Maintains persistent track IDs, selects a target for gimbal pointing, and publishes the ego camera's bearing ray to the chosen target.

## Nodes

### sort3d_tracking_node
**File:** `src/sort3d_node.cpp` | **Header:** `include/mas_tracker/sort3d_node.h`
**Executable:** `mas_tracker_node`
**Pattern:** Coupled (subscriber callback processes detections and publishes tracked objects immediately)
**Deployment:** Per-drone (each drone runs its own sort3d instance)

#### Subscriptions
- `input_detections/triangulated_points` (`mas_msgs/TriangulatedPointArray`) — triangulated 3D detections from mas_multiview
- `odom` (`nav_msgs/Odometry`) — robot odometry (optional, for motion compensation)
- `set_auto_pick_mode` (`std_msgs/Int8`) — enable/disable automatic target selection
- `set_target_position` (`geometry_msgs/PointStamped`) — manually select target by position; sort3d finds nearest local track and disables auto-pick
- `target_rays_w_{i}` (`mas_msgs/TargetRayArray`) — per-camera bearing rays (one subscription per camera, remapped to `{camera_name_prefix}{i}/target_rays_w` by launch file)

#### Publishers
- `tracked_objects/class_{i}` (`vision_msgs/Detection3DArray`) — per-class tracked objects with persistent IDs
- `chosen_target_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — selected target pose with triangulation covariance
- `target_region` (`geometry_msgs/PointStamped`) — selected target point (consumed by gimbal_controller)
- `chosen_target_ray_w` (`geometry_msgs/Vector3Stamped`) — bearing ray from ego camera to chosen target

#### Parameters
- `association_distance_threshold` (`double`, default: `1.0`) — max distance for detection-to-track association
- `max_track_age` (`int`, default: `30`) — frames before unmatched track is deleted
- `min_tracker_hits_for_valid` (`int`, default: `5`) — minimum hits before track is considered valid
- `number_of_object_classes` (`int`, default: `1`) — number of detection classes to track separately
- `num_cameras` (`int`, default: `3`) — number of cameras in the multiview system
- `camera_name_prefix` (`string`, default: `/px4_`) — prefix for per-camera ray topics
- `self_camera_index` (`int`, default: `1`) — 1-indexed camera index for this drone

#### Ray selection logic
The node selects the bearing ray from the ego camera that corresponds to the tracked target:
1. **ID-based (primary):** Finds the nearest TriangulatedPoint to the Kalman-filtered target position, extracts its `detection_ids`, matches against rays in the ego camera's TargetRayArray
2. **Angular proximity (fallback):** If no ID match, finds the ray with maximum dot product to the target direction from the ego camera origin

## Dependencies
- mas_multiview — provides TriangulatedPointArray and per-camera TargetRayArray
- mas_msgs — message definitions

## Key Files
- `src/sort3d_node.cpp` — ROS2 node implementation
- `include/mas_tracker/sort3d_node.h` — Node header
- `include/mas_tracker/sort3d.h` — SORT3D tracker (Kalman + Hungarian)
- `include/mas_tracker/kalman_filter.h` — Kalman filter (header-only)
- `src/hungarian.cpp` — Hungarian algorithm
- `src/sort3d.cpp` — SORT3D implementation
- `launch/sort3d.launch.py` — Launch file with per-camera ray remappings
