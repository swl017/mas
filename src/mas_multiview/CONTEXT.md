# mas_multiview — Node Interface Contract

## Purpose
Multi-view triangulation using Ceres nonlinear least squares. Subscribes to per-camera detections, camera info, odometry, and gimbal state from all vehicles, triangulates 3D points, and publishes results as TriangulatedPointArray with covariance. Supports precomputed ray input as an alternative to raw sensor topics.

## Nodes

### triangulation_node
**File:** `src/triangulation_node.cpp`
**Build:** ament_cmake with Ceres, Eigen3 (C++17)
**Pattern:** Decoupled (subscribers cache per-camera data, timer at `publish_rate` Hz triangulates + publishes)

#### Subscriptions

Per-camera (dynamically created for cameras 1..`num_camera`, topic = `{camera_name_prefix}{i}/{suffix}`):

**Raw mode** (default per camera):

| Topic Suffix | Type | QoS | Notes |
|-------------|------|-----|-------|
| `yolo_result_vision` | `vision_msgs/Detection2DArray` | BestEffort, Volatile (10) | 2D detections from YOLO |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` | Reliable (10) | Camera intrinsics (K matrix) |
| `camera/zoom_level` | `std_msgs/Float64` | Reliable (10) | Zoom level for focal length scaling |
| `camera_pose` | `geometry_msgs/PoseStamped` | Reliable (10) | Camera pose in world frame |
| `common_frame/odom` | `nav_msgs/Odometry` | BestEffort, Volatile (10) | Odometry with pose covariance from EKF |
| `gimbal_state_rpy_deg` | `geometry_msgs/Vector3` | BestEffort, Volatile (10) | Gimbal roll/pitch/yaw in degrees |

**Precomputed ray mode** (per camera, when `use_precomputed_rays[i]` is true):

| Topic Suffix | Type | QoS | Notes |
|-------------|------|-----|-------|
| `target_rays_w` | `mas_msgs/TargetRayArray` | BestEffort, Volatile (10) | Precomputed bearing rays (replaces all 4 raw topics for that camera) |

Default topic construction: `/px4_1/yolo_result_vision`, `/px4_2/common_frame/odom`, etc.

#### Publishers

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `triangulated_points` | `mas_msgs/TriangulatedPointArray` | Reliable (10) | All triangulated 3D points (consumed by mas_tracker) |
| `triangulated_points/visualization` | `visualization_msgs/MarkerArray` | Reliable (10) | Camera rays + reprojection lines for RViz |
| `triangulated_points/cam_{i}` | `visualization_msgs/MarkerArray` | Reliable (10) | Per-camera triangulation results |
| `{prefix}{i}/target_rays_w` | `mas_msgs/TargetRayArray` | Reliable (10) | Per-camera bearing rays with detection IDs (consumed by mas_tracker) |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"common_frame"` | Reference frame for published markers |
| `publish_rate` | double | `10.0` | Triangulation rate (Hz) |
| `num_camera` | int | `3` | Number of cameras to triangulate from |
| `camera_name_prefix` | string | `"/px4_"` | Prefix for per-camera topic construction |
| `detection_topic_suffix` | string | `"yolo_result_vision"` | Detection topic suffix |
| `camera_info_topic_suffix` | string | `"camera/color/camera_info"` | Camera info topic suffix |
| `camera_zoom_topic_suffix` | string | `"camera/zoom_level"` | Zoom level topic suffix |
| `camera_pose_topic_suffix` | string | `"camera_pose"` | Camera pose topic suffix |
| `camera_odom_topic_suffix` | string | `"common_frame/odom"` | Odometry topic suffix |
| `gimbal_topic_suffix` | string | `"gimbal_state_rpy_deg"` | Gimbal state topic suffix |
| `gimbal_angle_order` | string | `"zyx"` | Gimbal rotation order: `"zyx"`, `"zxy"`, or `"zy"` |
| `max_solve_time` | double | `0.1` | Max Ceres solver time (s) |
| `max_reprojection_error` | double | `100.0` | Max reprojection error (px) — rejects outliers |
| `cov.pix_std` | double | `7.0` | Pixel detection standard deviation |
| `cov.pos_std` | double | `0.1` | Position standard deviation |
| `cov.ori_std` | double | `0.001` | Orientation standard deviation |
| `cov.gimbal_std` | double | `0.001` | Gimbal angle standard deviation |
| `cov.use_pose_covariance` | bool | `true` | Use EKF pose covariance from odom |
| `cov.include_position_uncertainty` | bool | `true` | Include position in covariance propagation |
| `cov.include_orientation_uncertainty` | bool | `true` | Include orientation in covariance propagation |
| `cov.include_gimbal_uncertainty` | bool | `true` | Include gimbal angles in covariance propagation |
| `use_precomputed_rays` | bool[] | `[]` | Per-camera flag: if true, subscribe to `target_rays_w` instead of raw sensor topics |
| `bearing_sigma_deg` | double | `0.5` | Per-ray angular uncertainty (deg) for a transmitted (precomputed) bearing ray — scales the point-to-ray residual and the bearing covariance (ticket 020) |

#### Transmitted-ray fusion (cooperative peers — ticket 020)

**Interface decision (013 Q1 / 020 Q1 = A, transmitted ray).** Cooperative peers exchange
**bearing rays**, not raw detections or fused poses: a peer validates its own detection at the
source and transmits `origin + unit LOS + detection_id` as a `mas_msgs/TargetRayArray` on
`target_rays_w`. The interceptor's fusion honors the ray it receives as a **first-class geometric
constraint** — no camera intrinsics (`K_`) are exchanged or required.

**Residual.** A raw camera contributes a 2-DOF pixel-reprojection residual; a transmitted-ray
(precomputed) camera contributes a 1-DOF **point-to-ray angular residual**
`r = e / (rho * sigma_theta)`, where `e` is the perpendicular distance from the solved point to
the ray and `rho = (X - origin) . direction` is the along-ray range (guards: reject `rho <= 0`,
floor `rho`). Both residuals share one Ceres problem over the 3-D position, so a
`{raw ego + transmitted peer}` pair is jointly well-constrained. `sigma_theta` is the fusion-side
`bearing_sigma_deg` parameter (Q2 = param, no `mas_msgs` change).

**K_-free accounting.** For precomputed cameras the initial-guess weighting/gate, the results
gate metric (perpendicular ray distance in metres), and the covariance (a 2-DOF bearing Jacobian
about the LOS, variance `sigma_theta^2`) all branch on `Camera::is_precomputed_` and never read an
intrinsics matrix. Note: the results gate `max_reprojection_error` mixes pixels (raw) and metres
(ray distance) under one threshold — a documented v1 limitation; a dedicated ray-distance
threshold is a future refinement.

**Auditability.** Each fusion tick DEBUG-logs the fresh-ego vs stale-peer capture stamps and the
peer lag (`[ray-stamp] ...`) so the fresh-ego × stale-peer temporal inconsistency in the fair
peer-only latency experiment is traceable.

**Offline test:** `lib/multiview_triangulation/test/test_transmitted_ray.cpp` — a
`{raw + transmitted ray}` pair triangulates a known GT point, the fused range tracks the peer-ray
angle, and the covariance is PD and degrades as the rays approach parallel. Built by CMake
(`test_transmitted_ray`), GPU-free.

#### Services
None.

## Dependencies

**Upstream:** `mas_common_frame` (common_frame/odom), `gimbal_controller/siyi_gimbal_node` (gimbal_state_rpy_deg), `ultralytics_ros` (yolo_result_vision), camera driver (camera_info, camera_pose, zoom)
**Downstream:** `mas_tracker` consumes `triangulated_points` and per-camera `target_rays_w`

**Build:** Ceres Solver, Eigen3, rclcpp, sensor_msgs, geometry_msgs, nav_msgs, visualization_msgs, vision_msgs, std_msgs, mas_msgs, tf2_ros, tf2_geometry_msgs

## Key Files

| File | Role |
|------|------|
| `src/triangulation_node.cpp` | ROS2 node — subscribes, caches, triangulates on timer, publishes |
| `lib/multiview_triangulation/src/multiview_triangulation.cpp` | Core Ceres-based triangulation solver |
| `lib/multiview_triangulation/src/covariance_propagation.cpp` | Jacobian-based uncertainty propagation |
| `lib/multiview_triangulation/include/camera.h` | Camera model and intrinsics |
| `lib/multiview_triangulation/include/detection.h` | 2D detection structure |
| `lib/multiview_triangulation/include/ray.h` | Camera ray representation |
| `launch/triangulation.launch.py` | ROS2 launch configuration |
