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
