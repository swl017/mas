# Priority 4: Interface Conformance — Implementation Plan

## Context

The gap analysis (doc/active/gap_analysis.md) identified that cross-agent interfaces use oversized messages (full Detection2DArray for a boolean check), visualization-only formats (MarkerArray for structured 3D data), and missing topics (zoom_level, target_rays_w, combined_ang_vel). There's also a live bug: mas_tracker publishes `PoseStamped` but mas_policy subscribes to `PoseWithCovarianceStamped`, breaking the triangulation pipeline. This plan addresses all 6 Priority 4 tasks.

---

## Phase 1: Create `mas_msgs` package (foundation)

**New package:** `/home/usrg/mas/src/mas_msgs/` (ament_cmake)

**Message definitions:**
```
# msg/TriangulatedPoint.msg
geometry_msgs/Point position
float64[9] covariance    # 3x3 row-major

# msg/TriangulatedPointArray.msg
std_msgs/Header header
mas_msgs/TriangulatedPoint[] points

# msg/TargetRay.msg
geometry_msgs/Vector3 direction   # unit vector in world frame
string detection_id               # links back to detection/track ID

# msg/TargetRayArray.msg
std_msgs/Header header
geometry_msgs/Point origin        # camera position in world frame
mas_msgs/TargetRay[] rays
```

**Files to create:**
- `mas_msgs/package.xml` — deps: `std_msgs`, `geometry_msgs`, `rosidl_default_generators` (build), `rosidl_default_runtime` (exec)
- `mas_msgs/CMakeLists.txt` — `rosidl_generate_interfaces` (pattern from ultralytics_ros/CMakeLists.txt:29-35)
- `mas_msgs/msg/TriangulatedPoint.msg`
- `mas_msgs/msg/TriangulatedPointArray.msg`
- `mas_msgs/msg/TargetRay.msg`
- `mas_msgs/msg/TargetRayArray.msg`

**Verify:** `colcon build --packages-select mas_msgs`

---

## Phase 2: Task 1 — Structured triangulated_points (clean swap)

### 2a. Update mas_multiview publisher

**File:** `mas_multiview/src/triangulation_node.cpp`
- Add `#include <mas_msgs/msg/triangulated_point_array.hpp>`
- Replace the MarkerArray publisher on `triangulated_points` (line 190) with `mas_msgs::msg::TriangulatedPointArray`
- In the publish loop (lines 446-487): build `TriangulatedPointArray` from `multiview_triangulation_->results_`. For each result:
  - `point.position` = result.position (x,y,z)
  - `point.covariance` = result.covariance 3x3 row-major (9 doubles from Eigen::Matrix3d)
- Keep `triangulated_points/visualization` as MarkerArray (for RViz)
- Keep per-camera MarkerArray publishers unchanged

**File:** `mas_multiview/CMakeLists.txt` — add `find_package(mas_msgs REQUIRED)`, add to `ament_target_dependencies`
**File:** `mas_multiview/package.xml` — add `<depend>mas_msgs</depend>`

### 2b. Update mas_tracker subscriber + fix PoseStamped bug

**File:** `mas_tracker/src/sort3d_node.cpp`
- Change subscription (line 52) from `MarkerArray` to `mas_msgs::msg::TriangulatedPointArray`
- Replace `markers2dets()` (lines 169-213) with `triPoints2dets()`: directly map `point.position` to `det.bbox.center.position`, extract diagonal sqrt from `point.covariance` for `det.bbox.size`
- **Fix bug:** Change `target_pub_` (line 58) from `PoseStamped` to `PoseWithCovarianceStamped`
- Update `pubChosenTarget()` (lines 275-287): populate covariance from the tracked detection's stored covariance. Set diagonal entries `cov[0], cov[7], cov[14]` from the detection's variance values
- Store per-track covariance in the tracking pipeline (pass through from TriangulatedPoint)

**File:** `mas_tracker/include/mas_tracker/sort3d_node.h`
- Line 83: change `PoseStamped` → `PoseWithCovarianceStamped`
- Add `#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>`
- Add `#include <mas_msgs/msg/triangulated_point_array.hpp>`

**File:** `mas_tracker/CMakeLists.txt` — add `find_package(mas_msgs REQUIRED)`, add to deps
**File:** `mas_tracker/package.xml` — add `<depend>mas_msgs</depend>`

No changes to mas_policy — it already subscribes to `PoseWithCovarianceStamped` on `/chosen_target_pose` (observation_assembler.py:150-155). The type mismatch fix makes it work.

---

## Phase 3: Task 2 — Compact `detection_active` Bool topic

### 3a. Add publisher to ultralytics_ros

**File:** `ultralytics_ros/script/tracker_node.py`
- Add `from std_msgs.msg import Bool`
- After line 78 (`results_vision_msg_pub`), add: `self.detection_active_pub = self.create_publisher(Bool, result_topic + "_active", qos_profile_sensor_data)`
- After publishing Detection2DArray (~line 139), also publish: `Bool(data=len(detections_msg.detections) > 0)`

Topic will be e.g. `/{ns}/yolo_result_active`.

### 3b. Update mas_policy peer subscriptions

**File:** `mas_policy/mas_policy/observation_assembler.py`
- In `_create_peer_subscriptions()` (line 217-222): replace `Detection2DArray` subscription with `Bool` on `f'/{peer}/yolo_result_active'`
- Add callback `_peer_detection_active_callback(self, msg: Bool, veh: str)` that sets `state.bbox_empty = 0.0 if msg.data else 1.0` and updates `state.detection_timestamp`
- Keep ego `Detection2DArray` subscription unchanged (ego needs full bbox for 4D observation)
- Add `from std_msgs.msg import Bool` import

---

## Phase 4: Task 3 — Pre-computed `target_rays_w`

**Clarification:** `ray_w` in the semantic architecture means per-target bearing rays (camera origin → bbox center in world frame), NOT the gimbal LOS direction. There can be 0-N rays per agent per frame.

**Publisher:** `mas_multiview` — it is the only node that already has all required inputs (detections, camera intrinsics K, zoom level, gimbal angles, body pose) and already computes rays internally via `Ray::getRayFromPixels(K, R, t, pixel)` in `ray.h`. Each agent's multiview instance computes rays for its own camera(s) and publishes them; peer agents subscribe to those pre-computed rays.

**Two consumers with different needs:**
1. **mas_multiview (peer cameras)** — needs ALL rays from each peer for triangulation. Replaces subscribing to 4 raw cross-agent topics per peer camera (detections + camera_info + gimbal + odom) with a single `target_rays_w` topic.
2. **mas_policy** — needs ONE ray corresponding to the selected target for the observation vector (indices 17-19 ego, 6-8 peer).

### 4a. Publish own camera rays from mas_multiview

**File:** `mas_multiview/src/triangulation_node.cpp`
- Add `#include <mas_msgs/msg/target_ray_array.hpp>`
- Add per-camera publisher: `{camera_name_prefix}{i}/target_rays_w` (`mas_msgs::msg::TargetRayArray`)
- In the triangulation loop (lines 332-421), after computing camera extrinsics and processing detections, build and publish `TargetRayArray`:
  - `header.stamp` = current time, `header.frame_id` = `frame_id_`
  - `origin` = `camera_t_world_camera` (already computed at line 382)
  - For each detection processed by `multiview_triangulation_->addDetection(i, det_2d)`:
    - Compute ray via `Ray::getRayFromPixels(scaled_k, combined_R_world_gimbal, camera_t_world_camera, det_2d.center)`
    - `ray.direction` = the resulting `ray.ray_direction` (already a unit vector)
    - `ray.detection_id` = `det.id`
- Publish BEFORE calling `Triangulate()` — rays are inputs to triangulation, not outputs
- This adds zero new computation; the rays are already computed internally by the multiview library

### 4b. Cross-agent: multiview subscribes to peer `target_rays_w`

**File:** `mas_multiview/src/triangulation_node.cpp`

Currently, for each peer camera, multiview subscribes to 4 topics: `/{peer}/yolo_result_vision`, `/{peer}/camera/color/camera_info`, `/{peer}/common_frame/odom`, `/{peer}/gimbal_state_rpy_deg`. With pre-computed rays, a peer only sends `target_rays_w`.

- Add parameter `use_precomputed_rays` (bool, default false) for incremental rollout
- When `use_precomputed_rays` is true for a camera index:
  - Subscribe to `{camera_name_prefix}{i}/target_rays_w` (`TargetRayArray`) instead of the 4 raw topics
  - In `timerCallback()`, for peer cameras with pre-computed rays: skip raw detection → ray computation. Instead, directly set camera extrinsics from `origin` and feed `TargetRay.direction` as pre-computed ray directions into the triangulation solver
  - The `Ray` struct in `ray.h` can be populated directly: `ray.ray_origin = msg.origin`, `ray.ray_direction = target_ray.direction`
- When false: keep current behavior (subscribe to raw topics, compute rays locally)

### 4c. Policy: selected target ray from `target_rays_w`

**File:** `mas_policy/mas_policy/observation_assembler.py`

The policy observation vector uses a single 3D ray per agent. Currently this is the gimbal LOS direction (computed from gimbal angles + body quaternion). With `target_rays_w`, it becomes the bearing to the selected target.

- Add ego subscription: `TargetRayArray, 'target_rays_w'` (from own multiview)
- Add peer subscription: `TargetRayArray, f'/{peer}/target_rays_w'`
- Add `from mas_msgs.msg import TargetRayArray` import
- Add callback `_target_rays_callback(self, msg: TargetRayArray, veh: str)` that caches the full ray array per vehicle in a new dict `self._target_rays: dict[str, TargetRayArray]`
- In `assemble()`:
  - **Ego:** Look up the selected target's ray by matching `detection_id` against the tracked target ID (from `self._tri_state` or a new cached target_id). If found, use that ray for observation indices 17-19. If not found (no target selected or no detections), fall back to gimbal LOS (current computation).
  - **Peer:** Same logic — look up peer's selected target ray from `self._target_rays[peer]`. Fall back to gimbal LOS if unavailable. For peers, the "selected target" could be inferred from `/{peer}/chosen_target_pose` (the tracker's selected target ID).
- **Note:** Need to track which target_id each vehicle is tracking. Add subscription to `/{peer}/chosen_target_pose` for peers (or a simpler `/{peer}/selected_target_id` topic from tracker). For ego, the existing `chosen_target_pose` subscription already provides this.

---

## Phase 5: Task 5 — Pre-computed `combined_ang_vel_w`

Use `geometry_msgs/Vector3Stamped` — timestamps matter for cross-agent synchronization.

### 5a. Sim path: los_rate_controller.py

**File:** `/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py`
- Add `from geometry_msgs.msg import Vector3Stamped`
- Add publisher: `self._combined_ang_vel_w_pub = self.create_publisher(Vector3Stamped, 'combined_ang_vel_w', sensor_qos)`
- Cache body angular velocity in `_imu_callback` (line 192, currently only caches orientation):
  ```python
  self._body_angular_velocity_b = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
  ```
- In `_publish_state()` (line 354), after publishing gimbal state, compute and publish:
  - Gimbal rates already available: `yaw_rate = (self._yaw - self._yaw_prev) / dt`, `pitch_rate = (self._pitch - self._pitch_prev) / dt`
  - Combined body-frame: `gimbal_ang_vel_b = [0, pitch_rate, yaw_rate]`, `combined_b = body_ang_vel_b + gimbal_ang_vel_b`
  - Rotate to world: use `_quat_rotate_inverse` (already defined at line 64) with conjugate, or add a `_quat_rotate` helper
  - Stamp with `self.get_clock().now().to_msg()`

### 5b. Real path: siyi_ros_node.py

**File:** `gimbal_controller/gimbal_controller/siyi_ros_node.py`
- Use **encoder finite differences** for body-frame gimbal joint rates (consistent with sim). `getAttitudeSpeed()` from 0x0D reports world-frame IMU rates — different frame, reserved for diagnostics.
- Cache body angular velocity + quaternion from existing `odom_callback` (line 159):
  ```python
  self._body_ang_vel_b = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
  self._body_quat_wxyz = np.array([q.w, q.x, q.y, q.z])  # from msg.pose.pose.orientation
  ```
- Track previous encoder angles + timestamp for finite-difference rate estimation:
  ```python
  self._prev_enc_yaw = 0.0
  self._prev_enc_pitch = 0.0
  self._prev_enc_time = time.time()
  ```
- Add `from geometry_msgs.msg import Vector3Stamped` and `import numpy as np`
- Add publisher: `self.combined_ang_vel_w_pub = self.create_publisher(Vector3Stamped, 'combined_ang_vel_w', qos_profile)`
- In `publish_angles_callback()`, after encoder publish: compute gimbal rates from encoder finite differences, compute combined angular velocity, rotate to world frame, publish

### 5c. Update mas_policy peer subscriptions

**File:** `mas_policy/mas_policy/observation_assembler.py`
- Add peer subscription: `Vector3Stamped, f'/{peer}/combined_ang_vel_w'`
- Add `from geometry_msgs.msg import Vector3Stamped` import
- Add callback caching the 3D vector in `VehicleState.combined_ang_vel_w`
- **Remove** peer `gimbal_state_rpy_rad` subscription (line 211-215) — no longer needed for peers. Peer ray comes from `target_rays_w` (Phase 4), peer ang_vel from `combined_ang_vel_w` (this phase). Keep peer odom subscription for position/velocity.
- In `assemble()` peer loop (lines 374-379): use cached `other.combined_ang_vel_w` instead of computing from raw gimbal angles
- Add field to `VehicleState`:
  ```python
  combined_ang_vel_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
  ```
- **Ego** still computes combined_ang_vel_w locally (from ego gimbal state + IMU) — no change to ego path.

---

## Phase 6: Task 4 — Zoom level feedback

### 6a. Sim path: los_rate_controller.py

**File:** `/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py`
- Subscribe to `camera/zoom` (`std_msgs/Float64`) — already published in sim and consumed by mas_multiview. Simpler and more accurate than integrating `zoom_cmd` rate.
- Add `from std_msgs.msg import Float32, Float64`
- Cache zoom from subscription: `self._zoom_level = 1.0`, updated by `_zoom_callback`
- Add publisher: `self._zoom_level_pub = self.create_publisher(Float32, 'zoom_level', sensor_qos)`
- Publish cached zoom in `_publish_state()` (republish Float64 → Float32 for policy consistency)

### 6b. Real path: siyi_ros_node.py

**File:** `gimbal_controller/gimbal_controller/siyi_ros_node.py`
- Add `from std_msgs.msg import Float32`
- Add publisher: `self.zoom_level_pub = self.create_publisher(Float32, 'zoom_level', qos_profile)`
- In `publish_angles_callback()`: call `self.cam.getZoomLevel()` and publish

### 6c. Update mas_policy subscriptions

**File:** `mas_policy/mas_policy/observation_assembler.py`
- Add ego subscription: `Float32, 'zoom_level'` with callback setting `state.zoom_level`
- Add peer subscription: `Float32, f'/{peer}/zoom_level'` with same callback
- Default `zoom_level: float = 1.0` kept but now updated from subscription

---

## Phase 7: Task 6 — Operator script

**New file:** `scripts/operator.py` — a proper `rclpy` node (not subprocess-based) to avoid galactic/humble CLI flag differences.

- Implements an rclpy node with:
  - Publishers: `/mission_state_cmd` (Int8, RELIABLE + transient_local), `/{ns}/set_auto_pick_mode` (Int8)
  - Subscribers: `/{ns}/mission_state` for each vehicle (displays per-agent state feedback in terminal)
- Simple text menu loop:
  1. "Start Tracking" → pub TRACKING (1) to `/mission_state_cmd`
  2. "Approve Mission" → pub MISSION (2) to `/mission_state_cmd`
  3. "Abort (IDLE)" → pub IDLE (0) to `/mission_state_cmd`
  4. "Enable auto-pick on vehicle X" → pub to `/{ns}/set_auto_pick_mode`
  5. "Status" → display latest mission_state from each agent
- Reads vehicle namespaces from `mas_policy/config/vehicles.yaml`
- Runs as: `ros2 run mas_mission operator` (or standalone `python3 scripts/operator.py`)

---

## Phase 8: Update documentation

- Update CONTEXT.md for: mas_multiview, mas_tracker, mas_policy, gimbal_controller
- Update ARCHITECTURE.md with new topics (`target_rays_w`, `combined_ang_vel_w`, `zoom_level`, `yolo_result_active`)
- Update `doc/diagrams/mas_architecture_semantic.d2`: rename `ray_w` → `target_rays_w` in cross-agent edges
- Update gap_analysis.md to mark Priority 4 tasks as done
- Update feature_list.json with new feature entry
- Update progress.txt (end-of-session protocol)

---

## Review Notes (2026-03-26) — Incorporated

| Review Point | Section | Change Made |
|-------------|---------|-------------|
| PoseStamped bug | Phase 2b | Confirmed — fix included in same phase as TriangulatedPointArray swap |
| `getAttitudeSpeed()` frame | Phase 5b | Encoder finite differences for body-frame consistency; `getAttitudeSpeed()` reserved for diagnostics |
| Timestamped messages | Phase 5 | `Vector3Stamped` for `combined_ang_vel_w`; `TargetRayArray` has its own header |
| Sim zoom from `camera/zoom` | Phase 6a | Subscribe to existing `camera/zoom` (Float64) instead of integrating zoom_cmd rate |
| rclpy operator node | Phase 7 | Proper rclpy node, not subprocess-based |
| `ray_w` is per-target bearing rays | Phase 4 (rewritten) | Renamed to `target_rays_w`, new `TargetRayArray` message, published by mas_multiview (has all inputs: detections, K, zoom, gimbal, pose). Variable-length array with per-detection rays. Policy selects one ray per agent for obs vector. |

---

## Build & Verification

**Note:** mas_multiview requires cmake 3.29+ (for CUDAToolkit support via Ceres). The system cmake is 3.16. Use the pip-installed cmake:
```bash
export PATH=$HOME/.local/bin:$PATH  # cmake 3.29 from pip
```

```bash
# Build order (dependencies)
export PATH=$HOME/.local/bin:$PATH
colcon build --packages-select mas_msgs
colcon build --packages-select mas_multiview mas_tracker
colcon build --packages-select mas_policy ultralytics_ros
# In IsaacPX4 workspace:
cd ~/IsaacPX4/ros2_ws && colcon build --packages-select gimbal_stabilizer
```

**Verification checklist:**
1. `colcon build` succeeds for all modified packages
2. `ros2 interface show mas_msgs/msg/TriangulatedPointArray` and `mas_msgs/msg/TargetRayArray` show correct structure
3. Sim e2e: launch 2-3 agents, verify:
   - `triangulated_points` topic carries TriangulatedPointArray
   - `chosen_target_pose` carries PoseWithCovarianceStamped with non-zero covariance
   - `yolo_result_active` (Bool) publishes on each agent
   - `target_rays_w` (TargetRayArray) publishes per camera with ray count matching detection count
   - `combined_ang_vel_w` (Vector3Stamped) publishes on each agent
   - `zoom_level` (Float32) publishes on each agent
   - `/{peer}/yolo_result_active` received by mas_policy (not full Detection2DArray)
   - Observation vector dimensions unchanged (30 + 16*N + optional 6)
4. `ros2 run mas_mission operator` menu works for state transitions and shows per-agent state

---

## Key Files Modified

| File | Changes |
|------|---------|
| `mas_msgs/` (new) | Message package: TriangulatedPoint, TriangulatedPointArray, TargetRay, TargetRayArray |
| `mas_multiview/src/triangulation_node.cpp` | Publish TriangulatedPointArray + TargetRayArray; subscribe to peer TargetRayArray (`use_precomputed_rays` param) |
| `mas_multiview/CMakeLists.txt`, `package.xml` | Add mas_msgs dependency |
| `mas_tracker/src/sort3d_node.cpp` | Subscribe TriangulatedPointArray, publish PoseWithCovarianceStamped (bug fix) |
| `mas_tracker/include/mas_tracker/sort3d_node.h` | Type changes for publisher/subscriber |
| `mas_tracker/CMakeLists.txt`, `package.xml` | Add mas_msgs dependency |
| `ultralytics_ros/script/tracker_node.py` | Add `detection_active` Bool publisher |
| `mas_policy/mas_policy/observation_assembler.py` | Replace peer Detection2DArray with Bool; subscribe to target_rays_w + combined_ang_vel_w + zoom_level; select target ray for obs vector; remove peer gimbal_state_rpy_rad sub |
| `gimbal_stabilizer/los_rate_controller.py` | Add combined_ang_vel_w (Vector3Stamped), zoom_level (Float32) publishers; subscribe to camera/zoom |
| `gimbal_controller/siyi_ros_node.py` | Add combined_ang_vel_w (Vector3Stamped), zoom_level (Float32) publishers |
| `scripts/operator.py` (new) | rclpy-based operator CLI with mission state feedback |
| `doc/diagrams/mas_architecture_semantic.d2` | Rename `ray_w` → `target_rays_w` in cross-agent edges |
| CONTEXT.md (x4), ARCHITECTURE.md, gap_analysis.md, feature_list.json, progress.txt | Documentation updates |
