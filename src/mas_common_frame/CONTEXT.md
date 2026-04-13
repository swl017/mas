# mas_common_frame

## Purpose
Transforms multi-vehicle positions from EKF local coordinates into a shared mission reference frame using a one-time GPS-derived offset. Broadcasts TF2 transforms.

## Nodes

### common_frame_node
**File:** `mas_common_frame/common_frame_node.py`
**Pattern:** Decoupled (subscribers → cache per-vehicle state, timer at 10 Hz → compute + publish + broadcast TF)

#### Subscriptions (per vehicle `/{veh}/`)
- `mavros/home_position/home` (`mavros_msgs/HomePosition`) — EKF local frame origin (one-time init)
- `mavros/local_position/pose` (`geometry_msgs/PoseStamped`) — EKF local position + orientation
- `mavros/local_position/pose_cov` (`geometry_msgs/PoseWithCovarianceStamped`) — EKF pose covariance
- `mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`) — local velocity

#### Publishers (per vehicle `/{veh}/`)
- `common_frame/pose` (`geometry_msgs/PoseStamped`) — pose in mission frame
- `common_frame/odom` (`nav_msgs/Odometry`) — odometry in mission frame (pose + covariance + twist)

#### TF2 Broadcasts
- `common_frame` → `{veh}_base_link` transforms

#### Parameters
- `vehicle_name_prefix` (`string`, default: `"px4_"`) — namespace prefix
- `num_vehicles` (`int`, default: `2`) — number of vehicles
- `common_frame_origin` (`float[]`, default: `[37.7749, -122.4194, 0.0]`) — mission frame origin [lat, lon, alt]

---

### common_frame_node_single
**File:** `mas_common_frame/common_frame_node_single.py`
**Pattern:** Same as above, single vehicle variant (uses node namespace as vehicle name)

## Dependencies
None (standalone). Only needs MAVRos topics from flight controller.

## Key Files
- `mas_common_frame/common_frame_node.py` — Multi-vehicle node
- `mas_common_frame/common_frame_node_single.py` — Single-vehicle variant
- `mas_common_frame/common_frame.py` — Core coordinate transform logic
