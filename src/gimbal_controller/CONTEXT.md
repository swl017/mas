# gimbal_controller

## Purpose
Gimbal hardware interface (SIYI SDK) and camera pointing control for target tracking.

## Nodes

### siyi_gimbal_node
**File:** `gimbal_controller/siyi_ros_node.py`
**Pattern:** Decoupled (subscriber → cache command, timer → publish state + actuate)

#### Subscriptions
- `siyi_gimbal_angles/command_rpy_deg` (`geometry_msgs/Vector3`) — target gimbal angles from point_to_region_node
- `mavros/imu/data` (`sensor_msgs/Imu`) — aircraft attitude + angular velocity for joint angle derivation and combined_ang_vel_w (BEST_EFFORT QoS)
- `common_frame/odom` (`nav_msgs/Odometry`) — robot odometry for aircraft attitude injection to gimbal via 0x22 (BEST_EFFORT QoS)

#### Publishers
- `siyi_gimbal_angles/encoder_rpy_deg` (`geometry_msgs/Vector3`) — **primary gimbal state**: joint-frame angles (degrees) at 25 Hz. Remapped to `gimbal_state_rpy_deg` in launch. Direction multipliers (`yaw_direction`, `pitch_direction`) applied. Derived as: yaw from 0x0D (encoder-based joint angle in all modes), pitch/roll from `0x0D_heading - aircraft_attitude_ENU`. Requires `common_frame/odom` for pitch/roll derivation.
- `siyi_gimbal_angles/state_rpy_deg` (`geometry_msgs/Vector3`) — **secondary**: 0x0D raw heading-frame angles at 25 Hz. Remapped to `gimbal_imu_rpy_deg` in launch. Yaw=joint(encoder), pitch/roll=world(IMU).
- `combined_ang_vel_w` (`geometry_msgs/Vector3Stamped`) — gimbal angular velocity in world frame (finite-difference of joint angles)
- `camera/zoom_level` (`std_msgs/Float64`) — current zoom level from SDK `getZoomLevel()`

#### Parameters
- `server_ip` (`string`, default: `"192.168.144.25"`) — SIYI gimbal IP
- `server_port` (`int`, default: `37260`) — SIYI gimbal port
- `publish_rate_hz` (`double`, default: `25.0`) — state publish rate
- `yaw_direction` (`double`, default: `1.0`) — yaw sign convention
- `pitch_direction` (`double`, default: `-1.0`) — pitch sign convention
- `enable_encoder_stream` (`bool`, default: `true`) — enable magnetic encoder angle streaming (0x26)
- `enable_aircraft_attitude` (`bool`, default: `true`) — enable aircraft EKF attitude injection (0x22)
- `encoder_stream_freq` (`int`, default: `50`) — encoder stream frequency in Hz

---

### point_to_region_node
**File:** `gimbal_controller/point_to_region_node.py`
**Pattern:** Decoupled (subscribers → cache sensor data, timer at 10 Hz → compute + publish)

#### Subscriptions
- `gimbal_state_rpy_deg` (`geometry_msgs/Vector3`) — current gimbal angles from siyi_gimbal_node
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`) — camera intrinsics
- `common_frame/pose_` (`geometry_msgs/PoseStamped`) — robot pose from mas_common_frame
- `common_frame/odom` (`nav_msgs/Odometry`) — robot odometry from mas_common_frame
- `/target_region` (`geometry_msgs/PointStamped`) — target point from mas_tracker

#### Publishers
- `gimbal_command_los_world_deg` (`geometry_msgs/Vector3`) — world-frame azimuth/elevation to target (z=az, y=el, deg)

#### Dependencies
- mas_common_frame — provides robot pose/odom
- mas_tracker — provides target_region
- siyi_gimbal_node — provides gimbal state, consumes gimbal commands

## Key Files
- `gimbal_controller/siyi_ros_node.py` — Hardware interface node
- `gimbal_controller/point_to_region_node.py` — Pointing control node
- `gimbal_controller/point_to_region.py` — Core pointing computation logic
- `gimbal_controller/siyi_sdk.py` — SIYI gimbal SDK wrapper
- `launch/` — Launch files
