# gimbal_controller

## Purpose
Gimbal hardware interface (SIYI SDK) and camera pointing control for target tracking.

## Nodes

### siyi_gimbal_node
**File:** `gimbal_controller/siyi_ros_node.py`
**Pattern:** Decoupled (subscriber → cache command, timer → publish state + actuate)

#### Subscriptions
- `siyi_gimbal_angles/command_rpy_deg` (`geometry_msgs/Vector3`) — target gimbal angles (heading-frame, 0x0E)
- `gimbal_cmd_los_rate` (`geometry_msgs/Vector3`) — heading-frame LOS rate command (x=yaw, y=pitch, normalized -1..1, 0x07)
- `zoom_cmd` (`std_msgs/Float32`) — zoom rate command (+1=in, -1=out, 0=stop, 0x05)
- `mavros/imu/data` (`sensor_msgs/Imu`) — aircraft attitude for joint angle derivation + 0x22 injection (BEST_EFFORT)
- `common_frame/odom` (`nav_msgs/Odometry`) — velocity cache for 0x3E GPS injection (BEST_EFFORT)
- `mavros/global_position/global` (`sensor_msgs/NavSatFix`) — GPS for 0x3E injection (BEST_EFFORT)

#### Publishers
- `siyi_gimbal_angles/state_rpy_deg` (`geometry_msgs/Vector3`) — **primary gimbal state**: 0x0D angles at 100 Hz. Remapped to `gimbal_state_rpy_deg` in launch. Yaw=joint(encoder), pitch/roll=heading(IMU). Direction multipliers applied.
- `siyi_gimbal_angles/encoder_rpy_deg` (`geometry_msgs/Vector3`) — derived joint-frame angles at 100 Hz. Yaw from 0x0D (encoder), pitch/roll from `0x0D_heading - aircraft_attitude_ENU` rotated by yaw joint angle. Requires `mavros/imu/data` for pitch/roll derivation.
- `combined_ang_vel_w` (`geometry_msgs/Vector3Stamped`) — gimbal angular velocity in world frame (finite-difference of joint angles)
- `camera/zoom_level` (`std_msgs/Float64`) — current zoom level from SDK `getZoomLevel()`

#### Parameters
- `server_ip` (`string`, default: `"192.168.144.26"`) — SIYI gimbal IP
- `server_port` (`int`, default: `37260`) — SIYI gimbal port
- `publish_rate_hz` (`double`, default: `100.0`) — state publish rate
- `yaw_direction` (`double`, default: `1.0`) — yaw sign multiplier (left=positive)
- `pitch_direction` (`double`, default: `-1.0`) — pitch sign multiplier (down=positive after negation)
- `enable_aircraft_attitude` (`bool`, default: `true`) — enable 0x22 attitude injection (from IMU at 100 Hz) and 0x3E GPS injection

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

---

### gimbal_calibration
**File:** `gimbal_controller/gimbal_calibration.py`
**Pattern:** Bench executable (single-run session driver)

#### Purpose
- Runs ticket 026 bench calibration inside the package boundary
- Performs encoder verification, forward/reverse sweep, and optional checkerboard zero-offset estimation
- Writes session artifacts under `datasets/gimbal_calibration/<session_name>/`

#### Subscriptions
- `image_raw` (`sensor_msgs/Image`) — optional checkerboard image input
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`) — optional intrinsics for checkerboard pose estimation
- `gimbal_state_rpy_deg` (`geometry_msgs/Vector3`) — optional mirrored runtime state for bag capture / comparison

#### Outputs
- `datasets/gimbal_calibration/<session_name>/samples.csv`
- `datasets/gimbal_calibration/<session_name>/summary.json`
- `datasets/gimbal_calibration/<session_name>/bag/` via internal `ros2 bag record`
- `datasets/gimbal_calibration/<session_name>/notes.md`

#### Helper scripts
- `scripts/init_gimbal_calibration_session.py` — pre-creates the dataset layout and manifest
- `scripts/summarize_gimbal_calibration.py` — rebuilds summary JSON from `samples.csv`

## Key Files
- `gimbal_controller/siyi_ros_node.py` — Hardware interface node
- `gimbal_controller/point_to_region_node.py` — Pointing control node
- `gimbal_controller/point_to_region.py` — Core pointing computation logic
- `gimbal_controller/gimbal_calibration.py` — Bench calibration runner
- `gimbal_controller/siyi_sdk.py` — SIYI gimbal SDK wrapper
- `launch/` — Launch files
