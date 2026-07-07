# mavros_replicator

PX4 (`px4_msgs`, NED/FRD) ↔ MAVROS-shaped ROS topics (ENU/FLU) translator. Single node per vehicle.

**Spec**: [src/doc/mavros_replicator_spec.md](../doc/mavros_replicator_spec.md) — authoritative.

## Why

Replaces MAVROS on each vehicle so downstream MAS nodes (`mas_common_frame`, `mas_policy`, `mas_operator`) keep their MAVROS-shaped interface and frame conventions while the underlying transport switches to uXRCE-DDS. `px4_msgs` is a build dependency of this package only.

## Node: `mavros_replicator`

### Subscribes (PX4 side, BEST_EFFORT KEEP_LAST 5)

| Topic | Type |
|---|---|
| `/{robot_name}/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` |
| `/{robot_name}/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` |
| `/{robot_name}/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` |
| `/{robot_name}/fmu/out/vehicle_control_mode` | `px4_msgs/VehicleControlMode` |
| `/{robot_name}/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` (source for `mavros/home_position/home`; PX4 does not export `home_position` over uXRCE-DDS) |
| `/{robot_name}/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` |
| `/{robot_name}/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` (ENU position + FLU→ENU quaternion; converted to NED + yaw scalar) |

### Publishes (MAVROS side, RELIABLE KEEP_LAST 10 unless noted)

| Topic | Type | Notes |
|---|---|---|
| `/{robot_name}/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | |
| `/{robot_name}/mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | |
| `/{robot_name}/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | linear ENU world, angular FLU body |
| `/{robot_name}/mavros/local_position/odom` | `nav_msgs/Odometry` | pose ENU world, twist FLU body |
| `/{robot_name}/mavros/imu/data` | `sensor_msgs/Imu` | |
| `/{robot_name}/mavros/state` | `mavros_msgs/State` | 5 Hz (timer-driven, freshness-checked) |
| `/{robot_name}/mavros/home_position/home` | `mavros_msgs/HomePosition` | TRANSIENT_LOCAL depth=1, derived from `vehicle_local_position.ref_*`; republished only when EKF origin changes |
| `/{robot_name}/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | per cmd_vel arrival |
| `/{robot_name}/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | per cmd_vel arrival |

### Parameters

| Param | Default | Description |
|---|---|---|
| `robot_name` | env `ROBOT_NAME` or `"px4_1"` | Topic-tree prefix (drives both `/fmu/...` and `/mavros/...`). |
| `frame_id_world` | `{robot_name}/map` | `header.frame_id` for world-frame topics. |
| `frame_id_body` | `{robot_name}/base_link` | `child_frame_id` for body-frame fields. |
| `setpoint_timeout_ms` | `250` | Reserved (cmd_vel passthrough is per-message; PX4's own ~500 ms offboard timeout governs idle). |

### Dependencies

`rclpy`, `numpy`, `geometry_msgs`, `geographic_msgs`, `nav_msgs`, `sensor_msgs`, `mavros_msgs`, `px4_msgs`.

### px4_msgs branch must match firmware

`src/px4_msgs/` is vendored; check out the branch that matches the running PX4:

- **Pegasus simulation** (PX4 v1.14.x) → `git -C src/px4_msgs checkout release/1.14`
- **Real hardware** (PX4 v1.15.x) → `git -C src/px4_msgs checkout release/1.15`

A mismatch shows up as a stream of `Fast CDR exception deserializing message of type px4_msgs::msg::dds_::VehicleStatus_./VehicleLocalPosition_.` warnings on the replicator, with `ros2 topic echo /{robot}/fmu/out/...` returning nothing. Several layouts changed between 1.14 and 1.15 (incl. `VehicleStatus`, `VehicleLocalPosition`, and `OffboardControlMode`). The replicator code is written to be neutral across branches (e.g. it never sets the 1.14-only `OffboardControlMode.actuator` or the 1.15-only `thrust_and_torque`/`direct_actuator`).

## Files

- `mavros_replicator/frames.py` — pure-Python frame and quaternion math (NED↔ENU, FRD↔FLU). No ROS deps; unit-tested.
- `mavros_replicator/replicator_node.py` — the rclpy node.
- `test/test_frames.py` — math sanity tests.
- `launch/replicator.launch.py` — single-vehicle launch.
