# Frame and Coordinate Conventions (ROS2 Deployment)

**Applies to**: MAS ROS2 deployment (PegasusSimulator + MAVROS + los_rate_controller)
**Reference**: iris_ma6 `frame_conventions.md` for training environment conventions

This document defines coordinate frames, transformations, and sign conventions for the ROS2 deployment stack. Where conventions match iris_ma6 training, this is noted. Where they differ, the difference is explained.

---

## 1) World Frame

**Convention: ENU (East-North-Up)** — same as iris_ma6

| Axis | Direction |
|------|-----------|
| **X** | East |
| **Y** | North |
| **Z** | Up |

Source: MAVROS converts PX4 NED to ENU. `common_frame` publishes odometry in ENU.

---

## 2) Body Frame

**Convention: FLU (Forward-Left-Up)** — same as iris_ma6

| Axis | Direction | Rotation |
|------|-----------|----------|
| **X** | Forward | Roll axis |
| **Y** | Left | Pitch axis |
| **Z** | Up | Yaw axis |

### 2.1 IRIS USD Mesh Offset

The body **mesh** is rotated 90° CCW around Z relative to the physics frame:

```
Physics frame:          Visual (mesh):

     +X (fwd)                +Y (visual fwd)
      ^                       ^
      |                       |
+Y <--●                 +X <--●
```

The gimbal controller and all ROS2 nodes work in the **physics frame** (+X forward). The mesh offset is visual only.

### 2.2 Body Quaternion

MAVROS `imu/data.orientation` provides body-to-world quaternion in **xyzw** format (ROS convention). The los_rate_controller converts to **wxyz** internally:

```python
q_wxyz = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
```

---

## 3) Gimbal Frame and Joints

### 3.1 Kinematic Chain

```
body → yaw_link (yaw_joint, Z axis)
    → roll_link (roll_joint, X axis)
        → pitch_link (pitch_joint, Y axis)
            → camera
```

**Rotation order**: Yaw(Z) → Roll(X) → Pitch(Y) — ZXY intrinsic

### 3.2 Joint Axes (from iris_gimbal3.usda)

**Verified experimentally** (joint_sign_test, 2026-04-04): each joint commanded to ±20° with controller stopped.

| Joint | USD Name | USD Rotation Axis | Positive = | Jacobian convention |
|-------|----------|-------------------|------------|---------------------|
| Yaw | `yaw_joint` | **+Z** (body) | Pan LEFT (CCW from above) | Same → no negation |
| Roll | `roll_joint` | **-X** (body) | Tilt CCW (viewed from front) | +X → CW → **negate** |
| Pitch | `pitch_joint` | **-Y** (body) | Tilt UP | +Y → down → **negate** |

`los_rate_controller` negates roll and pitch at the read/write boundary to convert between USD and Jacobian conventions.

**LOS Convention**
The gimbal orientation is more intuitive and more coherent to represent in world-frame azimuth and elevation (positive elevation = up, opposite of pitch sign — this is the most confusing part of the convention). Nodes communicate in world-frame LOS angles or rates.

### 3.3 Joint Limits

| Joint | Range (deg) | Range (rad) |
|-------|-------------|-------------|
| Yaw | [-160, +160] | [-2.79, +2.79] |
| Roll | [-45, +45] | [-0.785, +0.785] |
| Pitch | [-45, +45] | [-0.785, +0.785] |

### 3.4 YAW_JOINT_OFFSET

```
YAW_JOINT_OFFSET = +π/2   (+90°)
```

With identity camera orientation on `pitch_link`, `yaw_joint=0` points the camera along body **-Y** (right side). The offset maps controller yaw=0 to body +X (forward):

```
Controller yaw=0  →  yaw_joint = +π/2  →  camera faces body +X (forward)
Controller yaw>0  →  camera pans left (CCW)
```

**Where applied**:
- `los_rate_controller.py`: add offset when writing joint commands, subtract when reading joint state
- `gimbal_stabilizer.py`: subtract offset when reading joint state
- `observation_assembler.py`: subtract offset from gimbal_state_rpy_deg for policy observation

**History**: Previously `-π/2` when the camera had a `-90°` yaw offset rotation. Changed to `+π/2` when camera orientation was set to identity to fix the pitch-turns-roll bug.

### 3.5 Camera Orientation on pitch_link

```
MonocularCamera orientation: [0, 0, 0]  (identity, ZYX Euler degrees)
```

The camera is mounted on `pitch_link` with **identity orientation**. This means the camera's optical axis aligns with pitch_link's native frame direction.

**History**: Previously `[0, 0, -90]` (pure Z rotation), which caused pitch_joint rotations to appear as roll in the camera image. Changed to identity after tracing the iris_ma6 `VISUALIZATION_DEBUG_LOG.md` (12 iterations to find the correct offset).

---

## 4) Gimbal State Topic (`gimbal_state_rpy_deg`)

Published by `los_rate_controller` at 100 Hz (BEST_EFFORT QoS).

| Field | Content | Units |
|-------|---------|-------|
| `x` | Roll (actual joint) | degrees |
| `y` | Pitch (actual joint) | degrees |
| `z` | Yaw (actual joint − YAW_JOINT_OFFSET) | degrees |

The yaw field has YAW_JOINT_OFFSET already subtracted: **0° = camera forward**.

Consumers: `observation_assembler`, `triangulation_node`, `point_to_region_node`.

---

## 5) Gimbal Command Interfaces

### 5.1 Rate Mode (`gimbal_cmd_los_rate`)

World-frame normalized rate commands (Vector3, BEST_EFFORT QoS).

| Field | Content | Range |
|-------|---------|-------|
| `x` | Azimuth rate | [-1, +1] |
| `y` | Elevation rate | [-1, +1] |
| `z` | Unused | 0 |

Scaled by `max_gimbal_rate` (default 2π rad/s = 360°/s).

**Flow**: FK actual joints → world az/el → apply rate delta → IK to body joints → publish.

Source: `policy_node` (via mission gate in MISSION state).

### 5.2 Position Mode (`gimbal_cmd_rpy_deg`)

Body-frame absolute angle commands (Vector3, RELIABLE QoS).

| Field | Content | Units |
|-------|---------|-------|
| `x` | Roll (ignored, auto-stabilized) | degrees |
| `y` | Pitch (positive = look down) | degrees |
| `z` | Yaw (0 = forward) | degrees |

**Pitch sign inversion**: The command convention is positive=down (matching `point_to_region` output), but the physical `pitch_joint` positive=up. The `los_rate_controller` negates pitch internally.

Position commands override rate commands. Any rate command clears position mode.

Source: `point_to_region_node` (via mission gate in TRACKING state).

---

## 6) Mission State Gate

`mas_mission` node controls which command source reaches the gimbal:

| State | Value | Gimbal source | Velocity source |
|-------|-------|---------------|-----------------|
| IDLE | 0 | None (blocked) | None (blocked) |
| TRACKING | 1 | `point_to_region` → `gimbal_cmd_rpy_deg` | None |
| MISSION | 2 | `policy_node` → `gimbal_cmd_los_rate` | `policy_node` → `cmd_vel` |

**Topic**: `/mission_state_cmd` (global, `std_msgs/msg/Int8`)
**State feedback**: `/{ns}/mission_state` (Int8, BEST_EFFORT)

---

## 7) IK / FK Math

### 7.1 World → Body (IK)

Used in rate mode to convert world-frame az/el to body-frame joint targets:

```python
dir_world = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]
dir_body = quat_rotate_inverse(q_body, dir_world)
yaw_body = atan2(dir_body[1], dir_body[0])
pitch_body = -atan2(dir_body[2], sqrt(dir_body[0]² + dir_body[1]²))
```

Note the **negative sign** on pitch: positive world elevation (above horizon) produces negative `pitch_body`, because positive `pitch_joint` tilts down and positive elevation is up (opposite signs).

### 7.2 Body → World (FK)

Used to derive world-frame LOS from actual joints:

```python
dir_body = [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch)]
dir_world = quat_rotate(q_body, dir_body)
az = atan2(dir_world[1], dir_world[0])
el = atan2(dir_world[2], sqrt(dir_world[0]² + dir_world[1]²))
```

### 7.3 Stabilizing Roll

Keeps horizon level by projecting world-up into the yawed gimbal frame:

```python
up_in_body = quat_rotate_inverse(q_body, [0, 0, 1])
up_y_yawed = -up_in_body[0]*sin(yaw) + up_in_body[1]*cos(yaw)
up_z_yawed = up_in_body[2]
roll = atan2(-up_y_yawed, up_z_yawed)
```

---

## 8) Differences from iris_ma6 Training Environment

| Aspect | iris_ma6 (training) | ROS2 deployment |
|--------|-------------------|-----------------|
| Camera offset | `(0.5, -0.5, 0.5, -0.5)` wxyz, ROS convention | Identity `[0, 0, 0]` |
| YAW_JOINT_OFFSET | `-π/2` | `+π/2` |
| Gimbal control | Direct joint write at sim rate | Joint position commands via OmniGraph ArticulationController |
| Pitch convention | Positive = look down | Same: positive `pitch_joint` = tilt down |
| IMU source | Isaac Sim articulation data | MAVROS `imu/data` (ENU, xyzw) |
| Control rate | 100 Hz (sim-time) | 100 Hz (sim-time via `use_sim_time: true`) |
| State source | `robot.data.joint_pos` (direct) | `isaac_joint_states` (OmniGraph ROS2 bridge) |

---

## 9) Topic Quick Reference

| Topic (relative to ns) | Type | QoS | Publisher | Description |
|------------------------|------|-----|-----------|-------------|
| `gimbal_state_rpy_deg` | Vector3 | BE | los_rate_controller | Actual joint angles (deg), yaw offset-corrected |
| `gimbal_los_state` | Vector3 | BE | los_rate_controller | World-frame az/el (rad) from FK |
| `gimbal_cmd_los_rate` | Vector3 | BE | policy_node (via mission) | Normalized rate commands [-1,1] |
| `gimbal_cmd_rpy_deg` | Vector3 | REL | mission_node | Body-frame angle commands (deg) |
| `isaac_joint_commands` | JointState | REL | los_rate_controller | Physical joint targets (rad) |
| `isaac_joint_states` | JointState | BE | OmniGraph | Physical joint feedback (rad) |
| `combined_ang_vel_w` | Vector3Stamped | BE | los_rate_controller | Body + gimbal angular velocity (world) |
| `zoom_level` | Float32 | BE | los_rate_controller | Current zoom level |
