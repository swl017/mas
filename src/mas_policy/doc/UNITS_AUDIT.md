# mas_policy Unit & Frame Audit vs iris_ma6 Training

Authoritative mapping of every slot that crosses the policy boundary. Training
reference: `iris_ma_env6_test.py::_get_observations` and `_pre_physics_step`.

Global conventions:
- World frame: ENU (X=East, Y=North, Z=Up).
- Body frame: FLU (X=Forward, Y=Left, Z=Up).
- Rotations: quaternion `(w, x, y, z)` throughout.
- `quat_rotate_inverse(q_body→world, v_world) = v_body` (world → body).
- Gravity vector in ENU world: `g = [0, 0, −9.81]` m/s².

---

## 1. Observation vector (ego, 31D)

| Idx | Field | Training source | Deploy source (topic / field) | Frame / units |
|:---:|-------|-----------------|-------------------------------|---------------|
| 0-2 | position | `body_position_w` | `common_frame/odom.pose.position` (or `mavros/local_position/odom`) | ENU world, m |
| 3-5 | linear velocity | `body_linear_velocity_w` | `common_frame/odom.twist.linear` passthrough from MAVROS `local_position/velocity_local.twist.linear` | ENU world, m/s |
| 6-8 | roll, pitch, yaw | `wrap_to_pi(euler_xyz_from_quat(body_orientation_w))` | same computation on cached quaternion | rad, `[-π, π]` |
| 9-11 | angular velocity | `body_angular_velocity_b` | `common_frame/odom.twist.angular` passthrough from MAVROS (FCU body frame) | body, rad/s |
| 12-14 | kinematic linear acceleration | `quat_rotate_inverse(body_orientation_w, body_lin_acc_w)` (net-force/mass, **no gravity**) | `mavros/imu/data.linear_acceleration` **+ g→body compensation** (see §3) | body, m/s² |
| 15 | gimbal yaw (body) | `joint_positions_b[:,1] − YAW_JOINT_OFFSET` (0=forward) | `gimbal_state_rpy_deg.z` (deg→rad); siyi_ros_node already publishes joint-frame (0=forward) with sign multipliers applied | body, rad |
| 16 | gimbal pitch (body) | `joint_positions_b[:,0]` | `gimbal_state_rpy_deg.y` (deg→rad) | body, rad |
| 17-19 | target ray | `camera_ray_directions_w[:,0,:]` from bbox raycast | `chosen_target_ray_w.vector` (selected by mas_tracker) | ENU world, unit vector |
| 20-22 | combined angular velocity | `body_combined_angular_velocity_w` (body + gimbal rate in world) | `combined_ang_vel_w.vector` published by los_rate_controller / siyi_ros_node | ENU world, rad/s |
| 23 | bbox age-of-information | `sim_time − timestamp_detection`, capped at `max_bbox_aoi` | `now − detection_timestamp`, clipped to `max_bbox_aoi` (20 s) | seconds |
| 24 | zoom level | `camera_zoom_level` | `camera/zoom_level` (Float64) | unitless (1×–10×) |
| 25 | effective HFOV | `2·atan2(W/2, fx_base · dr_scale · zoom)` | `2·atan2(W/2, fx · zoom)` with `fx = camera_info.K[0]` (dr_scale = 1 at deploy) | rad |
| 26-29 | bounding box (normalized) | `bboxes_2d / (W, H, W, H)` | `Detection2D.bbox` (cx, cy, w, h) divided by `image_width`, `image_height` | normalized `[0, 1]`, xywh |
| 30 | bbox empty flag | `1.0` if bbox is zero else `0.0` | `0.0` if ego has detection; peer flag from `yolo_result_active` Bool | 0 or 1 |

### Inter-agent block (16D per other agent)

| Offset | Field | Frame / units |
|:---:|-------|---------------|
| +0-2 | position | ENU world, m |
| +3-5 | linear velocity | ENU world, m/s |
| +6-8 | target ray | ENU world, unit vector |
| +9-11 | combined angular velocity | ENU world, rad/s |
| +12 | zoom level | unitless |
| +13 | bbox empty flag | 0 / 1 |
| +14 | motion data age | s since last `odom` |
| +15 | bbox data age | s since last detection |

### Optional triangulation tail (6D)

| Offset | Field | Frame / units |
|:---:|-------|---------------|
| +0-2 | triangulated target position | ENU world, m |
| +3-5 | per-axis std dev (`sqrt(cov diag)`, `−1` when invalid) | m |

---

## 2. Action vector (7D)

Policy outputs are tanh-squashed to `[-1, 1]` and scaled inside the deploy node.

| Idx | Action | Scaling | Published (topic · field) | Frame / units |
|:---:|--------|---------|---------------------------|---------------|
| 0 | vx | `· max_lin_vel` | `cmd_vel.twist.linear.x` | ENU world, m/s |
| 1 | vy | `· max_lin_vel` | `cmd_vel.twist.linear.y` | ENU world, m/s |
| 2 | vz | `· max_lin_vel` | `cmd_vel.twist.linear.z` | ENU world, m/s |
| 3 | yaw rate | `· max_yaw_rate` | `cmd_vel.twist.angular.z` | rad/s about world Z |
| 4 | gimbal azimuth rate | `· max_gimbal_rate` | `gimbal_cmd_los_rate.x` | rad/s, world LOS azimuth |
| 5 | gimbal elevation rate | `· max_gimbal_rate` | `gimbal_cmd_los_rate.y` | rad/s, world LOS elevation |
| 6 | zoom rate | `· max_zoom_rate` | `zoom_rate_cmd.data` | zoom-levels/s |

Training env scales identically:
`cmd_vel[:, idx, 0:3] = action[:, 0:3] * max_lin_vel` (world),
`cmd_vel[:, idx, 3] = action[:, 3] * max_yaw_rate`,
gimbal / zoom rates pass through the controller which applies `max_gimbal_rate` / `max_zoom_rate` internally.

Scaling constants must match `IrisMA6TestEnvCfg` / `GimbalControllerCfg` / `ZoomControllerCfg`:

| Param | Training | mas_policy |
|-------|---------:|-----------:|
| `max_lin_vel` | 10.0 m/s | 10.0 |
| `max_yaw_rate` | π/4 rad/s | 0.7854 |
| `max_gimbal_rate` | π rad/s | 3.14159… |
| `max_zoom_rate` | 2.0 /s | 2.0 |

---

## 3. Gravity compensation for `lin_acc_b`

**Problem.** Training fills obs[12:14] from IsaacLab’s `body_lin_acc_w` (kinematic acceleration, i.e. net force ÷ mass) rotated to body frame. At level hover, thrust cancels gravity, so the training value is ≈ `[0, 0, 0]`. MAVROS `sensor_msgs/Imu.linear_acceleration` reports the accelerometer’s **specific force** `f = a_kinematic − g_world` expressed in body FLU. At level hover, this is ≈ `[0, 0, +9.81]` m/s². A raw passthrough therefore feeds the policy a Z-axis bias of ≈ g that it never saw during training, and the RunningStandardScaler amplifies the shift.

**Fix.** Reverse the specific-force definition:

```
a_kinematic_body = f_body + quat_rotate_inverse(q_body→world, g_world)
                 = imu.linear_acceleration + quat_rotate_inverse(orientation_w, [0, 0, −9.81])
```

Sanity checks:
- Level hover, `q = I`: `[0, 0, +9.81] + [0, 0, −9.81] = [0, 0, 0]` ✓
- Free fall, `q = I`, IMU reads `[0, 0, 0]`: `[0, 0, 0] + [0, 0, −9.81] = [0, 0, −9.81]` ✓ (matches kinematic gravity accel in body)
- Level hover with +0.3 m/s² forward kinematic accel: IMU reads `[0.3, 0, +9.81]` → compensated `[0.3, 0, 0]` ✓

Implementation lives in [observation_assembler.py](../mas_policy/observation_assembler.py) inside `assemble()`:

```python
lin_acc_kinematic_b = ego.linear_acceleration_b + quat_rotate_inverse(
    ego.orientation_w, _GRAVITY_WORLD_ENU
)
```

Tests: `test_lin_acc_gravity_compensation_level_hover`,
`test_lin_acc_gravity_compensation_nonzero_kinematic`.

---

## 4. Non-obvious matches worth remembering

- `common_frame/odom` violates ROS REP-103 by keeping `twist.linear` in the parent (ENU world) frame rather than `child_frame_id` (body). That is intentional — MAVROS publishes `local_position/velocity_local` in world ENU, and mas_common_frame passes it through without rotation. Treat it as world. `twist.angular` stays body (MAVROS FCU body).
- `gimbal_state_rpy_deg` already encodes the training convention of `0 = forward` because `siyi_ros_node` subtracts the joint offset and applies `yaw_direction` / `pitch_direction` sign multipliers. Do **not** re-subtract `YAW_JOINT_OFFSET` in deployment.
- `gimbal_cmd_los_rate` is in the **world** LOS frame (azimuth, elevation), not body — because the training controller integrates `azimuth_rate = gimbal_yaw_rate_cmd · max_gimbal_rate` in world and does the 2-DOF analytic IK to body joints downstream.
- `effective_hfov` treats `camera_info.K[0]` as the 1× nominal focal length and multiplies by current zoom; this matches training (dr_scale = 1 at deployment). If a camera driver starts updating `camera_info` with a zoom-scaled fx, the multiplication must be removed.
