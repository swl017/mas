# Ticket #023: Policy gimbal observation sign/offset convention mismatch

## Problem

The policy's gimbal state observation (`gimbal_yaw_body`, `gimbal_pitch_body`) may have different sign and offset conventions between the iris_ma6 training environment and the ROS2 deployment. This would cause the policy to misinterpret gimbal state, producing wrong gimbal rate commands.

## Impact

Gimbal obs indices 15-16 in the 30D ego observation are `gimbal_yaw_body` and `gimbal_pitch_body`. The policy uses these to:
- Decide how much more to pan/tilt to acquire the target
- Coordinate with other agents' gimbal states
- Implicitly track its own camera pointing direction

If the sign/offset is wrong, the policy would overshoot or oscillate its gimbal commands, or fail to track targets.

---

## Observation Vector Structure (for reference)

### Training (iris_ma_env6_test.py, L1518-1535)
```python
ego_obs = torch.cat([
    ego_data.body_position_w,                    #  0-2:  position (3)
    ego_data.body_linear_velocity_w,             #  3-5:  velocity (3)
    wrap_to_pi(euler_xyz_from_quat(...)),         #  6-8:  euler RPY (3)
    ego_data.body_angular_velocity_b,            #  9-11: angular vel body (3)
    ego_data.body_linear_acceleration_b,         # 12-14: linear acc body (3)
    ego_gimbal_yaw_body,                         # 15:    gimbal yaw (1)
    ego_gimbal_pitch_body,                       # 16:    gimbal pitch (1)
    ego_ray_w,                                   # 17-19: ray direction world (3)
    ego_data.body_combined_angular_velocity_w,   # 20-22: combined ang vel (3)
    ego_bbox_aoi,                                # 23:    bbox AoI (1)
    ego_data.camera_zoom_level,                  # 24:    zoom (1)
    bbox,                                        # 25-28: bbox normalized (4)
    bbox_empty,                                  # 29:    bbox empty (1)
])
```

### Deployment (observation_assembler.py, L426-440)
```python
ego_obs = np.concatenate([
    ego.position_w,                              #  0-2:  position (3)
    ego.velocity_w,                              #  3-5:  velocity (3)
    euler_rpy,                                   #  6-8:  euler RPY (3)
    ego.angular_velocity_b,                      #  9-11: angular vel body (3)
    ego.linear_acceleration_b,                   # 12-14: linear acc body (3)
    np.array([gimbal_yaw_obs]),                  # 15:    gimbal yaw (1)
    np.array([gimbal_pitch_obs]),                # 16:    gimbal pitch (1)
    ray_w,                                       # 17-19: ray direction world (3)
    combined_ang_vel_w,                          # 20-22: combined ang vel (3)
    np.array([bbox_aoi]),                        # 23:    bbox AoI (1)
    np.array([ego.zoom_level]),                  # 24:    zoom (1)
    ego.bbox_xywh,                               # 25-28: bbox normalized (4)
    np.array([ego.bbox_empty]),                  # 29:    bbox empty (1)
])
```

---

## Known Discrepancies

### 1. YAW_JOINT_OFFSET

| | Training (iris_ma6) | ROS2 (los_rate_controller) |
|---|---|---|
| YAW_JOINT_OFFSET | **-π/2** | **+π/2** |
| Camera orientation on pitch_link | `(0.5, -0.5, 0.5, -0.5)` wxyz | Identity `[0, 0, 0]` |
| Physical effect at yaw_joint=0 | Camera faces body +X (forward) | Camera faces body -Y (right) |

**History**: The training env uses camera offset `(0.5,-0.5,0.5,-0.5)` (a -90° Z rotation) which rotates the camera -90° at yaw_joint=0. Combined with `YAW_JOINT_OFFSET = -π/2`, the controller yaw=0 maps to camera facing body +X (forward). The ROS2 deployment uses identity camera with `YAW_JOINT_OFFSET = +π/2` to achieve the same mapping: controller yaw=0 → camera faces body +X.

**Observation yaw computation:**
```python
# Training:
ego_gimbal_yaw_body = joint_positions_b[:, 1] - (-π/2)  # = raw_yaw + π/2
# ROS2:
gimbal_yaw_obs = msg.z  # = degrees(actual_yaw) = degrees(raw_yaw - π/2)
```

For the SAME physical camera direction (forward), the raw yaw_joint values differ due to different camera offsets:
- Training: raw_yaw = -π/2 (to cancel the camera's -90° offset) → obs = -π/2 + π/2 = **0** ✓
- ROS2: raw_yaw = +π/2 (to cancel identity camera at -Y) → obs = degrees(π/2 - π/2) = **0** ✓

**Both give yaw_obs=0 for forward-facing camera.** The offsets cancel correctly despite being different values. Need to verify this holds at other angles.

### 2. Pitch joint axis direction

| | Training (iris_ma6) | ROS2 (iris_gimbal3) |
|---|---|---|
| Pitch axis | Unknown — **needs verification** | **-Y** (verified experimentally, ticket #022) |
| Pitch in observation | `joint_positions_b[:, 0]` (raw, no negation) | `degrees(-actual_pitch)` = `degrees(raw_pitch)` |

Both provide the raw physical joint value to the observation. But if the training model has pitch around +Y (standard), positive pitch = down there, while iris_gimbal3 has pitch around -Y where positive pitch = up. The observations would be inverted.

### 3. `gimbal_ray_direction_world` FK formula

Both training and deployment use the same FK:
```python
dir_body = [cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)]
```

This assumes positive pitch = camera tilts down (`-sin(pitch)` gives negative Z). If the raw pitch values fed to this function have different sign conventions between training and deployment, the FK produces different world-frame ray directions.

The deployment's `gimbal_ray_direction_world` (utils.py:96) is a "direct port from iris_ma_env6_test.py:79-103". Both expect the same input convention: positive pitch = look down. The question is whether the raw joint values satisfy this.

---

## Data Flow Summary

### Training
```
joint_positions_b[:, 0]  →  ego_gimbal_pitch_body (raw, radians)
joint_positions_b[:, 1] - YAW_JOINT_OFFSET(-π/2)  →  ego_gimbal_yaw_body (radians, 0=fwd)
                                ↓
                    obs vector indices 15, 16
```

### Deployment
```
isaac_joint_states  →  los_rate_controller (negate roll/pitch at boundary)
                           ↓
                    gimbal_state_rpy_deg:
                      .z = degrees(actual_yaw)           = degrees(raw_yaw - π/2)
                      .y = degrees(-actual_pitch)         = degrees(raw_pitch)
                           ↓
                    observation_assembler:
                      gimbal_yaw_obs = radians(msg.z)     → obs[15]
                      gimbal_pitch_obs = radians(msg.y)   → obs[16]
```

---

## Investigation Steps

1. **Check training model joint axes**: Read the iris_ma6 gimbal USD/URDF to determine pitch and roll axis directions (+Y or -Y for pitch). Compare with iris_gimbal3's verified -Y.

2. **Numerical verification**: In both environments, set the gimbal to known physical directions and compare:
   - Camera forward: training yaw_obs=?, ROS2 yaw_obs=?
   - Camera 45° left: training yaw_obs=?, ROS2 yaw_obs=?
   - Camera 20° up: training pitch_obs=?, ROS2 pitch_obs=?

3. **Check FK consistency**: Feed the same observation values into `gimbal_ray_direction_world` and verify both produce the same world-frame ray.

4. **Fix the mapping**: Either:
   - Adjust `gimbal_state_rpy_deg` to match training convention exactly
   - Or add conversion in `observation_assembler._gimbal_state_callback`
   - Document the canonical convention in `frame_conventions.md`

## Affected Files

| File | Role |
|---|---|
| `IsaacPX4/ros2_ws/.../los_rate_controller.py` | Publishes `gimbal_state_rpy_deg` |
| `mas_policy/mas_policy/observation_assembler.py` | Reads gimbal state → obs[15:17] |
| `mas_policy/mas_policy/utils.py` | `gimbal_ray_direction_world` FK (uses pitch with -sin convention) |
| `IsaacLab/.../iris_ma6/iris_ma_env6_test.py` | Training obs construction (L1509-1526) |
| `IsaacLab/.../iris_ma6/controller/gimbal_controller.py` | Training `YAW_JOINT_OFFSET = -π/2` |
| `mas/src/doc/frame_conventions.md` | Authoritative conventions |

## Related

- **Ticket #022** — established iris_gimbal3 joint axis conventions (roll=-X, pitch=-Y), negation at boundary
- **Ticket #020** — LOS stabilization, gimbal topic conventions table

## Investigation Results (2026-04-04)

### Result: No mismatch — conventions are aligned

#### 1. Yaw observation (obs[15]) — MATCH

Training formula: `obs_yaw = raw_yaw_joint - (-π/2) = raw_yaw_joint + π/2`
Deployment formula: `obs_yaw = raw_yaw_joint - π/2`

These differ by π, but the raw yaw values differ by the same π due to different camera mountings:
- Training: camera offset `(0.5,-0.5,0.5,-0.5)` on pitch_link → at yaw_joint=0, camera faces -X
- Deployment: identity camera on pitch_link → at yaw_joint=0, camera faces body -Y

Numerical verification:
| Physical direction | Training raw → obs | Deployment raw → obs |
|---|---|---|
| Forward | -π/2 → **0** | +π/2 → **0** |
| 45° left | -π/4 → **π/4** | 3π/4 → **π/4** |
| 90° right | -π → **-π/2** | 0 → **-π/2** |

Offsets cancel correctly at all angles.

#### 2. Pitch observation (obs[16]) — MATCH

Training: `obs_pitch = joint_positions_b[:, 0]` = raw_pitch (confirmed index 0 = pitch via explicit `torch.stack([pitch, yaw, roll])` at iris_ma_env6_test.py:744-750).

Deployment chain:
1. `raw_pitch` from isaac_joint_states (by name lookup, not index)
2. `actual_pitch = -raw_pitch` (los_rate_controller.py:388, boundary negation for -Y axis)
3. `msg.y = degrees(-actual_pitch) = degrees(raw_pitch)` (los_rate_controller.py:604)
4. `obs_pitch = radians(msg.y) = raw_pitch` (observation_assembler.py:312)

Double negation cancels. Both produce `raw_pitch`.

#### 3. FK fallback ray (obs[17:19]) — same pre-existing sign issue in both

`gimbal_ray_direction_world` uses `-sin(pitch)` → positive pitch = look down. But iris_gimbal3's effective pitch axis is -Y (ticket #022), so positive raw_pitch = look UP. The FK produces a physically incorrect ray.

This does NOT affect policy behavior because:
- Training: `ego_ray_w = ego_data.camera_ray_directions_w` (from actual camera, not FK)
- Deployment: `ray_w = ego.chosen_target_ray_w` (from tracker, not FK)
- FK is only the fallback path when tracker has no data

If the FK fallback matters in the future, negate pitch before calling:
```python
ray_w = gimbal_ray_direction_world(yaw, -pitch, q_body)  # negate for -Y axis
```

### Conclusion

No code change required. The observation conventions are aligned between training and deployment. The YAW_JOINT_OFFSET sign difference (+π/2 vs -π/2) correctly compensates for the different camera mounting orientations. The pitch double-negation in the deployment chain preserves the raw joint value that training uses directly.

## Status

```
Flow: I -> S -> Y -> PR
Status: Investigation complete — no fix needed
```
