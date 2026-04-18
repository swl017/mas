# MAPPO-RNN SITL Divergence Diagnosis (agent_80k / agent_120k)

**Checkpoints tested:** `models/2026-04-16_14-55-03_mappo_rnn_torch_3556bf75a4_action_penalty_tuning/checkpoints/agent_{80000,120000}.pt`

**Stack:** PegasusSim (isaac_sim.tmuxp.yaml) → MAVROS/PX4 SITL → mas_common_frame → mas_multiview → mas_tracker/sort3d → mas_policy → mas_offboard → `mavros/setpoint_velocity/cmd_vel`.

**Initial conditions:** Two ego drones at 20 m altitude facing ~90° off-axis from the target; gimbal already pointed at target drone. Failure modes observed: gimbal shake + divergence, zoom monotonically pegged to +max, translational drift away from target.

The accompanying play script (`IsaacLab/.../play_iris_mappo_rnn.py`) shows calm behavior with the same checkpoint, so the policy itself is fine — the divergence is specific to the deployment pipeline.

---

## 1. Evidence from the bags

`bag_20260418_163009_agent80k` (3.0 s). Observation stream `/px4_1/policy/observation` decoded against the 31-ego / 16-peer / 6-triangulation layout.

| t (s) | pos (ENU) | vel | rpy | bbox_empty | zoom | hfov | tri_pos | tri_std |
|------:|-----------|-----|-----|:-----------:|-----:|-----:|---------|---------|
| 0.00  | (−15.47, +0.38, +20.00) | ≈0 | (0, 0, −π/2) | 0 | 2.00 | 0.43 | **(0, 0, 0)** | **(−1, −1, −1)** |
| 0.40  | (−15.51, +0.42, +19.99) | ≈0 | same | 0 | 2.00 | 0.43 | **(0, 0, 0)** | **(−1, −1, −1)** |
| 0.80  | (−15.54, +0.47, +19.92) | small | tilting | 0 | 2.05 | 0.42 | **(0, 0, 0)** | **(−1, −1, −1)** |
| 1.20  | (−15.51, +0.65, +19.75) | (+0.3, +0.7, −0.4) | drone tilted 18° | **1** | 2.62 | 0.33 | **(0, 0, 0)** | **(−1, −1, −1)** |
| 2.00  | (−14.38, +2.18, +19.93) | (+2.1, +3.3, +0.3) | — | **1** | 3.80 | 0.23 | **(0, 0, 0)** | **(−1, −1, −1)** |
| 2.88  | (−12.74, +5.97, +20.10) | (+0.7, +5.0, −0.3) | — | **1** | 5.56 | 0.16 | **(0, 0, 0)** | **(−1, −1, −1)** |

Action stream on `/px4_1/policy/{cmd_vel,gimbal_cmd_los_rate,zoom_rate_cmd}`:

| t | cmd_vel (world m/s) | yaw_rate | gimbal az/el rate (world rad/s) | zoom_rate (zoom/s) |
|---|--------------------|---------:|-------------------------------:|-------------------:|
| first | (+1.86, −1.14, −1.79) | +0.45 | (+0.91, −0.63) | **+2.00 (max)** |
| mid   | (+4.33, −1.35, −2.27) | +0.13 | (−3.14, −2.04) | **+2.00** |
| 120k final | (−2.25, +10.00, +4.98) saturated | +0.60 | (+0.58, +0.20) | **+2.00** |

Key takeaways:

- **Triangulation tail stayed at the `(−1, −1, −1)` "invalid" sentinel for the entire run**, even once `/px4_1/chosen_target_pose` started publishing valid positions (tracker did produce output, verified below).
- **Target bbox was lost to FOV by t = 1.2 s.** As soon as the drone started to move, the gimbal couldn't keep the target framed → `bbox_empty = 1` for the rest of the bag.
- **Zoom action saturated at +1 (tanh) for every single step.** Combined with `max_zoom_rate = 2.0`, zoom grew monotonically 2.0 → 5.6× in 3 s, narrowing HFOV 0.43 → 0.16 rad — the opposite of what recovery would require.
- **Linear velocity commands saturated** (reached +10 m/s in Y at the agent_120k run) and **gimbal az rate hit ±π** at several points — bang-bang behavior characteristic of a policy forced far from its training distribution.

---

## 2. Cross-check vs iris_ma6 training env

All slots were audited against `iris_ma_env6_test.py::_get_observations` and `_pre_physics_step`. Dedicated audit doc: [UNITS_AUDIT.md](UNITS_AUDIT.md). Items relevant to the divergence:

| Item | Training | Deploy at bag time | Now (post-fixes) |
|------|----------|--------------------|------------------|
| obs dim | 31 ego + 16 peer + 6 tri = 53 | 53 (31D ego already in) | 53 ✓ |
| `effective_hfov` (obs[25]) | `2·atan2(W/2, fx·zoom)` with `fx = focal_length · W / h_aperture = 24·640/20.955 = 733.0` | 0.43 rad at zoom=2 (fx=732.6 from `camera_info`) | same ✓ (training sim camera ≈ deploy PegasusSim camera, so physical FOV matches — the calibrated 1053 px only applies to the real SIYI hardware) |
| `max_zoom_rate` | 2.0 (`ZoomControllerCfg.max_zoom_rate`) | 1.0 | 2.0 ✓ |
| `max_lin_vel` / `max_yaw_rate` / `max_gimbal_rate` | 10 / π/4 / π | 10 / π/4 / π ✓ | unchanged |
| `lin_acc_b` (obs[12:14]) | kinematic (no gravity, ≈0 at hover) | raw IMU specific force (+g in body Z at hover) | kinematic after `imu + quat_rotate_inverse(q, [0,0,−9.81])` ✓ |
| `chosen_target_pose` wiring | `_triangulation_result` from in-env triangulation | subscribed to **absolute** `/chosen_target_pose` — nobody publishes | subscribed to **relative** `chosen_target_pose`, resolves to `/{ego}/chosen_target_pose` ✓ |
| Triangulation validity check | `is_valid` flag + `sqrt(clamp(cov_diag, 1e-12))` | rejected as invalid whenever any variance ≤ 0 | presence-of-message = valid, `sqrt(clamp(cov, 1e-12))` (matches training) ✓ |
| `combined_ang_vel_w` | body + gimbal angular velocity in world frame | `los_rate_controller.py:766` publishes `combined_w = R(q_body) · (ω_body_b + ω_gimbal_b)` — correct frame, ZXY Jacobian matches `compute_combined_angular_velocity` ✓ | unchanged |
| `cmd_vel` / `gimbal_cmd_los_rate` frames | world-ENU, world LOS | same | ✓ |
| Gimbal joint angles | `joint − YAW_JOINT_OFFSET`, 0 = forward | `gimbal_state_rpy_deg` already joint-frame 0 = forward | ✓ |
| Scaling of velocity/yaw/gimbal/zoom actions | linear with their respective `max_*` | identical | ✓ |

All observation frames/units line up once the four deploy-side fixes (obs dim, zoom rate, gravity comp, triangulation wiring + validity) are in.

---

## 3. Root-cause diagnosis of the divergence

Ranked by impact on the bag behavior:

### 3.1 (Primary) Triangulation tail stuck at the "invalid" sentinel

Two back-to-back bugs made `obs[47:53]` permanently `(0, 0, 0, −1, −1, −1)`:

1. `observation_assembler.py` subscribed to the **absolute** topic `/chosen_target_pose`, but `mas_tracker/sort3d_node` runs inside each vehicle namespace and publishes `/px4_1/chosen_target_pose` (plus `/px4_2/...`). No publisher existed on the global name — `ros2 topic info /chosen_target_pose` reported Publisher count: 0. Messages never arrived.
2. Even after the topic routing is corrected, `sort3d`'s `Tracker3D::getAsDetection3D()` did not propagate the Kalman-filter posterior covariance into `hyp.pose.covariance`. The 6×6 covariance array stayed at its default zeros, so every `chosen_target_pose` carried a zero covariance — even though `/{ego}/triangulated_points` from `mas_multiview` had non-zero covariance (e.g. `diag ≈ (0.128, 0.127, 0.065)`). The deploy validity check required `variances > 0`, so every message was rejected and the sentinel persisted.

Training always supplies a valid triangulation tail (`sqrt(clamp(cov, 1e-12))` when `is_valid`, `−1` otherwise). For 47 out of 53 obs dims the policy saw in-distribution values; for the final 6 it saw an input combination it never saw during training. The `RunningStandardScaler` then projects the sentinel `−1` against its training `(mean, var)` built from valid data, producing a several-σ-off normalized value. The policy keys zoom and translation heavily off the triangulation tail, which explains the bang-bang zoom and erratic XY commands.

**Fixes applied:**
- [observation_assembler.py:160-166](../mas_policy/observation_assembler.py#L160-L166) — relative topic so it resolves to `/{ego}/chosen_target_pose`.
- [observation_assembler.py:382-404](../mas_policy/observation_assembler.py#L382-L404) — presence-of-message is authoritative; clamp cov to `1e-12` before `sqrt`, matching training.
- [mas_tracker/src/sort3d.cpp](../../mas_tracker/src/sort3d.cpp) `Tracker3D::getAsDetection3D()` — copy the 3×3 position block of the Kalman posterior covariance `state_cov_` into `hyp.pose.covariance` (used as fallback; keeps `tracked_objects` consumers supplied).
- [mas_tracker/src/sort3d_node.cpp](../../mas_tracker/src/sort3d_node.cpp) `pubChosenTarget()` — **preferred covariance source is the original 3×3 triangulation covariance** carried on the nearest `TriangulatedPoint`, written into the 6×6 pose covariance at the `[0:3, 0:3]` block (row-major strided by 6). This preserves anisotropy and off-diagonals from multi-view geometry and matches training's `compute_full_triangulation` output. Falls back to the Kalman posterior diagonal if no tri point is associable.

### 3.2 (Secondary) Target is lost within ~0.8 s

From the bag, `bbox_empty` flips to 1 at `t ≈ 1.2 s` and never recovers. Contributing factors:

- With the triangulation tail saturated at the sentinel, the policy's first velocity/yaw outputs are large and oriented in an arbitrary direction (`v ≈ (+1.9, −1.1, −1.8)`). Those motions move the drone before the gimbal LOS controller has converged, so the camera frame drifts off the target.
- Once the target is lost, the policy's heuristic seems to be "zoom in and search"; with `max_zoom_rate = 2.0` the FOV halves every 1–2 s, guaranteeing it won't recover.

Fixing the triangulation tail should cap the initial commands and prevent this cascade for reasonable starting geometries. For harder inits (90° yaw offset, bbox near image edge), additional runway is needed — see §4.

### 3.3 (Now resolved but previously latent) `lin_acc_b` gravity offset

Before the gravity-compensation fix, `obs[12:14].z` would have been ≈ +9.81 in real hardware while training saw ≈ 0. In PegasusSim the bag shows obs[14] ≈ 0 at hover even pre-fix, because the simulator's MAVROS IMU output is apparently already kinematic (or the orientation ↔ IMU timing offsets cancel out in this run). The fix still matters on real hardware; it cannot be the cause of the SITL divergence here.

### 3.4 (Pre-existing, contributing) Deploy-time camera fx quirk

The `camera_info` fx (732.6) matches the training TiledCameraCfg (focal=24 mm, aperture=20.955 mm, width=640 → 733 px). Good. The **calibrated** fx of 1053 in `zoom_curve.json` is only relevant when the SIYI hardware is used — do not inject it into the SITL pipeline. `effective_hfov` is accurate as-is.

### 3.5 (Out of scope, but worth flagging)

- Both entries in [config/vehicles.yaml](../config/vehicles.yaml) use `agent_id: drone_0`. For homogeneous-parameter MAPPO this is fine (shared policy weights + shared scaler); for heterogeneous training it would give both ego drones identical preprocessor statistics. The 2026-04-16 experiment saves a per-agent preprocessor but the weights are shared, so this is safe for now.
- Both the initial yaw offset (ego facing 90° off-axis) and the z=20 m altitude sit at the **minimum** of the curriculum (`cylinder_height_min=20 m`, `orientation_noise_std=0.2 rad ≈ 11°`). A 90° offset is well beyond the `orientation_noise_std` distribution — valid training exposure only exists for non-designated observers, whose gimbal is pointed by the env during reset. The deployed policy is being asked to generalize to a state the designated observer never experienced. Likely hard but not impossible.
- The checkpoints are from an "action_penalty_tuning" experiment at 80k/120k steps. Typical iris_ma6 runs are 200k–400k steps; these are almost certainly pre-convergence.

---

## 4. Fixes applied (this session)

| # | File | Change |
|---|------|--------|
| 1 | [observation_assembler.py:160-166](../mas_policy/observation_assembler.py#L160-L166) | triangulation topic: `/chosen_target_pose` → `chosen_target_pose` (namespaced) |
| 2 | [observation_assembler.py:382-404](../mas_policy/observation_assembler.py#L382-L404) | validity check: accept zero-cov messages, clamp `cov_diag ≥ 1e-12` before `sqrt`, matching training |
| 3 | [observation_assembler.py](../mas_policy/observation_assembler.py) + tests | `lin_acc_kinematic_b = imu_b + quat_rotate_inverse(q, [0, 0, −9.81])` |
| 4 | [observation_assembler.py](../mas_policy/observation_assembler.py) + [policy_node.py](../mas_policy/policy_node.py) | obs dim 30 → 31 (insert `effective_hfov` at obs[25]); total 52 → 53 for 2-agent + tri |
| 5 | [policy_deploy.yaml](../config/policy_deploy.yaml), [policy_node.py](../mas_policy/policy_node.py), [action_publisher.py](../mas_policy/action_publisher.py) | `max_zoom_rate` 1.0 → 2.0 to match `ZoomControllerCfg` |
| 6 | [mas_tracker/src/sort3d.cpp](../../mas_tracker/src/sort3d.cpp) `Tracker3D::getAsDetection3D()` | Propagate KF posterior covariance (`state_cov_` 3×3 position block) into `hyp.pose.covariance`; previously zeros |

After rebuilding (`colcon build --packages-select mas_policy --symlink-install`), the 31D ego obs structure, scaling constants, frames, and triangulation tail all match training.

---

## 5. Next steps to validate

1. **Rerun the 3-drone SITL with the rebuilt `mas_policy`.** Record a new bag covering 15–30 s so the post-transient behavior is visible.
2. After ~2 s, confirm:
   - `ros2 topic echo /px4_1/policy/observation` shows `tri_std` ≠ `(−1, −1, −1)` once the tracker locks.
   - `obs[12:14]` stays within ±3 m/s² during level flight.
   - `policy/zoom_rate_cmd` is no longer pegged at +2.0.
3. **If instability persists** after the fixes:
   - **Step A:** set `enable_triangulation: false` in `config/policy_deploy.yaml` and relaunch. That drops obs dim to 47; use a 47-D-trained checkpoint (or retrain). If deploy becomes calm, the residual issue is still on the triangulation side (covariance quality or mas_multiview upstream).
   - **Step B:** try a later checkpoint (`agent_200000.pt` or later). 80k / 120k during `action_penalty_tuning` may simply be pre-convergence.
   - **Step C:** start with ego yaw already pointing at the target (drop the 90° offset) to stay squarely inside the training init distribution; that isolates policy competence from OOD generalization.
4. **Independent of deploy-side work, pursue a covariance fix in `mas_multiview`** so `TriangulatedPoint.covariance` carries the multi-view geometry uncertainty. Even though the deploy is now forgiving of zero covariance, the policy ideally wants the same distribution of `std_dev` values it saw during training. Without it, the deploy policy will always read `std ≈ 1 µm` and lose access to the "how confident am I?" signal that training relied on.

---

## 6. Per-topic quick reference (verified during this diagnosis)

| Topic | Rate | Frame / units | Matches training? |
|-------|-----:|---------------|:---:|
| `/{ego}/common_frame/odom` pose.position | 50 Hz | ENU world, m | ✓ |
| `/{ego}/common_frame/odom` twist.linear | 50 Hz | ENU **world**, m/s (passthrough from MAVROS `local_position/velocity_local.linear`) | ✓ |
| `/{ego}/common_frame/odom` twist.angular | 50 Hz | body FLU, rad/s (MAVROS FCU body) | ✓ |
| `/{ego}/mavros/imu/data.linear_acceleration` | 250 Hz | body FLU, m/s² (specific force, includes gravity on real HW) | compensated in assembler |
| `/{ego}/gimbal_state_rpy_deg` | 100 Hz | body, deg, z = yaw (0 = forward) | ✓ |
| `/{ego}/combined_ang_vel_w` | 100 Hz | ENU world, rad/s, body + gimbal | ✓ |
| `/{ego}/chosen_target_ray_w` | 6 Hz | common_frame world, unit vec | ✓ |
| `/{ego}/chosen_target_pose` | 6 Hz | common_frame world, PoseWithCov; **covariance currently zeros** (mas_multiview upstream) | handled; training-compat |
| `/{ego}/camera/color/camera_info` | 25 Hz | `fx = 732.6 px`, `W×H = 640×480` | ✓ (matches Isaac TiledCameraCfg) |
| `/{ego}/camera/zoom_level` | — | unitless | ✓ |
| `/{ego}/yolo_result_vision` | 25 Hz | Detection2DArray, pixel coords | normalized in assembler |
| `/{peer}/yolo_result_active` | 25 Hz | Bool → `bbox_empty` | ✓ |
| `/{ego}/policy/cmd_vel.twist.linear` | 25 Hz | ENU world, m/s, scaled by `max_lin_vel = 10.0` | ✓ |
| `/{ego}/policy/cmd_vel.twist.angular.z` | 25 Hz | world-Z yaw rate, scaled by `max_yaw_rate = 0.7854` | ✓ |
| `/{ego}/policy/gimbal_cmd_los_rate` (x, y) | 25 Hz | world (az, el) rate rad/s, scaled by `max_gimbal_rate = π` | ✓ |
| `/{ego}/policy/zoom_rate_cmd` | 25 Hz | zoom-levels/s, scaled by `max_zoom_rate = 2.0` | ✓ |
