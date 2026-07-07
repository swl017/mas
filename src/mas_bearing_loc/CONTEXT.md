# mas_bearing_loc

## Purpose

Single-view monocular bearing-only target localization via a Delay-Compensated
Extended Kalman Filter (DC-EKF), reproducing Liu et al. 2026
(arXiv 2606.10639v2, "Planar-Sector LOS Guidance"), §III-E.

This package is **not** part of the mainline cooperative-triangulation pipeline.
It exists as a *single-agent baseline* to:

- Quantitatively compare bearing-only EKF against `mas_multiview`
  (cooperative triangulation) on identical scenes.
- Reproduce the paper's Fig. 10–style image-plane tracking plots so the EKF's
  *demonstrated* behavior can be separated from the paper's *implied* 3-D
  position accuracy (which the paper never tabulates directly).
- Provide a clean ablation point ("what if we use only one drone's
  bearing-only filter, no cooperation?") for the RA-L / T-RO manuscript.

## Nodes overview

| Node | State | IMU? | Image feature in state? | Delay comp? | Recommended use |
|---|---|---|---|---|---|
| `dc_ekf_node` | 18D | yes | yes (IBVS dynamics) | yes | Paper reproduction; short interception scenes |
| `dc_ekf_node` (`disable_delay_compensation:=true`) | 18D | yes | yes | no | "Vanilla 18D" — isolates delay-comp as a confounder |
| `simple_ekf_node` | 6D | no (uses odom) | no | no | **Long-running production tracking**; see `doc/EKF_VARIANTS_BENCHMARK.md` |
| `direct_projection_ekf_node` | 6D (relative) | no (uses odom) | no | no | Maneuvering-observer tracking; injects observer accel as control, corrects via the direct unit-bearing tangent projection |

For the canonical 2-lap waypoint-mission benchmark
(`mas/bag/bag_20260622_164718_bearing_loc_test`, 70 s), only **SimpleEKF**
stays stable end-to-end (median 1.19 m).  Both 18D variants diverge to NaN
by ≈30 s due to the `1/Z` cross-covariance feedback in the IBVS interaction
matrix.  Full analysis in `doc/EKF_VARIANTS_BENCHMARK.md`.

**Re-tune (2026-07, offline replay on the same bag)** — deliverable
`research/bearing_localization/dc_ekf_retune/DC_EKF_RETUNE_001.md` in the
2026-RAL repo. Two outcomes:

- **SimpleEKF → 0.76 m** (from 1.19 m) via a new `sigma_norm_floor` knob. The
  node's `sigma_pix/(fx·zoom)` normalized 1σ is ~5× too tight at high zoom vs
  the true ~0.03 rad angular error, so the filter over-fits and the range
  collapses; flooring the effective σ fixes it. New param `sigma_norm_floor`
  on `SimpleEKFConfig` + `simple_ekf_node` (default 0 = legacy). Recommended
  ~0.03 for this YOLO chain.
- **DC-EKF still cannot hold range** on this regime. Re-tried in ~21 configs
  with new default-off stabilization knobs on `DCEKFConfig`/`dc_ekf_node`
  (`sigma_norm_floor`, `depth_floor`, `reject_mahalanobis`, `cov_eig_floor`,
  `cov_eig_ceil`) plus attitude/velocity overrides. These prevent the hard
  eigen-solver crash and slow the blow-up, but the relative range still
  inflates outward (|p_r| 18→50 m while true ~16 m) and the state NaNs by
  14–30 s; best finite window ~55 m. Confirms `EKF_VARIANTS_BENCHMARK.md`'s
  structural conclusion with a genuine fix attempt and a precise mechanism.

## Nodes

### dc_ekf_node
**File:** `mas_bearing_loc/dc_ekf_node.py`
**Pattern:** Event-driven on IMU/odom/detection; timer-driven publishing at 25 Hz.

#### Subscriptions (per vehicle `/{veh}/`)
- `mavros/imu/data` (`sensor_msgs/Imu`) — body-FLU ω, body-frame specific force
- `common_frame/odom` (`nav_msgs/Odometry`) — aircraft world pose (REP-147 ENU);
  used for attitude seed, target-world recovery (p_target = p_aircraft − p_r)
- `yolo_result_vision` (`vision_msgs/Detection2DArray`) — delayed image features;
  bbox center is normalized through CameraInfo + zoom into camera coords
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`) — fx, fy, cx, cy
- `gimbal_state_rpy_deg` (`geometry_msgs/Vector3`) — current gimbal RPY in deg
- `camera/zoom_level` (`std_msgs/Float64`) — multiplicative optical zoom factor

#### Publications (per vehicle `/{veh}/`)
- `bearing_loc/target_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — target
  position in `common_frame` ENU + 3×3 position covariance (upper-left of 6×6)
- `bearing_loc/target_twist` (`geometry_msgs/TwistStamped`) — target velocity
  in `common_frame` ENU; recovered as `v_aircraft − v_r`
- `bearing_loc/image_feature_meas` (`geometry_msgs/PointStamped`) — *raw*
  normalized image-feature measurement at detection time (for Fig.10-style
  reconstruction)
- `bearing_loc/image_feature_pred` (`geometry_msgs/PointStamped`) — EKF state's
  current `p̄` (filtered, delay-compensated image-plane prediction)
- `bearing_loc/diagnostics` (`std_msgs/Float64MultiArray`)
  — `[trace(P_pos), trace(P_vel), ‖p_r‖, ‖v_r‖]`

#### Parameters
- `target_class_name` (`string`, default `""`) — class filter on detections; empty = accept any. YOLO classes are *strings* (e.g. `'drone'`) per `vision_msgs/ObjectHypothesis.class_id`.
- `min_confidence` (`float`, default `0.25`)
- `init_range_guess` (`float`, default `30.0`) — m; range used to seed `p_r`
  from the first bearing
- `sigma_pix` (`float`, default `2.0`) — pixel-noise 1σ on the bbox center
- `sigma_target_acc` (`float`, default `1.5`) — m/s²/√Hz; lumped unmodeled
  target-acceleration noise driving `v_r` covariance growth
- `reject_mahalanobis` (`float`, default `16.0`) — gate threshold on innovation
- `use_odom_attitude_seed` (`bool`, default `true`) — seed q from
  `common_frame/odom`; thereafter q integrates from gyro (paper-faithful)
- `publish_rate_hz` (`float`, default `25.0`) — output topic rate
- `override_attitude_from_odom` (`bool`, default `true`) — on every
  `common_frame/odom` message, replace the EKF's integrated quaternion with
  the odom one.  Defeats the slow gyro-integration drift that the
  paper-faithful 18D state otherwise accumulates over multi-second episodes.
  Disable to recover strict paper fidelity (only for short interception runs).
- `t_cam_in_body` (`float[3]`, default `[0.0, 0.0, 0.0]`) — camera-in-body
  lever arm.  Default 0 matches the observed Pegasus sim behavior on
  IrisGimbal3 (see "Tuning notes" below).

### direct_projection_ekf_node
**File:** `mas_bearing_loc/direct_projection_ekf_node.py`
**Pattern:** Event-driven on odom/detection; timer-driven publishing (default 25 Hz).
Same inputs as `simple_ekf_node` (yolo + odom + gimbal + zoom + camera_info);
publishes under `{veh}/direct_loc/...` (same message set as the other nodes).

The 6D **relative-state** sibling of `simple_ekf_node`.  State is the
observer→target geometry `x = [q, q̇] = [p_obs − p_target, v_obs − v_target]`
(world ENU), so the observer's own acceleration enters the prediction as a known
control input — unlike `SimpleEKF`'s absolute-target state, which can only feel
the observer's motion through the measurement.  The YOLO bbox center is converted
to a 3D world unit bearing and corrected via the *direct* tangent-plane
projection `H = −(I − u·uᵀ)/r` (no in-state image feature `p̄`).  Math and
observability analysis:
`research/bearing_localization/moving_target/DIRECT_PROJECTION_EKF_EXPLAINED.md`;
reference implementation `…/benchmark_estimators.py::DirectProjectionEKF`.

Extra parameters beyond the `simple_ekf_node` set:
- `use_obs_accel` (`bool`, default `true`) — feed the observer acceleration
  (EMA-filtered finite difference of `common_frame/odom` velocity) as the
  prediction control input.  `false` falls back to a constant-relative-velocity
  model.
- `obs_accel_lpf_alpha` (`float`, default `0.3`) — EMA weight for that
  finite-difference acceleration estimate.

`sigma_pix` is reused (for parity with the sibling nodes) but converted to a
bearing 1σ per update: `sigma_bearing ≈ sigma_pix / (fx · zoom)` rad.

Run side-by-side with the other variants (publishes under `direct_loc/`):

```bash
ros2 launch mas_bearing_loc compare_ekfs.launch.py vehicle:=px4_1 \
    init_range_guess:=15.0 sigma_target_acc:=0.3 sigma_pix:=5.0
```

## Tuning notes — what was needed for `mas/bag/bag_20260622_143626_bearing_loc_test`

A multi-second outdoor circle around a near-stationary balloon target. With
naïve defaults (`sigma_target_acc=1.5`, `init_range=30`, no attitude override,
`t_cam_in_body=(0,-0.10,0.12)`) the EKF diverged to **>100 m position error**
by 14 s.  Root causes (in order of impact):

1. **Wrong `t_cam_in_body`.**  Although `iris_gimbal3.usda` places `pitch_link`
   at body `(0, -0.10, 0.12)`, the Isaac-Sim camera (mounted via
   `set_local_pose([0,0,0], …)` on `/pitch_link/camera`) appears to render
   from very close to body origin in practice.  An offline projection sweep
   over the bag (`/tmp/bearing_loc_runs/sweep_conventions.py`) showed that
   adding the (0, -0.10, 0.12) offset *increased* pixel residuals by 30 px;
   the no-offset case matched best.  **Fix:** default to `(0, 0, 0)`.
2. **Gyro-integration attitude drift.**  Over 14 s of coordinated turns the
   EKF's internally integrated quaternion drifted enough to bias the
   projected image feature by ~50 px (≈ 0.08 normalized).  **Fix:** set
   `override_attitude_from_odom=true` so the EKF's `q` is re-pinned to the
   external attitude every odom tick.
3. **`sigma_target_acc=1.5` too aggressive for a near-stationary target.**
   The unmodeled-target-acceleration noise drove `v_r` covariance growth
   unboundedly; the filter "explained" persistent innovation by inferring a
   non-zero target velocity, which then drifted the position estimate.
   **Fix:** lower to `0.3` (≈ 0.4 m/s² 1σ over the IMU period) — appropriate
   when the target is statically positioned or only mildly maneuvering.
4. **Init range too high.**  Default 30 m vs true ~10 m put the initial
   linearization in a poorly-conditioned regime.  **Fix:** pass
   `init_range_guess` matching the expected scene range.

Recommended launch line for this kind of scene:

```bash
ros2 launch mas_bearing_loc dc_ekf.launch.py \
    vehicle:=px4_1 target_class_name:=drone \
    init_range_guess:=12.0 sigma_target_acc:=0.3 sigma_pix:=5.0
```

Result on the bag: median EKF↔`chosen_target_pose` Euclidean error **≈ 1 m**
sustained for the full 14 s (vs ~30 m with naïve defaults).

## Calling Contract (per CLAUDE.md §"Stateful Component Rules")

Stateful methods on `DCEKF`:

- `predict_imu(t, ω, a, R_c_b)` — **WRITE**. Advances `(q, p_r, v_r, p̄, b_g, b_a)`
  and pushes a snapshot. Idempotent only if monotonically increasing `t`;
  reverse-time calls are no-ops.
- `update_bearing(t_img, p_bar_meas, R_c_b_at_img)` — **WRITE**. Rolls back to
  the snapshot at `t_img`, applies the measurement, replays forward.
  Idempotent within a single sim step (the same `t_img` re-applies the
  same correction, so guard the caller side).
- `target_position_world(p_aircraft)` / `target_velocity_world(v_aircraft)`
  — **READ**. Safe to call multiple times.

## Dependencies

- `vision_msgs` (Detection2DArray)
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
- numpy

## Documentation

- `doc/DC_EKF_PERFORMANCE_NOTES.md` — analysis of the paper's claimed
  performance and what is / is not actually validated
- `doc/dc_ekf_math.md` (TODO) — state, Jacobian blocks, noise model

## Key Files

- `mas_bearing_loc/dc_ekf.py` — 18D state EKF, 17D error-covariance, snapshot
  + IMU buffer delay compensation
- `mas_bearing_loc/camera_model.py` — pinhole + gimbal projection + IBVS L_s
- `mas_bearing_loc/imu_buffer.py` — ring buffers for IMU + state snapshots
- `mas_bearing_loc/quaternion.py` — wxyz quaternion helpers (project convention)
- `mas_bearing_loc/dc_ekf_node.py` — ROS2 node
- `mas_bearing_loc/simple_ekf.py` / `simple_ekf_node.py` — 6D absolute-target
  CV-EKF with 2D pinhole image-feature update
- `mas_bearing_loc/direct_projection_ekf.py` / `direct_projection_ekf_node.py`
  — 6D relative-state EKF, observer accel as control, direct unit-bearing update

## Deviations from the Paper

| Aspect | Paper | Here | Why |
|--------|-------|------|-----|
| Camera | Fixed (`t_b^c=0`, `R_b^c` constant) | Gimbal — `R_c^b` time-varying from `gimbal_state_rpy_deg` | We *have* a gimbal; this is what our hardware does |
| Image features | Custom IBVS feature(s) on a balloon | YOLO bbox center | Aligns with the rest of the MAS stack |
| Attitude correction | Implicit from external sensor or none | Optional seed from `common_frame/odom` at init only | Faithful integration vs long-horizon drift trade-off |
| Image feature in state | Full `L_s` coupling in F across all components | Dominant `∂p̄/∂v_r = dt·L_s[:,:3]·R_w^c` only | Cleanliness — the other couplings are second-order; document in dc_ekf_math.md if a re-implementation needs them |

## Verified topic conventions (2026-06-22)

| Topic / spec | What we assume / verified | Source |
|---|---|---|
| `mavros/imu/data` | `linear_acceleration` is **specific force, gravity included** (hover ≈ +9.84 m/s² on z). | Live sample |
| `gimbal_state_rpy_deg` | Body-frame RPY, **ZXY intrinsic** (`Rz(yaw)·Rx(roll)·Ry(pitch)`). Field map: `msg.x = roll, msg.y = pitch, msg.z = yaw`. | [`los_rate_controller.py:948-960`](/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py#L948) |
| `camera/zoom_level` | Magnification level in `[zoom_min, zoom_max] = [1.0, 6.0]`. `fx_eff = fx_baseline × zoom` assumed (i.e. `CameraInfo.K` is at zoom=1.0 baseline). | [`dynamics.py:141-228`](/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/dynamics.py#L141), [`los_rate_controller.py:1004-1011`](/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py#L1004) |
| **Camera-in-body offset `t_cam_in_body`** | Active variant is **IrisGimbal3** (used by [`px4_multi_world.isaac.py:207`](/home/usrg/IsaacPX4/PegasusSimulator/launch/px4_multi_world.isaac.py#L207)). Camera optical center at body **(0.0, -0.10, 0.12) m** in FLU at zero gimbal pose (= 10 cm right, 12 cm up). The small gimbal-rotation-induced motion of this point is **ignored** (≤ a few cm; sub-mrad at typical target range). Configure via the `t_cam_in_body` node parameter. For IrisGimbal2 use `(0.10, 0.0, 0.12)`. | [`iris_gimbal3.usda` pitch_link xform + yaw/roll/pitch joint anchors](/home/usrg/IsaacPX4/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris_gimbal3.usda) |
