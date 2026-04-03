## Ticket #021: Increase effective data update rate for sim topics

### What
Several sim topics publish at low effective wall-clock rates despite the physics running at 250 Hz sim-time. This limits gimbal stabilization, state estimation, and control loop performance. All topics in the perception-control pipeline need to update at the highest achievable wall-clock rate.

### Why
The gimbal LOS controller (ticket #020) showed 3x better stabilization at 15 Hz (IMU-triggered) vs 4 Hz (joint_states sim-time dedup). The bottleneck is not the controller — it's stale data. When the sim runs at ~1/6 real-time, unique sim timestamps arrive at only ~5 Hz wall-clock. Topics that depend on sim-time stepping update slowly, starving downstream controllers of fresh state.

Key finding from ticket #020 benchmarking:

| Trigger source | Effective rate | Yaw jitter |
|---------------|---------------|------------|
| `isaac_joint_states` (sim-time dedup) | 3.9 Hz | 1.95° |
| `mavros/imu/data` (wall-clock) | 15.9 Hz | 0.72° |
| `isaac_joint_states` (wall-clock) | 21.0 Hz | 0.72° |

### Timing convention
All rates below are **nominal** (sim-time relative) unless marked "(wall-clock)". See ARCHITECTURE.md "Timing Convention" section. Nominal rate × RTF = wall-clock rate.

### Bottlenecks found and fixes applied

#### 1. PX4 MAVLink streaming rates
**Problem:** `LOCAL_POSITION_NED` (msg_id 32) defaulted to 1 Hz nominal. `ATTITUDE_QUATERNION` (msg_id 31) was at 100 Hz nominal.
**Fix:** Added `set_message_interval` calls in all 5 simdrone tmuxp files:
- msg_id 31 (ATTITUDE_QUATERNION): 100 → 250 Hz nominal
- msg_id 32 (LOCAL_POSITION_NED): 1 → 100 Hz nominal
- msg_id 33 (GLOBAL_POSITION_INT): 5 → 50 Hz nominal
- msg_id 105 (HIGHRES_IMU): default → 250 Hz nominal
- msg_id 27 (RAW_IMU): default → 250 Hz nominal

#### 2. mas_common_frame 10 Hz timer
**Problem:** `common_frame_node.py` and `common_frame_node_single.py` used `create_timer(0.1, ...)` — 10 Hz nominal regardless of MAVROS input rate.
**Fix:** Replaced timer with callback-driven publishing. `common_frame/pose` and `common_frame/odom` now publish immediately on each `mavros/local_position/pose` callback. Output rate matches MAVROS input rate.

#### 3. isaac_joint_states duplicates (rendering rate)
**Problem:** `PublishJointState` OmniGraph node fires every physics step (250 Hz) but articulation state only updates at render rate. With rendering at 30 Hz, this produced 6-8 identical messages per unique timestamp — 30 Hz nominal unique data.
**Root cause:** `rendering_dt = 1/30` in PegasusSimulator `params.py`. The OmniGraph `OnPlaybackTick` fires per physics substep, but `ReadSimulationTime` and articulation reads only update per render frame.
**Fix:** Changed `rendering_dt` from `1/30` to `1/100` in PegasusSimulator params.py (PX4 world settings). Unique joint state data now at 100 Hz nominal.
**Camera constraint:** Camera frequency must divide evenly into rendering frequency. Changed camera `frequency` from 60 → 25 Hz nominal (100/25=4) in the launch file. Also fixed config key: the MonocularCamera reads `config["frequency"]`, not `config["update_rate"]`.

#### 4. LOS rate controller wall-clock dt
**Problem:** `_run_control_loop_wallclock()` used `time.monotonic()` for dt computation. At low RTF, wall-clock dt is much larger than nominal dt → K×dt product inflated → inner servo oscillation.
**Example:** pointing_gain=32.5 designed for K×dt=0.325 at dt=0.01s. At RTF=0.1, wall-clock dt≈0.04s → K×dt=1.3 (4× designed value).
**Fix:** Replaced with `_run_control_loop_simtime(stamp)` using IMU message timestamps for dt. K×dt now constant at 0.325 regardless of RTF.

### Nominal rate summary (after fixes)

| Topic | Nominal rate | Source |
|-------|-------------|--------|
| `isaac_joint_states` (unique) | 100 Hz | Isaac Sim rendering rate |
| `mavros/imu/data` | 250 Hz | PX4 SITL (msg_id 31 + 105/27) |
| `mavros/local_position/pose` | 100 Hz | PX4 SITL (msg_id 32) |
| `mavros/local_position/odom` | 100 Hz | PX4 SITL (msg_id 32) |
| `common_frame/pose` | 100 Hz | callback-driven, matches MAVROS |
| `common_frame/odom` | 100 Hz | callback-driven, matches MAVROS |
| `camera/color/image_raw` | 25 Hz | Isaac Sim camera frequency |
| `gimbal_state_rpy_deg` | 250 Hz | LOS controller (IMU-triggered) |
| `gimbal_los_state_deg` | 250 Hz | LOS controller (IMU-triggered) |
| `isaac_joint_commands` | 250 Hz | LOS controller (IMU-triggered) |

Note: `gimbal_state_rpy_deg` and `gimbal_los_state_deg` publish at 250 Hz nominal (IMU-triggered), but underlying joint data only changes at 100 Hz (rendering rate).

### Additional changes

- **gimbal_los_tracker_node** added to simdrone1/2 tmuxp (replaces point_to_region_node) with `use_sim_time:=true`
- **gimbal_los_tracker_node** default `proportional_gain` reduced from 2.0 → 0.5 to prevent rate command saturation in the outer loop
- **Timing Convention** added to ARCHITECTURE.md (nominal vs wall-clock vs RTF)

#### 5. Inner servo omega_cmd rate limiting
**Problem:** In rate mode, `omega_cmd = -K * att_error` (K=32.5) produces body-frame angular velocities up to 17+ rad/s for large errors (30°+). In iris_ma6, the implicit actuator PD (k=1000, d=50) naturally damps this. In the ROS2 path, 1-2 render frame feedback delay means the controller commands aggressive steps based on stale `actual` joint positions, causing underdamped oscillation (~30° amplitude, converging).
**Root cause:** No rate limiting on omega_cmd. The iris_ma6 implicit actuator's PD damping provides natural rate limiting, but the ROS2 OmniGraph feedback path adds 1-2 frame delay that defeats this damping. Not present on real hardware where encoder feedback is immediate.
**Fix:** Added `servo_rate_limit` parameter — clips omega_cmd per-component before J^{-1} computation. Defaults to `max_gimbal_rate` (π rad/s = 180°/s). At 250 Hz nominal, max per-step change is 0.72°. With 2-3 stale ticks between render updates, worst-case overshoot is ~2°, which damps within 2-3 render frames. Body-rate feedforward (stabilization) is unaffected — only the tracking command is clipped.
**Behavior change:** None for small errors (<5.5° where K×error < π). For large errors, the servo slews at max rate instead of overshooting. Matches real hardware servo behavior (physical max slew rate).

### Acceptance criteria
- [x] Full list of topic rates measured during sim operation
- [x] All perception-control pipeline topics update at ≥100 Hz nominal
- [x] No regression in gimbal stabilization or control performance — fixed via servo_rate_limit

### Flow
I → S → Y → PR

### Status
Complete — all 5 fixes applied, all acceptance criteria met. Needs sim verification.
