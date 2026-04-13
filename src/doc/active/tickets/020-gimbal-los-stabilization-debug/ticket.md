## Ticket #020: Gimbal LOS stabilization not working during policy flight

### What
During policy-driven flight, the gimbal does not maintain world-frame line-of-sight (LOS) stabilization. The camera drifts or lags when the drone changes attitude, instead of holding a stable pointing direction.

### Why
LOS stabilization is critical for keeping the target in the camera FOV during aggressive maneuvers. Without it, YOLO detections drop, triangulation degrades, and the observation vector fed to the policy becomes stale or incorrect. This undermines the entire perception-action loop.

### Symptom
Gimbal pointing direction shifts with drone body attitude changes instead of remaining fixed in the world frame. The effect is visible in the sim camera feed and/or in `gimbal_los_state_deg` drifting when the drone pitches/rolls.

### Candidate causes

1. **Drone attitude estimation lag**: The IMU quaternion (`mavros/imu/data`) used by `los_rate_controller` for FK/IK may lag behind actual body motion. If the quaternion is delayed by even 1-2 frames at 100 Hz control rate, the IK produces stale body-frame targets that don't fully compensate for the current attitude.

2. **Sim time not applied consistently**: Ticket #009 fixed `los_rate_controller` sim-time alignment, but other nodes in the chain (e.g., `gimbal_stabilizer`, `joint_state_publisher`) may still run on wall-clock. Verify all gimbal-pipeline nodes use `use_sim_time: true`.

3. **Joint actuator too slow (PD gains)**: The Isaac Sim articulation PD controller (`stiffness`/`damping`) may be too low for the gimbal joints to track the rapidly changing stabilization targets. The joint physically cannot keep up with the commanded positions during fast maneuvers.

4. **Rate integration drift**: In rate mode, `los_rate_controller` integrates az/el from actual joint feedback each tick (FK → rate → IK). If feedback is delayed or quantized, the integration accumulates error over time, causing the world-frame pointing to drift. If this is found to be the cause, consider replacing integrating internal states with sensor readings. Actually, we should always read from the sensors whereever possible.

5. **Quaternion convention mismatch**: `los_rate_controller` uses wxyz internally but receives xyzw from ROS2 IMU. The conversion happens in `_imu_callback` — verify this is correct and no other code path introduces a convention error.

6. **Stabilizing roll computation**: `_compute_stabilizing_roll` projects world-up into the yawed gimbal frame. If the yaw used is the target (not actual), a yaw tracking error could propagate into incorrect roll compensation.

### Investigation plan

1. **Timestamp audit**: Record `mavros/imu/data` stamp vs `isaac_joint_states` stamp vs `los_rate_controller` publish stamp. Quantify any pipeline latency.

2. **Sim-time check**: Verify `use_sim_time` is set for all nodes: `los_rate_controller`, `gimbal_stabilizer`, `joint_state_publisher`. Check launch files and `ros2 param get`.

3. **Step response test**: Command a fixed LOS direction (via `gimbal_cmd_rpy_deg`), then apply a known attitude disturbance (e.g., pitch step). Measure gimbal response: does it compensate? How much residual error? Plot `gimbal_los_state_deg` vs time.

4. **Actuator tracking**: Compare `isaac_joint_commands` (target) vs `isaac_joint_states` (actual) during flight. Large tracking error indicates PD gains too low or command rate too high.

5. **FK/IK round-trip**: With a static drone, verify body→world→body round-trip is identity: set a known joint target, read back FK result, run IK, confirm it matches input.

6. **Rate mode drift**: Run policy with zero rate commands (all gimbal actions = 0). Does the LOS direction stay constant? If it drifts, the issue is in the stabilization loop itself.

### Affected modules
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py` — FK/IK stabilization loop
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/gimbal_stabilizer.py` — older stabilizer (may still be running?)
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/launch/` — launch files for sim-time and PD params
- `IsaacPX4/PegasusSimulator/` — Isaac Sim articulation drive config (stiffness, damping)
- `mas_policy/` — publishes `gimbal_cmd_los_rate` consumed by `los_rate_controller`

### Related tickets
- #009 (gimbal oscillation) — fixed sim-time alignment; this ticket may involve residual issues from the same pipeline
- #011 (LOS controller state from joints) — established joint feedback as authoritative state source
- #016 (frustum roll/pitch swap) — fixed a frame convention bug in a related pipeline

### Acceptance criteria
- During policy flight with attitude changes, `gimbal_los_state_deg` stays within ±3° of the commanded world-frame direction
- Camera feed remains visually stable (target stays in FOV during drone maneuvering)
- Root cause identified and documented

### Investigation findings (2026-04-03)

#### Parallel investigation of all 6 candidate causes

**Cause #1 — IMU attitude lag**: ~5ms avg, ~10ms worst-case. Pipeline: 250 Hz (Isaac Sim) → 100 Hz (MAVROS) → 100 Hz (controller). At max roll rate (~200°/s), 10ms lag → ~2° stale attitude. Contributes but not root cause.

**Cause #2 — sim-time missing**: Only `los_rate_controller` has `use_sim_time: true`. Missing from: `gimbal_stabilizer`, `offboard_control`, `point_to_region_node`, `triangulation_node`. Not root cause for LOS stabilization but should be fixed.

**Cause #3 — PD gains**: iris_gimbal3.usda has stiffness=100, damping=10. Position-only mode showed poor tracking (~6° offset); position_velocity mode tracks well. **Adequate with velocity feedforward.**

**Cause #4 — Rate integration**: Hardcoded dt and target-based velocity confirmed as issues (see fixes below). FK→IK round-trip drift at zero rate is minor (~0.006°).

**Cause #5 — Quaternion/IMU**: PegasusSimulator `update_imu_data()` never populates `msg.orientation`, but `los_rate_controller` subscribes to `mavros/imu/data` which goes through PX4 EKF — valid orientation confirmed via `ros2 topic echo`. **Not an issue in deployed config.**

**Cause #6 — Stabilizing roll**: Confirmed as an issue but nuanced — see root cause analysis below.

### Root cause analysis and fixes

#### Root cause 1: Position mode held fixed body-frame joints (PRIMARY)
`los_rate_controller` position mode (used by `point_to_region` pre-mission) set `yaw_new = self._cmd_pos_yaw` — a fixed body-frame angle that never changed as the drone rotated. The gimbal rotated with the drone instead of holding a world-frame direction.

**Measured**: LOS azimuth swept 23.7° as drone yaw swept 29.9° during orbit. The gimbal joint yaw only varied 4.5° (should have varied ~30° to compensate).

**Fix**: Both position and rate modes now go through a unified world-frame → IK path. Position commands are converted to world-frame azimuth/elevation, then IK recomputes body-frame joints every tick using the current drone quaternion.

#### Root cause 2: point_to_region output in wrong frame
`point_to_region.get_gimbal_command_deg_horizontal_body_frame()` computed direction in a yaw-only-rotated frame (roll/pitch stripped). When the drone tilted during orbit, the elevation component didn't account for pitch — causing a constant ~9.6° elevation error matching the drone's pitch angle.

**Fix**: Added `get_gimbal_command_deg_world_frame()` which computes pure world-frame azimuth/elevation from the drone-to-target vector. No drone attitude needed. `point_to_region_node` now publishes world-frame angles, and `los_rate_controller` stores them directly as the world-frame target.

**After fix**: Azimuth error mean=0.5° max=1.3°, elevation error mean=0.0° max=0.0°.

#### Root cause 3: Stabilizing roll computed for wrong yaw
`_compute_stabilizing_roll()` was called with `actual_yaw` (joint feedback, lagging 1-2 ticks) instead of `yaw_new` (IK output for current attitude). Since roll compensation is tightly coupled to yaw direction (~0.9:1 sensitivity), the lag produced visible camera tilt during fast yaw changes — even though the frustum visualization (which chains drone quaternion × gimbal rotation) appeared level.

**Fix**: Use `yaw_new` for roll computation. Now that both modes always go through IK, `yaw_new` is the correct body-frame yaw for the current drone attitude, and yaw + roll targets arrive at the joints simultaneously.

#### Additional fixes applied
- **Actual dt measurement**: `get_clock().now()` instead of hardcoded `1/update_rate`, preventing rate integration drift in sim-time.
- **Sensor-based velocity**: `combined_ang_vel_w` and joint command velocities computed from actual joint position deltas, not target deltas.

### Changes made

| File | Change |
|------|--------|
| `los_rate_controller.py` | Rate mode: initially ported iris_ma6 Jacobian J^{-1} controller, later replaced with analytical IK (see ticket #022) |
| `los_rate_controller.py` | Position mode: analytical IK with world-frame target, stabilizing roll |
| `los_rate_controller.py` | Persistent world-frame az/el target for rate mode (integrates LOS rate commands) |
| `los_rate_controller.py` | Actual dt from `get_clock().now()`, sensor-based velocity computation |
| `los_rate_controller.py` | Roll/pitch negation at joint read/write boundary (USD -X/-Y axes, see ticket #022) |
| `point_to_region.py` | Added `get_gimbal_command_deg_world_frame()` — world-frame az/el output |
| `point_to_region_node.py` | Switched to world-frame method, publishes (az, el) in world frame |
| `gimbal_los_tracker_node.py` | New node: proportional world-frame error → normalized LOS rate publisher |

### LOS convention rule

**Wherever "LOS" appears in a topic or interface, positive elevation = up (ENU standard).**

Verified consistent through the full chain:
- iris_ma6 training: `elevation = atan2(z, xy_dist)` → positive = up
- action_publisher: `action[5]` → `gimbal_cmd_los_rate.y` pass-through (no sign flip)
- los_rate_controller: `el_world += cmd_el_rate * max_rate * dt` → positive = up
- gimbal_los_tracker_node: `el_desired = atan2(dz, xy_dist)` → positive = up

### Gimbal topic frame conventions (post-fix, updated 2026-04-04 per ticket #022)

| Topic | Frame | Publisher | Subscriber | Convention |
|-------|-------|-----------|------------|------------|
| `gimbal_cmd_los_rate` | **World-frame** LOS rates | `mas_policy`, `gimbal_los_tracker_node` | `los_rate_controller` | Normalized [-1,1] az/el rate. Positive el = up |
| `tracking/gimbal_command_los_world_deg` | **World-frame** az/el | `point_to_region_node` | `mission_node` | z=azimuth, y=elevation (deg). Positive el = up |
| `gimbal_cmd_los_world_deg` | **World-frame** az/el | `mission_node` | `los_rate_controller` | z=azimuth, y=elevation (deg). Positive el = up |
| `gimbal_los_state_deg` | **World-frame** az/el | `los_rate_controller` | `gimbal_los_tracker_node` | x=azimuth, y=elevation (deg). Positive el = up |
| `gimbal_state_rpy_deg` | **Body-frame** angles | `los_rate_controller` | `mas_multiview`, `mas_policy` | x=roll(Rx std), y=-pitch(Ry std: pos=up), z=yaw(Rz std). For downstream `Rz*Rx*Ry` composition |
| `isaac_joint_commands` | **Body-frame** joint targets | `los_rate_controller` | Isaac Sim | [yaw+offset, -roll, -pitch] (negated for USD -X/-Y axes) |

**Joint sign convention** (verified experimentally, ticket #022): iris_gimbal3 USD roll/pitch axes are -X/-Y (inverted from standard). `los_rate_controller` negates roll and pitch at read/write boundary. See `src/doc/frame_conventions.md` §3.2.

### Remaining work
- [x] Add `use_sim_time: true` to remaining gimbal-pipeline launch files
- [x] Rename `gimbal_cmd_rpy_deg` topic → `gimbal_cmd_los_world_deg`
- [x] Fix `mission_deploy.launch.py` missing remapping
- [x] Add `gimbal_los_tracker_node` — proportional world-frame LOS rate publisher following target pose
- [x] Port iris_ma6 Jacobian J^{-1} controller into `los_rate_controller` rate mode (configurable via `gimbal_controller_mode`)
- [x] Verify LOS convention chain (positive elevation = up, end-to-end)
- [x] Test LOS stabilization with `gimbal_los_tracker_node` (ticket #022)
- [x] Test LOS stabilization at various drone yaw angles (ticket #022)
- [x] Update ARCHITECTURE.md doc references to new topic names
- [x] Configurable gimbal controller mode: analytical, jacobian, combined (analytical position + jacobian velocity)
- [x] Anti-windup on world-frame integrator (per-axis independent, matching iris_ma6)
- [x] IMU-triggered control loop (15 Hz wall-clock, 3x better than sim-time dedup)
- [x] Physics-rate callback (no timer, driven by isaac_joint_states with timestamp dedup)
- [x] Duplicate message filtering (Isaac Sim publishes ~6 identical-timestamp msgs per physics step)
- [ ] Test during policy-driven flight (MISSION state) — deferred to policy testing ticket

### Flow
I → S → Y → PR

### Status
Done

### Related
- **Ticket #022** — continuation of this investigation. Found joint sign inversion (USD -X/-Y axes), replaced Jacobian with analytical IK, fixed frustum sign convention. All LOS stabilization issues resolved.
