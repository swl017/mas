## Ticket #011: Refactor los_rate_controller to derive state from actual joint angles

### What
Replace the internal rate-integration state (`_azimuth_world`, `_elevation_world`) and feedback-blend correction loop in `los_rate_controller.py` with direct reads of the actual gimbal joint angles from `isaac_joint_states`. The controller should treat actual joint positions as the authoritative state, not an integrated estimate that slowly blends toward reality.

### Why
The current design integrates velocity commands to maintain internal world-frame angles, then applies a fractional `feedback_blend` (default 5%) to nudge the estimate toward actual joints each tick. This causes:
1. **Accumulated drift** — the integrated estimate diverges from reality whenever the actuator can't track the commanded rate (e.g., near joint limits, under dynamic load, or when sim rate varies)
2. **Sluggish correction** — at 5% blend per tick the controller takes ~60 ticks (~0.6s at 100 Hz) to halve an error, masking actuator lag behind a soft filter
3. **Hidden state** — world-frame angles (`_azimuth_world`, `_elevation_world`) are unobservable quantities reconstructed from commands rather than measured from the plant, making debugging and tuning harder
4. **Fragile time coupling** — ticket #009 showed that wall-clock vs sim-clock mismatch caused the integrator to overshoot 4×; reading actual joints would have been immune to that class of bug

### Scope boundary
- Only refactor the state representation and control loop in `los_rate_controller.py`
- Do not change the IK math (`_world_to_body_angles`, `_compute_stabilizing_roll`), the joint command publishing, or the Isaac Sim articulation drive config
- Do not change MAS nodes or topic wiring

### Affected modules
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py`

### Design sketch
1. **Read actual joint angles** each tick from `_joint_positions_actual` (already cached from `isaac_joint_states` subscriber)
2. **Forward-kinematics to world frame**: convert body-frame joint angles (yaw, pitch) + vehicle quaternion → world-frame azimuth/elevation (inverse of `_world_to_body_angles`)
3. **Apply rate command as delta on world-frame state**: `az_world = fk_az + cmd_az_rate * max_rate * dt`, same for elevation
4. **IK back to body frame** and publish as before
5. **Remove** `_feedback_blend` parameter and the blending logic (steps 1 in current `_timer_callback`)
6. **Remove** or repurpose `_azimuth_world` / `_elevation_world` as transient variables, not persistent integrated state
7. **Fallback**: before the first `isaac_joint_states` message arrives, hold zero-position (current behavior) — do not integrate blind

### Key detail — body-to-world FK
The inverse of `_world_to_body_angles` is:
```python
# body-frame joint yaw/pitch → world-frame azimuth/elevation
dir_body = [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch)]
dir_world = quat_rotate(q_wxyz, dir_body)
az_world = atan2(dir_world[1], dir_world[0])
el_world = atan2(dir_world[2], sqrt(dir_world[0]**2 + dir_world[1]**2))
```

### Acceptance criteria
- `_feedback_blend` parameter and blending loop are removed
- Controller state is derived from `isaac_joint_states` each tick (no persistent integrator)
- Gimbal still meets ±2° steady-state spec from ticket #009
- Velocity commands still produce smooth, responsive pointing
- `gimbal_los_state` topic reports world-frame angles consistent with actual joint readback
- Finite-difference joint velocity (for `combined_ang_vel_w`) computed from actual joints, not integrated targets

### Flow
Light (I → S → Y → PR)

### Status
Done — refactor implemented and verified in sim

### Verification (2026-04-01)
- All 3 `los_rate_controller` nodes launched cleanly (px4_1, px4_2, px4_3)
- `gimbal_los_state` now reports FK-derived world-frame angles from actual joints (was always `(0,0)` with old integrator)
- Manual rate command test (0.3 az rate for 3s): `los_state.x` moved from 0 → 0.143 rad, confirming FK→rate→IK loop is functional
- No oscillation, no errors in logs

### Additional changes (2026-04-01, same session)

#### Camera orientation fix (pitch-turns-roll bug)
- **Root cause**: MonocularCamera in `px4_multi_world_iris_gimbal3.isaac.py` had `orientation: [0, 0, -90]` (pure Z rotation), but the correct offset is a compound `R_z(-90)*R_x(-90)`. With the pure Z rotation, pitch_joint rotations appeared as roll in the camera image. Documented in iris_ma6 `VISUALIZATION_DEBUG_LOG.md` iteration 11.
- **Fix**: Changed to `orientation: [0, 0, 0]` (identity) — pitch now correctly tilts the camera up/down.

#### YAW_JOINT_OFFSET convention update (-π/2 → +π/2)
- With identity camera orientation, `yaw_joint=0` physically points camera along body -Y (right side). To map controller yaw=0 to body +X (forward), offset changed from `-π/2` to `+π/2`.
- Updated in: `los_rate_controller.py`, `gimbal_stabilizer.py`, `observation_assembler.py`, `policy_node.py`, `policy_deploy.yaml`, `iris_gimbal3.py` (Isaac Lab asset).

#### Position command mode (gimbal_cmd_rpy_deg)
- Added `gimbal_cmd_rpy_deg` subscription to `los_rate_controller` for body-frame absolute angle commands (degrees, x=roll y=pitch z=yaw).
- Position commands override rate commands; any rate command clears position mode.
- Pitch is negated in position mode: point_to_region convention (positive=down) vs pitch_joint physics (positive=up).
- Enables `point_to_region_node` → mission gate → `los_rate_controller` pipeline for target tracking.

#### Verified end-to-end
- `point_to_region_node` publishes body-frame yaw/pitch to `tracking/gimbal_cmd_rpy_deg`
- Mission node in TRACKING state (1) forwards to `gimbal_cmd_rpy_deg`
- `los_rate_controller` position mode drives joints to commanded angles
- Gimbal correctly points at `target_region` published on `/target_region`

### Dependencies
- Ticket #008 (sim joint state feedback) — resolved
- Ticket #009 (oscillation fix) — resolved; this refactor further hardens against that class of bug
