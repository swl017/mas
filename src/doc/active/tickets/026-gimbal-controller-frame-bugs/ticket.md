## Ticket #026: Gimbal controller frame bugs (quaternion error + integrator windup)

### What
Two bugs in `iris_ma6` training env gimbal controller (`gimbal_controller_jacobian.py`) causing LOS stabilization divergence at non-zero gimbal yaw.

### Why
Gimbal pointing becomes unstable during training when the gimbal yaw moves away from zero, particularly diverging at yaw ~pi/2. This corrupts camera observations and degrades policy learning for target tracking.

### Bug 1: Quaternion error in wrong frame

**Location**: `gimbal_controller_jacobian.py:188`

**Before**: `q_err = quat_mul(quat_inv(q_desired_body), q_current)`

This computes `R_desired^{-1} * R_current`, whose axis-angle is in the **desired camera frame**. So `att_error` and `omega_cmd` were in the camera frame, but `omega_body` (gyro) is in **body frame**. The subtraction `omega_cmd - omega_body` at line 198 mixed frames.

At yaw~0, camera frame ~ body frame (error negligible). At yaw=pi/2, x and y components are swapped -> divergence.

**Fix**: Swap quaternion order: `q_err = quat_mul(q_current, quat_inv(q_desired_body))`

This gives `R_current * R_desired^{-1}` — same rotation but axis in the "from" frame of `q_current`, which is body frame. Now `omega_cmd` matches `omega_body` frame.

### Bug 2: World-frame azimuth integrator windup

**Location**: `gimbal_controller_jacobian.py:167-175`

`_azimuth_world` integrated continuously and wrapped to [-pi, pi] via `atan2`, but was never bounded by body-frame joint limits. When the user commanded yaw rate past the gimbal's reachable range:

1. `_azimuth_world` grew to e.g. 180 degrees
2. Desired camera quaternion pointed far beyond reachable workspace
3. Quaternion error became huge -> `qdot_ref` became huge
4. Position targets clamped at +/-45 deg, but **velocity targets were unclamped**
5. No USD-level joint stops on `iris_gimbal3.usda` yaw joint to catch it

**Fix**: Anti-windup — after integrating azimuth/elevation candidate, convert to body-frame yaw/pitch via `_world_to_body_angles()`. Only accept the update if resulting body-frame angles are within joint limits; otherwise freeze the world-frame setpoint.

### Changes made

| File | Line | Change |
|------|------|--------|
| `gimbal_controller_jacobian.py` | 188 | Swapped quat multiplication order for body-frame error |
| `gimbal_controller_jacobian.py` | 185-186 | Updated comment to match new formula |
| `gimbal_controller_jacobian.py` | 167-175 | Replaced direct integration with anti-windup: candidate integrate -> body-frame limit check -> conditional accept |

### Verification
- Teleop test (`teleop_iris_ma6.py --num_envs 4 --enable_camera`): gimbal yaw no longer exceeds configured limits
- LOS stabilization no longer diverges at non-zero yaw angles

### Related
- Ticket #020 (gimbal LOS stabilization debug) — deployment-side LOS issues, different codebase (`los_rate_controller.py`)
- Ticket #022 — analytical IK replacement for deployment controller

### Status
Done.
