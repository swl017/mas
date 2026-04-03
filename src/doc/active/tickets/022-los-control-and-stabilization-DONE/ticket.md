# Ticket #022: Gimbal LOS control and stabilization

## Problem (RESOLVED)

Three issues found and fixed in the `los_rate_controller.py` gimbal controller:

1. **Joint sign inversion**: iris_gimbal3 USD roll/pitch joints rotate around -X/-Y axes, but the Jacobian/FK code assumed +X/+Y. Caused pitch to point the wrong direction and roll/pitch instability at non-zero yaw.

2. **Jacobian instability at large gimbal yaw**: The Jacobian-inverse controller diverged at gimbal_yaw≈-90° due to cross-coupling amplification with PD actuator dynamics at ~20 Hz control rate. Replaced with analytical IK (same as position mode).

3. **RVIZ frustum mismatch**: `gimbal_state_rpy_deg` published values in the wrong sign convention for the triangulation_node's `Rz(yaw)*Rx(roll)*Ry(pitch)` composition.

---

## 1. Frame & Direction Convention Reference

Source: `src/doc/frame_conventions.md`

| Convention | Value |
|---|---|
| World | ENU (+X East, +Y North, +Z Up) |
| Body | FLU (+X Forward, +Y Left, +Z Up) |
| Gimbal chain | body → yaw(+Z) → roll(-X) → pitch(-Y) → camera |
| YAW_JOINT_OFFSET | +pi/2 |
| Quaternion | wxyz internal, xyzw from ROS IMU |

### Joint Sign Conventions (verified experimentally 2026-04-04)

| Joint | USD Axis | USD Positive = | Code Convention | Negate at boundary? |
|---|---|---|---|---|
| Yaw | +Z | Pan LEFT | Same | No |
| Roll | **-X** | Tilt CCW (front) | +X = CW | **Yes** |
| Pitch | **-Y** | Tilt UP | +Y = down | **Yes** |

### gimbal_state_rpy_deg Convention

Published for downstream `Rz(yaw)*Rx(roll)*Ry(pitch)` composition:

| Field | Content | Sign Convention |
|---|---|---|
| x (roll) | `actual_roll` (internal) | Rx standard: positive = CCW from front |
| y (pitch) | `-actual_pitch` (negated) | Ry standard: positive = tilt up |
| z (yaw) | `actual_yaw` (internal) | Rz standard: positive = CCW from above |

---

## 2. Root Causes Found

### A. Joint sign inversion (iris_gimbal3 USD)

The iris_gimbal3.usda defines roll_joint around **-X** and pitch_joint around **-Y** (body frame). The Jacobian controller assumed +X and +Y. This caused:
- Pitch pointed opposite direction (down instead of up for elevated targets)
- Roll correction inverted
- Both stable at yaw=0 (where coupling is minimal) but divergent at other yaw angles

**Fix**: Negate roll and pitch at read/write boundary in `los_rate_controller.py`:
```python
actual_roll = -self._joint_positions_actual['roll']
actual_pitch = -self._joint_positions_actual['pitch']
# ... and negate back when publishing commands:
positions = [self._yaw + YAW_OFFSET, -self._roll, -self._pitch]
```

### B. Jacobian instability at large gimbal yaw

The Jacobian-inverse controller (ported from iris_ma6 training) diverged at gimbal_yaw≈-90° because:
- Training env: direct joint write at 100 Hz → stable
- ROS2 deployment: PD ArticulationController at ~20 Hz → cross-coupling amplification

At gimbal_yaw=-90°, the J^{-1} maps pitch-axis errors to roll_dot and vice versa. Small errors compound through the PD actuator delay, creating a growing limit cycle.

**Fix**: Replaced Jacobian rate mode with **analytical IK** — the same `_world_to_body_angles()` + `_compute_stabilizing_roll()` used in position mode. This is algebraically stable at any gimbal yaw:
```python
# Rate mode now:
# 1. Integrate LOS rate → world az/el
# 2. Analytical IK → body yaw/pitch + stabilizing roll
# 3. Publish position targets directly
```

### C. RVIZ frustum sign mismatch

The triangulation_node composes gimbal angles as `Rz(yaw)*Rx(roll)*Ry(pitch)` with standard axis convention (+X, +Y, +Z). The internal pitch convention (positive=down) is opposite to Ry standard (positive=up), requiring negation in `gimbal_state_rpy_deg.y`.

---

## 3. Changes Made

### `los_rate_controller.py`
- Negate roll and pitch at joint read/write boundary (USD -X/-Y → code +X/+Y)
- Rate mode: replaced Jacobian J^{-1} with analytical IK (`_world_to_body_angles` + `_compute_stabilizing_roll`)
- `gimbal_state_rpy_deg`: roll=internal, pitch=negated(internal), yaw=internal — matches downstream Rx/Ry/Rz composition
- Diagnostic logging at 500-tick intervals (temporary)

### `los_rate_config.yaml`
- `control_mode: "position_velocity"` (position-only had poor tracking due to weak PD)

### `frame_conventions.md`
- Updated joint axes table: USD rotation axes are -X (roll), -Y (pitch), verified experimentally
- Added negation requirement for los_rate_controller

### `joint_sign_test.py` (new)
- Systematic test that commands each joint to ±20° and reads back state
- Used to definitively determine USD joint sign conventions

---

## 4. Verification

- [x] Camera image stable at drone yaw=0 (pitch, roll, yaw compensation)
- [x] Camera image stable at drone yaw=π/2
- [x] RVIZ frustum matches camera image during body tilt
- [x] LOS tracking converges to target at any drone yaw
- [x] Position_velocity mode: actual tracks commanded within ~1 deg

---

## Status

```
Flow: I -> S -> Y -> PR
Status: Resolved
```
