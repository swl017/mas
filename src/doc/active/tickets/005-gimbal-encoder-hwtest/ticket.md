## Ticket #005: Hardware verification — gimbal encoder wiring

### What
Verify 5 encoder behaviors on real SIYI hardware: sign convention, stream continuity, los_rate_controller compat, point_to_region closed-loop, multiview reprojection error

### Why
Encoder angles (0x26) replaced IMU angles as primary gimbal state; software is done but hardware behavior is unvalidated. Wrong sign convention would cause divergent pointing or triangulation error.

### Scope boundary
Fix sign multipliers or frame conventions if needed. Do not change the architectural decision (encoder as primary). Do not touch sim path.

### Affected modules
`gimbal_controller`, `mas_multiview`, `mas_policy`

### Acceptance criteria
All 5 checklist items in `gap_analysis.md` Priority 1 pass on real hardware

### Flow
Light (known scope, need I -> S -> Y if fixes are required)

### Status
In progress — A8 mini does not support 0x26. Workaround path identified: derive body-frame pitch from 0x0D minus aircraft pitch. Yaw already drift-free via 0x0D (magnetic encoder). Awaiting implementation + verification of Tests 4 & 5.

---

## Hardware Findings (2026-04-13)

### Gimbal identification
- **Model:** SIYI A8 mini (hardware ID `73`)
- **Gimbal firmware:** v7.3.0 (hex `07030073`)
- **Serial:** 7302253649
- **IP:** 192.168.144.26

### Critical finding: 0x26 NOT supported on A8 mini

The SIYI SDK Protocol V0.1.1 (2025.02.26) Appendix 2 "gimbal control command support list" confirms:
- **0x26 (Request Gimbal Magnetic Encoder Angle Data):** supported on **ZT30 only** — no checkmark for A8 mini
- **0x22 (Send Aircraft Attitude):** supported on ZT30, ZT6, ZR10, ZR30, A8 mini
- **0x3E (Send GPS Raw Data):** A8 mini accepts but does not ACK

The gimbal silently ignores 0x26 requests (no response, encoder values stay at 0.0). Confirmed via:
1. One-shot `requestGimbalEncoderAngle()` → returns (0, 0, 0)
2. Stream `requestDataStreamEncoderAngle(50)` → no data arrives
3. `parseGimbalEncoderMsg` never called (no 0x26 responses in buffer)
4. 0x0D IMU attitude works correctly (heading-frame angles)

### No acceleration compensation available

Neither 0x22 (attitude + angular rates) nor 0x3E (GPS position + velocity) provide linear acceleration data. The A8 mini's internal IMU accelerometers cannot distinguish gravity from centrifugal force during sustained maneuvers. This means:
- 0x0D world-frame angles drift during aggressive flight (orbits, banking turns)
- Deriving body-frame joints via `joint = 0x0D_world - aircraft_heading` inherits this drift
- This was the original motivation for wanting encoder angles

### Test 1: Sign convention — PASS
- Left yaw = positive z, down pitch = positive y on `encoder_rpy_deg` topic
- `state_rpy_deg` (0x0D IMU) in heading frame, `encoder_rpy_deg` was supposed to be body-frame joint angles
- `pitch_direction=-1.0` correctly negates raw value to match downstream convention (positive pitch = tilt down)
- Command topic produces matching physical movement

### Test 2: Encoder stream — FAIL (not supported)
- 0x26 not supported on A8 mini hardware; stream returns zeros
- 0x0D IMU stream works but is heading-frame, not body-frame

### Bugs fixed during testing
1. **`tf_transformations` missing on Jetson:** replaced with `transforms3d` (pure Python pip package). Import changed from `euler_from_quaternion([x,y,z,w])` to `quat2euler([w,x,y,z], axes='sxyz')`.
2. **SDK socket reconnection bug:** `disconnect()` closes socket but `connect()` never recreated it, causing "Bad file descriptor" cascade on retries. Fixed by adding `_create_socket()` helper, called in `__init__` and after `disconnect()`.

### Test 3: Mode behavior — 0x0D frame convention (bench test)

Tested Lock, Follow, FPV modes with manual base rotation. Recorded 0x0D (yaw, pitch, roll) at 10 Hz for 10s per mode.

**Results:**

| Mode | Yaw range | Pitch range | Roll range |
|------|-----------|-------------|------------|
| Lock | 77.0° | 0.0° | 0.0° |
| Follow | 59.7° | 0.0° | 0.0° |
| FPV | 32.7° | 9.2° | 10.4° |

**Conclusion: 0x0D yaw = joint angle (encoder-based) in ALL modes.**
- Yaw starts at ~0 when centered, changes proportionally as base rotates
- In Lock mode, gimbal counter-rotates to hold world orientation → joint angle tracks the rotation → 0x0D reports that joint angle
- This is consistent with SDK note "yaw angle is derived from a magnetic encoder"

**0x0D pitch/roll = heading-frame (world-stabilized):**
- Lock/Follow: pitch=0.0, roll=0.0 — camera stays level regardless of base rotation
- FPV: pitch/roll change — body-frame passthrough

**Frame convention for 0x0D:**

| Axis | Frame | Source | Drift risk |
|------|-------|--------|------------|
| Yaw | Joint (body-frame) | Magnetic encoder | None |
| Pitch | Heading (world-frame) | IMU | Under centrifugal force |
| Roll | Heading (world-frame) | IMU | Under centrifugal force |

### Controllability per mode

| | Yaw | Pitch | Roll |
|---|-----|-------|------|
| Lock | Commandable (world target) | Commandable (world target) | Auto-stabilized |
| Follow | Follows body + offset | Commandable (world target) | Auto-stabilized |
| FPV | Follows body + offset | Follows body + offset | Follows body |

Roll is never user-commandable (SDK only exposes yaw/pitch via 0x07, 0x0E, 0x41).

### Yaw drift under 0x22 attitude injection

Even with 0x22 injection at 100 Hz from MAVROS IMU, the yaw joint angle drifts slowly in Lock mode. Drift stops when siyi_ros_node is killed (no more 0x22 injection). This suggests the 0x22 data actively causes the drift — either the ENU→NED conversion produces a heading offset that the gimbal tries to track, or the A8 mini's internal heading fusion fights with the injected attitude.

Possible causes:
- MAVROS IMU yaw (magnetometer-based) disagrees with gimbal's internal heading → gimbal slowly rotates to match
- ENU→NED yaw conversion (`π/2 - yaw_enu`) produces a different heading reference than what the gimbal expects
- The A8 mini may not benefit from 0x22 injection on the bench (no GPS, no flight)

**Workaround**: set `enable_aircraft_attitude:=false` when 0x22 is not needed. In flight, the heading disagreement may be smaller since both use GPS-aided EKF.

### Revised plan

The situation is much better than initially feared:
- **Yaw**: encoder-based joint angle, drift-free without 0x22 injection — available directly from 0x0D
- **Pitch**: heading-frame, needs `pitch_joint = 0x0D_pitch - aircraft_pitch` derivation
- **Roll**: heading-frame, always stabilized to ~0 in Lock/Follow (auto-managed by gimbal)

Only pitch is vulnerable to centrifugal drift, and pitch centrifugal drift is smaller than yaw would be (gravity reference affected by vertical acceleration component, not horizontal centrifugal force during orbits).

**Next steps:**
1. Implement joint-frame pitch derivation in `siyi_ros_node.py`: replace the zero-valued `encoder_rpy_deg` output with `(roll_from_0x0D, pitch_joint = 0x0D_pitch - aircraft_pitch, yaw_from_0x0D)`. Keep the existing `pitch_direction` sign convention and downstream remapping to `gimbal_state_rpy_deg` so consumers are unaffected.
2. Decide default for `enable_aircraft_attitude`. Bench tests showed 0x22 injection actively causes yaw drift in Lock mode; recommend default `false` until the MAVROS-ENU → gimbal-NED heading disagreement is understood.
3. Characterize residual pitch drift under sustained rotation (bench), and under flight (centrifugal force).
4. Test 4 — `point_to_region` closed-loop: command gimbal at a known target, verify convergence and no oscillation with the derived joint angles.
5. Test 5 — `mas_multiview` triangulation: compare reprojection error of the derived joint-frame pitch path against the raw 0x0D heading-frame path on a real rosbag.
6. Future: implement 0x3E (GPS/velocity) injection to give the gimbal acceleration context (ArduPilot parity).