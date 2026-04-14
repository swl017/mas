## Ticket: Gimbal calibration script (D2)

**What**: Implement `gimbal_calibration.py` — a combined gimbal encoder verification and calibration sweep script that extends the mas/005 encoder hardware test.

**Why**: The iris_ma6 gimbal randomization assumes offset ranges (±0.1 rad yaw, ±0.05 rad pitch, ±0.02 rad roll) and dynamics parameters (stiffness ±20%, damping ±20%) that are not grounded in measured hardware data. The gimbal encoder is the primary gimbal state source but has not been validated on real hardware (mas/005 is blocked on hardware access). This script chains encoder verification → calibration sweep → dynamics characterization in a single bench test session.

**Blocked on**: `mas/005-gimbal-encoder-hwtest` (encoder sign convention and stream continuity must be validated first). This ticket can be specced and partially implemented, but cannot be tested until mas/005 is unblocked.

**Depends on**: Nothing in iris_ma6 or offboard_py. Interacts with gimbal_stabilizer package for SIYI SDK communication.

**Script workflow**:
1. **Encoder verification** (from mas/005):
   - Read raw encoder values via SIYI SDK
   - Verify sign convention: command positive yaw → encoder increases (or document actual convention)
   - Verify stream continuity: no jumps, gaps, or wraparound artifacts
   - Verify LOS rate controller compatibility
2. **Calibration sweep**:
   - Command gimbal to known angles via SIYI SDK (yaw: full range, 5° increments; pitch: full range, 5°; roll: read-only since auto-stabilized)
   - Log encoder readings in ROS2 bag
   - Measure hysteresis: sweep forward then backward, compare readings at same commanded angle
3. **Zero-offset estimation**:
   - Point camera at a checkerboard at known position
   - Compare encoder-reported angle vs. camera-based angle estimate (PnP from checkerboard)
   - Compute per-axis zero offset (encoder_angle - true_angle)
4. **Output**: CSV with commanded angle, encoder reading, camera-estimated angle, hysteresis, per-axis offset statistics

**Scope boundary**:
- DO: Implement encoder verification + calibration sweep in one script
- DO: Record all data to ROS2 bag with auto-naming
- DO: Use SIYI SDK for gimbal commands (via gimbal_stabilizer package interface)
- DO: Include camera-based ground truth via checkerboard (if feasible in single script)
- DO NOT: Modify gimbal_stabilizer package internals
- DO NOT: Implement gimbal dynamic response tests (collected from A1-A4 flight data, ticket-023)
- DO NOT: Implement acceleration compensation tests (collected during A2 flights)

**Affected files**:
- NEW: Script in `ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/` or `offboard_py/offboard_py/`
- Reference: `mas/005-gimbal-encoder-hwtest/gap_analysis.md` for encoder verification checklist

**Acceptance criteria**:
- Encoder sign convention documented (matches or corrected to match sim convention)
- Full range sweep completed for yaw and pitch with 5° resolution
- Hysteresis quantified (max forward-backward difference per axis)
- Zero offset estimated per axis with std across multiple measurements
- All data recorded in ROS2 bag + summary CSV

**Reference**: [checklist.md](../021-sim2real-measurement-checklist/checklist.md) item D2; `mas/005-gimbal-encoder-hwtest/ticket.md`

**Flow**: Light (clear scope, but blocked on hardware)
