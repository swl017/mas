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
   - Command gimbal to known angles via SIYI SDK (yaw: full range, 2° increments; pitch: full range, 2°; roll: read-only since auto-stabilized)
   - Log encoder readings in ROS2 bag
   - Measure hysteresis: sweep forward then backward, compare readings at same commanded angle
3. **Step response (dynamics)**:
   - Drive yaw/pitch through a matrix of step commands (±magnitudes, default [2°, 10°, 30°, 60°])
   - Skip commands outside MAS-convention per-axis bounds (pitch ∈ [-20°, +85°]) to prevent silent clamping
   - Record a scoped `bag_step/` rosbag (state, encoder, command, combined_ang_vel_w, mavros/imu/data, camera/zoom_level) across the whole step phase — bag is the primary trace for offline dynamics fitting
   - Compute per-step metrics in-script for quick inspection: 10-90% rise time, 2% settling time, overshoot %, 63.2% time constant τ
   - Emit `step_index.csv` mapping step_id to (t_cmd_ros, t_end_ros, initial angles) so the bag can be sliced per step
   - Output feeds a 2nd-order actuation model (stiffness/damping) for iris_ma6 sim randomization ranges. Note: τ scales with step magnitude on the A8 mini (observed 0.27 → 1.06 s for 2° → 60° yaw) — actuator is rate-limited, not a linear 2nd-order system. Use small-step data for the linear fit, large-step data for saturation rate.
4. **Zero-offset estimation**:
   - Point camera at a checkerboard at known position
   - Compare encoder-reported angle vs. camera-based angle estimate (PnP from checkerboard)
   - Compute per-axis zero offset (encoder_angle - true_angle)
5. **Output**: CSVs for sweep (commanded/state/encoder + hysteresis summary), step response (100 Hz trace + per-step metrics), and optional ROS2 bag

**Scope boundary**:
- DO: Implement encoder verification + calibration sweep + step-response dynamics in one script
- DO: Record all data to ROS2 bag with auto-naming (optional via flag)
- DO: Use SIYI SDK for gimbal commands (via gimbal_stabilizer package interface)
- DO: Include camera-based ground truth via checkerboard (if feasible in single script)
- DO: Capture step-response dynamics at 100 Hz for 2nd-order model fitting
- DO NOT: Modify gimbal_stabilizer package internals
- DO NOT: Implement acceleration compensation tests (collected during A2 flights)
- DEFERRED: Flight-dynamics tests under real aerodynamic loading (still collected from A1-A4 flight data, ticket-023). The bench step-response here is a linearization around static loading and feeds sim randomization, not a replacement for in-flight dynamics.

**Affected files**:
- NEW: Script in `ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/` or `offboard_py/offboard_py/`
- Reference: `mas/005-gimbal-encoder-hwtest/gap_analysis.md` for encoder verification checklist

**Acceptance criteria**:
- Encoder sign convention documented (matches or corrected to match sim convention)
- Full range sweep completed for yaw and pitch with ≤5° resolution (default 2°)
- Hysteresis quantified (max forward-backward difference per axis)
- Zero offset estimated per axis with std across multiple measurements
- Step response captured at 100 Hz for ±{2°, 10°, 30°, 60°} on yaw and pitch (pitch tilt-up skipped beyond -20° to avoid clamp); rise/settle/overshoot/τ reported per step
- Step-phase rosbag `bag_step/` covers all step topics + IMU; `step_index.csv` links step_id to bag time window for offline fitting
- All data recorded in ROS2 bag + summary CSV

**Reference**: [checklist.md](../021-sim2real-measurement-checklist/checklist.md) item D2; `mas/005-gimbal-encoder-hwtest/ticket.md`

**Flow**: Light (clear scope, but blocked on hardware)
