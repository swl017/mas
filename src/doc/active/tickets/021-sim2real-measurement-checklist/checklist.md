# Sim-to-Real Measurement Checklist

**Purpose**: Ground every DR range in measured real-world data. Each item maps to a specific DR parameter or model validation point in iris_ma6.

**Hardware**: Iris quadrotor with 3-axis gimbal + zoom camera, PX4 autopilot, SIYI HM30 radio link

**Test script requirements**: All offboard test scripts should:
- Wait for flight mode change to offboard before executing commands
- Trigger ROS2 bag recording automatically with test name prefix (no unrecorded or mislabeled experiments)
- Log to both PX4 `.ulg` (flight dynamics) and ROS2 bag (gimbal joints, camera, comms)

---

## Category A: Flight Dynamics (PX4 Logs)

### A1. Thrust-to-Weight Characterization

- [ ] **What**: Hover throttle percentage at known takeoff weight
- [ ] **Why**: Validates Isaac Sim thrust model; sets thrust DR baseline
- [ ] **How to collect**:
  1. Weigh drone with full payload (gimbal + camera + battery) on kitchen scale
  2. PX4 position mode → takeoff → hover 30s (minimum) → land or continue to other experiments
  3. Log `actuator_outputs` and `vehicle_local_position` from PX4 `.ulg`
  4. Repeat at 3 battery levels: full, 50%, 20% (voltage sag effect)
- [ ] **Test script**: None needed (position mode only)
- [ ] **Extract**: Mean hover throttle, throttle variance, thrust-per-motor at hover
- [ ] **Maps to**: Thrust model validation; mass DR baseline; battery sag model

### A2. Velocity Step Response

- [ ] **What**: Velocity tracking dynamics (rise time, overshoot, steady-state error)
- [ ] **Why**: Validates controller + drag model; calibrates velocity PID and drag feedforward
- [ ] **How to collect**:
  1. Position mode → takeoff → hover 10s to stabilize → switch to offboard mode
  2. Test script executes velocity steps: 0 → +3 → 0 → -3 m/s (returns toward origin to save space)
  3. Repeat pattern at 5, 8, 10 m/s
  4. Do in X, Y axes separately (2 flights)
  5. Log `vehicle_local_position` (pos + vel), `vehicle_attitude`, `actuator_outputs`
  6. Repeat 3x for statistical significance
- [ ] **Test script**: Offboard velocity command script (waits for offboard mode)
- [ ] **Extract**: Rise time (10-90%), overshoot %, steady-state error, settling time, velocity/attitude/rate setpoints
- [ ] **Maps to**: Controller gain DR (currently ±20%); drag coefficients; Ticket-004 feedforward
- [ ] **Also collects**: B1 (steady-state drag) from the constant-velocity segments, D3 (gimbal dynamic response during acceleration)

### A3. Attitude Step Response

- [ ] **What**: Roll/pitch step response characteristics
- [ ] **Why**: Validates attitude controller time constants in sim
- [ ] **Prerequisite**: A1 (thrust-to-weight for hover throttle), thrust curve
- [ ] **How to collect** (two options):
  - **Option A (test script, preferred)**: Position mode → takeoff → hover 10s → offboard mode → test script commands attitude setpoints at hover thrust → land or continue
  - **Option B (manual, for safety)**: Position mode → takeoff → stabilized mode → stick input (as step-like as possible) → hold 1-2s → position mode to recover → repeat → land
- [ ] **Test script**: Offboard attitude command script (Option A). Option B needs no script but is less reproducible.
- [ ] **Extract**: Attitude time constant (tau_roll, tau_pitch), rate limits (p_max, q_max), damping ratio, attitude/rate setpoints
- [ ] **Maps to**: Controller attitude gains; rate limit DR

### A4. Yaw Rate Response

- [ ] **What**: Yaw rate step response
- [ ] **Why**: Yaw dynamics are typically slower and less damped than roll/pitch
- [ ] **How to collect**:
  1. Position mode → takeoff → hover 10s to stabilize → offboard mode
  2. Test script commands yaw rate steps: 30, 60, 90 deg/s
  3. 5 repetitions per rate
- [ ] **Test script**: Offboard yaw rate command script (waits for offboard mode)
- [ ] **Extract**: Yaw time constant (tau_yaw), max yaw rate, steady-state yaw rate error, yaw rate setpoints
- [ ] **Maps to**: `max_yaw_rate` action limit; controller yaw gain DR

### A5. Motor RPM Characterization

- [ ] **What**: Motor speed vs. command mapping, motor time constant
- [ ] **Why**: Validates motor dynamics model in sim
- [ ] **How to collect**:
  - If ESC telemetry is available: data is **automatically collected** from A1-A4 flight logs (RPM logged alongside maneuvers)
  - If no ESC telemetry: use propeller tachometer on test stand with stepped PWM commands (sweep idle to max in 5% increments, hold 2s each)
- [ ] **Test script**: None needed (piggybacks on A1-A4 logs)
- [ ] **Extract**: RPM-vs-PWM curve, motor time constant (tau_motor), min/max RPM
- [ ] **Maps to**: Isaac Sim motor model validation, actuator time constant DR

### A6. Model Fitting and Validation

- [ ] **What**: Fit iris_ma6 sim model parameters to real flight data; quantify sim-real residual
- [ ] **Why**: The sim-real residual determines what DR must cover. Accurate nominal model + small DR transfers better than inaccurate model + large DR (Zhao 2024: SysID-only 0.028m vs SysID+30%DR 0.066m)
- [ ] **How**:
  1. From A1-A4 data, extract time-synchronized state trajectories (position, velocity, attitude, rates, motor commands)
  2. Replay the same motor commands / velocity commands through Isaac Sim and compare state trajectories
  3. Compute per-channel residual: `residual(t) = state_real(t) - state_sim(t)`
  4. Tune nominal sim parameters (thrust coefficient, drag Cd*A, controller gains) to minimize residual
  5. Remaining residual after tuning → sets principled DR ranges
- [ ] **Relationship to interception project sysid**: Their model is **different abstraction level** — direct motor RPM → forces via 23 empirical coefficients (no cascade controller). iris_ma6 uses white-box physics (Isaac Sim rigid body + PX4-like cascade controller). Their sysid script is not directly reusable, but the methodology (OLS regression + nonlinear optimization + 95% CI) applies. Parameters that map between models:
  - `k_w` (thrust coeff) ↔ Isaac Sim `k_f`
  - `k_x, k_y` (drag) ↔ `AerodynamicsCfg.C_d, A`
  - `tau_motor` ↔ `MotorDynamicsCfg.tau`
  - `w_min, w_max` ↔ `MotorDynamicsCfg.omega_min/max`
  - Their 16 moment coefficients (k_p1-k_r8) are **not directly portable** — iris_ma6 uses inertia matrix + mixer geometry instead
- [ ] **Test script**: Offline analysis script (not a flight test)
- [ ] **Maps to**: All nominal sim parameters; principled DR ranges for H2 (system identification) in sim2real_priority.md

---

## Category B: Drag and Aerodynamics

### B1. Steady-State Drag Force

- [ ] **What**: Velocity-dependent drag at multiple airspeeds
- [ ] **Why**: Validates `C_d * A` product; determines if quadratic drag model is sufficient
- [ ] **How to collect**: **Collected alongside A2** (velocity step response). Extract the constant-velocity segments where velocity has settled.
- [ ] **Extract**: Average pitch angle at each speed (pitch ≈ atan(drag/weight)); compute effective Cd*A from steady-state throttle offset
- [ ] **Maps to**: `AerodynamicsCfg.C_d` (nominal 0.03), `AerodynamicsCfg.A` (nominal 0.1 m^2)

### B2. Wind Disturbance Characterization

- [ ] **What**: Position/velocity tracking error under known wind conditions
- [ ] **Why**: Calibrates `sigma_gust` and `v_wind_mean` for aero level 2+
- [ ] **How to collect**:
  1. Fly hover + velocity tracking outdoors in measured wind (use anemometer)
  2. Record wind speed/direction at 1Hz concurrent with PX4 logs
  3. Log position error, velocity error, attitude disturbance
  4. 3 flights at different wind speeds (calm, moderate, gusty)
- [ ] **Note**: May require multiple days to get varied wind conditions
- [ ] **Extract**: Position error std vs. wind speed; gust bandwidth from PSD of position error
- [ ] **Maps to**: `AerodynamicsCfg.v_wind_mean`, `sigma_gust`, `gust_bandwidth`

---

## Category C: Sensor and Communication Latency Pipeline

### C1. IMU-to-Policy Latency (Proprioceptive)

- [ ] **What**: End-to-end latency from IMU measurement to policy input
- [ ] **Why**: Validates delay system `ego_motion_latency` (currently 5ms mean, 2ms std)
- [ ] **How to collect**:
  1. Instrument the ROS2 pipeline:
     - Timestamp at IMU driver publish
     - Timestamp at policy node receive
  2. Log 1000+ samples during active flight
  3. Use `ros2 topic delay` or custom latency logger
- [ ] **Test script**: ROS2 latency measurement node
- [ ] **Extract**: Mean, std, 95th/99th percentile latency; distribution shape
- [ ] **Maps to**: `delay_system_params.ego_motion_latency_ms_mean/std`

### C2. Detection Inference Latency

- [ ] **What**: YOLO/detector inference time on deployment GPU under realistic load
- [ ] **Why**: Validates `ego_detection_latency` (currently 100ms mean, 15ms std)
- [ ] **How to collect**:
  1. Collect representative outdoor images at 2 locations (varied backgrounds)
  2. Write a test script that feeds images at a configurable rate to the detector
  3. Run triangulation node concurrently for maximum realistic load
  4. Time 500+ inference calls (include preprocessing)
- [ ] **Test script**: Image feed + timing script with concurrent triangulation
- [ ] **Extract**: Mean, std, min, max inference time; distribution under load
- [ ] **Maps to**: `delay_system_params.ego_detection_latency_ms_mean/std`

### C3. Detection FPS and Dropout Rate

- [ ] **What**: Actual detection rate, missed detection probability, and correlating factors
- [ ] **Why**: Validates staleness model (currently 25 FPS ± 5) and dropout (5% base)
- [ ] **How to collect**:
  1. Fly target drone in camera frame, log target GPS position + camera detections
  2. Record: relative distance, camera zoom, apparent bbox size/center, background type
  3. Collect in two conditions: clean sky background, cluttered background (trees/buildings)
  4. 5-minute continuous operation minimum per condition
- [ ] **Note**: Complex setup — requires two-drone flight with coordinated logging
- [ ] **Test script**: Synchronized multi-drone logging script
- [ ] **Extract**: Detection rate vs. target distance, vs. bbox size, vs. zoom level, vs. background; miss rate; false positive rate; burst miss statistics (consecutive misses)
- [ ] **Maps to**: Detection staleness params; `dropout_base_probability`; burst dropout params; scale-dependent detection model

### C4. Raw Image Transport Latency (NEW)

- [ ] **What**: Latency from real-world event to image availability in ROS2 pipeline
- [ ] **Why**: Quantifies camera-to-ROS2 pipeline delay, separate from detector inference
- [ ] **How to collect**:
  1. Point camera at a high-resolution timer display (phone/monitor showing milliseconds)
  2. Compare displayed time vs. ROS2 image topic timestamp
  3. Log 500+ samples
- [ ] **Caveat**: Measures "camera sensor → image topic → display" pipeline. The actual "camera → detector input" path may differ. Need to verify if camera sensor processing dominates or ROS2 transport adds significant latency.
- [ ] **Test script**: Image timestamp comparison script
- [ ] **Extract**: Mean, std, distribution of image transport latency
- [ ] **Maps to**: Component of `ego_detection_latency` (image acquisition portion)

### C5. Datalink Latency and Dropout Rate (NEW)

- [ ] **What**: Agent-to-agent and agent-to-GCS communication latency, packet loss, burst characteristics
- [ ] **Why**: Validates `other_agent_latency` (currently 500ms mean, 80ms std) and inter-agent dropout model
- [ ] **How to collect**:
  1. Deploy 2 drones with ROS2 DDS communication
  2. Publish timestamped messages, measure one-way latency at receiver
  3. Collect agent-to-agent AND agent-to-GCS latencies
  4. Test at multiple locations with different RF environments
  5. Test with current link hardware (WiFi router initially; SIYI HM30 radio; LTE if WiFi range insufficient)
  6. Log 1000+ samples per condition per location
- [ ] **Note**: Link technology selection (WiFi vs radio vs LTE) is a separate ticket. WiFi has range issues, SIYI HM30 has throughput issues. Start with larger WiFi router, evaluate LTE if unsatisfactory.
- [ ] **Test script**: ROS2 latency ping-pong node with automatic logging
- [ ] **Extract**: Mean, std, 95th percentile per link type; packet loss rate; burst loss statistics (consecutive drops); latency vs. distance
- [ ] **Maps to**: `delay_system_params.other_agent_latency_ms_mean/std`; dropout parameters; burst dropout model

---

## Category D: Camera and Gimbal

### D1. Camera Intrinsic Calibration

- [ ] **What**: Focal length, principal point, distortion coefficients per zoom level
- [ ] **Why**: Validates `CameraRandomizationCfg.focal_length_range` (800-1200 px)
- [ ] **How to collect**:
  1. Standard checkerboard calibration (OpenCV `calibrateCamera`)
  2. Calibrate at each discrete zoom level (1x, 2x, 4x, etc.)
  3. Use 20+ images per zoom level from diverse angles
  4. Use `mrcal` to quantify calibration uncertainty
- [ ] **Test script**: None needed (bench test with existing calibration tools)
- [ ] **Extract**: f_x, f_y per zoom level; principal point offset; distortion k1-k5; calibration uncertainty from mrcal
- [ ] **Maps to**: `CameraRandomizationCfg.focal_length_range`, `fov_scale_range`

### D2. Gimbal Joint Calibration

- [ ] **What**: Gimbal encoder validation, zero-offset, range of motion, backlash
- [ ] **Why**: Validates `GimbalRandomizationCfg` offset ranges (±0.1 rad yaw, ±0.05 pitch)
- [ ] **Prerequisite**: Gimbal encoder hardware verification (ticket `mas/005-gimbal-encoder-hwtest` — currently blocked on hardware access). Encoder sign convention and stream continuity must be validated first.
- [ ] **How to collect**:
  1. Verify encoder readings work (per mas/005 checklist)
  2. Command gimbal to known angles via SIYI SDK, log encoder readings in ROS2 bag
  3. Sweep full range in yaw, pitch, roll at 5-degree increments
  4. Measure hysteresis: sweep forward vs backward
  5. Compare encoder readings against camera-based angle estimation (checkerboard at known position)
- [ ] **Test script**: Combined encoder verification + calibration sweep script (extends mas/005 hwtest)
- [ ] **Extract**: Zero offset per axis, max range, backlash magnitude, encoder-to-actual mapping
- [ ] **Maps to**: `GimbalRandomizationCfg.yaw_offset_range`, `pitch_offset_range`, `roll_offset_range`

### D3. Gimbal Dynamic Response

- [ ] **What**: Gimbal angular velocity limits, settling time, overshoot, acceleration compensation
- [ ] **Why**: Validates gimbal stiffness/damping DR (currently ±20%)
- [ ] **How to collect**:
  1. **Automatically collected** from A1-A4 flight data (gimbal compensates during maneuvers)
  2. Additionally: command step inputs to gimbal yaw/pitch at various amplitudes on bench, log at >100 Hz in ROS2 bag
  3. For acceleration compensation: compare gimbal LOS stability during A2 velocity steps (body pitches forward, gimbal should compensate). Metric: LOS angular deviation from target during acceleration phase.
- [ ] **Acceleration compensation verification**: Compare bbox center drift during velocity step transients. If gimbal compensates perfectly, bbox center stays fixed. Residual drift quantifies compensation error.
- [ ] **Test script**: Gimbal step command script (bench); flight data from A1-A4
- [ ] **Extract**: Max angular rate, rise time, overshoot, effective stiffness/damping, acceleration compensation residual (deg of LOS deviation per m/s² of body acceleration)
- [ ] **Maps to**: `GimbalRandomizationCfg.stiffness_scale_range`, `damping_scale_range`

---

## Category E: Mass and Payload

### E1. Drone Mass Budget

- [ ] **What**: Mass of each component and total takeoff weight
- [ ] **Why**: Sets nominal mass in sim; validates mass DR range
- [ ] **How to collect**:
  1. Weigh separately: frame, motors (x4), props (x4), battery, gimbal, camera, cables, misc
  2. Weigh complete assembled drone
  3. Weigh with each payload variant (different cameras, batteries)
- [ ] **Extract**: Component masses, total mass, payload variation range
- [ ] **Maps to**: `MassRandomizationCfg.body_mass_scale_range`, `payload_mass_range`

### E2. Center of Gravity with Gimbal

- [ ] **What**: CoG shift as gimbal rotates
- [ ] **Why**: Gimbal mass redistribution affects trim; not currently DR'd
- [ ] **How to collect**:
  1. Balance drone on a knife edge or hang from string (3-axis CoG)
  2. Measure CoG at gimbal neutral, max yaw left/right, max pitch up/down
- [ ] **Extract**: CoG position (x,y,z) at 5+ gimbal configurations; max CoG shift magnitude
- [ ] **Maps to**: Potential new DR parameter; validates inertia recomputation

---

## Category F: Controller Gains (PX4 Parameters)

### F1. PX4 Controller Gain Extraction

- [ ] **What**: Actual PID gains used on the real drone
- [ ] **Why**: Sets nominal gains for sim controller; determines if ±20% DR is appropriate
- [ ] **How to collect**:
  1. `param show MC_*` on PX4 shell (or from `.ulg` parameters section)
  2. Record all velocity, attitude, and rate PID gains
  3. If auto-tuned, record the auto-tune result
- [ ] **Extract**: All MC_PITCHRATE_*, MC_ROLLRATE_*, MC_YAWRATE_*, MPC_XY_VEL_*, MPC_Z_VEL_* gains
- [ ] **Maps to**: `GainRandomizationCfg` nominal values; ±20% range validation

### F2. Gain Sensitivity Analysis

- [ ] **What**: Flight performance under intentionally perturbed gains
- [ ] **Why**: Validates that ±20% DR range is survivable (not too aggressive)
- [ ] **How to collect**:
  1. Fly with nominal gains, record tracking performance
  2. Perturb velocity P gain by +20%, -20%, fly same trajectory
  3. Perturb rate D gain by +20%, -20%
  4. Check: does the drone remain stable? How much does tracking degrade?
- [ ] **Extract**: Tracking error vs. gain perturbation; stability margin
- [ ] **Maps to**: Confirms or adjusts ±20% gain DR range

---

## Category G: Uncertainty Modeling (NEW)

### G1. GPS Covariance Under Motion

- [ ] **What**: GPS position accuracy under different flight conditions and fix modes
- [ ] **Why**: Position noise feeds into triangulation uncertainty and inter-agent relative state accuracy
- [ ] **How to collect**:
  1. Log `vehicle_gps_position` (with EPH/EPV fields) and `estimator_status` during A1-A4 flights
  2. Compare GPS position against PX4 EKF fused estimate
  3. Record fix type (float, fix, RTK) and number of satellites
  4. Compare covariance during hover vs. constant velocity vs. acceleration transients
- [ ] **Questions to answer**:
  - Does GPS covariance change significantly under motion?
  - What are typical EPH/EPV values for float, fix, and RTK modes?
  - Is there a correlation between dynamics (acceleration) and position noise?
- [ ] **Test script**: None needed (data from A1-A4 `.ulg` logs)
- [ ] **Extract**: EPH/EPV statistics per fix mode; covariance vs. motion state; satellite count statistics
- [ ] **Maps to**: Delay system position noise std (currently 10cm); triangulation uncertainty model

---

## Data Collection Protocol

### Equipment Needed
- Kitchen scale (0.1g resolution) for mass measurements
- Anemometer for wind measurements
- Calibration checkerboard (A2 or larger)
- PX4 `.ulg` log enabled at max rate (`SDLOG_PROFILE = 1` for high-rate logging)
- ROS2 latency logging node
- Laptop with `pyulog` and `mrcal` installed
- High-resolution timer display (phone app) for C4 image latency test

### Recording Protocol
- **PX4 `.ulg`**: Always active for Category A/B/F flight dynamics data
- **ROS2 bag**: Always active for gimbal joints, camera images, inter-agent comms
- **Auto-naming**: Test scripts must auto-trigger `ros2 bag record` with test name prefix (e.g., `A2_vel_step_x_run1_20260415T1030`)
- **No unrecorded experiments**: Every flight must produce both `.ulg` + ROS2 bag

### Flight Test Procedure
1. **Pre-flight**: Record ambient temperature, wind (anemometer), battery voltage, GPS fix mode
2. **Arm and log**: Ensure PX4 logging active, ROS2 bag recording triggered by test script
3. **Execute maneuver**: Follow specific protocol per measurement item
4. **Post-flight**: Download `.ulg`, verify ROS2 bag completeness, record final battery voltage
5. **Parse**: Use `pyulog` (`ulog2csv`) to extract topic CSVs

### Log Parsing Tools
```bash
# Install tools
pip install pyulog

# Convert .ulg to CSV
ulog2csv flight.ulg

# Extract specific topics
ulog2csv flight.ulg -m vehicle_local_position,vehicle_attitude,actuator_outputs

# Quick flight summary
ulog_info flight.ulg
```

### Minimum Data Requirements
| Category | Minimum Flights | Minimum Duration | Repetitions | Recording |
|----------|----------------|------------------|-------------|-----------|
| A (Dynamics) | 6 | 60s each | 3x per condition | `.ulg` + ROS2 bag |
| B (Drag/Wind) | 4 | 30s per speed | 3x | `.ulg` + ROS2 bag |
| C (Latency) | 2 | 5 min continuous | 1000+ samples | ROS2 bag |
| D (Camera/Gimbal) | 0 (bench test) | N/A | 20+ images | ROS2 bag |
| E (Mass) | 0 (bench test) | N/A | 3x weighings | Manual log |
| F (Gains) | 3 | 60s each | 3x | `.ulg` |
| G (GPS) | 0 (from A1-A4) | N/A | N/A | `.ulg` |

---

## Test Scripts Required

| Script | Category | Input | Output | Notes |
|--------|----------|-------|--------|-------|
| `offboard_velocity_step.py` | A2 | Speed list, axis, hold time | Velocity commands via MAVROS/MAVSDK | Waits for offboard mode; executes 0→+v→0→-v pattern; auto-triggers ROS2 bag |
| `offboard_attitude_step.py` | A3 | Angle list, axis, hover thrust | Attitude + thrust commands | Requires A1 hover throttle as input |
| `offboard_yaw_rate.py` | A4 | Rate list, hold time | Yaw rate commands | Simpler than A2/A3 |
| `detector_latency_bench.py` | C2 | Image directory, feed rate | Timing CSV | Runs detector + triangulation under load |
| `image_transport_latency.py` | C4 | Camera topic | Latency CSV | Compares timer display vs. ROS2 timestamp |
| `datalink_latency_ping.py` | C5 | Agent IDs, duration | Latency + dropout CSV | Bidirectional timestamped ping |
| `gimbal_calibration.py` | D2 | Angle sweep params | Encoder readings + ROS2 bag | Extends mas/005 hwtest; auto-sweeps gimbal |

---

## Priority Order

**Phase 1 (Before first real-world test — bench work only)**:
1. E1 (mass budget) — 30 min bench work
2. F1 (PX4 gains) — 5 min param dump
3. D1 (camera calibration with mrcal) — 1 hr bench work
4. D2 (gimbal calibration) — 1 hr bench work (blocked on mas/005 encoder hwtest)

**Phase 2 (First flight campaign, calm conditions)**:
5. A1 (hover thrust) — 1 flight, position mode only
6. A2 (velocity steps) — 2 flights (also collects B1 drag + D3 gimbal response)
7. C1 (IMU latency) — instrument pipeline during above flights
8. G1 (GPS covariance) — from A1-A4 `.ulg` logs (no extra flights)

**Phase 3 (Second flight campaign, offboard attitude/yaw)**:
9. A3 (attitude step response) — 2 flights (requires A1 thrust data)
10. A4 (yaw rate response) — 1 flight
11. A5 (motor RPM) — from A1-A4 ESC telemetry (no extra flights)
12. A6 (model fitting) — offline analysis of all Phase 2-3 data

**Phase 4 (Multi-drone flights)**:
13. C5 (datalink latency/dropout) — 2-drone flight at multiple locations
14. C3 (detection FPS/dropout) — operational scenario with target drone
15. C2 (detection inference latency) — bench test with outdoor images from Phase 2-4

**Phase 5 (Outdoor / stress testing)**:
16. B2 (wind characterization) — 3+ outdoor flights, multiple days
17. F2 (gain sensitivity) — 3 flights with perturbed gains
18. E2 (CoG with gimbal) — bench test
19. C4 (image transport latency) — bench test