# Sim-to-Real Transfer Priority Record

**Context**: Informed by analysis of the Aerial_To_Aerial_Interception codebase (TU Delft PATS-X project), which achieved 95.6% real-world interception rate via system identification + modest domain randomization. Key lesson: *measure first, randomize second*.

**Current iris_ma6 status**: Extensive curriculum-gated DR (mass, gains, gimbal, camera, delay, noise, dropout). Aerodynamic model implemented at 4 fidelity levels. Missing: drag DR, wind activation, sys-id validation, some sensor pipeline measurements.

**Reference**: [`checklist.md`](checklist.md), [`Aerial_To_Aerial_Interception/`](/home/usrg/source/Aerial_To_Aerial_Interception/)

---

## High Impact

### H1. Wind and External Disturbance Model

**Gap**: Aero level 2+ supports wind (Dryden gust model) but `sigma_gust = 0.0` and `v_wind_mean = (0,0,0)` in default config. Wind is the single largest unmodeled real-world disturbance.

**What to do**:
1. Activate wind in training by setting `sigma_gust > 0` and adding wind to curriculum
2. Add Ornstein-Uhlenbeck process for slowly-varying wind direction (current Dryden is isotropic)
3. Gate via `progress_dynamics` phase (180k-200k steps) alongside other DR
4. Calibrate `sigma_gust` and `v_wind_mean` from outdoor flight data (see checklist B2)

**Expected impact**: High. Wind causes 0.5-2.0 m/s velocity tracking errors outdoors that the policy has never seen in training. Without this, the first outdoor flight will likely show significant performance degradation.

**Prerequisite measurements**: B2 (wind characterization)

### H2. System Identification of Iris Drone

**Gap**: Isaac Sim physics parameters (mass, inertia, thrust coefficients) come from URDF/USD defaults, not measured values. The sim-real mismatch is unknown.

**What to do**:
1. Collect PX4 `.ulg` logs from velocity step responses (checklist A2)
2. Run system identification: compare sim velocity response vs. real at matched controller gains
3. Compute the **residual** (sim vs. real tracking error) — this is what DR must cover
4. Adjust nominal drag parameters (`C_d`, `A`) to match measured drag-vs-speed curve
5. If residual is small (< 5% velocity error), current DR ranges are adequate
6. If residual is large (> 10%), either fix the nominal model or widen DR

**Expected impact**: High. The interception project found that well-identified nominal parameters + 10-20% DR transferred better than poorly-identified + 30% DR. Knowing your residual is the foundation for principled DR tuning.

**Prerequisite measurements**: A1, A2, A3, E1, F1

### H3. Aerodynamic Drag Domain Randomization

**Gap**: Drag parameters (`C_d`, `A`, `k_H`, `k_flap`) are fixed during training. The `progress_dynamics` phase randomizes mass/gains/gimbal/camera but NOT drag.

**What to do**:
1. Add drag coefficient DR to `_reset_idx()`, gated by `progress_dynamics`
2. Randomize `C_d` ±30% (drag is hard to measure precisely)
3. Randomize `A` ±20% (frontal area varies with attitude, payload)
4. Randomize `k_H` and `k_flap` ±50% (rotor effects are poorly characterized)
5. Optionally randomize `rho` ±5% (altitude/temperature variation)

**Expected impact**: High. At 10 m/s, drag force is ~6N. A 30% error in C_d means ~2N unmodeled force, which causes ~1.2 m/s steady-state velocity error. This is already observed in Ticket-004.

**Prerequisite measurements**: B1 (drag characterization)

### H4. Actuator Dynamics Mismatch

**Gap**: Isaac Sim applies forces directly to rigid bodies. Real motors have lag (tau ~ 50-100ms), saturation, and voltage-dependent thrust curves. The ±20% gain DR partially covers this but doesn't model the lag structure.

**What to do**:
1. Add a configurable first-order filter on commanded velocities before physics application
2. Parameterize as `v_applied(t+dt) = v_applied(t) + (v_cmd - v_applied) * dt / tau_actuator`
3. Randomize `tau_actuator` per episode (e.g., 20-80ms range)
4. This is separate from controller gain DR — it models the *plant*, not the *controller*

**Expected impact**: High for aggressive maneuvers. At slow velocities the effect is small, but when the policy commands rapid velocity changes (which it does during target tracking), actuator lag causes tracking overshoot that isn't present in sim.

**Prerequisite measurements**: A2 (velocity step response), A5 (motor RPM characterization, optional)

---

## Medium Impact

### M1. Observation Latency Validation

**Gap**: Delay system uses estimated latencies (5ms ego motion, 100ms detection, 500ms inter-agent). These may not match the actual deployment pipeline.

**What to do**:
1. Instrument the real ROS2 pipeline to measure actual latencies (checklist C1-C3)
2. Update delay system parameters to match measured values
3. If measured distribution is heavy-tailed (not Gaussian), consider switching to log-normal or adding occasional >3sigma spikes

**Expected impact**: Medium. The delay system is already sophisticated with curriculum-gated noise/dropout. But if real comms latency is 200ms (not 500ms) or detection is 50ms (not 100ms), the policy is training against wrong assumptions.

**Prerequisite measurements**: C1, C2, C3

### M2. Detector Realism (False Positives, Scale-Dependent Detection)

**Gap**: Current detector replicator models noise and dropout, but not:
- False positives (detecting non-target objects)
- Scale-dependent detection rate (small/distant targets missed more)
- Target confusion (detecting wrong target in multi-target scene)

**What to do**:
1. Run detector on representative deployment footage, compute per-scale detection rate
2. Add scale-dependent miss probability to detector replicator: `p_miss(bbox_area) = f(1/area)`
3. Add false positive model: random bbox at low rate (~1% per frame)
4. Gate via curriculum (currently FP/FN phase at 100k-120k steps)

**Expected impact**: Medium. The policy already handles dropout and noise, but false positives could cause erratic gimbal pointing. Scale-dependent detection directly affects zoom strategy.

**Prerequisite measurements**: C4 (detection FPS/dropout), plus offline detector evaluation

### M3. Battery Voltage Sag

**Gap**: Over a flight, battery voltage drops 10-15%, reducing max thrust. Not modeled.

**What to do**:
1. Model as slow linear decay of max thrust over episode time
2. `thrust_scale(t) = 1.0 - voltage_sag_rate * t` where `voltage_sag_rate ~ 0.01-0.02 per minute`
3. Randomize `voltage_sag_rate` per episode
4. Alternative: just widen thrust DR range, which implicitly covers this

**Expected impact**: Medium. Affects end-of-flight performance when battery is low. Policy trained at nominal thrust may not handle reduced authority gracefully.

**Prerequisite measurements**: A1 at multiple battery levels

### M4. Orientation Representation (Euler -> 6D Rotation Matrix)

**Gap**: Current observation uses Euler angles `[roll, pitch, yaw]` which have discontinuities at +-pi and gimbal lock near +-90 degree pitch. Quaternions have double-cover (q = -q). The 6D rotation matrix representation (first 2 columns) is the minimum continuous SO(3) representation.

**What to do**:
1. Replace `euler_xyz_from_quat()` -> `quat_to_matrix()`, extract columns 0 and 1 (6D)
2. Update observation dimension: +3 dims per agent (6 vs 3)
3. Retrain and compare learning curves
4. Particularly beneficial for RNN policies where discontinuous inputs corrupt hidden state

**Expected impact**: Medium. The policy currently works with Euler angles, but may exhibit instability near yaw wrapping boundaries or aggressive pitch. Zhou et al. (CVPR 2019) and the interception project both validate this representation.

**Prerequisite**: No measurements needed. Pure software change.

**Reference**: Zhou et al., "On the Continuity of Rotation Representations in Neural Networks", CVPR 2019

---

## Lower Impact

### L1. Ground Effect

**What**: Near-ground flight increases thrust efficiency (air cushion effect). Typically significant below 1 rotor diameter altitude (~0.5m for Iris).

**Relevance to iris_ma6**: Low. Altitude penalty threshold is 2.0m, so agents rarely fly below 2m. Only relevant if deployment involves low-altitude operations.

**If needed**: Add thrust multiplier `k_ground = 1 + (R/4h)^2` where R is rotor radius, h is altitude.

### L2. Propwash Between Agents

**What**: Downwash from one drone reduces thrust of another flying below/behind it.

**Relevance to iris_ma6**: Low-medium. Multi-agent, but agents typically maintain separation via CBF. Only relevant if agents operate in vertical stacks.

**If needed**: Model as proximity-dependent thrust reduction for drones within 45-degree cone below another.

### L3. Camera Rolling Shutter

**What**: CMOS sensors read rows sequentially, causing motion blur/skew during fast rotation.

**Relevance to iris_ma6**: Low. Bbox-based observation (not pixel-level) is somewhat robust to rolling shutter artifacts. Detector noise model implicitly covers some of this.

### L4. Gimbal Hysteresis and Backlash

**What**: Mechanical play in gimbal joints causing pointing error after direction reversal.

**Relevance to iris_ma6**: Low. Current gimbal offset DR (±0.1 rad) likely covers backlash effects. Only significant if gimbal is cheaply manufactured.

**Prerequisite measurements**: D2, D3 (gimbal calibration)

### L5. Thermal Effects on IMU

**What**: IMU bias drift with temperature (gyro drift ~0.01 deg/s/degC).

**Relevance to iris_ma6**: Very low. PX4's internal calibration handles thermal compensation. Only relevant for very long flights (>30 min) where temperature shifts significantly.

---

## Post-Deployment: Residual Model Fine-Tuning

### R1. Residual Dynamics Learning (Swift Approach)

**What**: After initial sim-to-real deployment, collect real flight data and learn a residual correction to the simulation dynamics model, then retrain the policy in the corrected sim.

**Evidence**: Kaufmann et al. (Nature 2023, "Swift") showed that DR alone achieved **0% track completion** on a real autonomous racing drone in realistic conditions, while residual model fine-tuning maintained ~100% completion. The residual approach learns `f_corrected = f_nominal + f_residual(state, action)` from real data, capturing unmodeled dynamics that DR cannot cover (aerodynamic interactions, flex, vibration modes, etc.). UZH RPG (arXiv: 2508.21065, RA-L 2026) showed a similar approach reduced hovering error by 55% (0.231m → 0.105m) with only 3 adaptation steps.

**Why this matters for iris_ma6**: DR covers parameter uncertainty (mass ±10%, gains ±20%) but cannot model structural mismatches — unmodeled coupling between gimbal and body dynamics, propwash effects on gimbal vibration, antenna-dependent communication latency patterns, etc. These are systematic errors that no amount of randomization will average out. A residual model captures them from data.

**What to do**:
1. Deploy policy from sim training (with DR) on real hardware
2. Collect 10-30 minutes of flight data with full state logging (PX4 `.ulg` + ROS2 bags)
3. Train a small residual network: `Δf = MLP(state, action)` that maps (state, action) → correction to next-state prediction
4. Inject the learned residual into the Isaac Sim environment's post-physics step (add `Δf` to the state after physics integration)
5. Retrain policy in the corrected simulator
6. Repeat 1-5 for 2-3 iterations (diminishing returns after that)

**Implementation in iris_ma6**: Add a `residual_dynamics` module that:
- Loads a trained residual MLP (or is bypassed when `None`)
- Applies `state_corrected = state_physics + residual_mlp(state, action)` after `_post_physics_step()`
- Gated by curriculum: residual correction strength ramps from 0→1 over `progress_dynamics` phase

**Expected impact**: High, but only after first deployment. This is the final stage of sim-to-real, not the first. All preceding items (H1-H4, M1-M4) reduce the residual that needs learning — the smaller the initial sim-real gap, the less data needed for residual fine-tuning.

**Prerequisite**: First real-world deployment with sufficient data collection infrastructure. Depends on all Phase 1-3 measurements being complete and baseline DR being tuned.

**Reference**: Kaufmann et al., "Champion-level drone racing using deep reinforcement learning", Nature 2023; Song et al., "Learning on the Fly", arXiv: 2508.21065, RA-L 2026

---

## Summary Matrix

| ID | Item | Impact | Effort | Needs Measurement? | Blocks On |
|----|------|--------|--------|-------------------|-----------|
| H1 | Wind disturbance model | High | Medium | B2 | - |
| H2 | System identification | High | High | A1,A2,A3,E1,F1 | - |
| H3 | Drag DR | High | Low | B1 | H2 (for range) |
| H4 | Actuator dynamics | High | Medium | A2,A5 | - |
| M1 | Latency validation | Medium | Low | C1,C2,C3 | - |
| M2 | Detector realism | Medium | Medium | C4 | - |
| M3 | Battery voltage sag | Medium | Low | A1 | - |
| M4 | 6D rotation repr. | Medium | Low | None | - |
| L1 | Ground effect | Low | Low | None | - |
| L2 | Propwash | Low | Medium | None | - |
| L3 | Rolling shutter | Low | Low | None | - |
| L4 | Gimbal hysteresis | Low | Low | D2,D3 | - |
| L5 | Thermal IMU drift | Very Low | Low | None | - |
| R1 | Residual dynamics learning | High (post-deploy) | High | Real flight data | H1-H4, M1-M4, first deployment |

## Recommended Execution Order

1. **M4** (6D rotation) — no measurements needed, pure software, do immediately
2. **H3** (drag DR) — low effort, use conservative ranges until B1 data available
3. **H1** (wind) — activate existing Dryden model with conservative sigma_gust
4. **Phase 1 measurements** (E1, F1, D1, D2) — bench tests, no flying
5. **Phase 2 measurements** (A1, A2, B1, C1, C2) — first flight campaign
6. **H2** (sys-id) — after Phase 2 data, compare sim vs. real
7. **H4** (actuator lag) — after A2 data reveals response characteristics
8. **M1** (latency validation) — update delay params from C1-C3 data
9. **M3** (battery sag) — after A1 multi-battery data
10. **Phase 3 measurements** (C3, C4) — multi-drone flights
11. **M2** (detector realism) — after C4 data
12. **First real-world deployment** — baseline policy with DR
13. **R1** (residual dynamics) — collect real flight data, train residual, retrain policy. Iterate 2-3x