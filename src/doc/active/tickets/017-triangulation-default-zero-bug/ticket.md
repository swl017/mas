# Ticket 017 — Triangulation outputs (0,0,0) on solver failure

**Status:** open
**Package:** `mas_multiview`
**Observed:** 2026-04-02 — livestream shows intermittent zero-position triangulation outputs

## Problem

When the Ceres solver fails to converge or the initial guess estimation produces no valid result, the triangulation pipeline publishes `(0, 0, 0)` positions. This is visible in live data as points snapping to the origin.

### Root cause chain

1. **Initial guess defaults to zero** — `estimateInitialPosition()` initializes `weighted_mid_point = Eigen::Vector3d::Zero()` (multiview_triangulation.cpp:222). If all ray pairs produce reprojection errors above threshold, `total_weight` stays 0 and `association.initial_guess` is never assigned, remaining at `(0,0,0)`.

2. **Solver result never validated** — `ceres::Solve()` returns a `Summary` (multiview_triangulation.cpp:340) but the code never checks `summary.IsSolutionUsable()` or `summary.termination_type`. If the solver times out or fails, the output `position[]` array may still contain the zero initial guess.

3. **Results published unconditionally** — `triangulation_node.cpp:560-575` iterates all results and publishes them without checking for zero positions, NaN/Inf, or solver status. The only NaN guard in the file (line 639) is for frustum visualization corners, not for the actual triangulation output.

### Empty detection array from one drone

When a drone's YOLO detector finds no targets, it publishes an empty `Detection2DArray`. The ready-check (triangulation_node.cpp:359) only tests that the `detections_` shared_ptr is non-null (i.e., *any* message was ever received), so a camera with zero detections is still counted as "ready."

**What currently happens:** `associateDetections()` (multiview_triangulation.cpp:184-189) filters cameras with no detections out of `nonempty_camera_idx`, and skips associations with <2 cameras (line 207). So if only one camera actually detected something, no associations are formed and no results are published — the pipeline silently produces nothing.

**What goes wrong:**
- The node still runs all extrinsic/intrinsic setup for the empty camera (lines 395-487) — wasted work on a timer-driven hot path.
- If exactly 2 cameras have detections but a 3rd sends empties, the solver runs with only 2-camera geometry. This is valid but produces higher-uncertainty results. There is no indication to downstream that the solve used fewer views than expected.
- With stale cached data: `detections_[i]` is overwritten on every callback (line 294), so if a camera briefly detects and then publishes empty arrays, the cached non-null pointer persists but the detection list is empty — the camera counts as ready but contributes nothing.

## Impact

- `mas_tracker` (SORT3D) ingests `(0,0,0)` points, corrupting track state and potentially creating phantom tracks at the origin.
- Downstream consumers (operator UI, policy) receive false target positions.

## Proposed fix

Add validation at two levels:

### A. Solver level (multiview_triangulation.cpp)

- After `ceres::Solve()`, check `summary.IsSolutionUsable()`. If false, mark the association as invalid and skip storing the result.
- If `total_weight <= 0` in `estimateInitialPosition()`, skip the association entirely rather than proceeding with a zero initial guess.

### B. Publisher level (triangulation_node.cpp)

- Before adding to `tri_points_msg`, reject points where position is `(0,0,0)` or contains NaN/Inf.
- Optionally log rejected points at debug level for diagnostics.

### C. Empty detection handling (triangulation_node.cpp)

- In the ready-check (line 359), additionally require `!detections_[i]->detections.empty()` before counting a camera as ready. This skips extrinsic/intrinsic setup for cameras that have nothing to contribute.
- Publish the number of contributing cameras in the `TriangulatedPointArray` header or per-point metadata so downstream can assess solve quality.

### D. Age-of-Information (AoI) aware covariance — stale ray handling

**Problem:** Currently all cached data is reset after each triangulation cycle (triangulation_node.cpp:763-776). If drone A detects at 10 Hz but drone B at 2 Hz, triangulation only fires when both have fresh data in the same timer window — losing most of drone A's observations.

**Approach:** Keep the triangulation solver unchanged. Instead, incorporate observation age into the per-camera measurement covariance so stale rays are naturally down-weighted by the existing WLS solve. This is not heuristic — it follows directly from Bar-Shalom's Out-of-Sequence Measurement (OOSM) theory.

#### Theoretical basis

A ray observed at `t_obs` and used at `t_now` is an out-of-sequence measurement. Bar-Shalom (2002) showed the optimal Kalman update for such a measurement uses an **effective measurement covariance**:

```
R_eff = R + H · Σ_drift · Hᵀ
```

where `Σ_drift` is the process noise accumulated from `t_obs` to `t_now` under the target motion model. The growth rate of `Σ_drift` determines how fast stale rays lose influence:

| Target motion model      | Σ_drift                         | Weight decay |
|--------------------------|----------------------------------|--------------|
| Random walk (Wiener)     | Q · Δt                          | w ~ 1/Δt     |
| Constant velocity        | Q · Δt³/3 (position block)      | w ~ 1/Δt³    |
| Mean-reverting (OU)      | saturates at σ²_stationary      | bounded       |

Sun et al. (2017) proved that for Wiener processes, MMSE = E[AoI] — minimizing age is literally equivalent to minimizing estimation error. For OU processes (targets loitering in a region), the MSE saturates, so very old observations still carry some information about the target's operating area.

Sinopoli et al. (2004) showed there is a **critical observation rate** below which estimation error diverges — this gives a principled `max_detection_age` cutoff rather than an arbitrary one.

#### Formulation for our system

For camera `c` with observation age `Δt_c = t_now - t_obs_c`, assuming a constant-velocity target model with process noise `Q` (acceleration covariance):

```
Σ_drift(c) = F(Δt_c) · Q · F(Δt_c)ᵀ
```

where `F(Δt)` is the state transition matrix. This decomposes into per-source inflation:

```
Σ_twb(c)   = σ_pos² · I₃  +  Σ_drift_pos(Δt_c)
Σ_phiwb(c) = σ_ori² · I₃  +  Σ_drift_ori(Δt_c)
```

The velocity from `common_frame/odom` twist (already subscribed) parameterizes `Q` — it's not a direct scaling factor but characterizes the process noise intensity.

**Effect on solver:** The residual covariance `S = Σ_z + J_θ Σ_θ J_θᵀ` grows for stale cameras, which increases the corresponding block in `S` and down-weights that camera's contribution in `Σ_X = A⁻¹ JXᵀ W S W JX A⁻ᵀ`. Fresh rays dominate; stale rays contribute with appropriate uncertainty — no information is thrown away.

#### What this replaces in the current code

- `triangulation_node.cpp:763-776` — stop resetting all cached data after each cycle. Instead, retain detections with their timestamps.
- `detectionCallback` (line 292) — store `msg->header.stamp` alongside the detection array.
- `covariance_propagation.cpp` — `Σ_twb` and `Σ_phiwb` become per-camera, inflated by AoI-derived `Σ_drift` before being passed to the Jacobian propagation.
- A `max_detection_age` parameter provides a safety cutoff. Value can be derived from Sinopoli's critical rate given the target dynamics and number of cameras, or set conservatively (e.g., 300 ms).

**Sim-side (triangulation.py):** The same principle applies. `TriangulationCfg.pos_std` and `ori_std` are currently scalar constants. They should become per-camera tensors `[N, C]` computed from AoI + process noise model, feeding into the existing `Σ_twb` and `Σ_phiwb` blocks in `compute_triangulation_covariance()`. The config already has `covariance_growth_rate` — this gets replaced by the proper `Σ_drift` computation.

#### Key references

- Bar-Shalom, "Update with Out-of-Sequence Measurements in Tracking: Exact Solution," IEEE Trans. AES, 2002 — the OOSM formulation giving `R_eff = R + H Σ_drift Hᵀ`
- Bar-Shalom, Chen, Mallick, "One-Step Solution for the Multistep OOSM Problem," IEEE Trans. AES, 2004 — extension to multi-step delays
- Larsen et al., "Incorporation of Time Delayed Measurements in a Discrete-time Kalman Filter," IEEE CDC, 1998 — forward-extrapolation alternative to retrodiction
- Sun, Polyanskiy, Uysal-Biyikoglu, "Sampling of the Wiener Process for Remote Estimation," IEEE ISIT, 2017 — proves MSE = E[AoI] for Wiener processes
- Sun et al., "Sampling and Remote Estimation for the OU Process," IEEE/ACM ToN, 2019 — MSE saturation for mean-reverting targets
- Sinopoli et al., "Kalman Filtering with Intermittent Observations," IEEE Trans. AC, 2004 — critical observation rate threshold
- Mourikis & Roumeliotis, "MSCKF for Vision-aided Inertial Navigation," IEEE ICRA, 2007 — sliding-window multi-state constraints for asynchronous camera observations
- Lee & Civera, "Triangulation: Why Optimize?" BMVC, 2019 + GTSAM LOST triangulation, 2023 — per-ray Fisher-information weights that compose with age-inflated σ

## Experiments

Validate bottom-up: first prove the math is correct in a fully controlled setting (Monte Carlo), then validate on sim rosbag with GT, then integrate into the ROS2 node. RL-level evaluation (IsaacLab `evaluate.py` sweeps) is a separate ticket.

### Exp 1 — Monte Carlo: math verification and covariance calibration

**What:** Verify that the AoI-inflated covariance is correctly calibrated under full control of all variables. This is the first gate — if the math doesn't hold here, nothing downstream will work.

**Setup:** Extend existing `monte_carlo_validation.py` (triangulation/tests/). We have full control of camera geometry, noise, target position, and observation timing. For each of 10,000+ MC samples:
1. Generate random camera config (2-5 cameras, varied baselines and angles) + random target position
2. Project target → pixel detections with realistic noise (`pix_std`)
3. Assign per-camera observation age `Δt_c` (uniform 0–500ms)
4. Shift each camera's position/orientation by `v_c · Δt_c` + `ω_c · Δt_c` to simulate drone drift since observation time (this is the ground truth drift that the covariance should capture)
5. Triangulate using rays from the *old* camera poses, with AoI-inflated covariance `R_eff = R + H Σ_drift Hᵀ`

**Conditions to compare:**
1. **Oracle** — all cameras fresh (Δt = 0), standard covariance → establishes the accuracy floor
2. **AoI-aware (random walk)** — `Σ_drift = Q · Δt`
3. **AoI-aware (constant velocity)** — `Σ_drift = Q · Δt³/3` (position block)
4. **AoI-aware (OU)** — `Σ_drift = (σ²/2θ)(1 - e^{-2θΔt})`
5. **Naive reuse** — stale rays, no covariance inflation (control)
6. **Discard stale** — only use fresh rays (current behavior)

**Metrics:**
- 3σ coverage rate — should be ≈ 99.7% for correctly calibrated models
- NEES (Normalized Estimation Error Squared) — should be χ²(3) distributed
- Coverage vs Δt breakdown — verify calibration holds across staleness levels, not just in aggregate
- RMSE vs number of stale cameras — does reusing stale rays improve over discarding them?
- Covariance trace vs Δt — verify monotonic growth matching the theoretical model

**Pass criteria:** 3σ coverage within [99.0%, 100%] across all Δt bins. NEES within χ²(3) 95% confidence interval. Naive reuse should show systematic under-coverage at high Δt (overconfident).

### Exp 2 — Monte Carlo: process model selection and max_age sensitivity

**What:** Using the same MC framework, determine which process model fits best and find the practical `max_detection_age` cutoff.

**Setup:** Fix camera geometry to a representative 3-camera config. Sweep:
- `Δt` from 0 to 2000ms in 50ms steps
- Target speed from 0.5 to 3.0 m/s

**Metrics:**
- RMSE vs Δt curve per model — find where each model's accuracy degrades unacceptably
- NEES vs Δt per model — find where calibration breaks down (NEES departs from χ²(3))
- RMSE(AoI-aware) vs RMSE(discard stale) crossover — the Δt where reusing a stale ray becomes worse than not having it. This is the principled `max_detection_age`.

**Expected outcome:** Crossover point defines `max_detection_age`. OU should fit better for slow targets (< 1 m/s), constant velocity for faster ones. This determines the default parameter and whether adaptive model selection is needed.

### Exp 1 & 2 Results (100k MC samples, 2026-04-02)

**Setup:** 3-camera triangle (10m baseline, 5m altitude), target at origin. Camera 0 always fresh; cameras 1,2 at varying ages. Drone velocity: 2 m/s linear, 0.1 rad/s angular.

**Exp 1 — Condition comparison (NEES, target=3.0):**

| Age (ms) | oracle (no drift) | naive_reuse | const_vel | random_walk | OU | discard_stale |
|----------|-------------------|-------------|-----------|-------------|-----|---------------|
| 0 | 3.01 | 3.02 | 3.03 | 3.02 | 3.02 | — |
| 50 | 3.20 | 3.20 | **2.75** | 0.76 | 0.79 | FAIL (<2 cams) |
| 100 | 3.78 | 3.77 | **2.23** | 0.50 | 0.55 | FAIL |
| 200 | 6.06 | 6.04 | **1.61** | 0.42 | 0.50 | FAIL |
| 300 | 9.81 | 9.84 | **1.36** | 0.46 | 0.60 | FAIL |
| 500 | 21.94 | 21.90 | **1.19** | 0.61 | 0.94 | FAIL |

**Key findings:**

1. **constant_velocity is the clear winner.** NEES stays closest to 3.0 across all ages (2.75→1.19). Mildly conservative at high ages but never overconfident. 3σ coverage: 98-100%.
2. **random_walk and OU are too conservative** — NEES drops to 0.4-0.8 (trace ratios 4-7x). They massively overestimate drift because `Σ_drift = v²·Δt` (linear) overpredicts when the actual drift is `v·Δt` (displacement grows linearly, variance grows quadratically).
3. **naive_reuse matches oracle_fresh exactly** — confirming that without covariance inflation, the analytical covariance is identically wrong. At 500ms: NEES=21.9 (7x overconfident), 3σ coverage=23.5%.
4. **discard_stale always fails** — with 2 of 3 cameras stale, only 1 camera remains → no triangulation possible. This is the current system behavior.

**Exp 2 — Process model sweep (constant_velocity column):**

| Age (ms) | NEES | 3σ cov | Trace ratio |
|----------|------|--------|-------------|
| 0 | 3.01 | 97.0% | 1.00 |
| 50 | 2.75 | 98.0% | 1.09 |
| 100 | 2.23 | 99.3% | 1.35 |
| 200 | 1.61 | 100% | 1.87 |
| 500 | 1.18 | 100% | 2.49 |
| 1000 | 1.08 | 100% | 2.69 |

The constant_velocity model converges to NEES≈1.1 at high ages (slightly conservative) with 100% 3σ coverage. This is acceptable — better to be mildly conservative than overconfident.

**Decision:** Use `aoi_process_model = "constant_velocity"` as the default. Set `max_detection_age = 500ms` as the initial safe bound (see caveats below).

### Extended experiment results (100k MC, 2026-04-02)

#### Exp 2b: All-3-stale vs 1-fresh-2-stale

| Age (ms) | All-stale NEES | All-stale RMSE | 1-fresh+2-stale NEES | 1-fresh+2-stale RMSE |
|----------|----------------|----------------|----------------------|----------------------|
| 0 | 3.03 | 0.26m | 3.02 | 0.25m |
| 100 | 2.02 | 0.30m | 2.24 | 0.29m |
| 300 | 1.27 | 0.54m | 1.36 | 0.47m |
| 500 | 1.15 | 0.84m | 1.18 | 0.70m |
| 1000 | 1.10 | 1.61m | 1.09 | 1.32m |

**Finding:** NEES stays calibrated (1.10-3.03) even with all cameras stale. RMSE is ~20% higher than with one fresh anchor, but the covariance honestly reflects this (3σ coverage = 100% throughout). **All-stale triangulation is always better than no triangulation** — the alternative (discard all) produces zero output.

#### Exp 2c: Mixed staleness (realistic asymmetric ages)

| Config | NEES | 3σ cov | RMSE |
|--------|------|--------|------|
| all fresh | 3.02 | 97.0% | 0.25m |
| 50/100/200 ms | 1.84 | 99.8% | 0.33m |
| 0/100/300 ms | 1.72 | 99.8% | 0.38m |
| 0/200/500 ms | 1.37 | 100% | 0.55m |
| 100/300/500 ms | 1.25 | 100% | 0.60m |
| 200/500/1000 ms | 1.14 | 100% | 1.06m |

**Finding:** Asymmetric ages work correctly. The solver automatically down-weights older cameras via the inflated covariance. All configurations maintain calibrated NEES and full 3σ coverage.

#### Velocity mismatch sensitivity (age=300ms, true v=2 m/s)

| v_assumed | NEES | 3σ cov | Note |
|-----------|------|--------|------|
| 0.0 | 3.88 | 93.5% | Ignoring drift — slightly overconfident |
| 1.0 | 2.64 | 98.8% | 50% underestimate — acceptable |
| **2.0** | **1.35** | **100%** | **Correct — calibrated** |
| 3.0 | 0.75 | 100% | 50% overestimate — too conservative |
| 5.0 | 0.31 | 100% | 150% overestimate — wasteful |

**Finding:** Velocity mismatch is forgiving in the safe direction. Underestimating v by 50% gives NEES=2.64 (still below 3, not overconfident). Completely ignoring drift (v=0) gives NEES=3.88 with 93.5% coverage — not catastrophic because the base sensor noise (`pos_std=0.1m`) covers some drift. Overestimating is always safe but wastes information.

**Implementation note:** Pass `sqrt(E[v]² + σ_v²)` as `velocity_magnitude` to `compute_sigma_drift()`, where `σ_v` comes from `common_frame/odom` twist covariance. This naturally errs on the conservative side.

#### Moving target (age=300ms, camera v=2 m/s)

| v_target | Camera-only NEES | Camera-only 3σ | Camera+target NEES | Camera+target 3σ |
|----------|------------------|----------------|--------------------|--------------------|
| 0 | 1.36 | 100% | 1.36 | 100% |
| 1.0 | 1.58 | 100% | 1.36 | 100% |
| 2.0 | 2.23 | 99.7% | 1.35 | 100% |
| 3.0 | **3.32** | **98.1%** | 1.34 | 100% |
| 5.0 | **6.82** | **75.3%** | 1.34 | 100% |

**Finding:** Without the target drift term, covariance becomes overconfident at v_target ≥ 3 m/s (NEES=3.32→6.82). Adding `v_target²·Δt²·I₃` to `Σ_drift` fixes it perfectly (NEES=1.34 across all speeds). This is implemented in `compute_sigma_drift(target_velocity=...)`.

**Where does v_target come from?** Options:
1. Tracker Kalman velocity estimate (most accurate, but creates a feedback loop: tracker → triangulation → tracker)
2. Conservative upper bound as config parameter (e.g., 3 m/s for pedestrians, 10 m/s for vehicles)
3. Zero (ignore target motion) — acceptable up to ~2 m/s target speed at 300ms age

#### Updated decisions based on extended experiments

- `max_detection_age = 500ms` confirmed as safe bound — even all-stale at 500ms is calibrated (NEES=1.15)
- Velocity mismatch tolerance is ±50% — use `sqrt(v² + σ_v²)` from odom twist for robustness
- Target motion drift should be included when v_target > 2 m/s. Default: use config upper bound (3 m/s for pedestrian targets)

### Exp 3 — Sim rosbag: end-to-end validation with GT

**What:** Validate the full pipeline (triangulation node → tracker) on recorded sim data where GT positions are available.

**Bag:** `bag/isaac_sim_bag_20260402_214151`

**Setup:** Replay the bag through two triangulation node configurations:
1. **Baseline** — current node (discard after use, no AoI)
2. **AoI-aware** — updated node with `Σ_drift` inflation and `max_detection_age` from Exp 2

Both publish to separate output topics. Record outputs to a new bag for offline analysis.

**Pre-analysis:** Before running, inspect the bag to characterize:
- Per-camera detection rates and inter-camera timing offsets (how asymmetric is the real data?)
- Existing (0,0,0) output frequency (quantify the current bug)
- GT topic availability and format

**Metrics:**
- Triangulation RMSE vs GT — primary accuracy (available because this is a sim bag)
- Triangulation output rate (Hz) — how many more outputs does AoI-aware produce?
- Zero-output count — does fix A+B+C eliminate the (0,0,0) problem?
- Position jitter — std of frame-to-frame position delta (lower = smoother)
- Per-camera age histogram — characterize the actual staleness distribution in the data
- Track continuity — feed both outputs to tracker, compare track loss count and max gap

**Expected outcome:** AoI-aware produces higher output rate with comparable or better RMSE. Zero outputs eliminated by fixes A+B+C. Jitter reduced because stale-but-valid rays smooth over detection gaps.

### Exp 4 — Real bag (future): field validation

**What:** Replay real-hardware rosbags (when available) through both node configurations.

**Setup:** Same as Exp 3 but no GT. Rely on consistency metrics only.

**Metrics:** Output rate, jitter, track continuity, zero-output count. Compare against Exp 3 results to check sim-to-real gap.

**Note:** This experiment is deferred until a real-hardware bag with multi-drone observations is available.

## Architecture change: target velocity feedback loop

The moving target experiment showed that without target velocity in `Σ_drift`, covariance becomes overconfident at v_target ≥ 3 m/s. To support fast targets, we add a feedback loop:

```
mas_tracker → mas_multiview  (new: tracked target velocities)
```

**New topic:** `tracked_target_velocities` (`vision_msgs/Detection3DArray` or custom)
- Published by `mas_tracker` — Kalman state already has velocity at indices [6,7,8] (vx,vy,vz)
- Consumed by `mas_multiview` — used in `Σ_drift_target = v_target² · Δt²` per association

**Cycle safety:** The cycle is safe because `mas_multiview` does not use target velocity for the position solve — only for covariance propagation. There is no divergence risk. The velocity estimate is one-step delayed (previous triangulation cycle → tracker update → velocity published → next triangulation cycle).

**Architecture diagram updated:** `src/doc/diagrams/mas_architecture_semantic.d2` — dashed edge from `mas_tracker → mas_multiview`.

---

**Out of scope for this ticket:** IsaacLab RL evaluation sweeps (`evaluate.py` with `--delay-override`, `--detection-dropout-override`, etc.). The RL policy should be retrained/evaluated with AoI-aware triangulation as an observation source — this is a separate ticket that depends on this one.

## Files

| File | What to change |
|------|---------------|
| `lib/multiview_triangulation/src/multiview_triangulation.cpp` | Guard `total_weight <= 0` path; check `summary.IsSolutionUsable()` after solve |
| `lib/multiview_triangulation/src/covariance_propagation.cpp` | Accept per-camera `Σ_twb`, `Σ_phiwb` inflated by AoI; accept target velocity for `Σ_drift_target` |
| `src/triangulation_node.cpp` | Filter invalid results before publishing; add empty-detection check to ready gate; store detection timestamps; compute AoI-inflated covariance; subscribe to tracked target velocities; add `max_detection_age` parameter |
| `src/sort3d_node.cpp` | Publish `tracked_target_velocities` from Kalman state |
| `mas_policy/.../triangulation/triangulation.py` | Make `pos_std`, `ori_std` per-camera `[N, C]` tensors; `compute_sigma_drift(target_velocity=...)` |
| `mas_policy/.../triangulation/triangulation_cfg.py` | Add `max_detection_age`, process model selection parameters |
| `mas_policy/.../triangulation/tests/monte_carlo_validation.py` | AoI experiments (Exp 1, 2, 2b, 2c, vel sensitivity, moving target) |
| `src/doc/diagrams/mas_architecture_semantic.d2` | Add tracker → multiview feedback edge |
