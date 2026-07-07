# EKF Variants — Benchmark on 2-Lap Waypoint Mission

Tests three target-localization filter variants against the recorded bag
`mas/bag/bag_20260622_164718_bearing_loc_test` (70 s, PX4 waypoint mission,
≈2 laps of ~17 m-radius orbits around a near-stationary balloon target at
~3.5 m relative altitude).

## TL;DR

| Filter | Outcome |
|---|---|
| **DC-EKF** (18D paper-faithful, delay-compensated) | **Diverges → NaN by t ≈ 30 s** |
| **Vanilla EKF** (18D, delay compensation disabled) | **Diverges → NaN by t ≈ 30 s** |
| **SimpleEKF** (6D bearing-only target tracker) | **Stable for the full 70 s.  Median error 1.19 m, max 4.88 m** |

The "vanilla" run isolated delay compensation as the suspected source of
divergence — but the result shows both 18D variants fail identically, so
delay compensation is **not** the culprit.  The 18D paper-faithful design
itself is unstable on this regime.

## Why does the 18D filter blow up here when it tracked the short bag fine?

The paper's design keeps the normalized image feature `p̄ = (x/Z, y/Z)` in
state and propagates it through the IBVS interaction matrix `L_s`.  `L_s`
has `1/Z` in three columns:

```
L_s = ┌ −1/Z   0    x/Z    x·y     −(1+x²)   y    ┐
      └  0   −1/Z   y/Z   1+y²     −x·y      −x   ┘
```

The cross-covariance `Cov(p̄, v_r)` that lets a pixel residual reach the 3D
state is built up in the prediction Jacobian's `∂p̄/∂v_r = dt · L_s[:,:3] · R_w^c`
term — and that whole term scales as `1/Z`.

In short-range, low-bearing-rate scenes (the short bag), Z is roughly
constant and well-known.  The cross-covariance scaling is consistent, the
Kalman gain distributes innovations correctly, and the filter tracks.

In long-running, high-bearing-rate scenes (the 2-lap waypoint), depth Z
varies rapidly as the orbit sweeps the field of view, and any small Z
error multiplies into a *wrong* cross-covariance.  A positive feedback
loop forms:

```
bad Z   →   wrong scaled L_s   →   wrong cross-cov   →   wrong Kalman gain
                                                     ↓
                                     wrong v_r and p_r updates
                                                     ↓
                                                worse Z
```

By ≈30 s the covariance is no longer positive-definite, `K · y` becomes
ill-conditioned, and the state goes to NaN.

The fix was *not* delay compensation, attitude override, or noise
re-tuning — all of which were tried and made marginal difference.  The
fix is to **stop putting the image feature in state**.

## What SimpleEKF does instead

State: `[ p_target (3), v_target (3) ]` in world ENU.  Aircraft pose and
velocity come straight from `common_frame/odom`; gimbal + camera_info give
the camera-to-world rotation.  The measurement model is a pinhole
projection:

```
h(x) = π( R_c_w⁻¹ · ( p_target − p_camera_world ) )
     = ( X/Z , Y/Z )

H = ∂h/∂p_target   (closed-form 2×3 Jacobian)
∂h/∂v_target = 0
```

No image feature in state.  No IMU integration.  No quaternion error state.
No delay compensation.  The 6×6 covariance updates cleanly under the
standard EKF predict + Joseph-form update.  Range observability still
comes from the same `v_perp / λ̇` geometry — but now it flows through the
projection Jacobian's well-conditioned 2×3 form, not through the IBVS
interaction matrix's `1/Z`-amplified cross-coupling.

## Quantitative comparison (1-second time bins)

| t-bin [s] | DC-EKF mean err [m] | Vanilla mean err [m] | SimpleEKF mean err [m] |
|---|---|---|---|
| 0 – 10   |  6.25 → NaN   |  6.21 → NaN   | **0.71** |
| 10 – 20  | 27.29 → NaN   | 27.80 → NaN   | **1.92** |
| 20 – 30  | 66.48 → NaN   | NaN           | **1.42** |
| 30 – 40  | NaN           | NaN           | **0.45** |
| 40 – 50  | NaN           | NaN           | **2.19** |
| 50 – 60  | NaN           | NaN           | **1.63** |
| 60 – 70  | NaN           | NaN           | **0.72** |

SimpleEKF's overall:  mean 1.29 m, median 1.19 m, p90 2.44 m, p99 3.50 m,
max 4.88 m.

## Decision

Promote `SimpleEKF` to the production single-agent bearing-only baseline.
Keep `DCEKF` in the package for two reasons:

1. **Paper reproduction**: it's a faithful (within the deviations
   documented in `CONTEXT.md`) implementation of Liu et al. 2026 §III-E,
   and the comparison is informative for the manuscript.
2. **Maneuvering target experiments**: the 18D design's first-order
   IMU integration is, in principle, the right tool for predicting
   short-horizon target state across a measurement gap; we expect it to
   beat the 6D constant-velocity model when target acceleration is
   genuinely non-zero and well-modeled.  We haven't shown that here.

For production use today on stationary or quasi-stationary targets, use
SimpleEKF:

```bash
ros2 run mas_bearing_loc simple_ekf_node --ros-args -r __ns:=/px4_1 \
    -p target_class_name:=drone -p init_range_guess:=15.0 \
    -p sigma_target_acc:=0.3 -p sigma_pix:=5.0
```

Or to run all three side-by-side (each publishes under its own prefix
`bearing_loc/`, `vanilla_loc/`, `simple_loc/`):

```bash
ros2 launch mas_bearing_loc compare_ekfs.launch.py vehicle:=px4_1 \
    init_range_guess:=15.0 sigma_target_acc:=0.3 sigma_pix:=5.0
```

## Files

- `mas_bearing_loc/dc_ekf.py`, `dc_ekf_node.py` — 18D paper-faithful DC-EKF.
  `disable_delay_compensation:=true` runs it as a "vanilla" 18D EKF
  (no rewind/replay).
- `mas_bearing_loc/simple_ekf.py`, `simple_ekf_node.py` — minimal 6D
  bearing-only target tracker.
- `launch/compare_ekfs.launch.py` — runs all three in a single launch.
- `doc/DC_EKF_VS_LOS_RATE_METHOD.md` — comparison of DC-EKF against the
  `d = v_tan / λ̇` direct-inversion method from KARI dronecop.
- `doc/DC_EKF_PERFORMANCE_NOTES.md` — analysis of the paper's claimed
  performance vs. what's actually validated.
