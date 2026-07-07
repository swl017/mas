# DC-EKF vs `d = v_tan / λ̇` (LOS-Rate Direct Method)

A side-by-side comparison of the current paper-faithful DC-EKF in
`mas_bearing_loc` against the prior LOS-rate direct-inversion method in
`kari_dronecop_ros_pd` ([observation_guidance.cpp:330-459][og],
[los_rate_observation.cpp:84-116][losobs]).

[og]: file:///home/usrg/kari_pd_ws/src/kari_dronecop_ros_pd/kari_dronecop_pursuit/tracking_guidance/src/observation_guidance.cpp
[losobs]: file:///home/usrg/kari_pd_ws/src/kari_dronecop_ros_pd/kari_dronecop_pursuit/tracking_guidance/src/observation/los_rate_observation.cpp

Both methods exploit the same kinematic identity:

```
        v_perp
ḋ ≈ 0,  ───── = λ̇       (target stationary, ego in lateral motion)
          d

⇒        d = v_perp / λ̇
```

The difference is in **how** they invert it:
*algebraic 1-shot* vs *recursive Bayesian update*.

---

## 1. Prior method — LOS rate, closed-form, then post-smooth

### Pipeline

1. Take YOLO bearing across two frames → numerical derivative `λ̇`.
2. Guard `|λ̇| ≥ 0.005` (singularity dodge).
3. Compute `d = v_tan / λ̇`, low-pass filter `d`.
4. Build 3D body-frame point:
   `(x, y, z) = (d, d·sin(β_img), d·sin(ε_img))`.
5. Rotate through gimbal pose + ego pose → world frame.
6. Optional: feed `(x, y, z)` plus its naive numerical-difference velocity into
   a 6D EKF as a *smoother*. The EKF here does NOT do the range inversion; it
   just denoises an already-computed Cartesian estimate.

### Mathematical model

- State (smoother EKF): `[x, y, z, vx, vy, vz]` (6D).
- Process: constant-velocity (no IMU integration in this 6D filter).
- Measurement: the closed-form `(x, y, z)` from step 5.
- Range observability lives **entirely** in step 3 — outside the filter.

### Strengths

- Crisp and interpretable: one ratio gives range.
- O(1) per step. No covariance bookkeeping.
- The range estimate is *instantaneously available* the moment the ratio is
  computed (no convergence transient).
- Easy to debug: print `v_tan`, `λ̇`, `d` and the geometry tells you what's
  wrong.

### Weaknesses

- **Numerical differentiation of bearing.** YOLO pixel noise gets amplified
  going through `λ̇ = Δλ/Δt`. Practical implementations lean hard on the
  low-pass filter; latency-vs-noise trade-off is yours to tune.
- **Singularity at `λ̇ → 0`.** The hard guard `|λ̇| ≥ 0.005` simply *stops
  updating* when bearing rate is small — i.e., precisely on a collision
  course or while standing still. Outside the guard window the filter is
  effectively blind.
- **Assumes target stationary.** A maneuvering target breaks the
  `d = v_perp / λ̇` identity; there is no clean way to inject target
  acceleration uncertainty.
- **No delay handling.** What you compute is what you publish, at "now"
  according to your wall clock; if YOLO is 150 ms behind, the position is
  150 ms stale.

---

## 2. Current method — DC-EKF, recursive, IBVS-based

### State (18D, paper-faithful)

```
x = [ q        (4)   aircraft attitude quaternion
      p_r      (3)   relative position p_aircraft − p_target, ENU
      v_r      (3)   relative velocity, ENU
      p̄        (2)   normalized image feature x/Z, y/Z
      b_gyr    (3)   gyro bias
      b_acc    (3)   accelerometer bias ]
```

Error-state covariance is 17×17 (the quaternion contributes 3D `δθ`).

### Predict step (per IMU sample, body-frame ω, a)

```
ω_corr  = ω − b_gyr
a_corr  = a − b_acc
q       ← δq(ω_corr · dt) ⊗ q              # small-angle quaternion update
R_b_w   = q_to_R(q)
a_world = R_b_w · a_corr + g_world          # g_world = (0, 0, −9.81) ENU
p_r     ← p_r + v_r·dt + 0.5·a_world·dt²
v_r     ← v_r + a_world·dt                  # target accel absorbed by Q
R_c_w   = R_b_w · R_c_b(gimbal)
v_cam   = R_c_w⁻¹ · v_r
ω_cam   = R_c_b⁻¹ · ω_corr
Z       = (R_c_w⁻¹ · −p_r)[2]                # target depth in camera frame
p̄       ← p̄ + L_s(p̄, Z) · [v_cam ; ω_cam] · dt
```

The interaction matrix `L_s` is the standard IBVS form:

```
        ┌  −1/Z   0     x/Z    x·y       −(1+x²)   y    ┐
L_s  =  │                                                │
        └   0    −1/Z   y/Z   1+y²       −x·y      −x   ┘
```

with `(x, y) = p̄`. **Note that `Z` lives in the denominator of the first
three columns** — exactly the place where the LOS-rate `d = v_perp / λ̇`
identity is encoded. Same physics, different syntax.

### Update step (delayed image feature arrives)

Measurement model: direct read of `p̄` from state.

```
H  =  [0 … I₂ … 0]          # selects the p̄ slice (2 × 17)
y  =  z_meas − p̄_predicted
S  =  H · P · Hᵀ + R_pix
K  =  P · Hᵀ · S⁻¹
δx =  K · y
inject δx into nominal state
P  ←  (I − K·H) · P · (I − K·H)ᵀ + K · R_pix · Kᵀ   (Joseph form)
```

The correction reaches `p_r`, `v_r`, even `q` because during *predict* the
covariance grew off-diagonal couplings:

```
∂p̄ / ∂v_r  =  dt · L_s[:, :3] · R_w_c⁻¹
```

So a `p̄`-only residual flows back to `v_r` and `p_r` through the
cross-covariance the Riccati equation has been accumulating. This is the
recursive equivalent of the prior method's `v_perp / λ̇` algebra.

### Delay compensation (paper Alg. 2)

- IMU and `(t, x, P)` snapshots in ring buffers (~0.5 s window).
- When a delayed image arrives with stamp `t_img`:
  1. Find snapshot at the largest `t_snap ≤ t_img`.
  2. Forward-predict snapshot to `t_img` using buffered IMU (small steps).
  3. EKF update at `t_img`.
  4. Replay all IMU from `t_img` to current time.
- This is what makes the filter's published estimate ~0.15 s **ahead** of the
  raw YOLO measurement.

### Strengths

- **No bearing differentiation.** YOLO noise enters as a pixel-space
  measurement covariance, not as derivative noise.
- **No singularity.** When `λ̇ → 0`, `L_s`'s rows just shrink and the
  Kalman gain naturally goes small; `P_pos` grows smoothly instead of the
  estimate blowing up.
- **Maneuvering target supported** via `sigma_target_acc` (unmodeled accel
  PSD). The Q matrix on `v_r` carries it.
- **Delay-compensated output** — publishes filtered estimate ahead of
  measurements.
- **Composes with multi-agent fusion.** Each agent's `P_pos` already
  encodes its bearing geometry; central fusion can do
  inverse-covariance-weighted averaging that reduces to the FIM sum of the
  research-concept §4 formalism.

### Weaknesses

- **More code, more parameters, more covariance bookkeeping.**
- **Range is observed through covariance, not directly.** The estimate has
  a convergence transient governed by the Riccati equation; if Q/R are
  mistuned, divergence is silent (until covariance trace blows up).
- **Linearization of `L_s` and quaternion dynamics.** EKF is first-order;
  if pixel noise σ is comparable to the linearization scale, you start to
  see bias.
- Requires good IMU. The prior method only uses ego velocity at update
  instants.

---

## 3. Side-by-side at a glance

| Aspect | Prior `d = v_tan / λ̇` | DC-EKF (this package) |
|---|---|---|
| Range derivation | Direct algebraic inversion | Implicit through `L_s`'s `1/Z` + Riccati |
| Bearing processing | Numerical derivative → divide → LPF | Statistical absorption (no differentiation) |
| Target model | Stationary (hard-coded) | Constant velocity + bounded accel via `Q` |
| `λ̇ ≈ 0` behavior | Hard singularity, guarded by `|λ̇| ≥ 0.005`, update halts | Covariance gently expands, smooth degradation |
| Output dimension | 6D (Cartesian + velocity) | 18D (attitude, position, velocity, p̄, biases) |
| Image-feature delay | Not handled | IMU backward-update + forward-replay |
| Ego motion required | Lateral / curved (`v_tan ≠ 0`) | Any informative IMU input |
| Failure mode | Diverges / freezes at singularity | Covariance trace inflates smoothly |
| Compute cost | `O(1)` per step | `O(n³)` — 17×17 matrices |
| Tuning surface | LPF time constant, `λ̇` threshold | `Q`, `R`, init covariance, delay window |
| Multi-agent extension | Average / GTSAM pose graph | Inverse-covariance fusion → FIM-sum |

---

## 4. Their fundamental equivalence

In the small-noise, well-conditioned limit, the two methods carry the **same**
information about range. To see it, write out the IBVS row that depends only
on tangential motion (set `x = y = 0`, i.e. target at image center):

```
p̄_dot  =  ( −v_cam_x / Z,  −v_cam_y / Z )
```

`p̄_dot` is exactly the (normalized) bearing rate, and `v_cam_{x,y}` are the
camera-frame tangential velocity components. So:

```
| p̄_dot | = | v_perp | / Z   ⇔   Z = | v_perp | / | p̄_dot |
```

which is the prior method's formula. The DC-EKF effectively *measures*
`p̄_dot` through the *cross-covariance* `Cov(p̄, v_r)` that the predict step
accumulates, then divides by `v_perp` implicitly via the Kalman gain. Same
geometry, recursive arithmetic.

---

## 5. When to prefer which

- **Use the prior method when:**
  - The target is genuinely stationary.
  - You can ensure persistent `λ̇` (curved or circular ego motion).
  - You want minimal code and a quick numerical estimate.
  - Bearing measurements arrive fresh (low YOLO latency).

- **Use the DC-EKF when:**
  - You need to fly *anywhere*, including collision-course / hover.
  - Bearing measurements have non-trivial delay (you want delay
    compensation).
  - The target may maneuver; you want a clean way to inject acceleration
    uncertainty.
  - You're planning multi-agent FIM-style fusion (one drone's covariance
    feeds a centralized estimator).

For the IROS / RA-L cooperative-counter-UAS paper, the DC-EKF is the
right single-agent baseline because:
1. It reproduces the paper's reference method exactly (`Liu et al. 2026,
   §III-E`).
2. Its output already lives in the format multi-agent fusion needs
   (3D pose + 3×3 covariance).
3. Its failure mode (large `P_pos` on collision course) is the
   *quantitative* substrate of the C1 claim — that cooperative baseline
   improves conditioning where single-view degrades.

---

## 6. One-paragraph summary

The **prior method** numerically differentiates the bearing and divides by
the tangential ego speed to invert `d = v_perp / λ̇` in closed form, then
post-smooths the resulting Cartesian point with a 6D constant-velocity EKF.
The **DC-EKF** never differentiates the bearing; instead it predicts the
image feature through the IBVS interaction matrix `L_s` (whose `1/Z`
denominator carries the same range information) and lets the Riccati
equation accumulate cross-covariance between the image feature and the 3D
state, so a pixel-level innovation is *automatically* distributed onto
range and velocity through the Kalman gain. They are mathematically
equivalent in the small-noise limit; the DC-EKF buys you graceful
behaviour at `λ̇ ≈ 0`, delay compensation, a clean way to model target
acceleration, and a covariance output that composes natively with
multi-agent FIM fusion.
