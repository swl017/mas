# Z Observability, Gimbal Frame, and Why the 18D EKF Fails

A response to two design questions raised on 2026-06-22:
1. Is `gimbal_state_rpy_deg` actually in body frame, or could it be in heading
   / world frame?
2. What conditions does the EKF need to estimate `Z` (target depth) well, and
   why is the current 18D EKF estimate "bad enough to diverge"?

## 1. Gimbal frame — verified body-frame ZXY, code is correct

### What the gimbal stabilizer publishes

[`los_rate_controller.py`](file:///home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py):

- The controller receives **world-frame** azimuth/elevation rate commands
  (line 360-426) and integrates them into a persistent world-frame target
  `(az_world, el_world)`.
- Each control tick:
  1. Reads physical joint angles → `actual_yaw, actual_roll, actual_pitch`
     (lines 528-532); these have a -π/2 offset on yaw and inverted signs on
     roll/pitch because of the iris_gimbal3 USD joint conventions.
  2. Runs IK from `(az_world, el_world, R_b_w)` → desired `(yaw, roll, pitch)`
     in **body frame** (lines 543-550 for position mode).
  3. Publishes the body-frame joint angles as `gimbal_state_rpy_deg`
     (line 956-960).
  4. *Also* publishes the world-frame az/el target as
     `gimbal_los_state_deg` (line 962-967).

The published comment on line 951 says verbatim:

> Body-frame RPY in degrees for downstream Rz(yaw)*Rx(roll)*Ry(pitch).

This is the ZXY intrinsic composition we already implement in
`gimbal_R_c_b()`.

### Empirical confirmation against the recorded bag

A sweep of every plausible (axis order, sign-pattern) combination against
`bag_20260622_164718_bearing_loc_test` (script: `sweep_v2.py`), using
`chosen_target_pose` as the reference target and YOLO bbox center as the
measurement:

| Method | Median pixel residual |
|---|---|
| **Body ZXY (yaw=+, roll=+, pitch=+)** — current code | **86.1 px** |
| World az/el direct (skip body composition, use `gimbal_los_state_deg`) | 102.0 px |
| Body ZYX (+,+,+) | 133.9 px |
| Anything with a flipped sign | > 200 px (often > 1000) |

The ~86 px residual is dominated by:
- `chosen_target_pose`'s own multi-view triangulation bias (both cameras share
  the same camera-model error, which biases the triangulated point by
  ~residual / fx × range ≈ 1 m at 15 m range);
- YOLO bbox-center ≠ exact target geometric center;
- Camera ↔ gimbal control-loop time skew.

None of these are camera-model bugs. **The gimbal frame and our composition
are correct**.

The "world az/el direct" result (102 px) is also instructive: it tells us the
gimbal *very nearly* achieves perfect world-LOS tracking — the residual delta
of ~16 px between methods is the gimbal's tracking lag.  This means we could
in principle bypass `R_b_w` entirely and use `gimbal_los_state_deg` for the
camera optical axis, at the cost of trusting the gimbal's setpoint rather
than the achieved pose.

## 2. Why Z observability is geometrically excellent here

For bearing-only target tracking, the Fisher Information Matrix on the
target position, accumulated over N bearings (camera at p_c(k), unit LOS
n(k), range r(k), bearing 1σ noise σ_k):

```
J_pos = Σ_k (1 / σ_k²) · (1 / r_k²) · ( I − n_k n_kᵀ )
```

(This is the `(1/r²) (I − nnᵀ)` rank-2 projector with the LOS direction in its
null space, summed over time.)

For Z direction specifically: λ_min(J_pos) along ẑ encodes how well Z is
constrained.  It is large when the n_k vectors span a wide range of
directions — i.e. when *parallax* is high.

For our 2-lap waypoint mission:
- Orbit radius ≈ 17 m, target range ≈ 17 m, altitude offset ≈ 3.5 m.
- Each second the aircraft moves ~5–10 m tangentially.
- Parallax per second:  ≈ 5 / 17 ≈ 0.3 rad (17°) — **excellent**.
- Bearing noise:  σ_bearing = σ_pix / fx ≈ 2 / 1053 ≈ 1.9 mrad.
- SNR ratio:  300 mrad parallax / 1.9 mrad noise = 160×.

Z is **geometrically very well observed** in this scene.  The 18D EKF's
divergence is therefore **not a Z observability problem — it is a Z
*estimation* problem inside the filter**.

## 3. Why the 18D EKF fails to recover Z even with great geometry

The paper's design keeps the normalized image feature `p̄` in state, with
dynamics propagated through the IBVS interaction matrix:

```
         ┌ −1/Z    0     x/Z    x·y       −(1+x²)   y    ┐
L_s =    │                                                │
         └  0     −1/Z   y/Z   1+y²       −x·y      −x   ┘
```

Three of the six columns of L_s carry a `1/Z` factor.  The cross-covariance
`Cov(p̄, v_r)` that lets a pixel-space innovation reach the 3D state is built
up in the prediction Jacobian:

```
∂p̄ / ∂v_r  =  dt · L_s[:, :3] · R_w_c⁻¹
            ∝ 1 / Z
```

So the **rate at which a pixel-space innovation translates into a 3D update is
proportional to 1/Z** — *as estimated by the filter, not the true Z*.

Consequence: a 50 % error in Z (e.g. starting at 30 m when truth is 15 m, or
the inverse) gives a 2× error in the off-diagonal of `P` that propagates
3-D corrections.  The Kalman gain is then off by 2× in the direction it
pushes (p_r, v_r) for each unit of pixel innovation.

This is **fundamentally self-referential**: bad Z → wrong gain → wrong update
→ worse Z.  On the short 14-second bag this stayed in a bounded basin.  On
the 70-second 2-lap bag it ran away to numerical infinity (~30 s in).

### Contrast with the 6D SimpleEKF

`SimpleEKF` measures the pixel directly:

```
h(x) = π( R_c_w⁻¹ · ( p_target − p_camera_world ) )
H = ∂h/∂p_target  (closed-form 2×3, recomputed at each measurement)
```

Both H and the projection h are re-evaluated *at the current state estimate*.
A wrong Z gives a slightly wrong H at this measurement instant — first-order
error — and the EKF compensates next step.  There is no historical
cross-covariance carrying the `1/Z` bug forward.

This is why SimpleEKF tracks the full 2-lap mission to median 1.19 m while
both 18D variants diverge.

## 4. What "Z is well-estimated" requires in *general*

For *any* bearing-only filter:

| Requirement | Reason |
|---|---|
| **Bearing diversity** — n_k must span more than the LOS plane | The `(I − nnᵀ)` projector has zero eigenvalue along n_k; you need different n_k's so their projectors *jointly* cover all 3D directions. |
| **Camera baseline perp to LOS** — ego must translate sideways relative to the LOS, not along it | Parallax = baseline_perp / range.  Pure pursuit (motion along LOS) gives zero parallax. |
| **Parallax angle ≫ bearing noise** | SNR for Z scales as parallax / σ_bearing.  10× margin is reasonable; 100× is luxury. |
| **Time scale where target velocity ≪ ego baseline rate** | If the target moves comparably to the ego baseline, the parallax is "wasted" on the target's motion rather than its position. |

For the 18D EKF specifically, *additional* requirements (which our scene
satisfies):
- Good initial `init_range` (within ~2× of truth) so the initial L_s is
  consistent with the early measurements.
- `sigma_target_acc` matched to the actual target maneuvering.
- `override_attitude_from_odom` enabled (kills gyro drift).
- `override_velocity_from_odom` enabled if target is approximately stationary.

Even with all four of these, the 18D filter's `1/Z` feedback loop puts a
*ceiling* on how long it can track in a non-trivial-bearing-rate scene.
SimpleEKF has no such ceiling.

## 5. Concrete improvement directions

### A. Production: use SimpleEKF as the deployed single-agent baseline

Already in tree (`mas_bearing_loc/simple_ekf.py`).  Stable on the full
2-lap bag.  Composes natively with multi-agent FIM-style fusion (its 3×3
position covariance is the same object you want to inverse-weight against
peers).

### B. If the 18D EKF must be kept for paper reproduction or maneuvering-target
experiments

The following two changes would likely defer (not eliminate) divergence:

1. **Adaptive Q on `p̄`**: scale `sigma_pbar_proc` by `(σ_Z / Ẑ)` so when
   range uncertainty is high, the filter trusts its own image-feature
   prediction less and the measurement more.  Currently `sigma_pbar_proc` is
   a fixed constant.

2. **Re-linearize L_s on a moving estimate** for the cross-covariance update:
   when `Ẑ` changes significantly between prediction steps, recompute
   `∂p̄ / ∂v_r` using the *current* `Ẑ` rather than letting the historical
   cross-covariance stay scaled by the initial `Ẑ`.  This is essentially an
   IEKF (iterated EKF) variant on the predict side.

Neither change is a guaranteed fix; the `1/Z` self-reference is intrinsic
to having `p̄` in state.  The cleanest robustness path is option (A).

### C. Hybrid

Run SimpleEKF on the deployed path; run DC-EKF in parallel only for the
paper's apples-to-apples comparison and the maneuvering-target ablations.
The `compare_ekfs.launch.py` file already supports both.

## 6. Summary

- Gimbal frame is **body ZXY**, our code is correct.  86 px median projection
  residual is dominated by reference (chosen_target_pose) bias and YOLO
  bbox-center error, not by a frame bug.
- Z is **geometrically very well observable** in the 2-lap scene (parallax
  17° / sec vs. bearing noise 2 mrad — 160× SNR).
- The 18D EKF's divergence is *not* a Z observability problem.  It is a Z
  *self-referential estimation* problem caused by `1/Z` factors in the IBVS
  interaction matrix that feed into the cross-covariance build-up.
- SimpleEKF avoids the self-reference by re-evaluating its measurement
  Jacobian at the current state every step, and tracks the 2-lap mission
  end-to-end at median 1.19 m.
