# DC-EKF Performance: What the Paper Actually Validates

Notes on Liu et al. 2026 (arXiv 2606.10639v2), §III-E and §IV-B.  Written from
the perspective of "we are about to use this as a baseline; how good is it,
really?"

## Claimed numbers

| Quantity | Value | Source |
|---|---|---|
| Vision-loop latency | 0.13 – 0.16 s | §III-E para 1 |
| Position shift at 8 m/s without delay-comp | 1.2 m | §III-E para 1 |
| EKF look-ahead vs measurement arrival | 0.1 – 0.2 s | §IV-B Fig. 10 caption |
| Interception success, static target | 90 % (9 / 10) | Table I |
| Interception success, dynamic target | 71 % (16 / 21), max 138 m | Table I |

## What Fig. 9 / Fig. 10 actually shows

- **Fig. 9 (a, b)**: Bird's-eye views of two intercepts.  No filter residuals.
- **Fig. 9 (c, d)**: Distribution of *image-feature* hits in the FOV across the
  episode.  Bearing-domain, not 3-D.
- **Fig. 9 (e, f)**: *Ego* velocity-trajectory profiles, not target estimates.
- **Fig. 9 (g, h)**: *Ego* attitude variation.
- **Fig. 10**: Image-plane `u, v` of YOLOv5s detections (blue) vs EKF estimate
  (red), over time.  The story is: EKF fills frame-drop gaps and leads
  measurement arrival by ~0.15 s.

## What is *not* in the paper

There is **no quantitative 3-D target-state error plot anywhere in the paper**.
No position-error vs. time, no range-error histogram, no velocity-error
comparison vs ground truth.  Interception success is the headline; estimate
quality is implicit.

## Why this matters for our work

Bearing-only on a single moving observer is a classic TMA problem in which
**range lies in the radial null space of the bearing measurement**
(`RESEARCH_CONCEPT.md §4`).  Range becomes observable only through
ego maneuver, an assumed motion model, or both:

1. **On a constant-bearing collision course**, `λ̇ → 0` is exactly the
   condition under which time-stacked bearing measurements *cannot* fill the
   radial null space.  The hit still occurs (this is the LOS-rate argument and
   the geometric reason proportional navigation works), but range stays
   weakly conditioned.

2. **In the paper, the filter does not have to deliver good range** to close
   the loop, because the loop is closed in *image space* (Eq. 27 keeps the
   normalized image feature `p̄` in state, and the PS-LOS law tracks `n_t`).
   The control law's objective — drive `|n_hd^T n_t| ≤ c_h` — is bearing-domain,
   not position-domain.

3. **Therefore the 90 % / 71 % success number tells you:** "image-space
   tracking is smooth enough through delay and frame drops that the IBVS-PN
   loop converges most of the time."  It does *not* tell you:
   - How accurate the 3-D position estimate is at the moment of impact.
   - How well the filter would deliver a *predicted intercept point* for a
     time-of-flight effector (net, projectile) — a task that genuinely needs
     range.
   - How the filter holds up across track re-acquisitions, FOV exits, or
     handoffs to a second drone.

## How this implementation reproduces the paper faithfully

- Full 18D state `[q, p_r, v_r, p̄, b_g, b_a]` as in Eq. 8.
- IMU-rate prediction (`mavros/imu/data`) with bias correction
  (`a_corr = a − b_a`, `ω_corr = ω − b_g`).
- Image-feature dynamics propagated through the IBVS interaction matrix `L_s`
  (Eq. 7), keeping the dominant `∂p̄/∂v_r` cross-coupling that lets the
  measurement update flow information back to the 3-D state.
- Delay compensation per Alg. 2: IMU ring buffer + snapshot rewind + forward
  replay.

## How this implementation lets us measure what the paper hid

Once a ground-truth target trajectory is available (Isaac Sim, or a
chase-cam-equipped run), the following are directly extractable from the
published topics:

| Metric | From topic | Compare against |
|---|---|---|
| Target position error vs t | `bearing_loc/target_pose.position − GT` | `mas_multiview/triangulated_points − GT` |
| Target velocity error vs t | `bearing_loc/target_twist − GT v` | (same, cooperative) |
| Range-error 1σ | `cov_position` trace, plus residual analysis | `mas_tracker/chosen_target_pose.covariance` |
| Image-plane look-ahead | `image_feature_pred` lead time over `image_feature_meas` | Reproduce Fig. 10 |
| Reaction to track loss | innovation history during YOLO frame gaps | Cooperative belief-sharing baseline |

The expected outcome — to be verified — is that the bearing-only filter is
**smooth in image space** (matching the paper) **while being a lot worse in
3-D** (the paper's blind spot).  That gap is the quantitative argument for the
cooperative-triangulation contribution.

## Camera-in-body lever arm

The paper sets `t_b^c = 0`: the camera optical center coincides with the body
frame origin.  For the **IrisGimbal3** asset actually used here
(`PegasusSimulator/.../iris_gimbal3.usda`, wired by
`px4_multi_world.isaac.py:207`), the kinematic chain places the camera at
`(0.0, -0.10, 0.12) m` in body-FLU at zero gimbal pose:

| Stage | Anchor in parent (m) | Cumulative pos in body |
|---|---|---|
| yaw_joint on body | (0.0, -0.1, 0.15) | — |
| yaw_link origin | (after `-R_yaw @ (0,0,0.01)`) | (0.0, -0.10, 0.14) |
| roll_joint on yaw_link | (0.0, 0.04, -0.02) | (0.0, -0.06, 0.12) |
| roll_link origin | (0, 0, 0) | (0.0, -0.06, 0.12) |
| pitch_joint on roll_link | (0.0, -0.04, 0.0) | (0.0, -0.10, 0.12) |
| **pitch_link (camera) origin** | (0, 0, 0) | **(0.0, -0.10, 0.12)** |

Implementation choice: treat the offset as **constant** (set via the
`t_cam_in_body` config field / node param).  The actual offset shifts by
≤ ~5 cm as the gimbal joints sweep their full range — at 30–100 m target
ranges this is < 1 mrad of bearing error, well below pixel noise.  If
sub-pixel accuracy at close range matters, swap in a real forward-kinematics
helper that consumes `gimbal_state_rpy_deg`.

For comparison, IrisGimbal2 (older variant) has the camera at
`(0.10, 0.0, 0.12)` — forward-mount.  Setting the parameter to that value
recovers the older platform.

## Environment assumptions — *confirmed* (2026-06-22)

1. **`mavros/imu/data` is specific force, gravity INCLUDED.** Hover sample:
   `linear_acceleration.z ≈ +9.84 m/s²`.  The implementation assumes this
   exact convention: `a_world_inertial = R_b_w @ (a_imu − b_a) + g_world`
   with `g_world = [0, 0, −9.81]` (ENU z-up).

2. **`gimbal_state_rpy_deg` is body-frame RPY, ZXY intrinsic
   (Rz(yaw)·Rx(roll)·Ry(pitch)).**  Confirmed at
   [`los_rate_controller.py`](/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py#L948-L960)
   `_publish_state`:
   > "Body-frame RPY in degrees for downstream `Rz(yaw)*Rx(roll)*Ry(pitch)`."
   Message field mapping: `msg.x = roll`, `msg.y = pitch`, `msg.z = yaw`.
   The implementation uses `quaternion.rpy_zxy_to_rot` and assumes the
   gimbal-zero frame coincides with body-FLU (camera optical axis along
   body +X at zero gimbal).

3. **`camera/zoom_level` is a multiplicative zoom level in [1.0, 6.0].**
   `ZoomDynamics` in
   [`dynamics.py`](/home/usrg/IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/dynamics.py#L141-L228)
   integrates `levels/s` into a magnification level (post-lag + post-quant for
   the SIYI A8 model).  The implementation assumes the published
   `CameraInfo.K` corresponds to the `zoom = 1.0` baseline, so
   `fx_eff = fx_baseline × zoom_level`.  If `CameraInfo.K` is recomputed
   inside the camera driver as zoom changes, the multiplier becomes a no-op
   and `zoom` must be forced to 1.0 — to be confirmed when first running
   against a live `rtsp_camera`.
