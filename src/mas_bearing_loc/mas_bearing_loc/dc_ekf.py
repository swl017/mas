"""Delay-Compensated EKF (DC-EKF) — reproduction of Liu et al. 2026 §III-E.

State (nominal, 18D):
    x = [ q (wxyz, 4),
          p_r  (relative position p_aircraft - p_target, ENU, 3),
          v_r  (relative velocity, ENU, 3),
          p_bar (normalized image feature x/z, y/z in camera frame, 2),
          b_gyr (gyro bias, body, 3),
          b_acc (accelerometer bias, body, 3) ]

Error state (17D, used for covariance / Jacobians):
    δx = [ δθ (3), δp_r (3), δv_r (3), δp_bar (2), δb_g (3), δb_a (3) ]

The prediction model puts the *image feature itself* in state so that delayed
visual measurements can be applied via a direct readout (H selects δp_bar);
cross-correlations between p_bar and (p_r, v_r, θ) are accumulated through the
prediction Jacobian's image-interaction-matrix coupling.

Delay compensation follows the paper's Algorithm 2: an IMU ring buffer and a
state-snapshot ring buffer make it cheap to roll the filter back to t_img, do
the measurement update there, and replay the IMU forward to the current time.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .camera_model import interaction_matrix
from .imu_buffer import IMUSample, RingBufferIMU, RingBufferSnapshot
from .quaternion import (
    quat_mul,
    quat_normalize,
    quat_to_rot,
    skew,
    small_angle_quat,
)


# Nominal-state index slices
IDX_Q = slice(0, 4)
IDX_PR = slice(4, 7)
IDX_VR = slice(7, 10)
IDX_PBAR = slice(10, 12)
IDX_BG = slice(12, 15)
IDX_BA = slice(15, 18)

# Error-state index slices (17D)
ERR_THETA = slice(0, 3)
ERR_PR = slice(3, 6)
ERR_VR = slice(6, 9)
ERR_PBAR = slice(9, 11)
ERR_BG = slice(11, 14)
ERR_BA = slice(14, 17)

NX = 18
NE = 17
G_WORLD = np.array([0.0, 0.0, -9.81])


@dataclass
class DCEKFConfig:
    # Process noise PSDs (continuous-time).  Applied as Q * dt at each predict.
    sigma_gyr: float = 5e-3          # rad/s/√Hz (gyro angle random walk)
    sigma_acc: float = 5e-2          # m/s^2/√Hz (accelerometer noise)
    sigma_target_acc: float = 1.5    # m/s^2/√Hz (unmodeled target accel)
    sigma_pbar_proc: float = 1e-3    # process noise on image feature
    sigma_b_gyr: float = 1e-4        # gyro bias random walk
    sigma_b_acc: float = 1e-3        # accel bias random walk

    # Measurement noise (in *normalized* image coords).  σ_px / (fx * zoom).
    sigma_pix: float = 2.0

    # --- Stabilization knobs (default off = paper-faithful) ---------------------
    # Floor on the effective normalized measurement 1σ (rad). The node passes
    # sigma_pix/(fx·zoom); at high zoom that underestimates the true angular error
    # (~0.03 rad), over-fitting the filter. Floors the effective σ. 0 = disabled.
    sigma_norm_floor: float = 0.0
    # Minimum |depth| (m) used in the IBVS interaction matrix L_s. The 1/Z terms in
    # L_s drive the p̄↔v_r cross-covariance; when the estimated depth is small or
    # poorly known (high-bearing-rate orbits), that cross-cov is mis-scaled and the
    # filter enters the positive-feedback loop documented in EKF_VARIANTS_BENCHMARK.md
    # (→ NaN by ~30 s). Flooring |depth| caps the 1/Z gain. 0 = disabled.
    depth_floor: float = 0.0
    # Innovation (Mahalanobis) gate: skip the measurement update when y'S⁻¹y exceeds
    # this, keeping the prediction. Rejects outlier detections. 0 = disabled.
    # 2-DOF chi² 99% ≈ 9.2.
    reject_mahalanobis: float = 0.0
    # Covariance PD projection: after each predict/update, clip the eigenvalues of P
    # into [cov_eig_floor, cov_eig_ceil]. The 1/Z IBVS coupling can drive P
    # non-positive-definite (→ NaN by ~30 s, EKF_VARIANTS_BENCHMARK.md); clipping
    # keeps P well-conditioned. 0 = disabled (each independently).
    cov_eig_floor: float = 0.0
    cov_eig_ceil: float = 0.0

    # Initialization
    init_range: float = 30.0         # m, fallback range when first detection arrives
    init_var_pos: float = 100.0      # m^2 (1σ ≈ 10 m on each axis)
    init_var_vel: float = 25.0       # (m/s)^2 (1σ = 5 m/s)
    init_var_pbar: float = 1e-2
    init_var_theta: float = 1e-4
    init_var_bg: float = 1e-6
    init_var_ba: float = 1e-4

    # Camera-in-body translational offset, body-FLU (m).
    # Default = (0, 0, 0): camera optical center coincident with body origin
    # (matches Liu et al. 2026's `t_b^c = 0`).
    # Note: while iris_gimbal3.usda places `pitch_link` at body (0, -0.10, 0.12),
    # an offline check against the recorded bag shows that *adding* this offset
    # INCREASES projection residuals.  The Isaac Sim camera, mounted via
    # `set_local_pose([0,0,0], ...)` on `/pitch_link/camera`, appears to render
    # from near the body origin in practice — likely the joint-driven motion of
    # the link is small at the gimbal pitch/yaw angles seen on this platform.
    # Override per platform if a real hardware mount differs.
    t_cam_in_body: tuple = (0.0, 0.0, 0.0)

    # Buffers
    buffer_window_sec: float = 0.5
    expected_imu_rate_hz: float = 200.0

    # When True, skip the snapshot rewind + IMU replay and apply the image
    # measurement directly to the current (x, P).  Useful as a "vanilla EKF"
    # comparison baseline — exposes any bug or numerical issue introduced by
    # the delay-compensation path.
    disable_delay_compensation: bool = False


class DCEKF:
    def __init__(self, cfg: Optional[DCEKFConfig] = None):
        self.cfg = cfg or DCEKFConfig()
        self.x = np.zeros(NX)
        self.x[0] = 1.0  # identity quaternion
        self.P = np.eye(NE) * 1e-3
        self.t: Optional[float] = None
        self.initialized = False
        self.t_cam_b = np.asarray(self.cfg.t_cam_in_body, dtype=float)

        self.imu_buf = RingBufferIMU(
            window_sec=self.cfg.buffer_window_sec,
            expected_rate_hz=self.cfg.expected_imu_rate_hz,
        )
        self.snap_buf = RingBufferSnapshot(
            window_sec=self.cfg.buffer_window_sec,
            expected_rate_hz=self.cfg.expected_imu_rate_hz,
        )

    # ---------- public API ----------

    def set_attitude(self, q_wxyz: np.ndarray) -> None:
        """Hot-start the aircraft attitude (e.g. from common_frame/odom)."""
        self.x[IDX_Q] = quat_normalize(q_wxyz)

    def override_attitude(self, q_wxyz: np.ndarray) -> None:
        """Replace the integrated quaternion with an external estimate."""
        self.x[IDX_Q] = quat_normalize(q_wxyz)

    def override_relative_velocity(self, v_aircraft_world: np.ndarray) -> None:
        """Pin `v_r` to the aircraft's world velocity (target assumed stationary).

        Like `override_attitude`, this trades strict paper fidelity for
        robustness in long-duration runs where the IMU integration on `v_r`
        would otherwise drift unboundedly.  Disable for true-maneuvering
        targets or when validating IMU-only propagation.
        """
        self.x[IDX_VR] = np.asarray(v_aircraft_world)

    def initialize_from_bearing(
        self,
        t: float,
        bearing_world: np.ndarray,
        q_wxyz: np.ndarray,
        range_guess: Optional[float] = None,
        v_aircraft_world: Optional[np.ndarray] = None,
    ) -> None:
        """Seed p_r, v_r, p_bar from the very first valid bearing.

        bearing_world: unit vector from camera toward target, in world ENU.
        Accounts for the camera-in-body lever arm:
            p_target = p_aircraft + R_b_w @ t_cb + range * bearing_world
            p_r = p_aircraft - p_target = -R_b_w @ t_cb - range * bearing_world
        """
        rng = range_guess if range_guess is not None else self.cfg.init_range
        self.x[IDX_Q] = quat_normalize(q_wxyz)
        R_b_w = quat_to_rot(self.x[IDX_Q])
        self.x[IDX_PR] = -R_b_w @ self.t_cam_b - rng * bearing_world
        # v_r = v_aircraft - v_target.  Assume v_target ≈ 0 at init, so v_r is
        # the *aircraft* world velocity.  If the bag starts mid-flight, seeding
        # this from common_frame/odom prevents a multi-second integrator
        # transient.
        if v_aircraft_world is not None:
            self.x[IDX_VR] = v_aircraft_world
        else:
            self.x[IDX_VR] = 0.0
        self.x[IDX_PBAR] = 0.0
        self.x[IDX_BG] = 0.0
        self.x[IDX_BA] = 0.0
        self.t = t

        P = np.zeros((NE, NE))
        P[ERR_THETA, ERR_THETA] = np.eye(3) * self.cfg.init_var_theta
        P[ERR_PR, ERR_PR] = np.eye(3) * self.cfg.init_var_pos
        P[ERR_VR, ERR_VR] = np.eye(3) * self.cfg.init_var_vel
        P[ERR_PBAR, ERR_PBAR] = np.eye(2) * self.cfg.init_var_pbar
        P[ERR_BG, ERR_BG] = np.eye(3) * self.cfg.init_var_bg
        P[ERR_BA, ERR_BA] = np.eye(3) * self.cfg.init_var_ba
        self.P = P
        self.initialized = True

        self.imu_buf = RingBufferIMU(
            window_sec=self.cfg.buffer_window_sec,
            expected_rate_hz=self.cfg.expected_imu_rate_hz,
        )
        self.snap_buf = RingBufferSnapshot(
            window_sec=self.cfg.buffer_window_sec,
            expected_rate_hz=self.cfg.expected_imu_rate_hz,
        )
        self.snap_buf.push(t, self.x, self.P)

    def predict_imu(self, t: float, omega: np.ndarray, accel: np.ndarray,
                    R_c_b: np.ndarray) -> None:
        """One IMU prediction step.  Pushes snapshot afterward."""
        if not self.initialized or self.t is None:
            return
        dt = t - self.t
        if dt <= 0:
            return
        self.x, self.P = _predict_step(self.x, self.P, omega, accel, R_c_b, dt,
                                       self.cfg, self.t_cam_b)
        self.t = t
        self.imu_buf.push(t, omega, accel)
        self.snap_buf.push(t, self.x, self.P)
        self._prune_buffers()

    def update_bearing(
        self,
        t_img: float,
        p_bar_meas: np.ndarray,
        R_c_b_at_img: np.ndarray,
        sigma_pix_eff: Optional[float] = None,
    ) -> Optional[dict]:
        """Apply an image-feature measurement.

        If `cfg.disable_delay_compensation` is False (default), the snapshot
        rewind + IMU replay path of Liu et al. 2026 Alg. 2 is used.  Otherwise
        the measurement is applied directly to the current `(x, P)` — a
        "vanilla EKF" path that ignores `t_img`.

        Returns a small diagnostic dict (innovation, mahalanobis distance,
        det P) or None if no snapshot was found in the buffer (measurement
        too stale).
        """
        if not self.initialized:
            return None
        if t_img > self.t + 1e-6:  # measurement from the future — clamp to current
            t_img = self.t

        # --- Vanilla path: skip rewind/replay, update current state in place.
        if self.cfg.disable_delay_compensation:
            sigma = sigma_pix_eff if sigma_pix_eff is not None else self.cfg.sigma_pix
            if self.cfg.sigma_norm_floor > 0.0:
                sigma = max(sigma, self.cfg.sigma_norm_floor)
            R_meas = np.eye(2) * (sigma * sigma)
            H = np.zeros((2, NE))
            H[:, ERR_PBAR] = np.eye(2)
            y = p_bar_meas - self.x[IDX_PBAR]
            S = H @ self.P @ H.T + R_meas
            try:
                mahal = float(y @ np.linalg.solve(S, y))
            except np.linalg.LinAlgError:
                return None
            if self.cfg.reject_mahalanobis > 0.0 and mahal > self.cfg.reject_mahalanobis:
                return {'innovation': y.copy(), 'mahalanobis': mahal, 'rejected': True,
                        'det_P_pos': float(np.linalg.det(self.P[ERR_PR, ERR_PR])),
                        'det_P_vel': float(np.linalg.det(self.P[ERR_VR, ERR_VR]))}
            try:
                K = self.P @ H.T @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return None
            dx = K @ y
            self.x = _inject(self.x, dx)
            I_KH = np.eye(NE) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T
            self.P = _regularize_cov(self.P, self.cfg)
            return {
                'innovation': y.copy(),
                'mahalanobis': mahal,
                'rejected': False,
                'det_P_pos': float(np.linalg.det(self.P[ERR_PR, ERR_PR])),
                'det_P_vel': float(np.linalg.det(self.P[ERR_VR, ERR_VR])),
            }

        # 1. find snapshot at or before t_img and rewind
        found = self.snap_buf.find_at_or_before(t_img)
        if found is None:
            return None
        snap_idx, snap = found
        x = snap.x.copy()
        P = snap.P.copy()
        t_curr = snap.t

        # 2. if snapshot is slightly before t_img, fast-forward using buffered IMU
        if t_img > t_curr + 1e-6:
            imu_window = self.imu_buf.samples_in(t_curr, t_img)
            for s in imu_window:
                dt = s.t - t_curr
                if dt <= 0:
                    continue
                x, P = _predict_step(x, P, s.omega, s.accel, R_c_b_at_img, dt,
                                     self.cfg, self.t_cam_b)
                t_curr = s.t
            # final partial step to land exactly on t_img
            if t_img > t_curr + 1e-6:
                last = imu_window[-1] if imu_window else self.imu_buf.latest()
                if last is not None:
                    x, P = _predict_step(
                        x, P, last.omega, last.accel, R_c_b_at_img,
                        t_img - t_curr, self.cfg, self.t_cam_b,
                    )
                    t_curr = t_img

        # 3. EKF measurement update — direct read of p_bar from state
        sigma = sigma_pix_eff if sigma_pix_eff is not None else self.cfg.sigma_pix
        if self.cfg.sigma_norm_floor > 0.0:
            sigma = max(sigma, self.cfg.sigma_norm_floor)
        R_meas = np.eye(2) * (sigma * sigma)
        H = np.zeros((2, NE))
        H[:, ERR_PBAR] = np.eye(2)
        y = p_bar_meas - x[IDX_PBAR]
        S = H @ P @ H.T + R_meas
        try:
            mahal = float(y @ np.linalg.solve(S, y))
        except np.linalg.LinAlgError:
            return None
        # Innovation gate: reject outliers, keep the current (predicted) state.
        if self.cfg.reject_mahalanobis > 0.0 and mahal > self.cfg.reject_mahalanobis:
            return {'innovation': y.copy(), 'mahalanobis': mahal, 'rejected': True,
                    'det_P_pos': float(np.linalg.det(self.P[ERR_PR, ERR_PR])),
                    'det_P_vel': float(np.linalg.det(self.P[ERR_VR, ERR_VR]))}
        try:
            K = P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None
        dx = K @ y
        x = _inject(x, dx)
        # Joseph form
        I_KH = np.eye(NE) - K @ H
        P = I_KH @ P @ I_KH.T + K @ R_meas @ K.T
        P = _regularize_cov(P, self.cfg)

        # 4. replay IMU from t_img to current time
        replay = self.imu_buf.samples_in(t_curr, self.t)
        for s in replay:
            dt = s.t - t_curr
            if dt <= 0:
                continue
            x, P = _predict_step(x, P, s.omega, s.accel, R_c_b_at_img, dt,
                                 self.cfg, self.t_cam_b)
            t_curr = s.t
        # ensure we land on current self.t
        if self.t > t_curr + 1e-6:
            last = self.imu_buf.latest()
            if last is not None:
                x, P = _predict_step(
                    x, P, last.omega, last.accel, R_c_b_at_img,
                    self.t - t_curr, self.cfg, self.t_cam_b,
                )

        self.x = x
        self.P = P

        # Refresh snapshot at current time (drop old snaps before t_img)
        self.snap_buf.prune_before(t_img)
        self.snap_buf.push(self.t, self.x, self.P)

        return {
            'innovation': y.copy(),
            'mahalanobis': mahal,
            'rejected': False,
            'det_P_pos': float(np.linalg.det(self.P[ERR_PR, ERR_PR])),
            'det_P_vel': float(np.linalg.det(self.P[ERR_VR, ERR_VR])),
        }

    # ---------- accessors ----------

    @property
    def relative_position(self) -> np.ndarray:
        return self.x[IDX_PR].copy()

    @property
    def relative_velocity(self) -> np.ndarray:
        return self.x[IDX_VR].copy()

    @property
    def quat(self) -> np.ndarray:
        return self.x[IDX_Q].copy()

    @property
    def p_bar(self) -> np.ndarray:
        return self.x[IDX_PBAR].copy()

    @property
    def cov_position(self) -> np.ndarray:
        return self.P[ERR_PR, ERR_PR].copy()

    @property
    def cov_velocity(self) -> np.ndarray:
        return self.P[ERR_VR, ERR_VR].copy()

    def target_position_world(self, p_aircraft_world: np.ndarray) -> np.ndarray:
        return p_aircraft_world - self.x[IDX_PR]

    def target_velocity_world(self, v_aircraft_world: np.ndarray) -> np.ndarray:
        return v_aircraft_world - self.x[IDX_VR]

    # ---------- internal ----------

    def _prune_buffers(self) -> None:
        if self.t is None:
            return
        keep_after = self.t - self.cfg.buffer_window_sec
        self.snap_buf.prune_before(keep_after)


# ============================================================================
#  Free functions (no self) — easier to unit-test than methods.
# ============================================================================

def _predict_step(
    x: np.ndarray,
    P: np.ndarray,
    omega: np.ndarray,
    accel: np.ndarray,
    R_c_b: np.ndarray,
    dt: float,
    cfg: DCEKFConfig,
    t_cam_b: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Nominal + error-covariance propagation through one IMU step.

    omega, accel: raw body-frame IMU samples (specific force for accel).
    R_c_b: camera-to-body rotation, treated as a known input (gimbal angles).
    t_cam_b: camera optical-center position in body frame (FLU, meters); if
        None or all-zero, behaves as the paper's t_b^c = 0 case.
    """
    q = x[IDX_Q]
    p_r = x[IDX_PR]
    v_r = x[IDX_VR]
    p_bar = x[IDX_PBAR]
    b_g = x[IDX_BG]
    b_a = x[IDX_BA]

    omega_corr = omega - b_g
    accel_corr = accel - b_a

    # --- attitude
    dq = small_angle_quat(omega_corr * dt)
    q_new = quat_normalize(quat_mul(dq, q))
    R_b_w = quat_to_rot(q_new)

    # --- inertial accel in world (specific force → inertial: a = R*f + g)
    a_world = R_b_w @ accel_corr + G_WORLD

    # --- relative kinematics: paper drops target accel
    p_r_new = p_r + v_r * dt + 0.5 * a_world * dt * dt
    v_r_new = v_r + a_world * dt

    # --- image feature dynamics
    R_c_w = R_b_w @ R_c_b
    R_w_c = R_c_w.T
    # Body→camera lever arm in body frame.  Used to (a) shift the projection
    # origin from body to camera optical center and (b) add the rigid-body
    # cross-product term ω × t_cam_b to the camera's translational velocity.
    if t_cam_b is None:
        t_cb = np.zeros(3)
    else:
        t_cb = np.asarray(t_cam_b)
    # Target position relative to camera, in camera frame.
    # p_r = aircraft - target; target - camera = -p_r - R_b_w @ t_cb.
    target_in_cam = R_w_c @ (-p_r - R_b_w @ t_cb)
    depth = target_in_cam[2]
    # Floor |depth| to cap the 1/Z gain in L_s (divergence guard; 0 = disabled).
    if cfg.depth_floor > 0.0 and abs(depth) < cfg.depth_floor:
        depth = cfg.depth_floor if depth >= 0.0 else -cfg.depth_floor
    # Camera linear velocity relative to target, in camera frame.
    # v_camera_world = v_aircraft_world + R_b_w @ (omega × t_cb).
    # Camera-to-target relative velocity in world = v_camera_world - v_target_world.
    # In our state, v_r = v_aircraft - v_target, so:
    #   v_camera_to_target_world = v_r + R_b_w @ (omega × t_cb)
    v_lever = R_b_w @ np.cross(omega_corr, t_cb)
    v_cam = R_w_c @ (v_r + v_lever)
    omega_cam = R_c_b.T @ omega_corr
    L = interaction_matrix(p_bar, depth)
    p_bar_dot = L @ np.concatenate([v_cam, omega_cam])
    p_bar_new = p_bar + dt * p_bar_dot

    # --- assemble new nominal state
    x_new = x.copy()
    x_new[IDX_Q] = q_new
    x_new[IDX_PR] = p_r_new
    x_new[IDX_VR] = v_r_new
    x_new[IDX_PBAR] = p_bar_new
    # biases unchanged in nominal (random walk lives in Q only)

    # --- error-state transition matrix F (17×17)
    F = np.eye(NE)

    # δθ
    F[ERR_THETA, ERR_THETA] = np.eye(3) - skew(omega_corr) * dt
    F[ERR_THETA, ERR_BG] = -np.eye(3) * dt

    # δp_r couples to δv_r (and weakly to δθ, δb_a through a_world)
    F[ERR_PR, ERR_VR] = np.eye(3) * dt
    F[ERR_PR, ERR_THETA] = -0.5 * dt * dt * R_b_w @ skew(accel_corr)
    F[ERR_PR, ERR_BA] = -0.5 * dt * dt * R_b_w

    # δv_r
    F[ERR_VR, ERR_THETA] = -dt * R_b_w @ skew(accel_corr)
    F[ERR_VR, ERR_BA] = -dt * R_b_w

    # δp_bar: image-Jacobian coupling.  We keep only the dominant
    # ∂p_bar / ∂v_r term, which is what carries 3-D state information back
    # through the image measurement.  See note in CONTEXT.md.
    F[ERR_PBAR, ERR_VR] = dt * L[:, :3] @ R_w_c

    # --- process noise Q (PSD form, applied as Q*dt at this step)
    Q = np.zeros((NE, NE))
    Q[ERR_THETA, ERR_THETA] = np.eye(3) * (cfg.sigma_gyr ** 2) * dt
    # Inject IMU acceleration noise into v_r and p_r (mapped through R_b_w)
    Qa = (cfg.sigma_acc ** 2) * dt * (R_b_w @ R_b_w.T)
    # plus dropped target-acceleration noise — diagonal, isotropic in world ENU
    Qa = Qa + np.eye(3) * (cfg.sigma_target_acc ** 2) * dt
    Q[ERR_VR, ERR_VR] = Qa
    Q[ERR_PR, ERR_PR] = 0.25 * dt * dt * Qa  # rough mapping
    Q[ERR_PBAR, ERR_PBAR] = np.eye(2) * (cfg.sigma_pbar_proc ** 2) * dt
    Q[ERR_BG, ERR_BG] = np.eye(3) * (cfg.sigma_b_gyr ** 2) * dt
    Q[ERR_BA, ERR_BA] = np.eye(3) * (cfg.sigma_b_acc ** 2) * dt

    P_new = F @ P @ F.T + Q
    P_new = _regularize_cov(P_new, cfg)
    return x_new, P_new


def _regularize_cov(P: np.ndarray, cfg: DCEKFConfig) -> np.ndarray:
    """Symmetrize and clip eigenvalues of P into [floor, ceil] (PD projection).

    Cheap 17×17 symmetric eig; guards the 1/Z-driven loss of positive-definiteness.
    """
    P = 0.5 * (P + P.T)
    if cfg.cov_eig_floor <= 0.0 and cfg.cov_eig_ceil <= 0.0:
        return P
    # Replace any inf/nan (a single small-depth step can overflow F@P@F.T) with a
    # finite cap so the symmetric eig converges; the eigenvalue clip then re-bounds P.
    cap = cfg.cov_eig_ceil if cfg.cov_eig_ceil > 0.0 else 1e6
    if not np.all(np.isfinite(P)):
        P = np.nan_to_num(P, nan=0.0, posinf=cap, neginf=-cap)
        P = 0.5 * (P + P.T)
    try:
        w, V = np.linalg.eigh(P)
    except np.linalg.LinAlgError:
        return np.eye(P.shape[0]) * cap
    if cfg.cov_eig_floor > 0.0:
        w = np.maximum(w, cfg.cov_eig_floor)
    if cfg.cov_eig_ceil > 0.0:
        w = np.minimum(w, cfg.cov_eig_ceil)
    return (V * w) @ V.T


def _inject(x: np.ndarray, dx: np.ndarray) -> np.ndarray:
    """Inject a 17D error-state correction into the 18D nominal state."""
    x_new = x.copy()
    dtheta = dx[ERR_THETA]
    dq = small_angle_quat(dtheta)
    x_new[IDX_Q] = quat_normalize(quat_mul(dq, x[IDX_Q]))
    x_new[IDX_PR] = x[IDX_PR] + dx[ERR_PR]
    x_new[IDX_VR] = x[IDX_VR] + dx[ERR_VR]
    x_new[IDX_PBAR] = x[IDX_PBAR] + dx[ERR_PBAR]
    x_new[IDX_BG] = x[IDX_BG] + dx[ERR_BG]
    x_new[IDX_BA] = x[IDX_BA] + dx[ERR_BA]
    return x_new
