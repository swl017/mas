"""Minimal "textbook" bearing-only EKF for comparison against the DC-EKF.

This is the *truly* vanilla baseline — no IBVS image-feature state, no IMU
integration, no quaternion error state, no delay compensation, no biases.

State (6D, world ENU):
    x = [ p_target_x, p_target_y, p_target_z,
          v_target_x, v_target_y, v_target_z ]

Aircraft pose and velocity (and gimbal/camera extrinsics) are treated as
*known inputs* from `common_frame/odom` + gimbal + camera_info — no error.

Predict (constant-velocity target with bounded acceleration noise):
    p_target ← p_target + v_target · dt
    v_target ← v_target                       (driven by Q)

Measurement (normalized image feature, 2D):
    h(x) = π( R_c_w⁻¹ · ( p_target − p_camera_world ) )
    H = ∂h/∂p_target (analytic), ∂h/∂v_target = 0

Compared to the DC-EKF this:
  * removes the IBVS interaction-matrix coupling (the dominant source of
    `Z`-scaled cross-correlations that build up wrongly when range is poorly
    known and the aircraft moves fast),
  * removes IMU integration (no chance for accelerometer bias to drift `v_r`
    over multi-second episodes),
  * removes quaternion as state (uses odom orientation as a known input),
  * removes the delay-compensation snapshot/replay path,
so every potential confounder discussed in the analysis docs is eliminated.
The result is the standard "single-observer bearing-only target tracker" a
textbook would describe.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


NX = 6  # [px, py, pz, vx, vy, vz]
IDX_P = slice(0, 3)
IDX_V = slice(3, 6)


@dataclass
class SimpleEKFConfig:
    sigma_target_acc: float = 0.5       # m/s²/√Hz, drives v_target covariance
    sigma_pix: float = 3.0              # 1σ pixel noise on bbox center
    # Floor on the effective *normalized* measurement 1σ (rad). The node passes
    # sigma_pix/(fx·zoom); at high zoom that underestimates the true angular error
    # (~0.03 rad, dominated by the zoom-invariant YOLO/target-centroid offset),
    # so the filter over-fits and the bearing-only range estimate collapses (the
    # target is inferred close-and-moving instead of far-and-static). Flooring the
    # effective σ at the true angular noise fixes it. 0 = disabled.
    sigma_norm_floor: float = 0.0
    init_range: float = 15.0
    init_var_pos: float = 100.0
    init_var_vel: float = 25.0
    t_cam_in_body: tuple = (0.0, 0.0, 0.0)
    # Position-covariance ceiling (m², on trace). With the deliberately loose
    # bearing R, the LOS/range direction is weakly constrained and trace(P_pos)
    # can run away; this caps it. 0 = disabled.
    pos_var_ceiling: float = 0.0
    # Innovation (Mahalanobis) gate: skip the correction when y'S⁻¹y exceeds
    # this, keeping the prediction. Rejects outlier detections (gimbal
    # oscillation / FOV-edge bboxes). 0 = disabled. 2-DOF chi² 99% ≈ 9.2.
    reject_mahalanobis: float = 0.0


class SimpleEKF:
    def __init__(self, cfg: Optional[SimpleEKFConfig] = None):
        self.cfg = cfg or SimpleEKFConfig()
        self.x = np.zeros(NX)
        self.P = np.eye(NX) * 1e-3
        self.t: Optional[float] = None
        self.initialized = False
        self.t_cam_b = np.asarray(self.cfg.t_cam_in_body, dtype=float)

    def initialize_from_bearing(
        self,
        t: float,
        bearing_world: np.ndarray,
        p_aircraft_world: np.ndarray,
        v_aircraft_world: Optional[np.ndarray] = None,
        R_b_w: Optional[np.ndarray] = None,
        range_guess: Optional[float] = None,
    ) -> None:
        """Seed target position from the first bearing.  `v_target` starts at 0."""
        rng = range_guess if range_guess is not None else self.cfg.init_range
        if R_b_w is None:
            p_cam = p_aircraft_world
        else:
            p_cam = p_aircraft_world + R_b_w @ self.t_cam_b
        self.x[IDX_P] = p_cam + rng * bearing_world
        self.x[IDX_V] = 0.0
        P = np.zeros((NX, NX))
        P[IDX_P, IDX_P] = np.eye(3) * self.cfg.init_var_pos
        P[IDX_V, IDX_V] = np.eye(3) * self.cfg.init_var_vel
        self.P = P
        self.t = t
        self.initialized = True

    def predict(self, t: float) -> None:
        """Propagate at the requested wall-clock time."""
        if not self.initialized or self.t is None:
            return
        dt = t - self.t
        if dt <= 0:
            return
        # Constant-velocity propagation
        F = np.eye(NX)
        F[IDX_P, IDX_V] = np.eye(3) * dt
        self.x = F @ self.x
        # Process noise: integral of target acceleration
        Qa = (self.cfg.sigma_target_acc ** 2) * dt
        Q = np.zeros((NX, NX))
        Q[IDX_P, IDX_P] = np.eye(3) * 0.25 * dt * dt * Qa
        Q[IDX_V, IDX_V] = np.eye(3) * Qa
        Q[IDX_P, IDX_V] = np.eye(3) * 0.5 * dt * Qa
        Q[IDX_V, IDX_P] = np.eye(3) * 0.5 * dt * Qa
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        self.t = t

    def update_bearing(
        self,
        t: float,
        p_bar_meas: np.ndarray,
        p_aircraft_world: np.ndarray,
        R_b_w: np.ndarray,
        R_c_b: np.ndarray,
        sigma_pix_eff: Optional[float] = None,
    ) -> Optional[dict]:
        """Apply normalized image-feature update at time `t`."""
        if not self.initialized:
            return None
        if self.t is None or t > self.t + 1e-6:
            self.predict(t)

        p_cam_world = p_aircraft_world + R_b_w @ self.t_cam_b
        R_c_w = R_b_w @ R_c_b
        # Vector from camera to target, in camera frame
        delta_world = self.x[IDX_P] - p_cam_world
        p_cam = R_c_w.T @ delta_world
        Z = p_cam[2]
        if abs(Z) < 0.5:
            return None
        u = p_cam[0] / Z
        v = p_cam[1] / Z

        # ∂h/∂p_target.  Let p_c = (X, Y, Z) = R_c_w⁻¹ · (p_t - p_cam_world).
        # ∂p_c/∂p_t = R_c_w⁻¹ (3×3).  Then
        # ∂(X/Z)/∂p_t = (1/Z) · [1 0 -X/Z] · R_c_w⁻¹
        # ∂(Y/Z)/∂p_t = (1/Z) · [0 1 -Y/Z] · R_c_w⁻¹
        R_w_c = R_c_w.T
        dpi_dpc = np.array([
            [1.0 / Z, 0.0, -u / Z],
            [0.0, 1.0 / Z, -v / Z],
        ])
        dh_dpt = dpi_dpc @ R_w_c
        H = np.zeros((2, NX))
        H[:, IDX_P] = dh_dpt

        sigma = sigma_pix_eff if sigma_pix_eff is not None else self.cfg.sigma_pix
        if self.cfg.sigma_norm_floor > 0.0:
            sigma = max(sigma, self.cfg.sigma_norm_floor)
        R_meas = np.eye(2) * (sigma * sigma)
        y = p_bar_meas - np.array([u, v])
        S = H @ self.P @ H.T + R_meas
        try:
            mahal = float(y @ np.linalg.solve(S, y))
        except np.linalg.LinAlgError:
            return None
        # Innovation gate: drop outlier measurements, keep the prediction.
        if self.cfg.reject_mahalanobis > 0.0 and mahal > self.cfg.reject_mahalanobis:
            return {"innovation": y.copy(), "mahalanobis": mahal, "rejected": True,
                    "det_P_pos": float(np.linalg.det(self.P[IDX_P, IDX_P])),
                    "det_P_vel": float(np.linalg.det(self.P[IDX_V, IDX_V]))}
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None
        dx = K @ y
        self.x = self.x + dx
        I_KH = np.eye(NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self._apply_pos_var_ceiling()
        return {
            "rejected": False,
            "innovation": y.copy(),
            "mahalanobis": mahal,
            "det_P_pos": float(np.linalg.det(self.P[IDX_P, IDX_P])),
            "det_P_vel": float(np.linalg.det(self.P[IDX_V, IDX_V])),
        }

    def _apply_pos_var_ceiling(self) -> None:
        """Cap trace(P_pos) without changing the mean or breaking PSD.

        Scales the position block (and its cross-covariance with velocity) by a
        diagonal congruence T = diag(s,s,s,1,1,1), which preserves PSD and
        leaves the velocity block untouched. Uniform across the 3 position axes
        — a safeguard against the runaway LOS-direction variance, not a
        per-direction model.
        """
        ceil = self.cfg.pos_var_ceiling
        if ceil <= 0.0:
            return
        tr = float(np.trace(self.P[IDX_P, IDX_P]))
        if tr > ceil:
            s = (ceil / tr) ** 0.5
            T = np.diag([s, s, s, 1.0, 1.0, 1.0])
            self.P = T @ self.P @ T.T
            self.P = 0.5 * (self.P + self.P.T)

    # ---- accessors ----
    @property
    def target_position(self) -> np.ndarray:
        return self.x[IDX_P].copy()

    @property
    def target_velocity(self) -> np.ndarray:
        return self.x[IDX_V].copy()

    @property
    def cov_position(self) -> np.ndarray:
        return self.P[IDX_P, IDX_P].copy()

    @property
    def cov_velocity(self) -> np.ndarray:
        return self.P[IDX_V, IDX_V].copy()
