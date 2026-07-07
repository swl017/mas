"""Direct-projection relative-state bearing-only EKF (6D).

A lighter sibling of the 18D DC-EKF and the 6D world-frame `SimpleEKF`.  The
defining idea ("direct projection") is to keep the state as the *relative*
observer→target geometry and map the unit bearing onto it by a single
geometric projection onto the unit sphere — no in-state image feature `p̄`.

State (6D, world ENU), relative to the observer (camera):
    x = [ q_x, q_y, q_z, q̇_x, q̇_y, q̇_z ]
    q  = p_observer − p_target          (relative position)
    q̇  = v_observer − v_target          (relative velocity)

Predict (constant-velocity *target*, observer accel as known control input):
    q  ← q + q̇·dt + ½·a_obs·dt²
    q̇  ← q̇ + a_obs·dt
Because q double-differentiates to  q̈ = a_obs − a_target, the observer's own
acceleration enters as a deterministic `B·u` term (we measure it), while the
target acceleration is the unmodeled part driven by the process noise Q.  This
is the key difference from `SimpleEKF`, whose absolute-target state has no
place to inject the observer's maneuver into the prediction.

Measurement (3D unit bearing, observer→target, world ENU):
    h(q) = −q / ‖q‖                     (project relative vector to unit sphere)
    H = ∂h/∂q = −(I − u·uᵀ) / r ,   u = −q/‖q‖ = predicted bearing,  r = ‖q‖
The tangent projector (I − u·uᵀ) zeros the line-of-sight (range) direction:
one bearing constrains only the two directions perpendicular to the LOS, so
range observability comes purely from observer-induced LOS rotation
(parallax) accumulated over time — exactly the classic bearing-only result.

Target world quantities are recovered against the *known* observer state:
    p_target = p_observer − q ,   v_target = v_observer − q̇

References:
- ``research/bearing_localization/moving_target/benchmark_estimators.py``
  (``DirectProjectionEKF``) — the synthetic-benchmark reference implementation.
- ``research/bearing_localization/moving_target/DIRECT_PROJECTION_EKF_EXPLAINED.md``
  — intuition, the (I − uuᵀ)/r Jacobian derivation, and the observability /
  Fisher-information analysis.
- ``doc/EKF_VARIANTS_BENCHMARK.md`` — why the in-state image feature (18D) is
  removed for long-running scenes.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


NX = 6  # [qx, qy, qz, q̇x, q̇y, q̇z]
IDX_P = slice(0, 3)  # relative position q = p_observer - p_target
IDX_V = slice(3, 6)  # relative velocity q̇ = v_observer - v_target


@dataclass
class DirectProjectionEKFConfig:
    sigma_target_acc: float = 0.5       # m/s²/√Hz, unmodeled target accel (Q)
    sigma_bearing: float = 0.02         # rad, 1σ bearing noise (fallback)
    init_range: float = 15.0
    init_var_pos: float = 100.0
    init_var_vel: float = 25.0
    # Robustness floors (fix #3): the direct-projection filter's H = (I-uuᵀ)/r
    # blows up as range→0, collapsing the relative state onto the observer.
    range_floor: float = 2.0            # m, lower bound on r used in the Jacobian
    pos_var_floor: float = 1.0          # m², per-axis floor on the position cov
    # Innovation (Mahalanobis) gate: skip the correction when y'S⁻¹y exceeds
    # this, keeping the prediction. Rejects outlier bearings (gimbal oscillation
    # / FOV-edge). 0 = disabled. 3-DOF chi² 99% ≈ 11.3.
    reject_mahalanobis: float = 0.0


def _unit(v: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > 1e-12:
        return v / n
    if fallback is None:
        return np.array([1.0, 0.0, 0.0])
    return np.asarray(fallback, dtype=float)


def bearing_jacobian_from_delta(delta: np.ndarray):
    """Unit bearing and its Jacobian for ``h(δ) = δ/‖δ‖``.

    Returns ``(u, projector/r, r)`` where ``u = δ/r``, ``r = ‖δ‖`` and
    ``projector/r = (I − u·uᵀ)/r = ∂h/∂δ``.  See the explained-doc §4 for the
    line-by-line derivation.
    """
    r = float(np.linalg.norm(delta))
    if r < 1e-6:
        r = 1e-6
    u = delta / r
    projector = np.eye(3) - np.outer(u, u)
    return u, projector / r, r


def _cv_process_noise(dt: float, sigma_acc: float) -> np.ndarray:
    """Continuous-white-noise-acceleration Q for a 6D [pos, vel] block."""
    q = sigma_acc ** 2
    Q = np.zeros((NX, NX))
    Q[IDX_P, IDX_P] = np.eye(3) * (0.25 * dt ** 4 * q)
    Q[IDX_P, IDX_V] = np.eye(3) * (0.5 * dt ** 3 * q)
    Q[IDX_V, IDX_P] = np.eye(3) * (0.5 * dt ** 3 * q)
    Q[IDX_V, IDX_V] = np.eye(3) * (dt ** 2 * q)
    return Q


class DirectProjectionEKF:
    def __init__(self, cfg: Optional[DirectProjectionEKFConfig] = None):
        self.cfg = cfg or DirectProjectionEKFConfig()
        self.x = np.zeros(NX)
        self.P = np.eye(NX) * 1e-3
        self.t: Optional[float] = None
        self.initialized = False

    def initialize_from_bearing(
        self,
        t: float,
        bearing_world: np.ndarray,
        v_observer_world: Optional[np.ndarray] = None,
        range_guess: Optional[float] = None,
    ) -> None:
        """Seed the relative geometry from the first observer→target bearing.

        ``q = p_observer − p_target = −range·bearing`` (bearing points toward the
        target, so the observer is ``range`` metres *behind* the target along it).
        ``q̇`` starts at the observer velocity (target velocity guessed as zero).
        """
        rng = range_guess if range_guess is not None else self.cfg.init_range
        b = _unit(bearing_world)
        self.x[IDX_P] = -rng * b
        if v_observer_world is None:
            self.x[IDX_V] = 0.0
        else:
            self.x[IDX_V] = np.asarray(v_observer_world, dtype=float)
        P = np.zeros((NX, NX))
        P[IDX_P, IDX_P] = np.eye(3) * self.cfg.init_var_pos
        P[IDX_V, IDX_V] = np.eye(3) * self.cfg.init_var_vel
        self.P = P
        self.t = t
        self.initialized = True

    def predict(self, t: float, obs_a: Optional[np.ndarray] = None) -> None:
        """Propagate to time ``t``; ``obs_a`` is the observer accel (world ENU)."""
        if not self.initialized or self.t is None:
            return
        dt = float(t - self.t)
        if dt <= 0.0:
            return
        a = np.zeros(3) if obs_a is None else np.asarray(obs_a, dtype=float)
        F = np.eye(NX)
        F[IDX_P, IDX_V] = np.eye(3) * dt
        # Relative kinematics with the observer's own accel as control input.
        self.x[IDX_P] = self.x[IDX_P] + self.x[IDX_V] * dt + 0.5 * a * dt * dt
        self.x[IDX_V] = self.x[IDX_V] + a * dt
        self.P = F @ self.P @ F.T + _cv_process_noise(dt, self.cfg.sigma_target_acc)
        self.P = 0.5 * (self.P + self.P.T)
        self.t = t

    def update_bearing(
        self,
        t: float,
        bearing_world: np.ndarray,
        obs_a: Optional[np.ndarray] = None,
        sigma_bearing_eff: Optional[float] = None,
    ) -> Optional[dict]:
        """Apply a 3D unit-bearing (observer→target, world ENU) update at ``t``.

        Predicts to ``t`` first (using ``obs_a``), then corrects the relative
        position along the two LOS-perpendicular directions via the direct
        tangent-plane projection.  Returns a diagnostics dict, or ``None`` if
        the filter is uninitialized or the innovation covariance is singular.
        """
        if not self.initialized:
            return None
        if self.t is None or t > self.t + 1e-6:
            self.predict(t, obs_a)

        z = _unit(bearing_world)
        # delta = -q = p_target - p_observer  → u = predicted bearing.
        delta = -self.x[IDX_P]
        r = float(np.linalg.norm(delta))
        u = delta / r if r > 1e-9 else np.array([1.0, 0.0, 0.0])
        h = u
        # Range floor (fix #3): cap the (I-uuᵀ)/r tangent Jacobian so an
        # underestimated range cannot blow it up and collapse the relative
        # state onto the observer.
        r_eff = max(r, self.cfg.range_floor)
        dh_ddelta = (np.eye(3) - np.outer(u, u)) / r_eff
        H = np.zeros((3, NX))
        # ∂h/∂q = ∂h/∂δ · (∂δ/∂q) = dh_ddelta · (−I) = −dh_ddelta.
        H[:, IDX_P] = -dh_ddelta

        sigma = sigma_bearing_eff if sigma_bearing_eff is not None else self.cfg.sigma_bearing
        R_meas = np.eye(3) * (sigma * sigma)
        y = z - h
        S = H @ self.P @ H.T + R_meas
        try:
            mahal = float(y @ np.linalg.solve(S, y))
        except np.linalg.LinAlgError:
            return None
        # Innovation gate: drop outlier bearings, keep the prediction.
        if self.cfg.reject_mahalanobis > 0.0 and mahal > self.cfg.reject_mahalanobis:
            return {"innovation": y.copy(), "mahalanobis": mahal, "rejected": True,
                    "det_P_pos": float(np.linalg.det(self.P[IDX_P, IDX_P])),
                    "det_P_vel": float(np.linalg.det(self.P[IDX_V, IDX_V]))}
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None
        self.x = self.x + K @ y
        I_KH = np.eye(NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self._apply_pos_var_floor()
        if not (np.isfinite(self.x).all() and np.isfinite(self.P).all()):
            return None
        return {
            "rejected": False,
            "innovation": y.copy(),
            "mahalanobis": mahal,
            "det_P_pos": float(np.linalg.det(self.P[IDX_P, IDX_P])),
            "det_P_vel": float(np.linalg.det(self.P[IDX_V, IDX_V])),
        }

    def _apply_pos_var_floor(self) -> None:
        """Keep the position covariance from collapsing to ~0 (fix #3).

        Raising a diagonal entry of a PSD matrix preserves PSD, so clamping the
        per-axis position variance up to ``pos_var_floor`` keeps the Kalman gain
        responsive instead of locking the filter once it over-collapses.
        """
        floor = self.cfg.pos_var_floor
        if floor <= 0.0:
            return
        for i in range(3):
            if self.P[i, i] < floor:
                self.P[i, i] = floor

    # ---- accessors (target world state needs the known observer state) ----
    def target_position(self, p_observer_world: np.ndarray) -> np.ndarray:
        """p_target = p_observer − q."""
        return np.asarray(p_observer_world, dtype=float) - self.x[IDX_P]

    def target_velocity(self, v_observer_world: np.ndarray) -> np.ndarray:
        """v_target = v_observer − q̇."""
        return np.asarray(v_observer_world, dtype=float) - self.x[IDX_V]

    @property
    def predicted_bearing_world(self) -> np.ndarray:
        """Current state's predicted observer→target unit bearing, h(q) = −q/‖q‖."""
        return _unit(-self.x[IDX_P])

    @property
    def rel_position(self) -> np.ndarray:
        return self.x[IDX_P].copy()

    @property
    def rel_velocity(self) -> np.ndarray:
        return self.x[IDX_V].copy()

    @property
    def range(self) -> float:
        return float(np.linalg.norm(self.x[IDX_P]))

    @property
    def cov_position(self) -> np.ndarray:
        # cov(p_target) = cov(q), since the observer state is treated as known.
        return self.P[IDX_P, IDX_P].copy()

    @property
    def cov_velocity(self) -> np.ndarray:
        return self.P[IDX_V, IDX_V].copy()
