"""Proportional-navigation guidance law — pure math, no ROS.

Mirrors EXACTLY the Ticket 003 point-mass reference
``research/bearing_localization/interception_baseline/pn_guidance.py`` so the
PegasusSimulator engagement uses an identical guidance law to the point-mass
study (only the vehicle dynamics differ: PX4 controller vs ideal tracking). Keep
the two in sync; `tests/test_pn_law.py` pins the behavior.

True PN: commanded acceleration is perpendicular to the LOS,
``a = N · max(Vc, 0) · (Ω × û)``, where ``û`` is the observer→target unit LOS,
``Vc`` the closing speed, and ``Ω`` the LOS rotation-rate vector. It does NOT add
a closing term — closing comes from the interceptor's speed (the node seeds the
commanded velocity as a v_max pursuit at engagement, matching the point-mass).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


def unit(v: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > 1e-12:
        return v / n
    if fallback is None:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return np.asarray(fallback, dtype=float)


def limit_norm(v: np.ndarray, limit: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if limit <= 0.0 or n <= limit or n <= 1e-12:
        return v.copy()
    return v * (limit / n)


@dataclass(frozen=True)
class PNCommand:
    acceleration_mps2: np.ndarray
    raw_acceleration_mps2: np.ndarray
    closing_speed_mps: float
    los_rate_radps: float
    saturated: bool
    range_est_m: float


def proportional_navigation(
    own_position_m: np.ndarray,
    own_velocity_mps: np.ndarray,
    target_position_est_m: np.ndarray,
    target_velocity_est_mps: np.ndarray,
    navigation_constant: float,
    accel_limit_mps2: float,
) -> PNCommand:
    """Compute 3D PN from estimated relative position and velocity."""
    r_hat = np.asarray(target_position_est_m, dtype=float) - np.asarray(own_position_m, dtype=float)
    v_rel_hat = np.asarray(target_velocity_est_mps, dtype=float) - np.asarray(own_velocity_mps, dtype=float)
    range_est = max(float(np.linalg.norm(r_hat)), 1e-9)
    n_hat = unit(r_hat)
    closing_speed = -float(np.dot(v_rel_hat, n_hat))
    omega_los = np.cross(r_hat, v_rel_hat) / max(float(np.dot(r_hat, r_hat)), 1e-9)
    raw_accel = navigation_constant * max(closing_speed, 0.0) * np.cross(omega_los, n_hat)
    accel = limit_norm(raw_accel, accel_limit_mps2)
    return PNCommand(
        acceleration_mps2=accel,
        raw_acceleration_mps2=raw_accel,
        closing_speed_mps=closing_speed,
        los_rate_radps=float(np.linalg.norm(omega_los)),
        saturated=bool(np.linalg.norm(raw_accel) > accel_limit_mps2 + 1e-9),
        range_est_m=range_est,
    )


def command_to_dict(command: PNCommand) -> Dict:
    return {
        "closing_speed_mps": command.closing_speed_mps,
        "los_rate_radps": command.los_rate_radps,
        "saturated": command.saturated,
        "range_est_m": command.range_est_m,
        "accel_norm_mps2": float(np.linalg.norm(command.acceleration_mps2)),
        "raw_accel_norm_mps2": float(np.linalg.norm(command.raw_acceleration_mps2)),
    }
