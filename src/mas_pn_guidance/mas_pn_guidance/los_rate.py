"""Stamped LOS-rate differentiation — pure math, no ROS.

Shared by both range-tolerant guidance modes so the exact same discipline is
applied to the two bearing sources:

- ``bearing_pn`` — the LOS is derived from the estimator's absolute 3-D
  ``target_pose``; the caller feeds the *arrival* timestamp of each estimate.
- ``raw_ibvs`` — the LOS is the raw detection bearing (``bearing_raw/los``,
  decoupled from the EKF); the caller feeds the *detection header* timestamp.

The discipline (ticket 011 review #2/#3, hardened in ticket 012 review #6):

1. Differentiate on a **new stamped sample only** (its actual elapsed time), not
   on the control tick — so a 20-25 Hz bearing consumed by a 50 Hz control loop
   is not aliased (repeat calls with the same stamp just hold the last rate).
2. EMA-smooth the raw rate; hold the filtered rate between samples.
3. Ignore out-of-order / same-stamp samples (no reverse-time differentiation).
4. ``reset()`` on any source/mode switch — never difference bearings across
   estimators or across a target-loss boundary.

``omega`` is the LOS rotation-rate vector ``Ω`` with ``Ω = n̂ × dn̂/dt`` (world
ENU), exactly the quantity ``pn_law.pn_from_los_rate`` expects. It is range-free:
it depends only on the unit LOS direction and its sample times, never on range.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .pn_law import unit


class StampedLosRateDifferentiator:
    """Differentiate a unit LOS on new stamped samples; EMA-filter; hold between.

    Parameters
    ----------
    ema_alpha : float
        EMA weight on the *previous* filtered rate, clamped to ``[0, 1]``.
        ``omega <- alpha*omega + (1-alpha)*omega_raw``. Higher = smoother/slower.
    min_dt_s : float
        Minimum sample spacing to differentiate over. A new stamp closer than
        this is held (the baseline is kept for the next, larger gap) so a burst
        of near-simultaneous samples cannot produce a huge finite difference.
    """

    def __init__(self, ema_alpha: float = 0.7, min_dt_s: float = 1e-3):
        self.ema_alpha = float(np.clip(ema_alpha, 0.0, 1.0))
        self.min_dt_s = float(min_dt_s)
        self.reset()

    def reset(self) -> None:
        self._last_n_hat: Optional[np.ndarray] = None
        self._last_t_ns: Optional[int] = None
        self.omega: np.ndarray = np.zeros(3)

    def update(self, n_hat, t_ns: int) -> np.ndarray:
        """Ingest a stamped LOS sample; return the current filtered LOS rate.

        Only differentiates when ``t_ns`` strictly advances past the last
        sample by more than ``min_dt_s``. A non-finite ``n_hat`` is ignored
        (the last rate is held). Idempotent for a repeated ``t_ns``.
        """
        n = np.asarray(n_hat, dtype=float)
        if n.shape != (3,) or not np.all(np.isfinite(n)):
            return self.omega
        norm = float(np.linalg.norm(n))
        if norm < 1e-12:
            return self.omega
        n = n / norm

        if self._last_t_ns is None:
            self._last_n_hat = n
            self._last_t_ns = int(t_ns)
            return self.omega
        if int(t_ns) <= self._last_t_ns:          # repeat / out-of-order -> hold
            return self.omega
        dt = (int(t_ns) - self._last_t_ns) * 1e-9
        if dt > self.min_dt_s and self._last_n_hat is not None:
            omega_raw = np.cross(n, (n - self._last_n_hat) / dt)
            self.omega = (self.ema_alpha * self.omega
                          + (1.0 - self.ema_alpha) * omega_raw)
            self._last_n_hat = n
            self._last_t_ns = int(t_ns)
        return self.omega


def coast_decay(age_s: float, timeout_s: float, lost_s: float) -> float:
    """Dropout coast factor in ``[0, 1]`` for the raw-IBVS LOS rate.

    While a fresh detection is arriving (``age <= timeout``) the measured LOS
    rate is used at full weight (``1.0``). Between ``timeout`` and ``lost`` the
    factor decays linearly to ``0`` so the held rate is coasted down rather than
    frozen; past ``lost`` it is ``0`` (the caller declares the target lost). The
    LOS *direction* is coasted separately (held at the last bearing); this only
    scales the rate-driven turn command.
    """
    if age_s <= timeout_s:
        return 1.0
    if age_s >= lost_s:
        return 0.0
    span = max(lost_s - timeout_s, 1e-9)
    return max(0.0, 1.0 - (age_s - timeout_s) / span)
