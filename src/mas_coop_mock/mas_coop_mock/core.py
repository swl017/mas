"""Pure logic for the ticket-019 mock-cooperative closed loop (no rclpy).

Kept ROS-free so the offline-provable surface (velocity recovery, observer
geometry, latency buffering) is unit-testable without a running graph or the sim.
The three rclpy node wrappers (cv_smoother, viewing_offset, ray_delay) import
from here.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np


class AlphaBetaVel:
    """Constant-velocity alpha-beta tracker over an irregular time grid.

    Recovers target velocity from a stream of position fixes (the fused
    `chosen_target_pose`, whose own SORT3D KF velocity is per-frame / event-rate
    and unpublished). `update(t, z)` returns the filtered (position, velocity);
    the first sample seeds position with zero velocity. Out-of-order / duplicate
    stamps (dt <= 0) are ignored (state unchanged).
    """

    def __init__(self, alpha: float = 0.5, beta: float = 0.1, v_max: float | None = None):
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha in (0,1]")
        if not (0.0 < beta <= 1.0):
            raise ValueError("beta in (0,1]")
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.v_max = None if v_max is None else float(v_max)
        self.t: float | None = None
        self.x: np.ndarray | None = None
        self.v: np.ndarray | None = None

    def update(self, t: float, z) -> tuple[np.ndarray, np.ndarray]:
        z = np.asarray(z, dtype=float)
        if self.x is None:
            self.t, self.x, self.v = float(t), z.copy(), np.zeros_like(z)
            return self.x.copy(), self.v.copy()
        dt = float(t) - self.t
        if dt <= 0.0:                       # out-of-order / duplicate stamp -> ignore
            return self.x.copy(), self.v.copy()
        xp = self.x + self.v * dt           # CV predict
        r = z - xp                          # residual
        self.x = xp + self.alpha * r
        self.v = self.v + (self.beta / dt) * r
        if self.v_max is not None:
            sp = float(np.linalg.norm(self.v))
            if sp > self.v_max:
                self.v *= self.v_max / sp
        self.t = float(t)
        return self.x.copy(), self.v.copy()


def viewing_pose(p_int, p_tgt, offset_deg: float, standoff_m: float,
                 height_m: float | None = None) -> np.ndarray:
    """Observer position giving `offset_deg` of parallax vs the interceptor.

    Sits on a circle of radius `standoff_m` about the target, at azimuth
    `offset_deg` rotated (about +z, ENU) from the target->interceptor bearing, so
    the angle interceptor-target-observer equals `offset_deg` (the parallax knob;
    0 deg = collinear/degenerate, ~90 deg = orthogonal/favorable). Height holds
    the target's z unless `height_m` is given. Returns a world (ENU) 3-vector.
    """
    p_int = np.asarray(p_int, dtype=float)
    p_tgt = np.asarray(p_tgt, dtype=float)
    d = p_int[:2] - p_tgt[:2]
    n = float(np.linalg.norm(d))
    u = d / n if n > 1e-6 else np.array([1.0, 0.0])
    a = math.radians(float(offset_deg))
    c, s = math.cos(a), math.sin(a)
    r = np.array([c * u[0] - s * u[1], s * u[0] + c * u[1]])   # rotate u by offset
    z = p_tgt[2] if height_m is None else float(height_m)
    return np.array([p_tgt[0] + standoff_m * r[0],
                     p_tgt[1] + standoff_m * r[1], z])


class DelayBuffer:
    """FIFO age-release buffer — the peer-communication (AoI) delay stage.

    `push(t_rx, item)` timestamps an item at receive time; `pop_ready(t_now)`
    releases (in order) every item whose age t_now - t_rx >= tau. tau = 0 is an
    immediate passthrough. Order is preserved (monotonic rx assumed).
    """

    def __init__(self, tau_s: float = 0.0):
        if tau_s < 0.0:
            raise ValueError("tau_s >= 0")
        self.tau = float(tau_s)
        self._q: deque = deque()

    def push(self, t_rx: float, item) -> None:
        self._q.append((float(t_rx), item))

    def pop_ready(self, t_now: float) -> list:
        out = []
        while self._q and (float(t_now) - self._q[0][0]) >= self.tau:
            out.append(self._q.popleft()[1])
        return out

    def __len__(self) -> int:
        return len(self._q)
