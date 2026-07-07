"""IMU and state-snapshot ring buffers for delay-compensated EKF.

Implements the buffering needed by Liu et al. 2026, Alg. 2:
- Keep a sliding window of recent IMU measurements (~0.5 s).
- Keep a sliding window of (timestamp, x, P) snapshots taken right after each
  IMU predict, indexed by the same timestamps.

When a delayed image feature arrives with stamp t_img:
1. Look up the snapshot at the largest t_snap ≤ t_img.
2. Predict that snapshot forward through the few IMU samples up to t_img.
3. Apply the EKF measurement update.
4. Replay IMU forward from t_img to current time using the buffered IMUs.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np


@dataclass
class IMUSample:
    t: float
    omega: np.ndarray  # 3, body frame
    accel: np.ndarray  # 3, body frame, specific force


@dataclass
class Snapshot:
    t: float
    x: np.ndarray  # state vector copy
    P: np.ndarray  # covariance copy


class RingBufferIMU:
    def __init__(self, window_sec: float = 0.5, expected_rate_hz: float = 200.0):
        cap = max(8, int(window_sec * expected_rate_hz * 1.5))
        self._buf: Deque[IMUSample] = deque(maxlen=cap)

    def push(self, t: float, omega: np.ndarray, accel: np.ndarray) -> None:
        self._buf.append(IMUSample(t=t, omega=omega.copy(), accel=accel.copy()))

    def samples_in(self, t_start: float, t_end: float) -> List[IMUSample]:
        """Samples strictly between (t_start, t_end].  Used for forward replay."""
        return [s for s in self._buf if t_start < s.t <= t_end]

    def latest(self) -> Optional[IMUSample]:
        return self._buf[-1] if self._buf else None

    def __len__(self) -> int:
        return len(self._buf)


class RingBufferSnapshot:
    def __init__(self, window_sec: float = 0.5, expected_rate_hz: float = 200.0):
        cap = max(8, int(window_sec * expected_rate_hz * 1.5))
        self._buf: Deque[Snapshot] = deque(maxlen=cap)

    def push(self, t: float, x: np.ndarray, P: np.ndarray) -> None:
        self._buf.append(Snapshot(t=t, x=x.copy(), P=P.copy()))

    def find_at_or_before(self, t: float) -> Optional[Tuple[int, Snapshot]]:
        """Index + snapshot whose timestamp is the largest one ≤ t."""
        if not self._buf:
            return None
        best_idx = -1
        for i, s in enumerate(self._buf):
            if s.t <= t:
                best_idx = i
            else:
                break
        if best_idx < 0:
            return None
        return best_idx, self._buf[best_idx]

    def prune_before(self, t: float) -> None:
        """Drop snapshots strictly older than `t`."""
        while self._buf and self._buf[0].t < t:
            self._buf.popleft()

    def __len__(self) -> int:
        return len(self._buf)
