"""Minimal first-order gimbal rate-loop model.

Calibrated on SIYI A8 mini via bench rate-step (run 1776262353, 2026-04-15):
  - Linear, symmetric command-to-rate map across |u| in [0.1, 1.0]
  - First-order response with ~100 ms time constant on both axes
  - No measurable deadband, no overshoot
  - Yaw and pitch gains match within 0.2% → single K suffices

Continuous-time model (per axis):
    dw/dt = (K·u(t − L) − w) / τ
    dθ/dt = w
where:
    u   — normalized command in [-1, +1]
    w   — angular rate [deg/s]
    θ   — angle [deg]
    K   — static gain [deg/s per unit u]
    τ   — time constant [s]
    L   — transport latency [s]

No ROS dependencies. Pure Python + math.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class AxisParams:
    """Per-axis rate-loop parameters."""
    K: float                # static gain, deg/s per unit u
    tau: float              # first-order time constant, s
    latency: float          # transport delay, s
    w_max: float            # absolute max rate, deg/s
    u_deadband: float = 0.0
    angle_min: float = -180.0
    angle_max: float = +180.0


# Defaults from run 1776262353 (A8 mini bench calibration).
YAW_DEFAULT = AxisParams(
    K=73.42, tau=0.101, latency=0.050, w_max=74.0,
    u_deadband=0.0, angle_min=-125.0, angle_max=+125.0)

PITCH_DEFAULT = AxisParams(
    K=73.56, tau=0.094, latency=0.050, w_max=74.0,
    u_deadband=0.0, angle_min=-20.0, angle_max=+85.0)


class GimbalAxis:
    """One axis. Integrates rate-command → rate → angle."""

    def __init__(self, params: AxisParams):
        self.p = params
        self.theta = 0.0
        self.w = 0.0
        self._delay_buf: deque[float] = deque()

    def reset(self, theta: float = 0.0, w: float = 0.0):
        self.theta = theta
        self.w = w
        self._delay_buf.clear()

    def cmd_to_rate_ss(self, u: float) -> float:
        """Steady-state rate for a held command u."""
        p = self.p
        if abs(u) < p.u_deadband:
            return 0.0
        return max(-p.w_max, min(p.w_max, p.K * u))

    def rate_to_cmd(self, w_desired: float) -> float:
        """Inverse static map: feed-forward u for a desired rate."""
        p = self.p
        w_clamped = max(-p.w_max, min(p.w_max, w_desired))
        return max(-1.0, min(1.0, w_clamped / p.K))

    def step(self, u: float, dt: float) -> tuple[float, float]:
        """Advance the model by dt. Returns (theta, w)."""
        if dt <= 0.0:
            return self.theta, self.w
        p = self.p

        # Transport latency as a fixed integer shift register sized by dt.
        # Zero latency -> no buffer; else pop the oldest once the buffer is
        # full, so the popped sample is exactly N = round(latency/dt) steps
        # old (equivalent to a pure delay of N·dt ≈ latency).
        n_delay = int(round(p.latency / dt)) if p.latency > 0 else 0
        self._delay_buf.append(float(u))
        if len(self._delay_buf) > n_delay:
            u_effective = self._delay_buf.popleft()
        else:
            u_effective = 0.0

        w_ss = self.cmd_to_rate_ss(u_effective)

        # Semi-implicit Euler: stable for any dt, matches exp response as dt→0.
        alpha = dt / (p.tau + dt)
        self.w += alpha * (w_ss - self.w)

        theta_new = self.theta + self.w * dt
        if theta_new <= p.angle_min:
            theta_new = p.angle_min
            if self.w < 0:
                self.w = 0.0
        elif theta_new >= p.angle_max:
            theta_new = p.angle_max
            if self.w > 0:
                self.w = 0.0
        self.theta = theta_new
        return self.theta, self.w


class Gimbal:
    """Two-axis gimbal: yaw + pitch rate-loop model."""

    def __init__(self,
                 yaw: AxisParams = YAW_DEFAULT,
                 pitch: AxisParams = PITCH_DEFAULT):
        self.yaw = GimbalAxis(yaw)
        self.pitch = GimbalAxis(pitch)

    def reset(self, yaw_deg: float = 0.0, pitch_deg: float = 0.0):
        self.yaw.reset(theta=yaw_deg)
        self.pitch.reset(theta=pitch_deg)

    def step(self, u_yaw: float, u_pitch: float,
             dt: float) -> tuple[float, float, float, float]:
        """Advance both axes. Returns (yaw_deg, pitch_deg, w_yaw, w_pitch)."""
        yaw_theta, yaw_w = self.yaw.step(u_yaw, dt)
        pitch_theta, pitch_w = self.pitch.step(u_pitch, dt)
        return yaw_theta, pitch_theta, yaw_w, pitch_w

    def rate_cmd_for(self, w_yaw_desired: float,
                     w_pitch_desired: float) -> tuple[float, float]:
        """Feed-forward (u_yaw, u_pitch) for desired rates."""
        return (self.yaw.rate_to_cmd(w_yaw_desired),
                self.pitch.rate_to_cmd(w_pitch_desired))


def simulate_step_response(axis: GimbalAxis, u_cmd: float,
                           duration: float = 1.0,
                           dt: float = 0.005) -> list[tuple[float, float, float]]:
    """Run a step-response simulation. Returns list of (t, w, theta)."""
    axis.reset()
    out = []
    t = 0.0
    n = int(math.ceil(duration / dt))
    for _ in range(n):
        theta, w = axis.step(u_cmd, dt)
        t += dt
        out.append((t, w, theta))
    return out


if __name__ == "__main__":
    g = Gimbal()
    ax = g.yaw

    # Validate against bench data: u=0.5 should yield w_ss≈36.7 deg/s
    trace = simulate_step_response(ax, u_cmd=0.5, duration=1.0, dt=0.005)
    t_end, w_end, theta_end = trace[-1]
    print(f"yaw @ u=0.5, t=1.0s: w={w_end:.2f} deg/s "
          f"(expected {ax.p.K * 0.5:.2f}), theta={theta_end:.2f} deg")

    # First-order progression check
    idx_01 = int(0.1 / 0.005) - 1
    idx_02 = int(0.2 / 0.005) - 1
    # After 0.1s effective integration (= 0.15s real due to 0.05s latency)
    # w(t) = w_ss * (1 - exp(-(t - L) / τ))
    expected_015 = ax.p.K * 0.5 * (1.0 - math.exp(-(0.15 - 0.05) / 0.101))
    _, w_015, _ = trace[int(0.15 / 0.005) - 1]
    print(f"yaw @ t=0.15s: w={w_015:.2f} "
          f"(expected ≈{expected_015:.2f})")

    # Inverse map check
    u = ax.rate_to_cmd(50.0)
    print(f"yaw rate_to_cmd(50 deg/s) = {u:.3f} "
          f"(expected {50.0 / ax.p.K:.3f})")
