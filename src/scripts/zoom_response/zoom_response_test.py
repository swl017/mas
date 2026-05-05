#!/usr/bin/env python3
"""Record SIYI A8 zoom step / rate response.

The A8 mini does pure digital zoom and the gimbal exposes two command
topics (Float32):

    <ns>/zoom_level_cmd   absolute target, 1.0 .. 6.0
    <ns>/zoom_rate_cmd    rate (zoom-levels/s; siyi_ros_node integrates this
                          into 0x0F absolute-zoom dispatches at 50 Hz)

State feedback is polled by siyi_ros_node and republished at the node's
publish_rate on:

    <ns>/camera/zoom_level   std_msgs/Float64 (one decimal)

Two modes, three profiles each:

    --mode level --profile step      discrete level steps with dwell
    --mode level --profile sine      continuously varying level (sinusoid)
    --mode rate  --profile const     constant rate, republished
    --mode rate  --profile sine      sinusoidal rate
    --mode rate  --profile chirp     linear-frequency-sweep rate

The command stream is published at --publish-hz (default 25 Hz, the policy
loop rate). For sine / chirp profiles every published sample is logged so
the plot can render the commanded signal as a curve.

Output (under <output_dir>/<run_name>/):
  states.csv     t_s, zoom_state
  commands.csv   t_s, kind, value          (kind = 'level' or 'rate')
  meta.json      run parameters
"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Float64


class ZoomResponseRecorder(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("zoom_response_recorder")
        ns = namespace.rstrip("/")
        qos = QoSProfile(depth=10)  # matches siyi_ros_node defaults (RELIABLE)

        self.level_pub = self.create_publisher(Float32, f"{ns}/zoom_level_cmd", qos)
        self.rate_pub = self.create_publisher(Float32, f"{ns}/zoom_rate_cmd", qos)
        self.state_sub = self.create_subscription(
            Float64, f"{ns}/camera/zoom_level", self._on_state, qos
        )

        self.t0: float | None = None
        self.states: list[tuple[float, float]] = []
        self.commands: list[tuple[float, str, float]] = []
        self._last_state: float | None = None

        self.get_logger().info(
            f"recorder ready: pub {ns}/zoom_level_cmd, {ns}/zoom_rate_cmd; "
            f"sub {ns}/camera/zoom_level"
        )

    # --- timing -----------------------------------------------------------
    def now(self) -> float:
        return time.monotonic()

    def start_clock(self) -> None:
        if self.t0 is None:
            self.t0 = self.now()

    def t(self) -> float:
        assert self.t0 is not None
        return self.now() - self.t0

    # --- callbacks --------------------------------------------------------
    def _on_state(self, msg: Float64) -> None:
        z = float(msg.data)
        self._last_state = z
        if self.t0 is not None:
            self.states.append((self.t(), z))

    # --- helpers ----------------------------------------------------------
    def wait_for_state(self, timeout_s: float = 15.0) -> bool:
        deadline = self.now() + timeout_s
        while rclpy.ok() and self.now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._last_state is not None:
                return True
        return False

    def spin_for(self, duration_s: float) -> None:
        end = self.now() + duration_s
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish_level(self, level: float, log: bool = True) -> None:
        msg = Float32(); msg.data = float(level)
        self.level_pub.publish(msg)
        if log:
            self.commands.append((self.t(), "level", float(level)))

    def publish_rate(self, rate: float, log: bool = True) -> None:
        msg = Float32(); msg.data = float(rate)
        self.rate_pub.publish(msg)
        if log:
            self.commands.append((self.t(), "rate", float(rate)))

    def announce_level(self, level: float) -> None:
        self.publish_level(level, log=True)
        self.get_logger().info(f"t={self.t():6.2f}s  cmd level={level:.2f}")

    def announce_rate(self, rate: float) -> None:
        self.publish_rate(rate, log=True)
        self.get_logger().info(f"t={self.t():6.2f}s  cmd rate={rate:+.2f}")


# --- profiles -------------------------------------------------------------

def _pre_condition_to_level(node: ZoomResponseRecorder, level: float, settle_pre_s: float) -> None:
    node.get_logger().info(f"pre-conditioning to {level:.2f} for {settle_pre_s:.1f}s")
    node.publish_level(level, log=False)
    end = time.monotonic() + settle_pre_s
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def run_level_step(node: ZoomResponseRecorder, steps: list[float], dwell_s: float,
                   settle_pre_s: float) -> None:
    if not steps:
        raise ValueError("--steps requires at least one target")
    _pre_condition_to_level(node, steps[0], settle_pre_s)
    node.start_clock()
    node.announce_level(steps[0])  # re-stamp at t=0+ so it appears in commands.csv
    node.spin_for(dwell_s)
    for target in steps[1:]:
        node.announce_level(target)
        node.spin_for(dwell_s)


def run_level_sine(node: ZoomResponseRecorder, center: float, amplitude: float,
                   period_s: float, duration_s: float, publish_hz: float,
                   settle_pre_s: float, zoom_min: float, zoom_max: float) -> None:
    """Continuously varying level command: center + amplitude * sin(2π t / period).

    Clamped to [zoom_min, zoom_max]. Published at publish_hz so commands.csv
    has a tight sample of the commanded curve.
    """
    start_level = max(zoom_min, min(zoom_max, center))
    _pre_condition_to_level(node, start_level, settle_pre_s)
    node.start_clock()

    period_pub = 1.0 / max(publish_hz, 1.0)
    omega = 2.0 * math.pi / max(period_s, 1e-3)
    t_start = node.now()
    next_pub = t_start
    end_t = t_start + duration_s
    while rclpy.ok() and node.now() < end_t:
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.now() >= next_pub:
            tau = node.now() - t_start
            level = max(zoom_min, min(zoom_max, center + amplitude * math.sin(omega * tau)))
            node.publish_level(level, log=True)
            next_pub += period_pub
    # Snap back to center to leave the camera in a tidy state.
    node.publish_level(start_level, log=True)
    node.spin_for(0.5)


def run_rate_const(node: ZoomResponseRecorder, rate_cmd: float, start_level: float,
                   duration_s: float, settle_pre_s: float, publish_hz: float) -> None:
    _pre_condition_to_level(node, start_level, settle_pre_s)
    node.start_clock()
    node.announce_rate(rate_cmd)  # stamped event at t=0
    period_pub = 1.0 / max(publish_hz, 1.0)
    next_pub = node.now() + period_pub
    end_t = node.now() + duration_s
    while rclpy.ok() and node.now() < end_t:
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.now() >= next_pub:
            node.publish_rate(rate_cmd, log=True)
            next_pub += period_pub
    _send_rate_stop(node)


def run_rate_sine(node: ZoomResponseRecorder, amplitude: float, bias: float,
                  period_s: float, start_level: float, duration_s: float,
                  publish_hz: float, settle_pre_s: float) -> None:
    """Sinusoidal rate command: bias + amplitude * sin(2π t / period)."""
    _pre_condition_to_level(node, start_level, settle_pre_s)
    node.start_clock()
    omega = 2.0 * math.pi / max(period_s, 1e-3)
    period_pub = 1.0 / max(publish_hz, 1.0)
    t_start = node.now()
    next_pub = t_start
    end_t = t_start + duration_s
    while rclpy.ok() and node.now() < end_t:
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.now() >= next_pub:
            tau = node.now() - t_start
            r = bias + amplitude * math.sin(omega * tau)
            node.publish_rate(r, log=True)
            next_pub += period_pub
    _send_rate_stop(node)


def run_rate_chirp(node: ZoomResponseRecorder, amplitude: float, f0_hz: float, f1_hz: float,
                   start_level: float, duration_s: float, publish_hz: float,
                   settle_pre_s: float) -> None:
    """Linear-frequency-sweep rate command: amp · sin(2π · (f0 + (f1-f0)·τ/T) · τ).

    Useful for system identification — covers a frequency band in a single run.
    """
    _pre_condition_to_level(node, start_level, settle_pre_s)
    node.start_clock()
    period_pub = 1.0 / max(publish_hz, 1.0)
    t_start = node.now()
    next_pub = t_start
    end_t = t_start + duration_s
    while rclpy.ok() and node.now() < end_t:
        rclpy.spin_once(node, timeout_sec=0.005)
        if node.now() >= next_pub:
            tau = node.now() - t_start
            f_inst = f0_hz + (f1_hz - f0_hz) * (tau / max(duration_s, 1e-3))
            phase = 2.0 * math.pi * f_inst * tau
            r = amplitude * math.sin(phase)
            node.publish_rate(r, log=True)
            next_pub += period_pub
    _send_rate_stop(node)


def _send_rate_stop(node: ZoomResponseRecorder) -> None:
    node.announce_rate(0.0)
    stop_msg = Float32(); stop_msg.data = 0.0
    for _ in range(5):
        node.rate_pub.publish(stop_msg)
        time.sleep(0.05)
    node.spin_for(1.5)


# --- io -------------------------------------------------------------------

def write_outputs(run_dir: Path, node: ZoomResponseRecorder, meta: dict) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)

    with (run_dir / "states.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "zoom_state"])
        for t, z in node.states:
            w.writerow([f"{t:.4f}", f"{z:.4f}"])

    with (run_dir / "commands.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "kind", "value"])
        for t, kind, v in node.commands:
            w.writerow([f"{t:.4f}", kind, f"{v:.4f}"])

    (run_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {run_dir}/states.csv ({len(node.states)} rows)")
    print(f"wrote {run_dir}/commands.csv ({len(node.commands)} rows)")
    print(f"wrote {run_dir}/meta.json")


# --- cli ------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--namespace", required=True, help="ROS namespace (e.g. /px4_2)")
    p.add_argument("--mode", choices=["level", "rate"], default="level")
    p.add_argument("--profile", choices=["step", "sine", "chirp", "const"], default=None,
                   help="step/sine for level; const/sine/chirp for rate. "
                        "Defaults: step (level mode), const (rate mode).")
    p.add_argument("--publish-hz", type=float, default=25.0,
                   help="command republish rate; default 25 Hz (matches policy loop)")

    # level/step
    p.add_argument("--steps", type=float, nargs="+",
                   default=[1.0, 6.0, 1.0, 4.0, 1.0, 2.0, 1.0],
                   help="level/step: sequence of target zooms")
    p.add_argument("--dwell-s", type=float, default=3.0,
                   help="level/step: seconds to hold each step")

    # rate/const
    p.add_argument("--rate-cmd", type=float, default=1.0,
                   help="rate/const: constant rate value")

    # rate (any) / level/sine — common
    p.add_argument("--start-level", type=float, default=1.0,
                   help="zoom level to pre-condition to before timing starts")
    p.add_argument("--duration-s", type=float, default=10.0,
                   help="total run duration for non-step profiles")

    # sine (level or rate)
    p.add_argument("--sine-period-s", type=float, default=4.0,
                   help="sine profile period in seconds")
    p.add_argument("--sine-amplitude", type=float, default=None,
                   help="sine amplitude (level units for level/sine, levels/s for rate/sine). "
                        "Default 1.5 (level) / 1.0 (rate)")
    p.add_argument("--sine-center", type=float, default=3.5,
                   help="level/sine: center level (default midpoint of A8 range)")
    p.add_argument("--sine-rate-bias", type=float, default=0.0,
                   help="rate/sine: DC offset on the rate")

    # chirp (rate)
    p.add_argument("--chirp-f0-hz", type=float, default=0.1)
    p.add_argument("--chirp-f1-hz", type=float, default=2.0)
    p.add_argument("--chirp-amplitude", type=float, default=1.0)

    p.add_argument("--zoom-min", type=float, default=1.0)
    p.add_argument("--zoom-max", type=float, default=6.0)
    p.add_argument("--settle-pre-s", type=float, default=2.5)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--run-name", type=str, default=None)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.profile is None:
        args.profile = "step" if args.mode == "level" else "const"
    if args.mode == "level" and args.profile not in ("step", "sine"):
        print(f"--profile {args.profile} not valid for --mode level", file=sys.stderr); return 2
    if args.mode == "rate" and args.profile not in ("const", "sine", "chirp"):
        print(f"--profile {args.profile} not valid for --mode rate", file=sys.stderr); return 2

    rclpy.init()
    node = ZoomResponseRecorder(args.namespace)

    if not node.wait_for_state(timeout_s=15.0):
        node.get_logger().error(
            f"no message on {args.namespace}/camera/zoom_level — is siyi_ros_node up?"
        )
        node.destroy_node(); rclpy.shutdown(); return 1

    run_name = args.run_name or (datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                                 + f"_{args.mode}_{args.profile}")
    run_dir = args.output.expanduser() / run_name

    meta = {
        "namespace": args.namespace,
        "mode": args.mode,
        "profile": args.profile,
        "publish_hz": args.publish_hz,
        "settle_pre_s": args.settle_pre_s,
        "started_at": datetime.now().isoformat(timespec="seconds"),
    }

    try:
        if args.mode == "level" and args.profile == "step":
            meta.update({"steps": args.steps, "dwell_s": args.dwell_s})
            run_level_step(node, args.steps, args.dwell_s, args.settle_pre_s)
        elif args.mode == "level" and args.profile == "sine":
            amp = args.sine_amplitude if args.sine_amplitude is not None else 1.5
            meta.update({
                "sine_center": args.sine_center, "sine_amplitude": amp,
                "sine_period_s": args.sine_period_s, "duration_s": args.duration_s,
                "zoom_min": args.zoom_min, "zoom_max": args.zoom_max,
            })
            run_level_sine(node, args.sine_center, amp, args.sine_period_s,
                           args.duration_s, args.publish_hz, args.settle_pre_s,
                           args.zoom_min, args.zoom_max)
        elif args.mode == "rate" and args.profile == "const":
            meta.update({
                "rate_cmd": args.rate_cmd, "start_level": args.start_level,
                "duration_s": args.duration_s,
            })
            run_rate_const(node, args.rate_cmd, args.start_level, args.duration_s,
                           args.settle_pre_s, args.publish_hz)
        elif args.mode == "rate" and args.profile == "sine":
            amp = args.sine_amplitude if args.sine_amplitude is not None else 1.0
            meta.update({
                "sine_amplitude": amp, "sine_rate_bias": args.sine_rate_bias,
                "sine_period_s": args.sine_period_s,
                "start_level": args.start_level, "duration_s": args.duration_s,
            })
            run_rate_sine(node, amp, args.sine_rate_bias, args.sine_period_s,
                          args.start_level, args.duration_s, args.publish_hz,
                          args.settle_pre_s)
        else:  # rate / chirp
            meta.update({
                "chirp_amplitude": args.chirp_amplitude,
                "chirp_f0_hz": args.chirp_f0_hz, "chirp_f1_hz": args.chirp_f1_hz,
                "start_level": args.start_level, "duration_s": args.duration_s,
            })
            run_rate_chirp(node, args.chirp_amplitude, args.chirp_f0_hz, args.chirp_f1_hz,
                           args.start_level, args.duration_s, args.publish_hz,
                           args.settle_pre_s)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted — saving partial data")
    finally:
        if args.mode == "rate":
            try:
                _send_rate_stop(node)
            except Exception:
                pass
        write_outputs(run_dir, node, meta)
        node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
