#!/usr/bin/env python3
"""
Gimbal calibration — encoder verification + calibration sweep + zero offset.

Ticket mas/026. Extends mas/005 encoder hardware test.

Phases:
  1. Verification: small step commands in each axis, confirm encoder response
     (sign convention + stream continuity).
  2. Sweep: yaw/pitch forward then backward at 5 deg increments. Records
     commanded, state (0x0D heading-frame), and encoder (derived joint-frame)
     angles at each hold. Forward-backward delta = hysteresis.
  3. Zero offset (optional, --checkerboard): capture camera images while
     pointing at a checkerboard, estimate camera pose via PnP, compare with
     commanded/encoder angles.

Output:
  <output-dir>/calibration.csv     — per-sample log
  <output-dir>/summary.csv         — per-axis hysteresis + zero-offset stats
  <output-dir>/bag/                — ros2 bag (if --record-bag)

Prereqs:
  ros2 run gimbal_controller siyi_ros_node   (in another terminal)

Usage:
  python3 gimbal_calibration.py --phase all --output-dir /tmp/cal_$(date +%s)
  python3 gimbal_calibration.py --phase verify
  python3 gimbal_calibration.py --phase sweep --yaw-step 5 --pitch-step 5
  python3 gimbal_calibration.py --phase checkerboard --checkerboard 9x6 --square-size 0.025
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, CameraInfo


# A8 mini limits (from cameras.py). Calibration stays inside these.
YAW_MIN, YAW_MAX = -135.0, 135.0
PITCH_MIN, PITCH_MAX = -90.0, 25.0
# Conservative sweep bounds (avoid endstops where hysteresis is hardware-limit).
SWEEP_YAW_MIN, SWEEP_YAW_MAX = -90.0, 90.0
SWEEP_PITCH_MIN, SWEEP_PITCH_MAX = -80.0, 20.0


@dataclass
class Sample:
    phase: str
    axis: str
    direction: str          # "fwd" / "bwd" / "-"
    cmd_yaw: float
    cmd_pitch: float
    state_roll: float       # 0x0D heading-frame
    state_pitch: float
    state_yaw: float
    enc_roll: float         # derived joint-frame
    enc_pitch: float
    enc_yaw: float
    cam_yaw: float = math.nan   # from PnP, if available
    cam_pitch: float = math.nan
    cam_roll: float = math.nan
    stamp: float = 0.0


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('gimbal_calibration')

        qos = QoSProfile(depth=10)
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.cmd_pub = self.create_publisher(
            Vector3, 'siyi_gimbal_angles/command_rpy_deg', qos)

        self._state = None
        self._encoder = None
        self._state_count = 0
        self._enc_count = 0
        self.create_subscription(
            Vector3, 'siyi_gimbal_angles/state_rpy_deg',
            self._state_cb, qos)
        self.create_subscription(
            Vector3, 'siyi_gimbal_angles/encoder_rpy_deg',
            self._encoder_cb, qos)

        self._latest_image = None
        self._camera_info = None
        self.create_subscription(
            Image, 'camera/color/image_raw',
            self._image_cb, best_effort)
        self.create_subscription(
            CameraInfo, 'camera/color/camera_info',
            self._camera_info_cb, best_effort)

    def _state_cb(self, msg):
        self._state = msg
        self._state_count += 1

    def _encoder_cb(self, msg):
        self._encoder = msg
        self._enc_count += 1

    def _image_cb(self, msg):
        self._latest_image = msg

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def send_command(self, yaw_deg: float, pitch_deg: float):
        """Publish setpoint. Subscriber flips sign via yaw_direction/pitch_direction."""
        msg = Vector3()
        msg.x = 0.0
        msg.y = float(pitch_deg)
        msg.z = float(yaw_deg)
        self.cmd_pub.publish(msg)

    def wait_for_topics(self, timeout=5.0):
        """Wait until both state and encoder topics are producing."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._state is not None and self._encoder is not None:
                return True
        return False

    def hold_and_sample(self, settle_s: float) -> tuple:
        """Spin for settle_s, then return (state, encoder) snapshots."""
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self._state, self._encoder


def phase_verify(node: CalibrationNode, samples: list, settle: float):
    """Send small known commands in each axis, verify encoder sign + response."""
    print("\n=== Phase 1: Encoder verification ===")
    test_pts = [
        ("yaw",   +15.0,  0.0),
        ("yaw",   -15.0,  0.0),
        ("pitch",   0.0, -15.0),
        ("pitch",   0.0, +15.0),
        ("center",  0.0,   0.0),
    ]
    baseline_state_cnt = node._state_count
    baseline_enc_cnt = node._enc_count

    for axis, y, p in test_pts:
        print(f"  cmd: yaw={y:+.1f} pitch={p:+.1f}  (axis={axis})")
        node.send_command(y, p)
        state, enc = node.hold_and_sample(settle)
        if state is None or enc is None:
            print("    WARN: no state/encoder data")
            continue
        samples.append(Sample(
            phase="verify", axis=axis, direction="-",
            cmd_yaw=y, cmd_pitch=p,
            state_roll=state.x, state_pitch=state.y, state_yaw=state.z,
            enc_roll=enc.x, enc_pitch=enc.y, enc_yaw=enc.z,
            stamp=time.time(),
        ))
        print(f"    state:   roll={state.x:+.2f} pitch={state.y:+.2f} yaw={state.z:+.2f}")
        print(f"    encoder: roll={enc.x:+.2f} pitch={enc.y:+.2f} yaw={enc.z:+.2f}")

    # Continuity check: ensure topics ticked during this phase.
    dstate = node._state_count - baseline_state_cnt
    denc = node._enc_count - baseline_enc_cnt
    print(f"  stream: state +{dstate} msgs, encoder +{denc} msgs during phase")
    if dstate == 0 or denc == 0:
        print("  FAIL: no stream activity — check siyi_ros_node")


def phase_sweep(node: CalibrationNode, samples: list,
                yaw_step: float, pitch_step: float, settle: float):
    """Forward + backward sweep in yaw and pitch. Measures hysteresis."""
    print("\n=== Phase 2: Calibration sweep ===")

    def _axis_sweep(axis: str, lo: float, hi: float, step: float, fixed_other: float):
        assert step > 0
        # inclusive of hi within floating tolerance
        fwd = [lo + i * step for i in range(int(round((hi - lo) / step)) + 1)]
        bwd = list(reversed(fwd))
        for direction, seq in (("fwd", fwd), ("bwd", bwd)):
            print(f"  {axis} {direction}: {seq[0]:+.1f} -> {seq[-1]:+.1f} "
                  f"(step={step}, n={len(seq)})")
            for v in seq:
                if axis == "yaw":
                    y, p = v, fixed_other
                else:
                    y, p = fixed_other, v
                node.send_command(y, p)
                state, enc = node.hold_and_sample(settle)
                if state is None or enc is None:
                    continue
                samples.append(Sample(
                    phase="sweep", axis=axis, direction=direction,
                    cmd_yaw=y, cmd_pitch=p,
                    state_roll=state.x, state_pitch=state.y, state_yaw=state.z,
                    enc_roll=enc.x, enc_pitch=enc.y, enc_yaw=enc.z,
                    stamp=time.time(),
                ))

    _axis_sweep("yaw", SWEEP_YAW_MIN, SWEEP_YAW_MAX, yaw_step, fixed_other=0.0)
    # Return to neutral before pitch sweep
    node.send_command(0.0, 0.0)
    node.hold_and_sample(settle)
    _axis_sweep("pitch", SWEEP_PITCH_MIN, SWEEP_PITCH_MAX, pitch_step, fixed_other=0.0)


def phase_checkerboard(node: CalibrationNode, samples: list,
                       checkerboard: tuple, square_size: float, settle: float):
    """Zero-offset estimation via checkerboard PnP at a few gimbal angles.

    Assumes a fixed checkerboard mounted at known direction relative to the drone
    body (operator aims drone/gimbal so it is roughly centered in frame). We
    don't attempt to recover checkerboard world pose — only the camera->board
    orientation delta between commanded positions.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("  checkerboard phase needs opencv-python + numpy — skipping")
        return

    from cv_bridge import CvBridge
    bridge = CvBridge()

    print("\n=== Phase 3: Zero-offset (checkerboard PnP) ===")
    if node._camera_info is None:
        print("  no camera_info received — skipping")
        return

    K = np.array(node._camera_info.k, dtype=np.float64).reshape(3, 3)
    D = np.array(node._camera_info.d, dtype=np.float64)

    cb_cols, cb_rows = checkerboard  # inner corners
    objp = np.zeros((cb_rows * cb_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cb_cols, 0:cb_rows].T.reshape(-1, 2)
    objp *= square_size

    # A small grid that keeps the board in the FOV. Operator aligns at start.
    test_pts = [
        (0.0, 0.0),
        (+10.0, 0.0), (-10.0, 0.0),
        (0.0, +5.0), (0.0, -5.0),
    ]
    for y, p in test_pts:
        node.send_command(y, p)
        state, enc = node.hold_and_sample(settle)
        # pull the latest image
        rclpy.spin_once(node, timeout_sec=0.2)
        img_msg = node._latest_image
        if img_msg is None:
            print(f"    cmd yaw={y:+.1f} pitch={p:+.1f}: no image — skip")
            continue
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cb_cols, cb_rows))
        if not found:
            print(f"    cmd yaw={y:+.1f} pitch={p:+.1f}: checkerboard not found")
            continue
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
        if not ok:
            print(f"    cmd yaw={y:+.1f} pitch={p:+.1f}: PnP failed")
            continue
        R, _ = cv2.Rodrigues(rvec)
        # Convert camera->board rotation to Euler (XYZ). Gimbal yaw ≈ -cam_yaw.
        sy = math.hypot(R[0, 0], R[1, 0])
        cam_roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        cam_pitch = math.degrees(math.atan2(-R[2, 0], sy))
        cam_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

        samples.append(Sample(
            phase="checkerboard", axis="both", direction="-",
            cmd_yaw=y, cmd_pitch=p,
            state_roll=state.x if state else math.nan,
            state_pitch=state.y if state else math.nan,
            state_yaw=state.z if state else math.nan,
            enc_roll=enc.x if enc else math.nan,
            enc_pitch=enc.y if enc else math.nan,
            enc_yaw=enc.z if enc else math.nan,
            cam_yaw=cam_yaw, cam_pitch=cam_pitch, cam_roll=cam_roll,
            stamp=time.time(),
        ))
        print(f"    cmd yaw={y:+.1f} pitch={p:+.1f}  "
              f"enc=({enc.z:+.2f},{enc.y:+.2f})  "
              f"cam=({cam_yaw:+.2f},{cam_pitch:+.2f},{cam_roll:+.2f})")


def write_samples_csv(path: Path, samples: list):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "phase", "axis", "direction",
            "cmd_yaw", "cmd_pitch",
            "state_roll", "state_pitch", "state_yaw",
            "enc_roll", "enc_pitch", "enc_yaw",
            "cam_yaw", "cam_pitch", "cam_roll",
            "stamp",
        ])
        for s in samples:
            w.writerow([
                s.phase, s.axis, s.direction,
                s.cmd_yaw, s.cmd_pitch,
                s.state_roll, s.state_pitch, s.state_yaw,
                s.enc_roll, s.enc_pitch, s.enc_yaw,
                s.cam_yaw, s.cam_pitch, s.cam_roll,
                s.stamp,
            ])


def write_summary_csv(path: Path, samples: list):
    """Hysteresis + zero-offset per axis."""
    # Group sweep samples by axis + commanded value, pair fwd vs bwd.
    def _hysteresis(axis: str, meas_yaw: bool):
        fwd = {}
        bwd = {}
        for s in samples:
            if s.phase != "sweep" or s.axis != axis:
                continue
            cmd = s.cmd_yaw if meas_yaw else s.cmd_pitch
            meas = s.enc_yaw if meas_yaw else s.enc_pitch
            bucket = fwd if s.direction == "fwd" else bwd
            bucket[round(cmd, 2)] = meas
        pairs = [(k, fwd[k], bwd[k]) for k in fwd if k in bwd]
        if not pairs:
            return None
        diffs = [abs(a - b) for _, a, b in pairs]
        return dict(
            n=len(pairs),
            max_hyst=max(diffs),
            mean_hyst=sum(diffs) / len(diffs),
        )

    def _offset(meas_yaw: bool):
        """mean(enc - cmd) over sweep for this axis."""
        axis = "yaw" if meas_yaw else "pitch"
        errs = []
        for s in samples:
            if s.phase != "sweep" or s.axis != axis:
                continue
            cmd = s.cmd_yaw if meas_yaw else s.cmd_pitch
            meas = s.enc_yaw if meas_yaw else s.enc_pitch
            errs.append(meas - cmd)
        if not errs:
            return None
        mean = sum(errs) / len(errs)
        var = sum((e - mean) ** 2 for e in errs) / max(1, len(errs) - 1)
        return dict(n=len(errs), mean=mean, std=math.sqrt(var))

    def _cam_offset(meas_yaw: bool):
        errs = []
        for s in samples:
            if s.phase != "checkerboard":
                continue
            cmd = s.cmd_yaw if meas_yaw else s.cmd_pitch
            cam = s.cam_yaw if meas_yaw else s.cam_pitch
            if math.isnan(cam):
                continue
            # cmd is gimbal joint, cam is camera->board. Take delta vs cmd=0 sample if present.
            errs.append(cam - cmd)
        if not errs:
            return None
        mean = sum(errs) / len(errs)
        var = sum((e - mean) ** 2 for e in errs) / max(1, len(errs) - 1)
        return dict(n=len(errs), mean=mean, std=math.sqrt(var))

    rows = []
    for axis, meas_yaw in (("yaw", True), ("pitch", False)):
        hyst = _hysteresis(axis, meas_yaw)
        off = _offset(meas_yaw)
        cam_off = _cam_offset(meas_yaw)
        rows.append((
            axis,
            hyst["n"] if hyst else 0,
            hyst["max_hyst"] if hyst else math.nan,
            hyst["mean_hyst"] if hyst else math.nan,
            off["mean"] if off else math.nan,
            off["std"] if off else math.nan,
            cam_off["mean"] if cam_off else math.nan,
            cam_off["std"] if cam_off else math.nan,
        ))

    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "axis",
            "hyst_n", "hyst_max_deg", "hyst_mean_deg",
            "enc_minus_cmd_mean_deg", "enc_minus_cmd_std_deg",
            "cam_minus_cmd_mean_deg", "cam_minus_cmd_std_deg",
        ])
        w.writerows(rows)


def parse_checkerboard(s: str) -> tuple:
    cols, rows = s.lower().split("x")
    return int(cols), int(rows)


def start_rosbag(bag_dir: Path) -> subprocess.Popen:
    topics = [
        "siyi_gimbal_angles/state_rpy_deg",
        "siyi_gimbal_angles/encoder_rpy_deg",
        "siyi_gimbal_angles/command_rpy_deg",
        "gimbal_cmd_los_rate",
        "camera/color/camera_info",
        "mavros/imu/data",
    ]
    cmd = ["ros2", "bag", "record", "-o", str(bag_dir)] + topics
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", choices=["verify", "sweep", "checkerboard", "all"],
                    default="all")
    ap.add_argument("--yaw-step", type=float, default=5.0)
    ap.add_argument("--pitch-step", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=0.8,
                    help="seconds to wait at each setpoint before sampling")
    ap.add_argument("--output-dir", type=str,
                    default=f"/tmp/gimbal_calibration_{int(time.time())}")
    ap.add_argument("--record-bag", action="store_true",
                    help="record ros2 bag of gimbal + imu topics during run")
    ap.add_argument("--checkerboard", type=parse_checkerboard, default=(9, 6),
                    help="inner-corner grid, e.g. 9x6")
    ap.add_argument("--square-size", type=float, default=0.025,
                    help="checkerboard square size in meters")
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {out_dir}")

    rclpy.init()
    node = CalibrationNode()

    bag_proc = None
    if args.record_bag:
        bag_proc = start_rosbag(out_dir / "bag")
        time.sleep(0.5)  # give recorder time to subscribe

    # Graceful shutdown: return to neutral + stop bag
    def _shutdown(*_):
        try:
            node.send_command(0.0, 0.0)
        except Exception:
            pass
        if bag_proc is not None:
            bag_proc.send_signal(signal.SIGINT)
            try:
                bag_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                bag_proc.kill()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    signal.signal(signal.SIGINT, lambda *a: (_shutdown(), sys.exit(0)))

    if not node.wait_for_topics(timeout=10.0):
        print("ERROR: state_rpy_deg or encoder_rpy_deg not received — "
              "is siyi_ros_node running?")
        _shutdown()
        return 1

    samples = []

    # Always start by homing to neutral
    node.send_command(0.0, 0.0)
    node.hold_and_sample(args.settle)

    try:
        if args.phase in ("verify", "all"):
            phase_verify(node, samples, settle=args.settle)
        if args.phase in ("sweep", "all"):
            phase_sweep(node, samples,
                        yaw_step=args.yaw_step,
                        pitch_step=args.pitch_step,
                        settle=args.settle)
        if args.phase in ("checkerboard", "all"):
            phase_checkerboard(node, samples,
                               checkerboard=args.checkerboard,
                               square_size=args.square_size,
                               settle=args.settle)
    finally:
        node.send_command(0.0, 0.0)
        node.hold_and_sample(args.settle)

        write_samples_csv(out_dir / "calibration.csv", samples)
        write_summary_csv(out_dir / "summary.csv", samples)
        print(f"\nWrote {len(samples)} samples to {out_dir / 'calibration.csv'}")
        print(f"Wrote summary to {out_dir / 'summary.csv'}")

        _shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
