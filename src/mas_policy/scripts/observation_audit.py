#!/usr/bin/env python3
"""Sample N ticks of the deployed policy observation and compare against the
inputs that should have produced it. Report mean/std/min/max per slot to
detect drift and jitter relative to the IL training distribution.

Run:
    python3 observation_audit.py --ns /px4_1 --ticks 20

Topics consumed:
    {ns}/policy/observation        Float32MultiArray   # the assembled obs
    {ns}/common_frame/odom         Odometry            # body pose+vel
    {ns}/gimbal_state_rpy_deg      Vector3             # gimbal yaw/pitch (deg)
    {ns}/chosen_target_ray_w       Vector3Stamped      # target ray (world)
    {ns}/combined_ang_vel_w        Vector3Stamped      # body+gimbal ang vel
    {ns}/camera/zoom_level         Float64
    {ns}/mavros/imu/data           Imu                 # body lin acc
    {ns}/yolo_result_vision        Detection2DArray
"""

import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray


def quat_to_euler(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def quat_rotate_inverse(q_wxyz, v):
    w = q_wxyz[0]
    u = -q_wxyz[1:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


# IL training camera params (from iris_ma_env6_test_cfg.py)
IL_FOCAL_LENGTH = 24.0
IL_HORIZONTAL_APERTURE = 20.955
IL_IMAGE_WIDTH = 640
IL_IMAGE_HEIGHT = 480
IL_FX = IL_FOCAL_LENGTH * IL_IMAGE_WIDTH / IL_HORIZONTAL_APERTURE   # ≈ 732.96


class Sampler(Node):
    def __init__(self, ns: str, n_ticks: int):
        super().__init__('observation_audit')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        rel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.n_ticks = n_ticks
        self.samples = []   # list of dicts, one per obs tick

        # Cached latest-message state
        self._latest = {
            'obs': None, 'odom': None, 'gimbal': None, 'ray': None,
            'cav': None, 'zoom': None, 'imu': None, 'det': None,
        }

        self.create_subscription(Float32MultiArray, f'{ns}/policy/observation',
                                 self._on_obs, sensor_qos)
        self.create_subscription(Odometry, f'{ns}/common_frame/odom',
                                 lambda m: self._set('odom', m), sensor_qos)
        self.create_subscription(Vector3, f'{ns}/gimbal_state_rpy_deg',
                                 lambda m: self._set('gimbal', m), sensor_qos)
        self.create_subscription(Vector3Stamped, f'{ns}/chosen_target_ray_w',
                                 lambda m: self._set('ray', m), rel_qos)
        self.create_subscription(Vector3Stamped, f'{ns}/combined_ang_vel_w',
                                 lambda m: self._set('cav', m), sensor_qos)
        self.create_subscription(Float64, f'{ns}/camera/zoom_level',
                                 lambda m: self._set('zoom', m), sensor_qos)
        self.create_subscription(Imu, f'{ns}/mavros/imu/data',
                                 lambda m: self._set('imu', m), sensor_qos)
        self.create_subscription(Detection2DArray, f'{ns}/yolo_result_vision',
                                 lambda m: self._set('det', m), sensor_qos)

    def _set(self, key, msg):
        self._latest[key] = msg

    def _on_obs(self, msg):
        # Capture an aligned snapshot when each obs tick arrives
        if any(self._latest[k] is None for k in ['odom', 'gimbal', 'cav', 'zoom', 'imu']):
            return  # not all inputs ready yet

        sample = {
            't_wall': time.time(),
            't_sim': self.get_clock().now().nanoseconds / 1e9,
            'obs': np.array(msg.data, dtype=np.float64),
            'odom': self._latest['odom'],
            'gimbal': self._latest['gimbal'],
            'ray': self._latest['ray'],
            'cav': self._latest['cav'],
            'zoom': self._latest['zoom'].data,
            'imu': self._latest['imu'],
            'det': self._latest['det'],
        }
        self.samples.append(sample)

    def collect(self, max_wall: float = 30.0):
        t0 = time.time()
        while len(self.samples) < self.n_ticks:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > max_wall:
                break


def stats(arr):
    a = np.asarray(arr, dtype=np.float64)
    if a.size == 0:
        return float('nan'), float('nan'), float('nan'), float('nan')
    return float(a.mean()), float(a.std()), float(a.min()), float(a.max())


def fmt_stat(arr, label, expected_range=None, unit=""):
    m, s, lo, hi = stats(arr)
    spread = hi - lo
    note = ""
    if expected_range is not None:
        emin, emax = expected_range
        if m < emin or m > emax:
            note += " ✗OOD"
    return (f"  {label:<32s}  "
            f"mean={m:>+9.4f} ± {s:<8.4f}  "
            f"range=[{lo:>+8.3f}, {hi:>+8.3f}]  "
            f"jitter={spread:>7.4f}{unit}{note}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='/px4_1', help='Drone namespace')
    parser.add_argument('--ticks', type=int, default=20, help='Number of obs ticks to sample')
    args = parser.parse_args()

    rclpy.init()
    node = Sampler(args.ns, args.ticks)
    print(f"Sampling {args.ticks} obs ticks from {args.ns}/policy/observation ...")
    node.collect()
    if len(node.samples) < 2:
        print(f"ERROR: only {len(node.samples)} samples collected. Is the policy publishing?")
        rclpy.shutdown()
        return 1

    # --- Per-sample slot extraction ---
    rows = []
    for s in node.samples:
        obs = s['obs']
        o = s['odom']
        g = s['gimbal']
        ca = s['cav']
        imu = s['imu']
        det = s['det']

        pos_w = np.array([o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z])
        vel_w = np.array([o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z])
        qw = o.pose.pose.orientation.w; qx = o.pose.pose.orientation.x
        qy = o.pose.pose.orientation.y; qz = o.pose.pose.orientation.z
        ang_b = np.array([o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z])
        roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)

        gyaw_body = math.radians(g.z)
        gpitch_body = math.radians(g.y)

        if s['ray'] is not None:
            ray_w = np.array([s['ray'].vector.x, s['ray'].vector.y, s['ray'].vector.z])
        else:
            ray_w = np.zeros(3)
        cav_w = np.array([ca.vector.x, ca.vector.y, ca.vector.z])

        acc_b = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
        g_world = np.array([0.0, 0.0, -9.81])
        g_body = quat_rotate_inverse(np.array([qw, qx, qy, qz]), g_world)
        acc_kin_b = acc_b + g_body

        n_det = len(det.detections) if det is not None else 0

        rows.append({
            't_wall': s['t_wall'], 't_sim': s['t_sim'],
            'pos_w': pos_w, 'vel_w': vel_w,
            'rpy': np.array([roll, pitch, yaw]),
            'ang_b': ang_b, 'acc_kin_b': acc_kin_b,
            'gyaw_body': gyaw_body, 'gpitch_body': gpitch_body,
            'ray_w': ray_w, 'cav_w': cav_w,
            'zoom': s['zoom'], 'n_det': n_det,
            'obs': s['obs'],
        })

    n = len(rows)
    print(f"Collected {n} samples. sim_t spans [{rows[0]['t_sim']:.3f}, {rows[-1]['t_sim']:.3f}]s "
          f"(wall {rows[-1]['t_wall'] - rows[0]['t_wall']:.2f}s)")

    obs_dim = len(rows[0]['obs'])
    n_peers = (obs_dim - 31) // 16
    print(f"obs_dim = {obs_dim} = 31 ego + 16 × {n_peers} peer\n")

    print("=" * 100)
    print(f"DEPLOYED OBS — drift/jitter over {n} ticks")
    print("=" * 100)

    obs_arr = np.array([r['obs'] for r in rows])  # shape (n, obs_dim)

    # Slot-by-slot stats
    slot_labels = (
        [(f"obs[{i}] pos_w[{xyz}]",  None) for i, xyz in zip(range(0, 3), 'xyz')]
        + [(f"obs[{i}] vel_w[{xyz}]",  None) for i, xyz in zip(range(3, 6), 'xyz')]
        + [(f"obs[{i}] rpy[{r}]",      None) for i, r in zip(range(6, 9), ['roll', 'pitch', 'yaw'])]
        + [(f"obs[{i}] ang_b[{xyz}]",  None) for i, xyz in zip(range(9, 12), 'xyz')]
        + [(f"obs[{i}] acc_kin_b[{xyz}]", None) for i, xyz in zip(range(12, 15), 'xyz')]
        + [(f"obs[15] gimbal_yaw_body",   None)]
        + [(f"obs[16] gimbal_pitch_body", None)]
        + [(f"obs[{i}] ray_w[{xyz}]",  None) for i, xyz in zip(range(17, 20), 'xyz')]
        + [(f"obs[{i}] combined_av_w[{xyz}]", None) for i, xyz in zip(range(20, 23), 'xyz')]
        + [(f"obs[23] bbox_aoi (s)",      None)]
        + [(f"obs[24] zoom_level",         (1.0, 6.0))]
        + [(f"obs[25] effective_hfov (rad)", None)]
        + [(f"obs[{i}] bbox[{c}]", None) for i, c in zip(range(26, 30), ['cx', 'cy', 'w', 'h'])]
        + [(f"obs[30] bbox_empty",         None)]
    )

    print("--- EGO obs slots (0–30) ---")
    for i, (label, expr) in enumerate(slot_labels):
        print(fmt_stat(obs_arr[:, i], label, expected_range=expr))

    if n_peers > 0:
        for p in range(n_peers):
            base = 31 + 16 * p
            print(f"\n--- PEER {p} obs slots ({base}–{base + 15}) ---")
            peer_labels = (
                [f"peer{p} pos_w[{xyz}]"  for xyz in 'xyz']
                + [f"peer{p} vel_w[{xyz}]"  for xyz in 'xyz']
                + [f"peer{p} ray_w[{xyz}]"  for xyz in 'xyz']
                + [f"peer{p} cav_w[{xyz}]"  for xyz in 'xyz']
                + [f"peer{p} zoom",
                   f"peer{p} bbox_empty",
                   f"peer{p} data_age",
                   f"peer{p} bbox_age"]
            )
            for k, lab in enumerate(peer_labels):
                print(fmt_stat(obs_arr[:, base + k], lab))

    # ---- Distributional sanity vs IL training ----
    print("\n" + "=" * 100)
    print("OOD CHECKS vs IL training distribution")
    print("=" * 100)

    rpy = np.array([r['rpy'] for r in rows])
    yaw_deg = np.degrees(rpy[:, 2])
    body_yaw_il_range_deg = 11.0  # orientation_noise_std=0.2 rad
    pos = np.array([r['pos_w'] for r in rows])
    speed = np.linalg.norm(np.array([r['vel_w'] for r in rows]), axis=1)
    gyaw = np.array([r['gyaw_body'] for r in rows])
    gpitch = np.array([r['gpitch_body'] for r in rows])
    ray_norm = np.linalg.norm(np.array([r['ray_w'] for r in rows]), axis=1)
    n_det_arr = np.array([r['n_det'] for r in rows])

    checks = [
        ("altitude_z (m)",     pos[:, 2],                "training: 20–25",  (20.0, 35.0)),
        ("speed (m/s)",        speed,                    "training: 0–5",    (0.0, 8.0)),
        ("body_yaw (deg)",     yaw_deg,                  f"training: ±{body_yaw_il_range_deg}",
                                                                              (-body_yaw_il_range_deg, body_yaw_il_range_deg)),
        ("body_roll (deg)",    np.degrees(rpy[:, 0]),    "training: small",  (-30, 30)),
        ("body_pitch (deg)",   np.degrees(rpy[:, 1]),    "training: small",  (-30, 30)),
        ("gimbal_yaw_body (°)", np.degrees(gyaw),        "training: 0=fwd",  (-160, 160)),
        ("gimbal_pitch_body (°)", np.degrees(gpitch),    "training: ±45",    (-45, 45)),
        ("|ray_w|",            ray_norm,                 "training: 0 or 1", None),
        ("n_detections",       n_det_arr.astype(float), "0 or 1+",           None),
    ]
    for lab, arr, expected, range_ in checks:
        m, s, lo, hi = stats(arr)
        if range_ is not None:
            ood_frac = float(((arr < range_[0]) | (arr > range_[1])).mean())
            ood_marker = "✗OOD" if ood_frac > 0.5 else ("⚠" if ood_frac > 0.0 else "✓")
            ood_str = f"  {ood_marker} OOD-fraction={ood_frac:.2f}"
        else:
            ood_str = ""
        print(f"  {lab:<24s}  mean={m:>+8.3f} ± {s:<7.3f}  range=[{lo:>+8.2f}, {hi:>+8.2f}]    {expected}{ood_str}")

    # ---- Drift detection: linear trend over time ----
    print("\n" + "=" * 100)
    print("DRIFT DETECTION (linear regression over sim-time, should be ~0 at hover)")
    print("=" * 100)
    t_sim = np.array([r['t_sim'] for r in rows])
    t_sim = t_sim - t_sim[0]
    drift_targets = [
        ("pos_w_z slope (m/s)", pos[:, 2]),
        ("body_yaw slope (deg/s)", yaw_deg),
        ("gimbal_yaw_body slope (deg/s)", np.degrees(gyaw)),
        ("gimbal_pitch_body slope (deg/s)", np.degrees(gpitch)),
    ]
    for lab, arr in drift_targets:
        if len(arr) >= 3 and t_sim[-1] > 0:
            slope = np.polyfit(t_sim, arr, 1)[0]
        else:
            slope = float('nan')
        print(f"  {lab:<35s}  slope = {slope:+.4f}")

    print("\n" + "=" * 100)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
