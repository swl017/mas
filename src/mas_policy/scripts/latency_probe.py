#!/usr/bin/env python3
"""Measure latency through the deployment chain, including the mas_mission hop.

Two questions answered:

1. Action latency (policy → mas_mission → downstream):
   - For TwistStamped (cmd_vel), match upstream/downstream by header.stamp
     and compare wall-clock arrival times.
   - For Vector3 (gimbal_cmd_los_rate), match by exact (x, y, z) tuple
     and compare wall-clock arrival times.

2. Observation input freshness (per-input msg-stamp staleness vs sim-time):
   - For each topic that feeds the obs assembler, capture
     (sim_time_when_received) − msg.header.stamp. This is how stale that
     input is when it lands in the policy obs.

Run:
    python3 latency_probe.py --ns /px4_1 --duration 10
"""

import argparse
import sys
import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import Float32, Float64, Float32MultiArray
from geometry_msgs.msg import Vector3, Vector3Stamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from rosgraph_msgs.msg import Clock


def stat_summary(arr):
    a = np.asarray(arr, dtype=np.float64)
    if a.size == 0:
        return None
    return {
        'n': len(a), 'mean': float(a.mean()), 'std': float(a.std()),
        'min': float(a.min()), 'p50': float(np.median(a)),
        'p95': float(np.percentile(a, 95)), 'max': float(a.max()),
    }


def print_stats(label, st, unit="ms"):
    if st is None or st['n'] == 0:
        print(f"  {label:<55s}  no samples")
        return
    print(f"  {label:<55s}  "
          f"n={st['n']:>3d}  mean={st['mean']*1000:>6.2f} ± {st['std']*1000:>5.2f} "
          f"p50={st['p50']*1000:>6.2f}  p95={st['p95']*1000:>6.2f}  "
          f"max={st['max']*1000:>6.2f} {unit}")


class LatencyProbe(Node):
    def __init__(self, ns: str):
        super().__init__('latency_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=20)
        rel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=20)

        self._sim_now = None
        self.create_subscription(Clock, '/clock',
                                 lambda m: setattr(self, '_sim_now', m.clock.sec + m.clock.nanosec*1e-9),
                                 sensor_qos)

        # ---- Action chain: upstream (policy) and downstream (post-mission) ----
        # cmd_vel: TwistStamped, has header.stamp
        self._upstream_cmdvel = {}      # stamp_key -> (wall_arr, msg)
        self._downstream_cmdvel = {}
        self._cmdvel_pairs = []          # list of (wall_pub, wall_recv, dt)
        # gimbal_cmd_los_rate: Vector3, no header → match by data tuple
        self._upstream_gimbal = {}
        self._downstream_gimbal = {}
        self._gimbal_pairs = []
        # zoom_rate_cmd: Float32
        self._upstream_zoom = {}
        self._downstream_zoom = {}
        self._zoom_pairs = []

        self.create_subscription(TwistStamped, f'{ns}/policy/cmd_vel',
                                 lambda m: self._on_cmdvel(m, upstream=True), sensor_qos)
        self.create_subscription(TwistStamped, f'{ns}/cmd_vel',
                                 lambda m: self._on_cmdvel(m, upstream=False), sensor_qos)
        self.create_subscription(Vector3, f'{ns}/policy/gimbal_cmd_los_rate',
                                 lambda m: self._on_gimbal(m, upstream=True), rel_qos)
        self.create_subscription(Vector3, f'{ns}/gimbal_cmd_los_rate',
                                 lambda m: self._on_gimbal(m, upstream=False), rel_qos)
        self.create_subscription(Float32, f'{ns}/policy/zoom_rate_cmd',
                                 lambda m: self._on_zoom(m, upstream=True), rel_qos)
        self.create_subscription(Float32, f'{ns}/zoom_rate_cmd',
                                 lambda m: self._on_zoom(m, upstream=False), rel_qos)

        # ---- Observation input freshness: each topic's stamp vs sim time on receipt ----
        self._obs_stale = defaultdict(list)  # name -> list of (sim_time_recv − msg_stamp_sim)
        self._obs_arrival_jitter = defaultdict(list)  # name -> list of inter-arrival wall dt
        self._obs_last_wall = {}

        def stamp_sec(stamp):
            return stamp.sec + stamp.nanosec * 1e-9

        def make_stamp_cb(name):
            def cb(msg):
                if self._sim_now is None:
                    return
                if hasattr(msg, 'header'):
                    msg_t = stamp_sec(msg.header.stamp)
                    if msg_t > 0:
                        self._obs_stale[name].append(self._sim_now - msg_t)
                w = time.time()
                if name in self._obs_last_wall:
                    self._obs_arrival_jitter[name].append(w - self._obs_last_wall[name])
                self._obs_last_wall[name] = w
            return cb

        self.create_subscription(Odometry, f'{ns}/common_frame/odom',
                                 make_stamp_cb('odom'), sensor_qos)
        self.create_subscription(Vector3Stamped, f'{ns}/chosen_target_ray_w',
                                 make_stamp_cb('chosen_target_ray_w'), rel_qos)
        self.create_subscription(Vector3Stamped, f'{ns}/combined_ang_vel_w',
                                 make_stamp_cb('combined_ang_vel_w'), sensor_qos)
        self.create_subscription(Imu, f'{ns}/mavros/imu/data',
                                 make_stamp_cb('imu/data'), sensor_qos)
        self.create_subscription(Detection2DArray, f'{ns}/yolo_result_vision',
                                 make_stamp_cb('yolo_result_vision'), sensor_qos)
        # gimbal_state_rpy_deg is Vector3 (no header) — track only inter-arrival jitter
        self.create_subscription(Vector3, f'{ns}/gimbal_state_rpy_deg',
                                 make_stamp_cb('gimbal_state_rpy_deg'), sensor_qos)

        # The published policy/observation also lacks a header. Track its rate.
        self.create_subscription(Float32MultiArray, f'{ns}/policy/observation',
                                 make_stamp_cb('policy/observation'), sensor_qos)

    # ---- action chain pairing ----
    def _on_cmdvel(self, msg: TwistStamped, upstream: bool):
        wall = time.time()
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec,
               msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
               msg.twist.angular.z)
        if upstream:
            self._upstream_cmdvel[key] = wall
            if key in self._downstream_cmdvel:
                # Already arrived downstream first? unusual; record anyway
                dt = self._downstream_cmdvel[key] - wall
                self._cmdvel_pairs.append(dt)
        else:
            if key in self._upstream_cmdvel:
                dt = wall - self._upstream_cmdvel[key]
                self._cmdvel_pairs.append(dt)
            else:
                self._downstream_cmdvel[key] = wall
        # Cap memory
        if len(self._upstream_cmdvel) > 200:
            self._upstream_cmdvel = dict(list(self._upstream_cmdvel.items())[-100:])
        if len(self._downstream_cmdvel) > 200:
            self._downstream_cmdvel = dict(list(self._downstream_cmdvel.items())[-100:])

    def _on_gimbal(self, msg: Vector3, upstream: bool):
        wall = time.time()
        key = (round(msg.x, 6), round(msg.y, 6), round(msg.z, 6))
        if upstream:
            self._upstream_gimbal[key] = wall
            if key in self._downstream_gimbal:
                dt = self._downstream_gimbal[key] - wall
                self._gimbal_pairs.append(dt)
        else:
            if key in self._upstream_gimbal:
                dt = wall - self._upstream_gimbal[key]
                self._gimbal_pairs.append(dt)
            else:
                self._downstream_gimbal[key] = wall
        if len(self._upstream_gimbal) > 200:
            self._upstream_gimbal = dict(list(self._upstream_gimbal.items())[-100:])
        if len(self._downstream_gimbal) > 200:
            self._downstream_gimbal = dict(list(self._downstream_gimbal.items())[-100:])

    def _on_zoom(self, msg: Float32, upstream: bool):
        wall = time.time()
        key = round(msg.data, 6)
        if upstream:
            self._upstream_zoom[key] = wall
            if key in self._downstream_zoom:
                dt = self._downstream_zoom[key] - wall
                self._zoom_pairs.append(dt)
        else:
            if key in self._upstream_zoom:
                dt = wall - self._upstream_zoom[key]
                self._zoom_pairs.append(dt)
            else:
                self._downstream_zoom[key] = wall
        if len(self._upstream_zoom) > 200:
            self._upstream_zoom = dict(list(self._upstream_zoom.items())[-100:])
        if len(self._downstream_zoom) > 200:
            self._downstream_zoom = dict(list(self._downstream_zoom.items())[-100:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='/px4_1', help='Drone namespace')
    parser.add_argument('--duration', type=float, default=10.0, help='Wall-seconds to sample')
    args = parser.parse_args()

    rclpy.init()
    node = LatencyProbe(args.ns)

    print(f"Probing {args.ns} for {args.duration}s wall (then reporting)...")
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    print()
    print("=" * 100)
    print("ACTION CHAIN: policy/X  →  mas_mission  →  X    (wall-clock latency)")
    print("=" * 100)
    print_stats("policy/cmd_vel             → cmd_vel",
                stat_summary(node._cmdvel_pairs))
    print_stats("policy/gimbal_cmd_los_rate → gimbal_cmd_los_rate",
                stat_summary(node._gimbal_pairs))
    print_stats("policy/zoom_rate_cmd       → zoom_rate_cmd",
                stat_summary(node._zoom_pairs))

    print()
    print("=" * 100)
    print("OBS INPUT FRESHNESS: (sim_now − msg.header.stamp)  on each receipt")
    print("=" * 100)
    for name in ['odom', 'imu/data', 'chosen_target_ray_w', 'combined_ang_vel_w', 'yolo_result_vision']:
        print_stats(f"{name} staleness", stat_summary(node._obs_stale[name]))

    print()
    print("=" * 100)
    print("OBS INPUT INTER-ARRIVAL JITTER (wall-time gap between consecutive messages)")
    print("=" * 100)
    for name in ['odom', 'imu/data', 'gimbal_state_rpy_deg', 'chosen_target_ray_w',
                 'combined_ang_vel_w', 'yolo_result_vision', 'policy/observation']:
        print_stats(f"{name}", stat_summary(node._obs_arrival_jitter[name]))

    print()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
