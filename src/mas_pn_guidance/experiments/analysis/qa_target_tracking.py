#!/usr/bin/env python3
"""qa_target_tracking.py — realized-dynamics QA for capability-grid boots.

For each eng_<boot>_*_vf*_alat*_* bag, parse the REQUESTED (forward speed,
lateral accel) from the condition id and measure the target's REALIZED dynamics
over the mission==2 window: forward speed (mean v_x, heading=0), total 3-D speed
p95, lateral accel p95. Flags cells where realized materially differs from
requested (PX4/tilt saturation). settle_error trials have no engagement window
and drop out automatically.

Generalized from ticket-004 `analyze_target_tracking.py` (same math, CLI args).

Usage (sourced ROS2 shell):
  python3 qa_target_tracking.py <BOOT_ID> [--bag-dir /home/usrg/mas/bag]
                                          [--target-ns px4_2]
"""
import argparse
import glob
import os
import re

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import Int8


def _num(s):  # "7p1" -> 7.1, "6" -> 6.0
    return float(s.replace("p", "."))


def realized(bag, target_ns):
    db3 = glob.glob(os.path.join(bag, "*.db3"))
    if not db3:
        return None
    r = SequentialReader()
    r.open(StorageOptions(uri=db3[0], storage_id="sqlite3"), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=[f"/{target_ns}/common_frame/odom",
                                       f"/{target_ns}/mission_state"]))
    T, V, miss = [], [], []
    while r.has_next():
        tp, d, t = r.read_next()
        if tp.endswith("odom"):
            m = deserialize_message(d, Odometry)
            lv = m.twist.twist.linear
            T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            V.append([lv.x, lv.y, lv.z])
        elif tp.endswith("mission_state"):
            miss.append((t * 1e-9, deserialize_message(d, Int8).data))
    if len(T) < 5:
        return None
    T, V = np.array(T), np.array(V)
    on = [t for t, s in miss if s == 2]
    if on:
        t0 = min(on)
        off = [t for t, s in miss if s != 2 and t > t0]
        sel = (T >= t0) & (T <= (min(off) if off else T[-1]))
        if sel.sum() > 5:
            T, V = T[sel], V[sel]
    spd = np.linalg.norm(V, axis=1)
    dt = np.diff(T)
    dt[dt <= 0] = 1e-3
    acc = np.linalg.norm(np.diff(V, axis=0), axis=1) / dt
    fwd = float(np.mean(V[:, 0]))                  # heading=0 -> forward is +x
    return fwd, float(np.percentile(spd, 95)), float(np.percentile(acc, 95))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("boot")
    ap.add_argument("--bag-dir", default="/home/usrg/mas/bag")
    ap.add_argument("--target-ns", default="px4_2")
    a = ap.parse_args()

    bags = sorted(set(glob.glob(os.path.join(a.bag_dir, f"bag_*eng_{a.boot}_*"))))
    seen = {}
    for b in bags:
        m = re.search(r"vf([0-9p]+)_alat([0-9p]+)_f", os.path.basename(b))
        if not m:
            continue
        key = (_num(m.group(1)), _num(m.group(2)))
        if key in seen:
            continue
        rz = realized(b, a.target_ns)
        if rz:
            seen[key] = rz

    print(f"target-tracking QA boot={a.boot} — {len(seen)} conditions\n"
          f"{'vf_req':>6s} {'vf_real':>7s} | {'alat_req':>8s} {'alat_p95':>8s} | "
          f"{'tot_spd_p95':>11s}  flag")
    for (vf, al) in sorted(seen):
        fwd, tot, acc = seen[(vf, al)]
        flag = ""
        if abs(fwd - vf) / max(vf, 1e-3) > 0.15:
            flag += " FWD-SAT"
        if acc < 0.7 * al:
            flag += " ALAT-SAT"      # realized lateral accel well below request
        print(f"{vf:6.1f} {fwd:7.2f} | {al:8.2f} {acc:8.2f} | {tot:11.2f} {flag}")
    print("\n(ALAT p95 includes fwd-ramp/noise so it can read high; "
          "FWD-SAT or ALAT-SAT<0.7x = unrealized cell)")


if __name__ == "__main__":
    main()
