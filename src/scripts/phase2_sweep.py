#!/usr/bin/env python3
"""Phase 2 single-knob ablation for ticket 032.

Assumes:
- rtsp_camera + tracker_node already running (e.g., drone.tmuxp.yaml).
- tracker_node built from the post-Phase-0 source so `ros2 param set`
  propagates to its cached slots via the on-set callback.

Flow:
1. Reset tracker params to baseline.
2. For each run in RUNS: apply overrides, settle, probe stamp_age on
   /px4_2/yolo_result_vision for DURATION seconds, summarize CSV.
3. Restore baseline.

Writes per-run CSVs to /tmp/032/phase2_<name>.csv and prints a single
summary table at the end.
"""

import csv
import os
import subprocess
import sys
import time

import numpy as np

TRACKER_NODE = "/px4_2/tracker_node"
PROBE_TOPIC = "/px4_2/yolo_result_vision"
PROBE_TYPE = "vision_msgs/msg/Detection2DArray"
CSV_DIR = "/tmp/032"
DURATION_SEC = 60
SETTLE_SEC = 3

BASELINE = {
    "conf_thres": 0.25,
    "iou_thres": 0.45,
    "max_det": 300,
    "classes": list(range(80)),
}

# yolov11m-drone.pt has `names = {0: 'drone'}` — single-class model,
# class id 0.
RUNS = [
    ("baseline_reconfirm", {}),
    ("max_det_2", {"max_det": 2}),
    ("max_det_1", {"max_det": 1}),
    ("conf_0.35", {"conf_thres": 0.35}),
    ("conf_0.45", {"conf_thres": 0.45}),
    ("iou_0.6", {"iou_thres": 0.6}),
    ("classes_drone", {"classes": [0]}),
]

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PROBE = os.path.join(SCRIPT_DIR, "stamp_age.py")


def _fmt_value(v):
    if isinstance(v, list):
        return "[" + ",".join(str(x) for x in v) + "]"
    return str(v)


def set_param(name, value):
    subprocess.run(
        ["ros2", "param", "set", TRACKER_NODE, name, _fmt_value(value)],
        check=True,
        stdout=subprocess.DEVNULL,
    )


def apply(params):
    for k, v in params.items():
        set_param(k, v)


def probe(name):
    csv_path = os.path.join(CSV_DIR, f"phase2_{name}.csv")
    if os.path.exists(csv_path):
        os.remove(csv_path)
    proc = subprocess.Popen(
        ["python3", PROBE, PROBE_TOPIC, PROBE_TYPE, "--csv", csv_path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(DURATION_SEC)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    return summarize(csv_path)


def summarize(path):
    try:
        rows = list(csv.DictReader(open(path)))
    except FileNotFoundError:
        return None
    if not rows:
        return None
    ages = np.array([float(r["age_ms"]) for r in rows])
    ts = np.array([float(r["t_received"]) for r in rows])
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    rate = len(ages) / dur if dur > 0 else 0.0
    return {
        "N": len(ages),
        "dur": dur,
        "rate": rate,
        "p50": float(np.median(ages)),
        "p95": float(np.quantile(ages, 0.95)),
        "p99": float(np.quantile(ages, 0.99)),
        "mean": float(ages.mean()),
        "std": float(ages.std()),
    }


def fmt_row(name, r):
    if r is None:
        return f"{name:20s} NO SAMPLES"
    return (
        f"{name:20s} "
        f"N={r['N']:4d}  rate={r['rate']:5.2f}Hz  "
        f"p50={r['p50']:6.1f}  p95={r['p95']:6.1f}  p99={r['p99']:6.1f}  "
        f"mean={r['mean']:6.1f}±{r['std']:5.1f}"
    )


def main():
    os.makedirs(CSV_DIR, exist_ok=True)
    total = len(RUNS) * (DURATION_SEC + SETTLE_SEC)
    print(f"# Phase 2 single-knob sweep — {len(RUNS)} runs × {DURATION_SEC}s "
          f"(~{total/60:.1f} min total, CSVs → {CSV_DIR}/phase2_*.csv)")
    print(f"# tracker={TRACKER_NODE}  probe={PROBE_TOPIC}")
    print()

    results = []
    try:
        for name, overrides in RUNS:
            apply(BASELINE)
            if overrides:
                apply(overrides)
            applied = {**BASELINE, **overrides}
            change = ", ".join(f"{k}={_fmt_value(v)}" for k, v in overrides.items()) or "(baseline)"
            print(f"→ {name}: {change}", flush=True)
            time.sleep(SETTLE_SEC)
            r = probe(name)
            results.append((name, r, applied))
            print("  " + fmt_row(name, r), flush=True)
    finally:
        apply(BASELINE)
        print("\n# baseline restored.")

    print("\n## Summary")
    for name, r, _ in results:
        print(fmt_row(name, r))


if __name__ == "__main__":
    main()
