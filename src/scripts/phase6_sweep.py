#!/usr/bin/env python3
"""Phase 6: custom rectangular-imgsz engines matched to rtsp shape.

Depends on `src/ultralytics_ros/models/export_engines.sh` having been
run so the 12 rectangular engines exist in install/share:
    {yolov11{s,m,l}-drone,dronecop9-2}-384x640.engine
    {yolov11{s,m,l}-drone,dronecop9-2}-544x960.engine
    {yolov11{s,m,l}-drone,dronecop9-2}-1088x1920.engine

Each cell pairs an engine with its matched rtsp resolution:
    *-384x640.engine    →  rtsp  640×360
    *-544x960.engine    →  rtsp  960×540
    *-1088x1920.engine  →  rtsp 1920×1080

Scope trimmed from {n,s,m,l,x} to {s,m,l} yolov11-drone + dronecop9-2
to keep the sweep tractable (12 cells ≈ 17 min) after adding the 1080p
resolution axis. n (fast but low accuracy) and x (impractically slow
at 1080p) were both off the Pareto frontier after Phase 4 / 5.

Protocol: same as phase4_sweep.py / phase5_sweep.py — owns node
lifecycle, 20 s warmup, 60 s probe, 3 s teardown, process-group kill.
CSVs at /tmp/032/phase6_*.csv.
"""

import csv
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

ROBOT_NAME = os.environ.get("ROBOT_NAME", "px4_2")
RTSP_URL = os.environ.get("RTSP_URL", "rtsp://192.168.144.26:8554/main.264")
PROBE_TOPIC = f"/{ROBOT_NAME}/yolo_result_vision"
PROBE_TYPE = "vision_msgs/msg/Detection2DArray"
CSV_DIR = "/tmp/032"
DURATION_SEC = 60
WARMUP_SEC = 20
TEARDOWN_SEC = 3
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PROBE = os.path.join(SCRIPT_DIR, "stamp_age.py")

# (engine_suffix, rtsp_width, rtsp_height)
IMGSZ_PAIRS = [
    ("384x640", 640, 360),
    ("544x960", 960, 540),
    ("1088x1920", 1920, 1080),
]
MODEL_BASENAMES = [
    "yolov11s-drone",
    "yolov11m-drone",
    "yolov11l-drone",
    "dronecop9-2",
]


def check_conflicting_nodes():
    try:
        out = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=10,
        ).stdout
    except Exception as e:
        print(f"# WARNING: ros2 node list failed ({e}); continuing without precheck")
        return
    conflict = [n for n in out.splitlines() if "rtsp_camera_node" in n or "tracker_node" in n]
    if conflict:
        print("ERROR: the following nodes are already running and must be stopped:")
        for n in conflict:
            print(f"  {n}")
        sys.exit(2)


def check_engines_installed():
    share = subprocess.run(
        ["ros2", "pkg", "prefix", "ultralytics_ros"],
        capture_output=True, text=True, timeout=5,
    ).stdout.strip()
    if not share:
        print("ERROR: cannot resolve ultralytics_ros install prefix. Did you source install/setup.bash?")
        sys.exit(2)
    models_dir = os.path.join(share, "share", "ultralytics_ros", "models")
    missing = []
    for base in MODEL_BASENAMES:
        for suffix, _, _ in IMGSZ_PAIRS:
            name = f"{base}-{suffix}.engine"
            if not os.path.exists(os.path.join(models_dir, name)):
                missing.append(name)
    if missing:
        print("ERROR: missing engines in install/share:")
        for m in missing:
            print(f"  {m}")
        print(f"\nRun: cd src/ultralytics_ros/models && ./export_engines.sh")
        print("Then: colcon build --packages-select ultralytics_ros")
        sys.exit(2)


def spawn(cmd, log_path):
    log = open(log_path, "w", buffering=1)
    proc = subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )
    return proc, log


def kill_group(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=3)
        except Exception:
            pass
    except Exception:
        pass


def run_probe(csv_path):
    if os.path.exists(csv_path):
        os.remove(csv_path)
    p = subprocess.Popen(
        ["python3", PROBE, PROBE_TOPIC, PROBE_TYPE, "--csv", csv_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    try:
        time.sleep(DURATION_SEC)
    finally:
        kill_group(p)


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
    return {
        "N": len(ages),
        "dur": dur,
        "rate": len(ages) / dur if dur > 0 else 0.0,
        "p50": float(np.median(ages)),
        "p95": float(np.quantile(ages, 0.95)),
        "p99": float(np.quantile(ages, 0.99)),
        "mean": float(ages.mean()),
        "std": float(ages.std()),
    }


def parse_infer(log_path):
    try:
        last = None
        with open(log_path) as f:
            for line in f:
                if "infer N=" in line:
                    last = line
        if not last:
            return None
        m = re.search(r"p50=\s*([\d.]+)\s+p95=\s*([\d.]+)", last)
        if not m:
            return None
        return float(m.group(1)), float(m.group(2))
    except Exception:
        return None


def main():
    os.makedirs(CSV_DIR, exist_ok=True)
    check_conflicting_nodes()
    check_engines_installed()

    cells = []
    for base in MODEL_BASENAMES:
        # "yolov11s-drone" → "s"; "dronecop9-2" → "dronecop9-2"
        short = base.replace("yolov11", "").replace("-drone", "") if base.startswith("yolov11") else base
        for suffix, w, h in IMGSZ_PAIRS:
            model = f"{base}-{suffix}.engine"
            cells.append((short, suffix, model, w, h))

    total_min = len(cells) * (WARMUP_SEC + DURATION_SEC + TEARDOWN_SEC) / 60.0
    print(f"# Phase 6 custom-imgsz sweep — {len(cells)} cells ≈ {total_min:.1f} min")
    print(f"# ROBOT_NAME={ROBOT_NAME}  probe={PROBE_TOPIC}")
    print(f"# csv_dir={CSV_DIR}")
    print()

    results = []
    for short, suffix, model, w, h in cells:
        name = f"{short}_{suffix}"
        csv_path = os.path.join(CSV_DIR, f"phase6_{name}.csv")
        rtsp_log = os.path.join(CSV_DIR, f"phase6_{name}_rtsp.log")
        trk_log = os.path.join(CSV_DIR, f"phase6_{name}_tracker.log")

        print(f"→ {name}: model={model} rtsp={w}x{h}", flush=True)

        rtsp_cmd = [
            "ros2", "run", "rtsp_camera", "rtsp_camera_node",
            "--ros-args",
            "-r", f"__ns:=/{ROBOT_NAME}/camera",
            "-p", f"rtsp_url:={RTSP_URL}",
            "-p", "camera_name:=color",
            "-p", f"width:={w}", "-p", f"height:={h}",
            "-p", "latency_ms:=200",
            "-p", "drop_on_latency:=false",
            "-p", "use_tcp:=false",
            "-p", "do_retransmission:=false",
            "-p", "codec:=h264",
            "-p", "decoder:=avdec_h264",
        ]
        trk_cmd = [
            "ros2", "launch", "ultralytics_ros", "tracker_drone.launch.xml",
            f"ns:=/{ROBOT_NAME}",
            "input_topic:=camera/color/image_raw/compressed",
            f"yolo_model:={model}",
        ]

        rtsp_proc, rtsp_fh = spawn(rtsp_cmd, rtsp_log)
        trk_proc, trk_fh = spawn(trk_cmd, trk_log)
        try:
            time.sleep(WARMUP_SEC)
            run_probe(csv_path)
        finally:
            kill_group(trk_proc)
            kill_group(rtsp_proc)
            try: trk_fh.close()
            except Exception: pass
            try: rtsp_fh.close()
            except Exception: pass
            time.sleep(TEARDOWN_SEC)

        r = summarize(csv_path)
        infer = parse_infer(trk_log)
        results.append((name, r, infer))
        if r is None:
            print(f"  {name:22s} NO SAMPLES", flush=True)
        else:
            ip = f"{infer[0]:.1f}/{infer[1]:.1f}" if infer else "?/?"
            print(f"  {name:22s} N={r['N']:4d} rate={r['rate']:5.2f}Hz "
                  f"age p50={r['p50']:6.1f} p95={r['p95']:6.1f} p99={r['p99']:6.1f} "
                  f"infer p50/p95={ip}", flush=True)

    print("\n## Summary")
    hdr = f"{'cell':22s} {'N':>5} {'rate':>7} {'age_p50':>8} {'age_p95':>8} {'age_p99':>8} {'infer_p50':>10} {'infer_p95':>10}"
    print(hdr)
    print("-" * len(hdr))
    for name, r, infer in results:
        if r is None:
            print(f"{name:22s} NO SAMPLES")
            continue
        ip50 = f"{infer[0]:.1f}" if infer else "—"
        ip95 = f"{infer[1]:.1f}" if infer else "—"
        print(f"{name:22s} {r['N']:5d} {r['rate']:6.2f}Hz "
              f"{r['p50']:7.1f} {r['p95']:7.1f} {r['p99']:7.1f} "
              f"{ip50:>10} {ip95:>10}")


if __name__ == "__main__":
    main()
