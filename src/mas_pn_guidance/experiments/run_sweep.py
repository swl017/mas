#!/usr/bin/env python3
"""run_sweep.py — manifest-driven batch runner for the PN engagement conductor.

Runs a YAML manifest of conductor boots against a live Isaac Sim + PX4 SITL
stack, encoding the operational discipline that used to be tribal knowledge
(ticket-007 REPRODUCE.md; ticket-008/010 sweep logs):

  * one conductor at a time — waits until the previous conductor process is gone
  * readiness gate before every boot — both vehicles armed, interceptor at IC
  * health gate — a 1-trial oracle boot whose CPA must beat a reference bound
  * settle_error-streak watchdog — kills a boot that enters the stranded-chase
    cascade instead of burning the remaining trials (ticket-010 N2CT1 incident)
  * grounded-vehicle detection — a disarmed interceptor on the ground is
    reported explicitly (PX4 may then refuse re-arm: "High Accelerometer Bias";
    remedy is a sim reboot — see EXPERIMENT.md §failure-modes)
  * archiving + provenance — boot CSV/JSONL copied to the results dir with a
    provenance JSON (sha256 of the installed EKF launch + PN yaml, manifest)
  * QA hook — analysis/qa_target_tracking.py per boot (realized dynamics)

Usage:
  source /home/usrg/mas/install/setup.bash
  python3 run_sweep.py manifests/my_sweep.yaml [--dry-run] [--only BOOT_ID]
                                               [--skip-gate]

Boot ids are never reused: the runner refuses a boot whose results CSV already
exists in the bag dir or the results dir.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
EKF_INSTALLED = Path("/home/usrg/mas/install/mas_bearing_loc/share/mas_bearing_loc/"
                     "launch/engagement_ekf.launch.py")
PN_INSTALLED = Path("/home/usrg/mas/install/mas_pn_guidance/share/mas_pn_guidance/"
                    "config/pn_guidance.yaml")


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# --------------------------------------------------------------- ros helpers --
def topic_once(topic: str, field: str | None = None, timeout: int = 12) -> dict:
    """Parse `ros2 topic echo --once` output into {leaf_name: float|str}."""
    cmd = ["timeout", str(timeout), "ros2", "topic", "echo", "--once", topic]
    if field:
        cmd += ["--field", field]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True).stdout
    except OSError:
        return {}
    vals: dict = {}
    for line in out.splitlines():
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if not key or not val or key.startswith("-"):
            continue
        try:
            vals[key] = float(val)
        except ValueError:
            vals[key] = val
    return vals


def conductor_running() -> bool:
    out = subprocess.run(
        "pgrep -af experiment_conductor | grep -vE 'bash -c|/bin/bash|pgrep|grep'",
        shell=True, capture_output=True, text=True).stdout.strip()
    return bool(out)


# --------------------------------------------------------------- readiness ----
class NotReady(RuntimeError):
    pass


def wait_ready(ses: dict, timeout_s: float = 900.0) -> None:
    """Block until no conductor runs, both vehicles are armed, and the
    interceptor holds its IC. Raises NotReady on timeout or a grounded vehicle."""
    icx, icy = ses.get("interceptor_ic", [0.0, -50.0])
    tol = float(ses.get("ic_tol_m", 1.5))
    ns = ses.get("interceptor_ns", "px4_1")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if conductor_running():
            time.sleep(15)
            continue
        state = topic_once(f"/{ns}/mavros/state")
        pos = topic_once(f"/{ns}/common_frame/odom", "pose.pose.position")
        armed = state.get("armed") in (True, "true", "True", 1.0)
        x, y, z = pos.get("x"), pos.get("y"), pos.get("z")
        if x is None:
            time.sleep(10)
            continue
        at_ic = abs(x - icx) < tol and abs(y - icy) < tol
        if armed and at_ic:
            return
        if not armed and z is not None and z < 0.5:
            raise NotReady(
                f"{ns} is DISARMED ON THE GROUND at ({x:.1f},{y:.1f},{z:.2f}) — "
                "stranded-chase outcome. PX4 may refuse re-arm (High Accelerometer "
                "Bias); see EXPERIMENT.md §failure-modes (usual remedy: sim reboot).")
        time.sleep(10)
    raise NotReady(f"vehicles not ready within {timeout_s:.0f}s")


# --------------------------------------------------------------- boot runner --
def fmt_list(values) -> str:
    """Conductor numeric-list args must stay STRING: join + trailing comma."""
    if isinstance(values, (int, float, str)):
        values = [values]
    return ",".join(str(v) for v in values) + ","


def launch_cmd(ses: dict, boot: dict) -> list[str]:
    return [
        "ros2", "launch", "mas_pn_guidance", "experiment_conductor.launch.py",
        f"boot_id:={boot['id']}",
        f"geometries:={boot['geometries']}",
        "target_condition_mode:=capability_grid",
        f"target_forward_speeds:={fmt_list(boot['forward_speeds'])}",
        f"target_lateral_accels:={fmt_list(boot['lateral_accels'])}",
        f"estimators:={','.join(boot['estimators'])}",
        f"repeats:={boot.get('repeats', 1)}",
        f"seed:={ses.get('seed', 42)}",
        f"record:={'true' if ses.get('record', True) else 'false'}",
        f"bag_script:={ses.get('bag_script', '/home/usrg/mas/bag/rosbag_record_reduced.sh')}",
        f"use_sim_time:={'true' if ses.get('use_sim_time', True) else 'false'}",
    ]


def read_rows(csv_path: Path) -> list[dict]:
    if not csv_path.exists():
        return []
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def run_boot(ses: dict, boot: dict, policies: dict) -> tuple[str, list[dict]]:
    """Launch one conductor boot; watch for settle_error streaks.
    Returns (status, rows) with status in {ok, aborted_settle_streak, error}."""
    bag_dir = Path(ses.get("bag_dir", "/home/usrg/mas/bag"))
    csv_path = bag_dir / f"boot_{boot['id']}_results.csv"
    if csv_path.exists():
        raise RuntimeError(f"boot id {boot['id']} already used ({csv_path}); "
                           "boot ids are never reused")
    streak_limit = int(policies.get("settle_error_streak_abort", 3))
    cmd = launch_cmd(ses, boot)
    log(f"boot {boot['id']}: {' '.join(cmd[3:])}")
    # Keep the conductor's own log — settle_error causes are invisible without it.
    logf = open(bag_dir / f"boot_{boot['id']}_conductor.log", "w")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                            start_new_session=True)
    status = "ok"
    streak = 0
    seen = 0
    while proc.poll() is None:
        time.sleep(20)
        rows = read_rows(csv_path)
        for row in rows[seen:]:
            tag = (f"  {row.get('estimator', ''):18s} {row.get('target_condition', ''):22s} "
                   f"{row.get('result', ''):12s} {row.get('min_range_m', '')[:6]}")
            log(tag)
            streak = streak + 1 if row.get("result") == "settle_error" else 0
        seen = len(rows)
        if streak >= streak_limit:
            log(f"boot {boot['id']}: {streak} consecutive settle_error rows — "
                "stranded-chase cascade, killing conductor")
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            status = "aborted_settle_streak"
            break
    if proc.poll() not in (0, None) and status == "ok":
        status = "error"
    logf.close()
    return status, read_rows(csv_path)


# --------------------------------------------------------------- gate ---------
def run_gate(ses: dict, gate: dict, policies: dict, stamp: str) -> bool:
    boot = {
        "id": gate.get("id", f"GATE{stamp}"),
        "geometries": gate.get("geometry", "crossing"),
        "forward_speeds": [gate.get("forward_speed", 4.0)],
        "lateral_accels": [gate.get("lateral_accel", 0.0)],
        "estimators": [gate.get("estimator", "oracle")],
        "repeats": 1,
    }
    status, rows = run_boot(ses, boot, policies)
    ok = False
    for row in rows:
        if row.get("result") == "hit" and \
                float(row.get("min_range_m") or 99) < float(gate.get("max_cpa_m", 0.5)):
            ok = True
    log(f"health gate: {'PASS' if ok else 'FAIL'} "
        f"({[(r.get('result'), r.get('min_range_m')) for r in rows]})")
    return ok


# --------------------------------------------------------------- archiving ----
def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else "MISSING"


def archive(ses: dict, boot_id: str, status: str, out_dir: Path,
            manifest_path: Path) -> None:
    bag_dir = Path(ses.get("bag_dir", "/home/usrg/mas/bag"))
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in (f"boot_{boot_id}_results.csv", f"boot_{boot_id}_results.jsonl",
                 f"boot_{boot_id}_conductor.log"):
        src = bag_dir / name
        if src.exists():
            shutil.copy2(src, out_dir / src.name)
    prov = {
        "boot_id": boot_id,
        "status": status,
        "archived_at": datetime.now().isoformat(timespec="seconds"),
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "installed_ekf_launch": {"path": str(EKF_INSTALLED), "sha256": sha256(EKF_INSTALLED)},
        "installed_pn_yaml": {"path": str(PN_INSTALLED), "sha256": sha256(PN_INSTALLED)},
    }
    with open(out_dir / f"boot_{boot_id}_provenance.json", "w") as f:
        json.dump(prov, f, indent=2)


def run_qa(ses: dict, boot_id: str, out_dir: Path) -> None:
    qa = HERE / "analysis" / "qa_target_tracking.py"
    res = subprocess.run(
        [sys.executable, str(qa), boot_id,
         "--bag-dir", ses.get("bag_dir", "/home/usrg/mas/bag"),
         "--target-ns", ses.get("target_ns", "px4_2")],
        capture_output=True, text=True)
    (out_dir / f"boot_{boot_id}_qa.txt").write_text(res.stdout + res.stderr)
    flags = [ln for ln in res.stdout.splitlines() if "SAT" in ln]
    log(f"QA {boot_id}: {'flags: ' + '; '.join(flags) if flags else 'no SAT flags'}")


# --------------------------------------------------------------- main ---------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("manifest", type=Path)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the launch commands and exit")
    ap.add_argument("--only", metavar="BOOT_ID",
                    help="run a single boot from the manifest")
    ap.add_argument("--skip-gate", action="store_true")
    args = ap.parse_args()

    m = yaml.safe_load(args.manifest.read_text())
    ses = m.get("session", {})
    policies = m.get("policies", {})
    boots = [b for b in m.get("boots", [])
             if not args.only or b["id"] == args.only]
    if not boots:
        log("nothing to run (check --only spelling / manifest boots)")
        return 1
    out_dir = Path(m.get("output", {}).get("results_dir",
                                           str(HERE / "results" / args.manifest.stem)))
    stamp = datetime.now().strftime("%m%d%H%M")

    if args.dry_run:
        for b in boots:
            print(" ".join(launch_cmd(ses, b)))
        return 0

    gate = m.get("gate")
    if gate and not args.skip_gate:
        wait_ready(ses)
        gate = dict(gate)
        gate.setdefault("id", f"GATE{stamp}")
        if not run_gate(ses, gate, policies, stamp):
            log("gate failed — session unhealthy; aborting sweep "
                "(restart the sim and re-run)")
            return 2
        archive(ses, gate["id"], "gate", out_dir, args.manifest)

    for b in boots:
        try:
            wait_ready(ses)
        except NotReady as e:
            log(f"ABORT before boot {b['id']}: {e}")
            return 3
        status, rows = run_boot(ses, b, policies)
        archive(ses, b["id"], status, out_dir, args.manifest)
        try:
            run_qa(ses, b["id"], out_dir)
        except Exception as e:  # QA must never kill the sweep
            log(f"QA {b['id']} failed: {e}")
        n_valid = sum(r.get("result") in ("hit", "miss", "timeout") for r in rows)
        log(f"boot {b['id']} {status}: {len(rows)} rows ({n_valid} valid)")
        if status != "ok" and policies.get("on_boot_abort", "stop") == "stop":
            log("stopping sweep (policies.on_boot_abort: stop)")
            return 4
    log(f"sweep complete — results in {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
