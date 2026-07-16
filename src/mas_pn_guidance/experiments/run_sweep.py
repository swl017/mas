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


def param_get(node: str, param: str, timeout: int = 8) -> str:
    """`ros2 param get <node> <param>` -> the bare value token, or 'NA'."""
    try:
        out = subprocess.run(["ros2", "param", "get", node, param],
                             capture_output=True, text=True, timeout=timeout).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return "NA"
    # "Double value is: 75.0" / "String value is: x" -> the trailing token
    return out.rsplit(":", 1)[-1].strip() if ":" in out else (out or "NA")


def _schedule_fp(pn: str) -> dict:
    """Fingerprint the replayed active-sensing schedule (023 opt_weave/fim_mpc proxy
    arms) so a boot's schedule is auditable without archiving the full knot vector:
    (n_knots, checksum). Faithful 026 F1/F2/F3 arms carry no schedule — their scalar
    `as_*` params fully define them."""
    import ast
    tok = param_get(pn, "as_schedule_u_mps2")
    try:
        arr = [float(x) for x in ast.literal_eval(tok.strip())]
        return {"n_knots": len(arr), "checksum": round(sum(abs(x) for x in arr), 6)}
    except (ValueError, SyntaxError, TypeError):
        return {"n_knots": "NA", "checksum": "NA", "raw": tok[:80]}


def capture_live_config(ses: dict) -> dict:
    """Snapshot the live node params in effect for a boot so its ACTUAL config is
    auditable from the archive, not just implied by the boot id (ticket 019 M4/B1).
    Always reads the PN node (guidance mode/source/nav-constant, the ego-active weave
    params, AND the ticket 023/026 active-sensing arm config); adds the mock-cooperative
    geometry/latency when an observer is present. The nodes persist across boots in their
    own tmux sessions, so a post-boot read reflects the config the boot used.

    Active-sensing params that a node build predates (e.g. the 026 `as_aopn_*`/`as_dev_*`
    keys before that slice lands) read back as ``"NA"`` — a graceful gap, not a crash."""
    interc = ses.get("interceptor_ns", "px4_1")
    pn = f"/{interc}/pn_guidance_node"
    cfg = {
        "pn_guidance_node": pn,
        "pn": {p: param_get(pn, p) for p in
               ("guidance_mode", "estimate_source", "nav_constant",
                "ego_weave_amp_mps", "ego_weave_freq_hz", "ego_weave_taper_range_m",
                # ticket 023/026 active-sensing arm config — WITHOUT these the archived
                # live_config cannot prove which law produced a row (023 review finding 6).
                "active_sensing_class", "as_amp_mps2", "as_freq_hz", "as_taper_range_m",
                "as_aopn_n2", "as_aopn_sign", "as_dev_delta_deg", "as_dev_wash_range_m",
                "as_fim_lambda", "as_fim_horizon_s", "as_fim_samples",
                "as_fim_replan_ticks", "as_fim_hit_r_m", "as_schedule_dt_s")},
        "pn_schedule_fp": _schedule_fp(pn),
    }
    obs = ses.get("observer_ns", "")
    if obs:
        vo, cs = f"/{obs}/viewing_offset", f"/{interc}/cv_smoother"
        cfg["viewing_offset_node"] = vo
        cfg["viewing_offset"] = {p: param_get(vo, p)
                                 for p in ("offset_deg", "standoff_m", "height_m")}
        cfg["cv_smoother_node"] = cs
        cfg["cv_smoother"] = {p: param_get(cs, p)
                              for p in ("latency_s", "latency_jitter_s", "drop_p", "drop_burst")}
    return cfg


def _fmt_double(x) -> str:
    """Render a float as a fixed-point DOUBLE literal `ros2 param set` accepts — NO
    scientific notation (ros2 rejects e.g. `5e-05` -> param set FAILS -> boot skipped),
    always with a decimal point (`15` -> INTEGER, rejected by a DOUBLE param)."""
    s = f"{float(x):.12f}".rstrip("0").rstrip(".")
    return s if "." in s else s + ".0"


def _param_literals(v) -> list:
    """Candidate `ros2 param set` literals to try, in order. ros2 infers the type from
    the literal and REJECTS a mismatch (`64.0` -> a DOUBLE, rejected by an INTEGER param;
    `64` -> an INTEGER, rejected by a DOUBLE param; `5e-05` -> rejected outright). We
    don't know the param's declared type here, so for an integral value we offer BOTH the
    int and double literal (one matches); non-integral floats have only the double form."""
    if isinstance(v, bool):
        return [str(v)]
    if isinstance(v, int):
        return [str(v), _fmt_double(v)]            # int param first ('64'), double fallback
    if isinstance(v, float):
        if v == int(v):
            return [_fmt_double(v), str(int(v))]    # double first ('3.0'), int fallback
        return [_fmt_double(v)]                     # non-integral: fixed-point ('0.00005')
    return [str(v)]


def _param_set(node: str, p: str, v, timeout: int = 40, tries: int = 3) -> bool:
    """`ros2 param set` with retries — the CLI's per-call discovery can spike well
    past a few seconds when the graph is busy (a conductor/bag still tearing down at
    a boot boundary), so a single tight timeout is not robust for an unattended sweep.
    Tries each type-candidate literal (int vs double) so both int- and double-typed
    params set cleanly (ticket 026: as_fim_samples is INTEGER, as_aopn_n2 tiny DOUBLE)."""
    for _ in range(tries):
        for val in _param_literals(v):
            try:
                r = subprocess.run(["ros2", "param", "set", node, p, val],
                                   capture_output=True, text=True, timeout=timeout)
                if "successful" in (r.stdout + r.stderr).lower():
                    return True
            except (OSError, subprocess.TimeoutExpired):
                pass
        time.sleep(3)
    return False


def set_live_params(boot: dict, settle_s: float) -> None:
    """Apply a boot's declared live node params (mock-cooperative geometry/latency)
    BEFORE the boot, so the sweep is fully declarative + turnkey (no manual `ros2
    param set`) and every point is reproducible (ticket 019 B3: the zero-latency
    point goes through the identical cv_smoother node as the rest). Default-inert:
    a boot with no `live_params` key is a no-op. Raises RuntimeError if a param never
    takes (caller skips the boot rather than crashing the whole sweep). After a
    geometry change the observer must fly to the new parallax, so settle before engaging."""
    lp = boot.get("live_params") or {}
    if not lp:
        return
    changed = False
    for node, params in lp.items():
        for p, v in params.items():
            ok = _param_set(node, p, v)
            changed = changed or ok
            log(f"  set {node} {p}={v}: {'ok' if ok else 'FAILED (after retries)'}")
            if not ok:
                raise RuntimeError(f"live_params: could not set {node} {p}={v}")
    if changed and settle_s > 0:
        log(f"  live_params settle {settle_s:.0f}s (observer repositions to new geometry)")
        time.sleep(settle_s)


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
    cmd = [
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
        # forward the session namespaces so a non-default role layout (e.g. ticket-019
        # cooperative, target=px4_3) reaches the conductor instead of its px4_1/px4_2 default
        f"interceptor_ns:={ses.get('interceptor_ns', 'px4_1')}",
        f"target_ns:={ses.get('target_ns', 'px4_2')}",
    ]
    # observer ns (empty=non-coop, e.g. the ego-active baseline): flips on target-aware
    # recording of the observer odom + cooperative belief (ticket 019 M4). Only append
    # when set — `ros2 launch` rejects an empty-valued arg 'observer_ns:='.
    obs = ses.get("observer_ns", "")
    if obs:
        cmd.append(f"observer_ns:={obs}")
    return cmd


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
        # Live mock-cooperative config actually in effect for this boot (ticket 019 M4)
        "live_config": capture_live_config(ses),
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

    settle_s = float(ses.get("live_params_settle_s", 12.0))
    bag_dir = Path(ses.get("bag_dir", "/home/usrg/mas/bag"))
    for b in boots:
        # Resume-skip: a boot whose results already exist (e.g. after a mid-sweep
        # crash) is left as-is rather than crashing on the never-reuse guard.
        if (bag_dir / f"boot_{b['id']}_results.csv").exists():
            log(f"skip boot {b['id']} (results already exist — resume)")
            continue
        try:
            wait_ready(ses)
        except NotReady as e:
            log(f"ABORT before boot {b['id']}: {e}")
            return 3
        try:
            set_live_params(b, settle_s)   # declarative mock-coop geometry/latency (ticket 019)
        except Exception as e:             # one bad param-set skips one boot, not the sweep
            log(f"live_params {b['id']} failed: {e} — skipping boot")
            continue
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
