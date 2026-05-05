#!/usr/bin/env python3
"""Fit a SIYI A8 zoom-response sim2real model from bench runs.

Inputs (each a directory written by zoom_response_test.py):

    level_step_quantum    single- and few-quantum step responses  → τ_d
    level_step_paired     paired in/out big-step responses        → v_max⁺ / v_max⁻
    rate_chirp            linear-frequency-sweep rate chirp       → bandwidth, τ₁
    rate_sine_saturated   rate amp >> v_max                       → saturation symmetry check

Output:

    src/scripts/sim2real_model_fitting/output/zoom_model.json

Model form (matches the on-vehicle integrator deployed in siyi_ros_node.py):

    target(t)       = clip( ∫ rate_cmd(t' − τ_d) dt' , [zoom_min, zoom_max] )
    pre_quantize(t) = first_order_lag( target , τ₁ )         # τ₁ may be 0
    state(t)        = round_to_quantum( saturate_slew( pre_quantize, v_max⁺/⁻ ), 0.1 )

τ_d, v_max⁺/⁻, τ₁ come from the runs above. quantum is a hardware constant.

Reference: ticket [src/doc/active/tickets/037-zoom-response-characterization/]
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import median

import numpy as np


REPO_ROOT = Path("/home/usrg/mas")
DATASETS_ROOT = REPO_ROOT / "datasets" / "zoom_response"
OUTPUT_PATH = Path(__file__).parent / "output" / "zoom_model.json"

QUANTUM = 0.1


def load_run(run_dir: Path):
    with (run_dir / "states.csv").open() as f:
        rows = [(float(r["t_s"]), float(r["zoom_state"])) for r in csv.DictReader(f)]
    t = np.array([r[0] for r in rows]); z = np.array([r[1] for r in rows])
    cmds: list[tuple[float, str, float]] = []
    with (run_dir / "commands.csv").open() as f:
        for r in csv.DictReader(f):
            cmds.append((float(r["t_s"]), r["kind"], float(r["value"])))
    meta = json.loads((run_dir / "meta.json").read_text())
    return t, z, cmds, meta


def state_at(t_query: float, t: np.ndarray, z: np.ndarray) -> float:
    idx = np.searchsorted(t, t_query, side="right") - 1
    return float(z[idx]) if idx >= 0 else float(z[0])


# --- τ_d from single-quantum step run ----------------------------------------

def measure_step_dead_times(run_dir: Path) -> tuple[list[dict], dict]:
    """Return per-step measurements + summary for the level_step_quantum run.

    Dead-time = first time |state - z0| ≥ one quantum (0.1) after t_cmd, minus
    t_cmd. Using one-quantum threshold (rather than 5% of the step) gives a
    measurement that is independent of step magnitude.
    """
    t, z, cmds, _meta = load_run(run_dir)
    level_cmds = [(tc, v) for tc, k, v in cmds if k == "level"]
    rows: list[dict] = []
    for i, (t_cmd, target) in enumerate(level_cmds):
        t_end = level_cmds[i + 1][0] if i + 1 < len(level_cmds) else float(t[-1])
        z0 = state_at(t_cmd, t, z)
        if abs(target - z0) < 1e-6:
            continue
        in_win = (t >= t_cmd) & (t <= t_end)
        tt = t[in_win]; zz = z[in_win]
        if tt.size < 2:
            continue
        moved = np.abs(zz - z0) >= QUANTUM - 1e-6
        if not np.any(moved):
            continue
        first_move_t = float(tt[np.argmax(moved)])
        rows.append({
            "t_cmd": float(t_cmd),
            "z0": float(z0),
            "target": float(target),
            "delta": float(target - z0),
            "dead_time_s": first_move_t - t_cmd,
        })

    if not rows:
        return rows, {"n": 0, "median_s": 0.0, "p05_s": 0.0, "p95_s": 0.0, "std_s": 0.0}

    samples = sorted(r["dead_time_s"] for r in rows)
    n = len(samples)
    summary = {
        "n": n,
        "median_s": float(samples[n // 2]),
        "p05_s": float(samples[max(0, int(0.05 * n))]),
        "p95_s": float(samples[min(n - 1, int(0.95 * n))]),
        "mean_s": float(sum(samples) / n),
        "std_s": float(np.std(samples, ddof=1)) if n > 1 else 0.0,
        "min_s": float(samples[0]),
        "max_s": float(samples[-1]),
    }
    return rows, summary


# --- v_max⁺ / v_max⁻ from paired in/out run -----------------------------------

def measure_v_max(run_dir: Path) -> dict:
    """Estimate v_max from large paired in/out step responses.

    Use 200 ms-window peak rate during the transient — robust against the
    100 Hz / 0.1-quantum quantization spikes. Separate by direction so we
    can detect asymmetry > 10 % (per ticket acceptance criterion).
    """
    t, z, cmds, _meta = load_run(run_dir)
    level_cmds = [(tc, v) for tc, k, v in cmds if k == "level"]
    in_peaks: list[float] = []
    out_peaks: list[float] = []
    in_avgs: list[float] = []
    out_avgs: list[float] = []
    for i, (t_cmd, target) in enumerate(level_cmds):
        t_end = level_cmds[i + 1][0] if i + 1 < len(level_cmds) else float(t[-1])
        z0 = state_at(t_cmd, t, z)
        delta = target - z0
        if abs(delta) < QUANTUM:
            continue
        in_win = (t >= t_cmd) & (t <= t_end)
        tt = t[in_win]; zz = z[in_win]
        if tt.size < 5:
            continue
        peak = smoothed_peak_rate(tt, zz, window_s=0.2)
        # avg rate keyed off the 90 % rise time (matches the plot's "avgrate")
        thr90 = z0 + 0.90 * delta
        sign = 1.0 if delta > 0 else -1.0
        crossed = (zz - thr90) * sign >= 0
        if np.any(crossed):
            t_rise = float(tt[np.argmax(crossed)] - t_cmd)
            avg = abs(delta) / t_rise if t_rise > 0 else 0.0
        else:
            avg = 0.0
        if delta > 0:
            in_peaks.append(peak); in_avgs.append(avg)
        else:
            out_peaks.append(peak); out_avgs.append(avg)

    return {
        "in":  _v_summary(in_peaks, in_avgs),
        "out": _v_summary(out_peaks, out_avgs),
    }


def _v_summary(peaks: list[float], avgs: list[float]) -> dict:
    if not peaks:
        return {"n": 0, "v_max_peak200ms_per_s": 0.0, "avg_rate_per_s": 0.0}
    return {
        "n": len(peaks),
        "v_max_peak200ms_per_s": float(median(peaks)),
        "v_max_peak200ms_max_per_s": float(max(peaks)),
        "avg_rate_per_s": float(median(avgs)),
    }


def smoothed_peak_rate(tt: np.ndarray, zz: np.ndarray, window_s: float) -> float:
    if tt.size < 2:
        return 0.0
    peak = 0.0; j = 0
    for i in range(tt.size):
        while j < i and tt[i] - tt[j] > window_s:
            j += 1
        if j == i:
            continue
        dt = tt[i] - tt[j]
        if dt > 0:
            r = abs(zz[i] - zz[j]) / dt
            if r > peak:
                peak = r
    return float(peak)


# --- First-order τ₁ from a clean constant-frequency rate-sine run ------------

def measure_first_order_tau(run_dir: Path, t_d_s: float) -> dict:
    """Estimate τ₁ from a constant-frequency rate-sine run.

    The chirp's sliding-window single-frequency fits are noisy because the
    instantaneous frequency moves *within* each window. A constant-frequency
    rate-sine run, well below v_max and well below the dead-time / bandwidth
    knee, gives a clean phase-lag measurement:

        observed_lag(ω) = τ_d  +  atan(ωτ₁) / ω
        ⇒ τ₁ = tan(ω · (observed_lag − τ_d)) / ω

    We compute observed_lag via cross-correlation of dz/dt (smoothed) vs
    rate_cmd. The result is more trustworthy than the chirp fit but only
    available when a constant-freq sine run exists in datasets/.
    """
    if not run_dir.exists():
        return {"available": False}
    t, z, cmds, meta = load_run(run_dir)
    rate_cmds = [(tc, v) for tc, k, v in cmds if k == "rate"]
    if not rate_cmds or t.size < 20:
        return {"available": False}
    period = float(meta.get("sine_period_s", 0.0))
    if period <= 0.0:
        return {"available": False}
    f_hz = 1.0 / period
    rc_t = np.array([tc for tc, _ in rate_cmds])
    rc_v = np.array([v for _, v in rate_cmds])

    # Smoothed dz/dt aligned to a 100 Hz grid over the active window.
    active = rc_v != 0.0
    if not np.any(active):
        return {"available": False}
    t_a, t_b = float(rc_t[active][0]), float(rc_t[active][-1])
    grid = np.arange(t_a + 0.5, t_b - 0.5, 0.01)
    if grid.size < 50:
        return {"available": False}
    z_grid = np.interp(grid, t, z)
    box_n = max(2, int(0.2 / 0.01))
    z_smooth = np.convolve(z_grid, np.ones(box_n) / box_n, mode="same")
    dz_dt = np.zeros_like(z_smooth)
    dz_dt[:-1] = (np.diff(z_smooth)) / 0.01
    cmd_grid = np.interp(grid, rc_t, rc_v)
    a = cmd_grid - cmd_grid.mean()
    b = dz_dt - dz_dt.mean()
    max_lag_steps = int(1.0 / 0.01)
    best_lag = 0; best_corr = -1.0
    for L in range(0, max_lag_steps + 1):
        x = a[: a.size - L]; y = b[L:]
        denom = float(np.sqrt((x * x).sum() * (y * y).sum()))
        if denom <= 0:
            continue
        c = float((x * y).sum() / denom)
        if c > best_corr:
            best_corr = c; best_lag = L
    observed_lag_s = best_lag * 0.01
    omega = 2.0 * math.pi * f_hz
    excess = observed_lag_s - t_d_s
    if excess <= 0.005:
        tau1 = 0.0
    else:
        tau1 = float(math.tan(min(omega * excess, math.pi / 2 - 0.05)) / omega)
    return {
        "available": True,
        "f_hz": f_hz,
        "observed_lag_s": observed_lag_s,
        "best_corr": best_corr,
        "tau1_s": tau1,
    }


# --- Bandwidth + first-order τ from chirp -------------------------------------

def estimate_bandwidth_and_tau(run_dir: Path,
                               t_d_estimate: float) -> dict:
    """Sliding-window amplitude/phase analysis of the chirp.

    For each window: fit a single-frequency sinusoid (instantaneous freq from
    chirp meta) to (a) the integrated rate command and (b) the zoom_state.
    Amplitude ratio gives the magnitude transfer; phase difference compared
    against the dead-time prediction gives the first-order content.

    Returns headline (bandwidth_hz_3db, first_order_tau_s) plus the per-bin
    samples for posterity.
    """
    t, z, cmds, meta = load_run(run_dir)
    rate_cmds = [(tc, v) for tc, k, v in cmds if k == "rate"]
    if not rate_cmds:
        return {"available": False}
    rc_t = np.array([tc for tc, _ in rate_cmds])
    rc_v = np.array([v for _, v in rate_cmds])

    # Build the integrated commanded level signal (clamped).
    zoom_min = float(meta.get("zoom_min", 1.0))
    zoom_max = float(meta.get("zoom_max", 6.0))
    z0 = state_at(rc_t[0], t, z)
    cmd_level = np.empty_like(rc_t)
    cmd_level[0] = max(zoom_min, min(zoom_max, z0))
    for k in range(1, rc_t.size):
        dt = rc_t[k] - rc_t[k - 1]
        avg = 0.5 * (rc_v[k] + rc_v[k - 1])
        cmd_level[k] = max(zoom_min, min(zoom_max, cmd_level[k - 1] + avg * dt))
    cmd_mean = float(cmd_level.mean())
    cmd_demean = cmd_level - cmd_mean

    # Resample state to the cmd grid for direct comparison.
    state_on_cmd = np.interp(rc_t, t, z)
    state_demean = state_on_cmd - state_on_cmd.mean()

    f0 = float(meta.get("chirp_f0_hz", 0.05))
    f1 = float(meta.get("chirp_f1_hz", 2.0))
    duration = float(meta.get("duration_s", rc_t[-1] - rc_t[0]))

    # Bin the run by 5 instantaneous-frequency windows at logarithmically
    # spaced centers. Skip the first 0.5 s (transient settling) and the
    # last 0.5 s (rate goes to 0).
    f_centers = np.geomspace(max(f0, 0.05), min(f1, 1.5), 6)
    bins: list[dict] = []
    for fc in f_centers:
        # Linear chirp: instantaneous f = f0 + (f1 - f0) * τ / T
        tau_center = (fc - f0) / max(f1 - f0, 1e-6) * duration
        # Window must hold at least 2 cycles or 0.5 s, whichever is longer.
        win = max(2.0 / fc, 0.5)
        t_win0 = max(0.5, tau_center - win / 2)
        t_win1 = min(duration - 0.5, tau_center + win / 2)
        if t_win1 - t_win0 < 0.4:
            continue
        m = (rc_t >= rc_t[0] + t_win0) & (rc_t <= rc_t[0] + t_win1)
        if m.sum() < 10:
            continue
        ttw = rc_t[m] - rc_t[0]
        cmd_w = cmd_demean[m]
        state_w = state_demean[m]
        # Fit cmd_w ≈ a_c * cos(2π f τ) + b_c * sin(2π f τ); same for state.
        omega = 2 * math.pi * fc
        c = np.cos(omega * ttw); s = np.sin(omega * ttw)
        D = np.column_stack([c, s])
        # Least-squares fit
        cmd_coef, *_ = np.linalg.lstsq(D, cmd_w, rcond=None)
        state_coef, *_ = np.linalg.lstsq(D, state_w, rcond=None)
        amp_cmd = float(math.hypot(*cmd_coef))
        amp_state = float(math.hypot(*state_coef))
        if amp_cmd < QUANTUM * 0.5:
            continue  # unobservable — cmd amplitude below half a quantum
        # Decomposition: signal ≈ a cos(ωt) + b sin(ωt) = A cos(ωt − φ),
        # where φ = atan2(b, a). A more-positive φ means the peak occurs
        # later in time, i.e. that signal lags. So state-lags-cmd is
        # (phase_state − phase_cmd), not the other way around.
        phase_cmd = math.atan2(cmd_coef[1], cmd_coef[0])
        phase_state = math.atan2(state_coef[1], state_coef[0])
        phase_lag = phase_state - phase_cmd
        # Wrap to [-π, π].
        phase_lag = ((phase_lag + math.pi) % (2 * math.pi)) - math.pi
        bins.append({
            "f_hz": float(fc),
            "amp_cmd": amp_cmd,
            "amp_state": amp_state,
            "ratio": amp_state / amp_cmd,
            "phase_lag_rad": float(phase_lag),
            "phase_lag_s": float(phase_lag) / (2 * math.pi * fc),
            "n_samples": int(m.sum()),
        })

    if not bins:
        return {"available": False}

    # 3 dB bandwidth: lowest f where ratio drops to 1/√2 ≈ 0.707.
    bandwidth_hz = None
    for k in range(1, len(bins)):
        if bins[k - 1]["ratio"] >= 0.707 and bins[k]["ratio"] < 0.707:
            # Linear interpolate in log-f.
            f_lo, r_lo = bins[k - 1]["f_hz"], bins[k - 1]["ratio"]
            f_hi, r_hi = bins[k]["f_hz"], bins[k]["ratio"]
            t_frac = (0.707 - r_lo) / (r_hi - r_lo) if r_hi != r_lo else 0.5
            bandwidth_hz = float(math.exp(
                math.log(f_lo) + t_frac * (math.log(f_hi) - math.log(f_lo))
            ))
            break
    if bandwidth_hz is None:
        # Either no rolloff seen (band too narrow) or all bins already attenuated.
        bandwidth_hz = bins[-1]["f_hz"] if bins[-1]["ratio"] >= 0.707 else bins[0]["f_hz"]

    # First-order τ₁ estimate: extra phase lag beyond pure dead-time should
    # behave as atan(ωτ₁). Solve for τ₁ at the highest-frequency observable bin.
    # Pure dead-time predicts phase_lag_s = τ_d (constant in seconds).
    # Total predicted phase_lag(ω) = ωτ_d + atan(ωτ₁).  Solve at the highest f.
    last = bins[-1]
    omega = 2 * math.pi * last["f_hz"]
    excess_rad = last["phase_lag_rad"] - omega * t_d_estimate
    # Allow only positive τ₁; if excess is ≤ 0 within numerical noise, set 0.
    if excess_rad <= 0.02:
        first_order_tau_s = 0.0
    else:
        # atan(ωτ₁) = excess_rad → τ₁ = tan(excess_rad) / ω
        first_order_tau_s = float(max(0.0, math.tan(min(excess_rad, math.pi / 2 - 0.05)) / omega))

    return {
        "available": True,
        "bandwidth_hz_3db": float(bandwidth_hz),
        "first_order_tau_s": float(first_order_tau_s),
        "bins": bins,
    }


# --- Saturation symmetry check ------------------------------------------------

def check_saturation(run_dir: Path, v_max_in: float, v_max_out: float) -> dict:
    t, z, cmds, _meta = load_run(run_dir)
    rate_cmds = [(tc, v) for tc, k, v in cmds if k == "rate"]
    if not rate_cmds or t.size < 5:
        return {"available": False}
    rc_t = np.array([tc for tc, _ in rate_cmds])
    rc_v = np.array([v for _, v in rate_cmds])
    active = rc_v != 0.0
    if not np.any(active):
        return {"available": False}
    t_a, t_s = float(rc_t[active][0]), float(rc_t[active][-1])
    mask = (t >= t_a + 0.3) & (t <= t_s - 0.3)
    if mask.sum() < 5:
        return {"available": False}
    tt = t[mask]; zz = z[mask]
    # Smoothed dz/dt
    dt = np.diff(tt)
    box_n = max(2, int(0.2 / max(np.median(dt), 1e-3)))
    kernel = np.ones(box_n) / box_n
    z_s = np.convolve(zz, kernel, mode="same")
    dz_s = np.diff(z_s) / np.maximum(dt, 1e-9)
    half = box_n // 2
    inner = dz_s[half : dz_s.size - half] if dz_s.size > 2 * half else dz_s
    if inner.size == 0:
        return {"available": False}
    pos = inner[inner > 0]; neg = inner[inner < 0]
    return {
        "available": True,
        "max_dz_dt_in_per_s": float(pos.max()) if pos.size else 0.0,
        "max_dz_dt_out_per_s": float(-neg.min()) if neg.size else 0.0,
        "max_in_vs_v_max":  float(pos.max() / v_max_in)  if (pos.size and v_max_in)  else 0.0,
        "max_out_vs_v_max": float(-neg.min() / v_max_out) if (neg.size and v_max_out) else 0.0,
    }


# --- Main ---------------------------------------------------------------------

def main() -> int:
    runs = {
        "step_quantum_dir": DATASETS_ROOT / "level_step_quantum",
        "step_paired_dir":  DATASETS_ROOT / "level_step_paired",
        "chirp_dir":        DATASETS_ROOT / "rate_chirp",
        "saturated_dir":    DATASETS_ROOT / "rate_sine_saturated",
    }
    rate_sine_dir = DATASETS_ROOT / "rate_sine"  # optional, for τ₁ cross-check
    for k, p in runs.items():
        if not p.exists():
            print(f"missing {k}: {p}")
            return 1

    quantum_rows, quantum_summary = measure_step_dead_times(runs["step_quantum_dir"])
    print(f"τ_d (one-quantum threshold, n={quantum_summary['n']}): "
          f"median={quantum_summary['median_s']*1000:.0f} ms  "
          f"p05={quantum_summary['p05_s']*1000:.0f}  "
          f"p95={quantum_summary['p95_s']*1000:.0f}  "
          f"std={quantum_summary['std_s']*1000:.0f}")

    paired = measure_v_max(runs["step_paired_dir"])
    v_in  = paired["in"]["v_max_peak200ms_per_s"]
    v_out = paired["out"]["v_max_peak200ms_per_s"]
    print(f"v_max  zoom-in:  {v_in:.2f}/s  (n={paired['in']['n']})")
    print(f"v_max  zoom-out: {v_out:.2f}/s  (n={paired['out']['n']})")
    if v_in > 0:
        asym = abs(v_in - v_out) / v_in
        print(f"v_max asymmetry: {asym*100:.1f} %  "
              f"({'symmetric' if asym < 0.10 else 'ASYMMETRIC > 10 %'})")

    chirp = estimate_bandwidth_and_tau(
        runs["chirp_dir"], t_d_estimate=quantum_summary["median_s"]
    )
    if chirp.get("available"):
        print(f"chirp:  bandwidth_3dB ≈ {chirp['bandwidth_hz_3db']:.3f} Hz  "
              f"τ₁(chirp) ≈ {chirp['first_order_tau_s']*1000:.0f} ms  (noisy estimate)")
        for b in chirp["bins"]:
            print(f"  f={b['f_hz']:.2f} Hz  ratio={b['ratio']:.3f}  "
                  f"lag={b['phase_lag_s']*1000:+.0f} ms  n={b['n_samples']}")
    else:
        print("chirp analysis unavailable (insufficient data)")

    sine_tau = measure_first_order_tau(rate_sine_dir, quantum_summary["median_s"])
    if sine_tau.get("available"):
        print(f"rate-sine cross-check (f={sine_tau['f_hz']:.2f} Hz): "
              f"observed_lag={sine_tau['observed_lag_s']*1000:.0f} ms  "
              f"τ₁={sine_tau['tau1_s']*1000:.0f} ms  ρ={sine_tau['best_corr']:.3f}")
        # Prefer the constant-freq estimate when available.
        first_order_tau_s = sine_tau["tau1_s"]
    else:
        first_order_tau_s = chirp.get("first_order_tau_s", 0.0)

    sat = check_saturation(runs["saturated_dir"], v_in or 3.18, v_out or 3.18)
    if sat.get("available"):
        print(f"saturation: max in={sat['max_dz_dt_in_per_s']:.2f}/s  "
              f"max out={sat['max_dz_dt_out_per_s']:.2f}/s  "
              f"in/v_max={sat['max_in_vs_v_max']:.2f}  out/v_max={sat['max_out_vs_v_max']:.2f}")

    out = {
        "description": (
            "SIYI A8 mini zoom-response sim2real model. Fit from bench runs in "
            "datasets/zoom_response/. Hardware-side integrator is software-emulated "
            "in siyi_ros_node.zoom_rate_step_callback because 0x05 MANUAL_ZOOM is "
            "direction-only; only 0x0F ABSOLUTE_ZOOM accepts magnitudes. Sim should "
            "match this integrator shape (clamped, quantized, dead-time-buffered)."
        ),
        "deadtime_s_in":  round(quantum_summary["median_s"], 3),
        "deadtime_s_out": round(quantum_summary["median_s"], 3),
        "deadtime_std_s": round(quantum_summary["std_s"], 3),
        "deadtime_min_s": round(quantum_summary["min_s"], 3),
        "deadtime_max_s": round(quantum_summary["max_s"], 3),
        "v_max_in_per_s":  round(v_in, 2),
        "v_max_out_per_s": round(v_out, 2),
        "v_max_asymmetry": round(abs(v_in - v_out) / v_in, 3) if v_in else 0.0,
        "quantum": QUANTUM,
        "first_order_tau_s": round(first_order_tau_s, 3),
        "first_order_tau_source": "rate_sine" if sine_tau.get("available") else "chirp",
        "bandwidth_hz_3db": round(chirp.get("bandwidth_hz_3db", 0.0), 3),
        "bandwidth_note": (
            "approximate — chirp bins show single-frequency-fit noise; "
            "use a multi-point constant-freq sweep for tighter bound."
        ),
        "saturation_check": sat,
        "fit_provenance": {
            "step_quantum_dir": str(runs["step_quantum_dir"].relative_to(REPO_ROOT)),
            "step_paired_dir":  str(runs["step_paired_dir"].relative_to(REPO_ROOT)),
            "chirp_dir":        str(runs["chirp_dir"].relative_to(REPO_ROOT)),
            "saturated_dir":    str(runs["saturated_dir"].relative_to(REPO_ROOT)),
        },
        "raw_quantum_step_rows": quantum_rows,
        "raw_paired_summary":    paired,
        "raw_chirp_bins":        chirp.get("bins", []),
        "raw_rate_sine_tau":     sine_tau,
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
