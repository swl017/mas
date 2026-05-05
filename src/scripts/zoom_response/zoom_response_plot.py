#!/usr/bin/env python3
"""Plot zoom step / rate response logs from zoom_response_test.py.

Reads a run directory containing states.csv + commands.csv + meta.json.
Adapts to the run's profile:

  * level / step   step-hold reference + per-step metrics (dead, rise90, settle, rates)
  * level / sine   continuous level reference vs state, mean/peak tracking error
  * rate  / const  state vs time, peak rate during the rate-cmd window
  * rate  / sine   commanded rate curve + integrated-cmd-level overlay vs state
  * rate  / chirp  same as sine, plus instantaneous-frequency annotation
"""

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class StepMetrics:
    t_cmd: float
    target: float
    z0: float
    dead_time_s: float | None
    rise_time_90_s: float | None
    settle_time_s: float | None      # |z - target| <= 0.1 (one A8 quantization step)
    avg_rate: float                  # |target - z0| / time-to-reach-90%, units/s
    peak_rate_smoothed: float        # max |dz/dt| over a 200 ms boxcar, units/s


def load_run(run_dir: Path):
    with (run_dir / "states.csv").open() as f:
        rows = [(float(r["t_s"]), float(r["zoom_state"])) for r in csv.DictReader(f)]
    states_t = np.array([r[0] for r in rows])
    states_z = np.array([r[1] for r in rows])

    cmds: list[tuple[float, str, float]] = []
    with (run_dir / "commands.csv").open() as f:
        for r in csv.DictReader(f):
            cmds.append((float(r["t_s"]), r["kind"], float(r["value"])))

    meta = json.loads((run_dir / "meta.json").read_text())
    return states_t, states_z, cmds, meta


def state_at(t_query: float, t: np.ndarray, z: np.ndarray) -> float:
    idx = np.searchsorted(t, t_query, side="right") - 1
    if idx < 0:
        return float(z[0]) if len(z) else float("nan")
    return float(z[idx])


def compute_step_metrics(t: np.ndarray, z: np.ndarray, level_cmds: list[tuple[float, float]],
                         total_t: float) -> list[StepMetrics]:
    out: list[StepMetrics] = []
    for i, (t_cmd, target) in enumerate(level_cmds):
        t_end = level_cmds[i + 1][0] if i + 1 < len(level_cmds) else total_t
        z0 = state_at(t_cmd, t, z)
        delta = target - z0
        in_win = (t >= t_cmd) & (t <= t_end)
        tt = t[in_win]; zz = z[in_win]
        if tt.size < 2 or abs(delta) < 1e-6:
            out.append(StepMetrics(t_cmd, target, z0, None, None, None, 0.0, 0.0))
            continue

        thr5 = z0 + 0.05 * delta
        thr90 = z0 + 0.90 * delta
        sign = 1.0 if delta > 0 else -1.0

        def first_crossing(threshold: float) -> float | None:
            crossed = (zz - threshold) * sign >= 0
            if not np.any(crossed):
                return None
            return float(tt[np.argmax(crossed)] - t_cmd)

        dead = first_crossing(thr5)
        rise = first_crossing(thr90)

        within = np.abs(zz - target) <= 0.1
        settle = None
        if np.any(within):
            stable_run = 0
            need = 0.3
            for k in range(len(tt)):
                if within[k]:
                    stable_run = stable_run if stable_run else k
                    if tt[k] - tt[stable_run] >= need:
                        settle = float(tt[stable_run] - t_cmd)
                        break
                else:
                    stable_run = 0

        peak_smoothed = smoothed_peak_rate(tt, zz, window_s=0.2)
        avg = abs(delta) / rise if (rise is not None and rise > 0) else 0.0
        out.append(StepMetrics(t_cmd, target, z0, dead, rise, settle, avg, peak_smoothed))
    return out


def smoothed_peak_rate(tt: np.ndarray, zz: np.ndarray, window_s: float) -> float:
    if tt.size < 2:
        return 0.0
    peak = 0.0
    j = 0
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


def cumulative_integrate(tc: np.ndarray, vc: np.ndarray, z0: float,
                         lo: float, hi: float) -> np.ndarray:
    """Trapezoidal integration of rate cmd vs time, clipped to [lo, hi].

    Mimics the gimbal node's integrator (without the watchdog) so the user
    can compare commanded-trajectory vs state. Clip is applied each step so
    the integrated reference doesn't run away when state saturates.
    """
    if tc.size == 0:
        return np.array([])
    out = np.empty_like(tc)
    out[0] = max(lo, min(hi, z0))
    for k in range(1, tc.size):
        dt = tc[k] - tc[k - 1]
        avg_rate = 0.5 * (vc[k] + vc[k - 1])
        out[k] = max(lo, min(hi, out[k - 1] + avg_rate * dt))
    return out


def cross_corr_lag(t_ref: np.ndarray, ref: np.ndarray,
                   t_state: np.ndarray, state: np.ndarray,
                   max_lag_s: float = 1.0) -> tuple[float, float]:
    """Estimate the lag at which state best aligns with reference.

    Returns (best_lag_s, normalized_correlation_at_lag).
    """
    if t_ref.size < 4 or t_state.size < 4:
        return 0.0, 0.0
    # Resample both to a common 100 Hz grid over the overlap window.
    t0 = max(t_ref.min(), t_state.min())
    t1 = min(t_ref.max(), t_state.max())
    if t1 - t0 < 0.5:
        return 0.0, 0.0
    grid = np.arange(t0, t1, 0.01)
    a = np.interp(grid, t_ref, ref)
    b = np.interp(grid, t_state, state)
    a = a - a.mean(); b = b - b.mean()
    max_lag = int(max_lag_s / 0.01)
    best = (0, -1.0)
    for L in range(-max_lag, max_lag + 1):
        if L >= 0:
            x = a[: a.size - L]; y = b[L:]
        else:
            x = a[-L:]; y = b[: b.size + L]
        denom = np.sqrt((x * x).sum() * (y * y).sum())
        if denom <= 0:
            continue
        c = float((x * y).sum() / denom)
        if c > best[1]:
            best = (L, c)
    return best[0] * 0.01, best[1]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("run_dir", type=Path)
    p.add_argument("--output", type=Path, default=None)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    out_path = args.output or (args.run_dir / "response.png")

    t, z, cmds, meta = load_run(args.run_dir)
    if t.size == 0:
        print("no state samples in states.csv"); return 1
    total_t = float(t[-1])
    mode = meta.get("mode", "?")
    profile = meta.get("profile", "step" if mode == "level" else "const")

    level_cmds = [(tc, v) for (tc, kind, v) in cmds if kind == "level"]
    rate_cmds = [(tc, v) for (tc, kind, v) in cmds if kind == "rate"]
    level_continuous = profile == "sine" and mode == "level"
    rate_continuous = profile in ("sine", "chirp") and mode == "rate"

    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(11, 7.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )

    # --- top panel: state + reference ---------------------------------------
    if level_cmds and not level_continuous:
        ref_t, ref_v = [], []
        for i, (tc, v) in enumerate(level_cmds):
            ref_t.append(tc); ref_v.append(v)
            t_next = level_cmds[i + 1][0] if i + 1 < len(level_cmds) else total_t
            ref_t.append(t_next); ref_v.append(v)
        ax_top.step(ref_t, ref_v, where="post", color="tab:red", linestyle="--",
                    alpha=0.6, label="commanded target")
        for tc, _ in level_cmds:
            ax_top.axvline(tc, color="gray", alpha=0.25, linewidth=0.8)
    elif level_continuous:
        lc_t = np.array([t for (t, _v) in level_cmds])
        lc_v = np.array([v for (_t, v) in level_cmds])
        ax_top.plot(lc_t, lc_v, color="tab:red", linestyle="--", alpha=0.7,
                    linewidth=1.0, label="commanded level")

    if rate_continuous and rate_cmds:
        # Integrate the commanded rate to a "virtual commanded level" for
        # comparison with state on the same axis. Use the first state sample
        # as the integration baseline so the curves are aligned.
        rc_t = np.array([t for (t, _v) in rate_cmds])
        rc_v = np.array([v for (_t, v) in rate_cmds])
        zoom_min = float(meta.get("zoom_min", 1.0))
        zoom_max = float(meta.get("zoom_max", 6.0))
        z0 = state_at(rc_t[0], t, z) if rc_t.size else float(z[0])
        cmd_level = cumulative_integrate(rc_t, rc_v, z0, zoom_min, zoom_max)
        ax_top.plot(rc_t, cmd_level, color="tab:red", linestyle="--", alpha=0.7,
                    linewidth=1.0, label="∫ rate_cmd dt (clipped)")

    ax_top.plot(t, z, color="tab:blue", linewidth=1.6, label="zoom_state")

    if not rate_continuous:
        # Condense consecutive same-value rate cmds (republished at publish_hz)
        # into a single annotation at the transition. Without this, a 25 Hz
        # republish for 5 s draws 128 overlapping vlines and labels.
        rate_transitions: list[tuple[float, float]] = []
        prev_v: float | None = None
        for tc, v in rate_cmds:
            if prev_v is None or abs(v - prev_v) > 1e-9:
                rate_transitions.append((tc, v))
                prev_v = v
        for tc, v in rate_transitions:
            ax_top.axvline(tc, color="tab:green" if v != 0 else "tab:gray",
                           alpha=0.5, linewidth=0.8)
            ax_top.text(tc, max(z) + 0.05, f"rate={v:+g}",
                        color="tab:green", fontsize=8, rotation=90,
                        va="bottom", ha="right")

    ax_top.set_title(f"SIYI A8 zoom response — mode={mode}, profile={profile}, "
                     f"ns={meta.get('namespace','?')}")
    ax_top.set_ylabel("zoom level")
    ax_top.grid(alpha=0.3)
    ax_top.legend(loc="upper right")

    # --- bottom panel: state derivative + (optional) rate cmd overlay -------
    if t.size >= 2:
        dz = np.diff(z); dt = np.diff(t)
        with np.errstate(divide="ignore", invalid="ignore"):
            rates = np.where(dt > 0, dz / dt, 0.0)
        t_mid = 0.5 * (t[1:] + t[:-1])
        ax_bot.plot(t_mid, rates, color="tab:purple", linewidth=0.8, alpha=0.6,
                    label="dz/dt (raw)")
        # 200 ms boxcar of state to suppress the 100 Hz quantization spikes.
        # The convolution's 'same' mode tapers samples within half a window
        # of each edge, producing artificial slope; trim that span on display.
        if t.size > 5:
            med_dt = float(np.median(dt))
            box_n = max(2, int(0.2 / max(med_dt, 1e-3)))
            kernel = np.ones(box_n) / box_n
            z_smooth = np.convolve(z, kernel, mode="same")
            dz_s = np.diff(z_smooth)
            with np.errstate(divide="ignore", invalid="ignore"):
                rates_s = np.where(dt > 0, dz_s / dt, 0.0)
            half = box_n // 2
            t_mid_disp = t_mid[half : t_mid.size - half] if t_mid.size > 2 * half else t_mid
            rates_s_disp = rates_s[half : rates_s.size - half] if rates_s.size > 2 * half else rates_s
            ax_bot.plot(t_mid_disp, rates_s_disp, color="tab:purple", linewidth=1.4,
                        label="dz/dt (200 ms smoothed)")

    if rate_continuous and rate_cmds:
        rc_t = np.array([t for (t, _v) in rate_cmds])
        rc_v = np.array([v for (_t, v) in rate_cmds])
        ax_bot.plot(rc_t, rc_v, color="tab:red", linestyle="--", linewidth=1.0,
                    alpha=0.7, label="rate_cmd")

    ax_bot.axhline(0, color="gray", linewidth=0.6)
    ax_bot.set_ylabel("dz/dt (1/s)")
    ax_bot.set_xlabel("time (s)")
    ax_bot.grid(alpha=0.3)
    ax_bot.legend(loc="upper right", fontsize=8)

    # --- metrics overlay ----------------------------------------------------
    metrics_text = build_metrics(mode, profile, t, z, level_cmds, rate_cmds, total_t, meta)
    if metrics_text:
        ax_top.text(
            0.01, 0.98, metrics_text,
            transform=ax_top.transAxes, fontsize=8, family="monospace",
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85, edgecolor="gray"),
        )

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"wrote {out_path}")
    if metrics_text:
        print(); print(metrics_text)
    return 0


def build_metrics(mode: str, profile: str,
                  t: np.ndarray, z: np.ndarray,
                  level_cmds: list[tuple[float, float]],
                  rate_cmds: list[tuple[float, float]],
                  total_t: float, meta: dict) -> str:
    if mode == "level" and profile == "step" and level_cmds:
        ms = compute_step_metrics(t, z, level_cmds, total_t)
        lines = [
            f"{'step':>4} {'cmd':>5} {'z0':>5} {'dead':>7} {'rise90':>8} {'settle':>8} "
            f"{'avgrate':>9} {'peak200ms':>10}"
        ]
        for i, m in enumerate(ms):
            dead = "n/a" if m.dead_time_s is None else f"{m.dead_time_s:.2f}s"
            rise = "n/a" if m.rise_time_90_s is None else f"{m.rise_time_90_s:.2f}s"
            settle = "n/a" if m.settle_time_s is None else f"{m.settle_time_s:.2f}s"
            lines.append(
                f"{i:>4} {m.target:>5.2f} {m.z0:>5.2f} {dead:>7} {rise:>8} {settle:>8} "
                f"{m.avg_rate:>7.2f}/s {m.peak_rate_smoothed:>8.2f}/s"
            )
        moving = [m for m in ms if abs(m.target - m.z0) > 0.1]
        if moving:
            lines.append(
                f"max avg-rate / max 200ms-peak: "
                f"{max(m.avg_rate for m in moving):.2f}/s  /  "
                f"{max(m.peak_rate_smoothed for m in moving):.2f}/s"
            )
        return "\n".join(lines)

    if mode == "level" and profile == "sine" and level_cmds:
        lc_t = np.array([tc for tc, _v in level_cmds])
        lc_v = np.array([v for _t, v in level_cmds])
        # Tracking error vs commanded level on the state grid.
        cmd_on_state = np.interp(t, lc_t, lc_v)
        # Drop a 0.5 s warmup so initial dead-time doesn't dominate the stats.
        warm = t > (t[0] + 0.5)
        if warm.sum() < 4:
            return ""
        err = z[warm] - cmd_on_state[warm]
        lag, corr = cross_corr_lag(lc_t, lc_v, t, z)
        return (
            f"level/sine: period={meta.get('sine_period_s','?')}s  "
            f"amp={meta.get('sine_amplitude','?')}  center={meta.get('sine_center','?')}\n"
            f"mean |state - cmd|: {np.mean(np.abs(err)):.3f}  "
            f"max |error|: {np.max(np.abs(err)):.3f}\n"
            f"best xcorr lag (state vs cmd): {lag:+.3f}s  ρ={corr:.3f}"
        )

    if mode == "rate" and profile == "const":
        ts_active = [tc for tc, v in rate_cmds if v != 0.0]
        ts_stop = [tc for tc, v in rate_cmds if v == 0.0]
        if not ts_active or not ts_stop or t.size < 2:
            return ""
        t_a, t_s = ts_active[0], ts_stop[0]
        mask = (t >= t_a) & (t <= t_s)
        if mask.sum() < 2:
            return ""
        tt = t[mask]; zz = z[mask]
        peak = smoothed_peak_rate(tt, zz, window_s=0.2)
        avg = (zz[-1] - zz[0]) / (tt[-1] - tt[0]) if tt[-1] > tt[0] else 0.0
        return (
            f"rate cmd active {t_a:.2f}s .. {t_s:.2f}s  cmd={meta.get('rate_cmd','?')}/s\n"
            f"z range: {zz.min():.2f} -> {zz.max():.2f}\n"
            f"avg dz/dt: {avg:+.2f}/s\n"
            f"peak |dz/dt| (200 ms window): {peak:.2f}/s"
        )

    if mode == "rate" and profile in ("sine", "chirp") and rate_cmds:
        rc_t = np.array([tc for tc, _v in rate_cmds])
        rc_v = np.array([v for _t, v in rate_cmds])
        if t.size < 5:
            return ""
        # Smoothed state derivative via forward differences. np.gradient with
        # mode='same' boxcar smoothing produces ~20/s spikes at the array
        # edges; np.diff plus an edge trim avoids those polluting the metrics.
        dt = np.diff(t)
        med_dt = float(np.median(dt))
        box_n = max(2, int(0.2 / max(med_dt, 1e-3)))
        kernel = np.ones(box_n) / box_n
        z_smooth = np.convolve(z, kernel, mode="same")
        dz_dt = np.zeros_like(z_smooth)
        dz_dt[:-1] = np.where(dt > 0, np.diff(z_smooth) / dt, 0.0)
        dz_dt[-1] = dz_dt[-2]
        active = rc_v != 0.0
        if not np.any(active):
            return ""
        t_a, t_s = float(rc_t[active][0]), float(rc_t[active][-1])
        edge_pad_s = 0.5 * box_n * med_dt
        mask = (t >= t_a + edge_pad_s) & (t <= t_s - edge_pad_s)
        if mask.sum() < 4:
            return ""
        cmd_on_state = np.interp(t[mask], rc_t, rc_v)
        err = dz_dt[mask] - cmd_on_state
        lag, corr = cross_corr_lag(rc_t, rc_v, t[mask], dz_dt[mask])
        head = (
            f"rate/{profile}: cmd active {t_a:.2f}s .. {t_s:.2f}s\n"
            f"|cmd|_max={np.max(np.abs(rc_v)):.2f}/s  "
            f"|dz/dt|_max(smoothed)={np.max(np.abs(dz_dt[mask])):.2f}/s\n"
            f"mean |dz/dt - cmd|: {np.mean(np.abs(err)):.3f}/s  "
            f"max |error|: {np.max(np.abs(err)):.3f}/s\n"
            f"best xcorr lag (dz/dt vs cmd): {lag:+.3f}s  ρ={corr:.3f}"
        )
        if profile == "chirp":
            f0 = meta.get("chirp_f0_hz", "?")
            f1 = meta.get("chirp_f1_hz", "?")
            head += f"\nchirp: f0={f0} Hz → f1={f1} Hz"
        return head

    return ""


if __name__ == "__main__":
    raise SystemExit(main())
