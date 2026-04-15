#!/usr/bin/env python3
"""
Analyze bag_step/ + step_index.csv produced by gimbal_calibration.py.

Reads the scoped step-phase rosbag, slices it per step using step_index.csv,
and produces plots of the dynamics: position & angular-velocity trajectories,
rise time / τ / overshoot vs step magnitude, and (from calibration.csv)
sweep linearity + hysteresis.

Usage:
  source /opt/ros/humble/setup.bash
  python3 gimbal_analysis.py <run-dir>

Where <run-dir> contains bag_step/, step_index.csv, step_summary.csv,
and (optionally) calibration.csv.
"""

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_dir: Path):
    """Return dict: topic -> list of (t_ns, msg)."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = defaultdict(list)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        out[topic].append((t_ns, msg))
    return dict(out), type_map


def as_arrays(topic_msgs):
    """Vector3 topic: return t (s), x, y, z as np arrays."""
    t = np.array([m[0] for m in topic_msgs], dtype=np.float64) / 1e9
    x = np.array([m[1].x for m in topic_msgs], dtype=np.float64)
    y = np.array([m[1].y for m in topic_msgs], dtype=np.float64)
    z = np.array([m[1].z for m in topic_msgs], dtype=np.float64)
    return t, x, y, z


def as_stamped_arrays(topic_msgs):
    """Vector3Stamped: t, x, y, z."""
    t = np.array([m[0] for m in topic_msgs], dtype=np.float64) / 1e9
    x = np.array([m[1].vector.x for m in topic_msgs], dtype=np.float64)
    y = np.array([m[1].vector.y for m in topic_msgs], dtype=np.float64)
    z = np.array([m[1].vector.z for m in topic_msgs], dtype=np.float64)
    return t, x, y, z


def read_step_index(path: Path):
    rows = []
    with path.open() as f:
        for row in csv.DictReader(f):
            rows.append(dict(
                step_id=int(row["step_id"]),
                axis=row["axis"],
                cmd_to=float(row["cmd_to_deg"]),
                t_cmd=float(row["t_cmd_ros_s"]),
                t_end=float(row["t_end_ros_s"]),
                s0_yaw=float(row["s0_yaw"]),
                s0_pitch=float(row["s0_pitch"]),
            ))
    return rows


def read_rate_step_index(path: Path):
    rows = []
    with path.open() as f:
        for row in csv.DictReader(f):
            rows.append(dict(
                step_id=int(row["step_id"]),
                axis=row["axis"],
                u_cmd=float(row["u_cmd"]),
                t_cmd=float(row["t_cmd_ros_s"]),
                t_end=float(row["t_end_ros_s"]),
                s0_yaw=float(row["s0_yaw"]),
                s0_pitch=float(row["s0_pitch"]),
                railed=bool(int(row.get("railed", 0))),
            ))
    return rows


def read_rate_step_summary(path: Path):
    rows = []
    with path.open() as f:
        for row in csv.DictReader(f):
            def flt(v):
                try:
                    return float(v)
                except ValueError:
                    return float("nan")
            rows.append(dict(
                step_id=int(row["step_id"]),
                axis=row["axis"],
                u_cmd=flt(row["u_cmd"]),
                w_ss=flt(row["w_ss_deg_s"]),
                w_peak=flt(row["w_peak_deg_s"]),
                rise=flt(row["rise_time_s"]),
                latency=flt(row["latency_s"]),
                railed=bool(int(row.get("railed", "0"))),
            ))
    return rows


def _fit_first_order(t_rel, w, w_ss):
    """Fit τ, latency to w(t) = w_ss * (1 - exp(-(t-t0)/τ)) via 63.2% crossing.
    Returns (t0, tau) or (nan, nan) if signal too small.
    """
    if abs(w_ss) < 5.0 or len(t_rel) < 5:
        return float("nan"), float("nan")
    sign = 1.0 if w_ss > 0 else -1.0
    # First-motion latency: first sample where |w| exceeds 5 deg/s
    t0 = float("nan")
    for i, v in enumerate(w):
        if abs(v) >= 5.0:
            t0 = t_rel[i]
            break
    # 63.2% crossing → τ
    thr = 0.632 * w_ss
    tau = float("nan")
    for i, v in enumerate(w):
        if sign * (v - thr) >= 0 and not math.isnan(t0):
            tau = t_rel[i] - t0
            break
    return t0, tau


def plot_rate_step_trajectories(steps, rate_t, rate_y, rate_z,
                                summary_by_id, out_path: Path,
                                show_fit: bool = True):
    """Rate response trajectories from state_rate_rpy_deg.

    Split by axis (cols) × sign (rows) so the positive/negative branches don't
    overplot. Each trace is colored by |u_cmd|. Steady-state (w_ss) and a
    first-order fit (dotted) are overlaid per trace when show_fit is True.
    """
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), sharex=True)
    fig.suptitle("Rate-step response (A8 mini, bench)")

    mags = sorted({abs(s["u_cmd"]) for s in steps})
    cmap = plt.get_cmap("viridis")
    color_of = {m: cmap(i / max(1, len(mags) - 1)) for i, m in enumerate(mags)}

    for col, axis in enumerate(("yaw", "pitch")):
        for row, sign_label in enumerate(("positive u", "negative u")):
            ax = axes[row, col]
            want_positive = (row == 0)
            for s in steps:
                if s["axis"] != axis:
                    continue
                if (s["u_cmd"] > 0) != want_positive:
                    continue
                arr = rate_z if axis == "yaw" else rate_y
                t_rel, w = slice_window(rate_t, arr,
                                        s["t_cmd"], s["t_end"])
                if len(t_rel) < 4:
                    continue
                color = color_of[abs(s["u_cmd"])]
                ax.plot(t_rel, w, color=color, linewidth=1.2,
                        label=f"u={s['u_cmd']:+.2f}")
                # Overlay w_ss + first-order fit from summary
                m = summary_by_id.get(s["step_id"])
                if m is None or math.isnan(m["w_ss"]):
                    continue
                ax.axhline(m["w_ss"], color=color, linestyle=":",
                           linewidth=0.6, alpha=0.5)
                if show_fit:
                    t0, tau = _fit_first_order(t_rel, w, m["w_ss"])
                    if not (math.isnan(t0) or math.isnan(tau)) and tau > 0:
                        tt = np.linspace(t0, t_rel[-1], 60)
                        ww = m["w_ss"] * (1 - np.exp(-(tt - t0) / tau))
                        ax.plot(tt, ww, color=color, linestyle="--",
                                linewidth=0.9, alpha=0.7)
            ax.set_title(f"{axis.upper()} — {sign_label}")
            if col == 0:
                ax.set_ylabel("rate [deg/s]")
            if row == 1:
                ax.set_xlabel("t since cmd [s]")
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color="k", linewidth=0.4)
            ax.axvline(0, color="k", linewidth=0.4, alpha=0.4)
            handles, labels = ax.get_legend_handles_labels()
            seen = {}
            for h, l in zip(handles, labels):
                seen.setdefault(l, h)
            if seen:
                ax.legend(seen.values(), seen.keys(), fontsize=7, ncol=2,
                          loc="best")

    # Legend hint for overlays
    fig.text(0.5, 0.015,
             "dotted horizontal = w_ss;  dashed = first-order fit "
             "w_ss·(1 − e^−(t−t₀)/τ)",
             ha="center", fontsize=8, style="italic")
    fig.tight_layout(rect=(0, 0.03, 1, 0.94))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_rate_step_metrics(summary, out_path: Path):
    """Rise time, latency, effective gain, and peak/ss vs |u_cmd|.

    K_eff = w_ss / u_cmd is the local gain; a horizontal line means linear.
    w_peak / w_ss tracks overshoot (>1 means overshoot).
    """
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    fig.suptitle("Rate-step metrics vs |u_cmd|")

    # axis → list of rows; colors for axes, markers for sign of u_cmd
    by_axis = defaultdict(list)
    for r in summary:
        if r.get("railed"):
            continue
        by_axis[r["axis"]].append(r)

    for axis, color in (("yaw", "tab:blue"), ("pitch", "tab:orange")):
        for sign_val, marker, sign_label in (
                (+1.0, "o", "+u"), (-1.0, "s", "−u")):
            rows = [r for r in by_axis.get(axis, [])
                    if (r["u_cmd"] > 0) == (sign_val > 0)]
            rows = sorted(rows, key=lambda r: abs(r["u_cmd"]))
            if not rows:
                continue
            umag = np.array([abs(r["u_cmd"]) for r in rows])
            rise = np.array([r["rise"] for r in rows])
            latency = np.array([r["latency"] for r in rows])
            w_ss = np.array([r["w_ss"] for r in rows])
            w_peak = np.array([r["w_peak"] for r in rows])
            u = np.array([r["u_cmd"] for r in rows])
            with np.errstate(divide="ignore", invalid="ignore"):
                k_eff = np.where(np.abs(u) > 1e-6, w_ss / u, np.nan)
                peak_ratio = np.where(
                    np.abs(w_ss) > 5.0, w_peak / w_ss, np.nan)
            label = f"{axis} {sign_label}"
            axes[0, 0].plot(umag, rise, marker=marker, linestyle="-",
                            color=color, label=label)
            axes[0, 1].plot(umag, latency, marker=marker, linestyle="-",
                            color=color, label=label)
            axes[1, 0].plot(umag, k_eff, marker=marker, linestyle="-",
                            color=color, label=label)
            axes[1, 1].plot(umag, peak_ratio, marker=marker, linestyle="-",
                            color=color, label=label)

    axes[0, 0].set_title("10–90% rise time")
    axes[0, 0].set_ylabel("rise [s]")
    axes[0, 1].set_title("First-motion latency")
    axes[0, 1].set_ylabel("latency [s]")
    axes[1, 0].set_title("Effective gain K = w_ss / u_cmd")
    axes[1, 0].set_ylabel("K [deg/s per u]")
    axes[1, 1].set_title("Peak / steady-state ratio (overshoot proxy)")
    axes[1, 1].set_ylabel("w_peak / w_ss")
    axes[1, 1].axhline(1.0, color="k", linestyle=":", linewidth=0.6)
    for ax in axes.flat:
        ax.set_xlabel("|u_cmd|")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_rate_k_curve(summary, out_path: Path):
    """Steady-state rate vs normalized command — the 0x07 calibration curve.
    Fits positive and negative branches separately so asymmetry is visible.
    """
    fig, ax = plt.subplots(figsize=(8.5, 6))
    fig.suptitle("Rate calibration: w_ss vs u_cmd")

    by_axis = defaultdict(list)
    for r in summary:
        if r.get("railed"):
            continue
        by_axis[r["axis"]].append(r)

    for axis, color in (("yaw", "tab:blue"), ("pitch", "tab:orange")):
        rows = sorted(by_axis.get(axis, []), key=lambda r: r["u_cmd"])
        if not rows:
            continue
        u = np.array([r["u_cmd"] for r in rows])
        w = np.array([r["w_ss"] for r in rows])
        ax.plot(u, w, "o", color=color, label=f"{axis}")

        # Separate linear fits for u>0 and u<0 to reveal asymmetry
        for sign_val, style in ((+1.0, "-"), (-1.0, "--")):
            branch = (u * sign_val > 0) & (np.abs(w) >= 5.0)
            if branch.sum() < 2:
                continue
            k, b = np.polyfit(u[branch], w[branch], 1)
            xs = np.linspace(u[branch].min(), u[branch].max(), 30)
            sign_label = "+u" if sign_val > 0 else "−u"
            ax.plot(xs, k * xs + b, style, color=color, linewidth=1.1,
                    alpha=0.8,
                    label=f"{axis} {sign_label}: k={k:.1f}, b={b:.1f}")

        # Deadband markers: smallest |u| where |w_ss| >= 5 (per sign)
        for sign_val in (+1.0, -1.0):
            branch_rows = [(abs(r["u_cmd"]), r["w_ss"]) for r in rows
                           if r["u_cmd"] * sign_val > 0
                           and abs(r["w_ss"]) >= 5.0]
            if branch_rows:
                u_min = min(branch_rows)[0] * sign_val
                ax.axvline(u_min, color=color, linestyle=":",
                           linewidth=0.6, alpha=0.5)

    ax.set_xlabel("u_cmd (normalized)")
    ax.set_ylabel("w_ss [deg/s]")
    ax.axhline(0, color="k", linewidth=0.4)
    ax.axvline(0, color="k", linewidth=0.4)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def read_step_summary(path: Path):
    rows = []
    with path.open() as f:
        for row in csv.DictReader(f):
            def flt(v):
                try:
                    return float(v)
                except ValueError:
                    return float("nan")
            rows.append(dict(
                step_id=int(row["step_id"]),
                axis=row["axis"],
                cmd_to=flt(row["cmd_to_deg"]),
                rise=flt(row["rise_time_s"]),
                settle=flt(row["settle_time_s"]),
                overshoot=flt(row["overshoot_pct"]),
                tau=flt(row["tau_s"]),
            ))
    return rows


def slice_window(t, y, t0, t1):
    """Return samples in [t0, t1]. t is absolute ROS time."""
    mask = (t >= t0) & (t <= t1)
    return t[mask] - t0, y[mask]


def axis_signal(axis, state_pitch, state_yaw):
    return state_yaw if axis == "yaw" else state_pitch


def plot_step_trajectories(steps, state_t, state_y, state_z,
                           enc_t, enc_y, enc_z, out_path: Path):
    """Two panels (yaw, pitch). Each step overlaid, colored by |cmd_to|."""
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.suptitle("Step response trajectories (A8 mini, bench)")

    # Distinct color per unique |cmd_to|.
    mags = sorted({abs(s["cmd_to"]) for s in steps})
    cmap = plt.get_cmap("viridis")
    color_of = {m: cmap(i / max(1, len(mags) - 1)) for i, m in enumerate(mags)}

    for col, axis in enumerate(("yaw", "pitch")):
        ax_pos = axes[0, col]
        ax_vel = axes[1, col]
        for s in steps:
            if s["axis"] != axis:
                continue
            st = (s["s0_yaw"] if axis == "yaw" else s["s0_pitch"])
            # Position trace
            if axis == "yaw":
                t_rel, y = slice_window(state_t, state_z, s["t_cmd"], s["t_end"])
            else:
                t_rel, y = slice_window(state_t, state_y, s["t_cmd"], s["t_end"])
            if len(t_rel) < 4:
                continue
            color = color_of[abs(s["cmd_to"])]
            ls = "-" if s["cmd_to"] > 0 else "--"
            ax_pos.plot(t_rel, y - st, color=color, linestyle=ls,
                        linewidth=1.3,
                        label=f"{s['cmd_to']:+.0f}°")
            # Velocity (finite diff of position)
            if len(t_rel) >= 3:
                v = np.gradient(y, t_rel)
                ax_vel.plot(t_rel, v, color=color, linestyle=ls,
                            linewidth=1.1)
            # Mark commanded level
            ax_pos.axhline(s["cmd_to"], color=color, linestyle=":",
                           linewidth=0.5, alpha=0.5)

        ax_pos.set_title(f"{axis.upper()} — position (state - s0)")
        ax_pos.set_ylabel("angle [deg]")
        ax_pos.grid(True, alpha=0.3)
        # Dedup legend
        handles, labels = ax_pos.get_legend_handles_labels()
        seen = {}
        for h, l in zip(handles, labels):
            seen.setdefault(l, h)
        ax_pos.legend(seen.values(), seen.keys(), fontsize=8, ncol=2,
                      loc="lower right")

        ax_vel.set_title(f"{axis.upper()} — angular rate (d state / dt)")
        ax_vel.set_ylabel("rate [deg/s]")
        ax_vel.set_xlabel("t since cmd [s]")
        ax_vel.grid(True, alpha=0.3)
        ax_vel.axhline(0, color="k", linewidth=0.4)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_dynamics_metrics(summary, out_path: Path):
    """Rise/τ/settle/overshoot vs |cmd_to|."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    fig.suptitle("Step-response metrics vs magnitude")

    by_axis = defaultdict(list)
    for r in summary:
        by_axis[r["axis"]].append(r)

    for axis, color in (("yaw", "tab:blue"), ("pitch", "tab:orange")):
        rows = sorted(by_axis.get(axis, []), key=lambda r: abs(r["cmd_to"]))
        if not rows:
            continue
        mag = np.array([abs(r["cmd_to"]) for r in rows])
        rise = np.array([r["rise"] for r in rows])
        tau = np.array([r["tau"] for r in rows])
        settle = np.array([r["settle"] for r in rows])
        over = np.array([r["overshoot"] for r in rows])

        axes[0, 0].plot(mag, rise, "o-", color=color, label=axis)
        axes[0, 1].plot(mag, tau, "o-", color=color, label=axis)
        axes[1, 0].plot(mag, settle, "o-", color=color, label=axis)
        axes[1, 1].plot(mag, over, "o-", color=color, label=axis)

    axes[0, 0].set_title("10-90% rise time")
    axes[0, 0].set_ylabel("rise [s]")
    axes[0, 1].set_title("63.2% time constant τ")
    axes[0, 1].set_ylabel("τ [s]")
    axes[1, 0].set_title("2% settling time")
    axes[1, 0].set_ylabel("settle [s]")
    axes[1, 1].set_title("Overshoot")
    axes[1, 1].set_ylabel("overshoot [%]")
    for ax in axes.flat:
        ax.set_xlabel("|cmd_to| [deg]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

    # Annotation: straight line through τ vs mag indicates rate limit.
    # Estimate saturation rate from yaw large-step slope.
    yaw_rows = sorted(by_axis.get("yaw", []), key=lambda r: abs(r["cmd_to"]))
    big = [r for r in yaw_rows if abs(r["cmd_to"]) >= 10]
    if len(big) >= 2:
        mag = np.array([abs(r["cmd_to"]) for r in big])
        tau = np.array([r["tau"] for r in big])
        # If actuator is rate-limited, τ ≈ 0.632 * mag / rate_sat.
        # So rate_sat ≈ 0.632 * slope⁻¹.
        slope, _ = np.polyfit(mag, tau, 1)
        if slope > 0:
            rate_sat = 0.632 / slope
            axes[0, 1].text(
                0.05, 0.95,
                f"yaw rate_sat ≈ {rate_sat:.1f} °/s\n(from large-step τ slope)",
                transform=axes[0, 1].transAxes,
                verticalalignment="top", fontsize=9,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_sweep(sweep_csv: Path, out_path: Path):
    """Linearity + hysteresis from calibration.csv (sweep phase)."""
    if not sweep_csv.exists():
        return False
    rows = []
    with sweep_csv.open() as f:
        for r in csv.DictReader(f):
            if r["phase"] != "sweep":
                continue
            rows.append(r)
    if not rows:
        return False

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Sweep: linearity + hysteresis")

    def _panel(axis, ax_lin, ax_hyst, cmd_key, state_key):
        fwd_c, fwd_s = [], []
        bwd_c, bwd_s = [], []
        for r in rows:
            if r["axis"] != axis:
                continue
            c = float(r[cmd_key])
            v = float(r[state_key])
            (fwd_c if r["direction"] == "fwd" else bwd_c).append(c)
            (fwd_s if r["direction"] == "fwd" else bwd_s).append(v)
        fwd_c = np.array(fwd_c); fwd_s = np.array(fwd_s)
        bwd_c = np.array(bwd_c); bwd_s = np.array(bwd_s)

        ax_lin.plot(fwd_c, fwd_s, "o-", ms=3, label="fwd", color="tab:blue")
        ax_lin.plot(bwd_c, bwd_s, "o-", ms=3, label="bwd", color="tab:orange")
        lo = min(fwd_c.min(), bwd_c.min())
        hi = max(fwd_c.max(), bwd_c.max())
        ax_lin.plot([lo, hi], [lo, hi], "k:", linewidth=0.8, label="y=x")
        ax_lin.set_title(f"{axis.upper()} — state vs cmd")
        ax_lin.set_xlabel("cmd [deg]")
        ax_lin.set_ylabel("state [deg]")
        ax_lin.grid(True, alpha=0.3)
        ax_lin.legend(fontsize=9)

        # Hysteresis: fwd - bwd at matched cmd
        matched = []
        fwd_map = dict(zip(np.round(fwd_c, 2), fwd_s))
        bwd_map = dict(zip(np.round(bwd_c, 2), bwd_s))
        for k in sorted(set(fwd_map) & set(bwd_map)):
            matched.append((k, fwd_map[k] - bwd_map[k]))
        if matched:
            mc = np.array([m[0] for m in matched])
            md = np.array([m[1] for m in matched])
            ax_hyst.plot(mc, md, "o-", ms=3, color="tab:red")
            ax_hyst.axhline(0, color="k", linewidth=0.5)
            ax_hyst.set_title(f"{axis.upper()} — hysteresis (fwd − bwd)")
            ax_hyst.set_xlabel("cmd [deg]")
            ax_hyst.set_ylabel("Δ [deg]")
            ax_hyst.grid(True, alpha=0.3)
            ax_hyst.text(
                0.02, 0.95,
                f"max|Δ| = {np.max(np.abs(md)):.2f}°\nmean|Δ| = "
                f"{np.mean(np.abs(md)):.2f}°",
                transform=ax_hyst.transAxes,
                verticalalignment="top", fontsize=9,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    _panel("yaw", axes[0, 0], axes[0, 1], "cmd_yaw", "state_yaw")
    _panel("pitch", axes[1, 0], axes[1, 1], "cmd_pitch", "state_pitch")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="defaults to <run-dir>/plots/")
    args = ap.parse_args()

    run_dir = args.run_dir
    plots_dir = args.out_dir or (run_dir / "plots")
    plots_dir.mkdir(parents=True, exist_ok=True)

    sweep_path = run_dir / "calibration.csv"

    # Angle-step phase (optional): bag_step/ + step_index.csv
    angle_bag_dir = run_dir / "bag_step"
    angle_index_path = run_dir / "step_index.csv"
    angle_summary_path = run_dir / "step_summary.csv"
    summary = []
    if angle_bag_dir.exists() and angle_index_path.exists():
        print(f"Reading angle-step bag: {angle_bag_dir}")
        topics, _ = read_bag(angle_bag_dir)
        print("  topics:", list(topics.keys()))
        print(f"  total msgs: {sum(len(v) for v in topics.values())}")

        state = topics.get("/siyi_gimbal_angles/state_rpy_deg", [])
        enc = topics.get("/siyi_gimbal_angles/encoder_rpy_deg", [])
        if not state:
            print("WARN: no state_rpy_deg in angle-step bag", file=sys.stderr)
        else:
            state_t, state_x, state_y, state_z = as_arrays(state)
            enc_t = enc_x = enc_y = enc_z = None
            if enc:
                enc_t, enc_x, enc_y, enc_z = as_arrays(enc)

            steps = read_step_index(angle_index_path)
            summary = (read_step_summary(angle_summary_path)
                       if angle_summary_path.exists() else [])
            print(f"  steps: {len(steps)}")

            p1 = plots_dir / "step_trajectories.png"
            plot_step_trajectories(
                steps, state_t, state_y, state_z,
                enc_t, enc_y, enc_z, p1)
            print(f"  wrote {p1}")

            if summary:
                p2 = plots_dir / "step_metrics.png"
                plot_dynamics_metrics(summary, p2)
                print(f"  wrote {p2}")
    else:
        print(f"(no angle-step data at {angle_bag_dir} — skipping)")

    # Sweep plot
    p3 = plots_dir / "sweep.png"
    if plot_sweep(sweep_path, p3):
        print(f"  wrote {p3}")
    else:
        print(f"  (no sweep data at {sweep_path})")

    # Rate-step phase (optional): bag_rate_step/ + rate_step_index.csv
    rate_bag_dir = run_dir / "bag_rate_step"
    rate_index_path = run_dir / "rate_step_index.csv"
    rate_summary_path = run_dir / "rate_step_summary.csv"
    if rate_bag_dir.exists() and rate_index_path.exists():
        print(f"\nReading rate bag: {rate_bag_dir}")
        rate_topics, _ = read_bag(rate_bag_dir)
        state_rate = rate_topics.get("/siyi_gimbal_angles/state_rate_rpy_deg",
                                     [])
        if state_rate:
            rt_t, rt_x, rt_y, rt_z = as_arrays(state_rate)
            rsteps = read_rate_step_index(rate_index_path)
            rsummary = (read_rate_step_summary(rate_summary_path)
                        if rate_summary_path.exists() else [])
            summary_by_id = {m["step_id"]: m for m in rsummary}

            p4 = plots_dir / "rate_step_trajectories.png"
            plot_rate_step_trajectories(rsteps, rt_t, rt_y, rt_z,
                                        summary_by_id, p4)
            print(f"  wrote {p4}")

            if rsummary:
                p5 = plots_dir / "rate_k_curve.png"
                plot_rate_k_curve(rsummary, p5)
                print(f"  wrote {p5}")
                p6 = plots_dir / "rate_step_metrics.png"
                plot_rate_step_metrics(rsummary, p6)
                print(f"  wrote {p6}")
        else:
            print("  (no state_rate_rpy_deg in rate bag — is the SIYI node "
                  "publishing it?)")

    # Print short analysis summary to stdout (angle-step only)
    if not summary:
        return 0
    print("\n=== Analysis summary ===")
    by_axis = defaultdict(list)
    for r in summary:
        by_axis[r["axis"]].append(r)
    for axis in ("yaw", "pitch"):
        rows = sorted(by_axis.get(axis, []), key=lambda r: abs(r["cmd_to"]))
        if not rows:
            continue
        mag = np.array([abs(r["cmd_to"]) for r in rows])
        tau = np.array([r["tau"] for r in rows])
        # Linear fit τ(|mag|) on |mag| >= 10 => rate-limited regime
        big = mag >= 10
        if big.sum() >= 2:
            slope, intercept = np.polyfit(mag[big], tau[big], 1)
            rate_sat = 0.632 / slope if slope > 0 else float("nan")
            print(f"  {axis}: τ ≈ {slope:.4f}·|mag| + {intercept:.3f}  "
                  f"→ rate_sat ≈ {rate_sat:.1f} °/s")
        # Per-magnitude mean from both signs
        for m in sorted(set(mag)):
            sel = [r for r in rows if abs(r["cmd_to"]) == m]
            tau_m = np.mean([r["tau"] for r in sel])
            rise_m = np.mean([r["rise"] for r in sel])
            over_m = np.mean([r["overshoot"] for r in sel])
            print(f"    |mag|={m:5.1f}°  τ={tau_m:.3f}s  "
                  f"rise={rise_m:.3f}s  overshoot={over_m:5.2f}%")

    return 0


if __name__ == "__main__":
    sys.exit(main())
