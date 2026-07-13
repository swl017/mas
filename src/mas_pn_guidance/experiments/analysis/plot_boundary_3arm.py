#!/usr/bin/env python3
"""plot_boundary_3arm.py — 3-arm capability-grid boundary figure (canonical tool).

Generalized from the ticket-008/010 script: identical classification and marker
scheme, with the run-specific text supplied as CLI args instead of edited into
per-ticket copies. Does NOT default to any paper-figure path — pass --output
explicitly when regenerating a cornerstone figure (snapshot the old one first).

Per (cell, estimator) over all reps (result in {hit,miss,timeout};
settle_error/config_error excluded):
  frac_hit = (#reps with CPA<threshold)/n
  robust_hit  : frac_hit >= 0.8   (>=4/5)     robust_miss : frac_hit <= 0.2
  uncertain   : otherwise (hollow ring)       1-rep cells: hit/miss directly

Primary marker (World-Frame CV = node `simple_ekf`):
  green circle    CV robust_hit
  yellow triangle oracle robust_hit AND CV robust_miss (ego-only limit)
  magenta square  oracle median CPA in [0.5,2) AND CV not robust_hit
  gray x          oracle median CPA >= 2
Overlays: hollow ring = CV uncertain; red diamond = Direct Projection robust_hit.

Score is CPA (`min_range_m`) vs target GT — the conductor already writes this.

Usage:
  python3 plot_boundary_3arm.py --data-dir <dir with boot_*_results.csv> \
      --output fig.png [--title ...] [--subtitle ...] [--footer ...] [--hit 0.5]
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-experiments")
sys.dont_write_bytecode = True

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

CV = "simple_ekf"           # node name of the World-Frame CV-EKF arm
VALID = {"hit", "miss", "timeout"}   # settle_error / config_error excluded

CATEGORIES = {
    "simple_hit":          {"label": "World-Frame CV hit",                    "color": "#238B45", "marker": "o", "size": 95},
    "oracle_hit_ekf_miss": {"label": "Oracle hit + CV miss (ego-only limit)", "color": "#F2B705", "marker": "^", "size": 120, "edgecolor": "#765900"},
    "oracle_near":         {"label": "Oracle near (0.5-2 m) + CV miss",       "color": "#B0006D", "marker": "s", "size": 92, "edgecolor": "white"},
    "oracle_miss":         {"label": "Oracle miss / not isolatable",          "color": "#777777", "marker": "x", "size": 82},
}


def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float("nan")
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def load_all(data_dir: Path):
    """cell -> estimator -> list[cpa].  cell = (fwd, alat, geometry)."""
    cells: dict = defaultdict(lambda: defaultdict(list))
    files = sorted(glob.glob(str(data_dir / "boot_*_results.csv")))
    if not files:
        raise SystemExit(f"no boot_*_results.csv in {data_dir}")
    n_rows = 0
    for f in files:
        with open(f, newline="") as fh:
            for row in csv.DictReader(fh):
                if row.get("result") not in VALID:
                    continue
                if not row.get("min_range_m"):
                    continue
                cell = (
                    float(row["target_forward_speed_cmd_mps"]),
                    float(row["target_lat_accel_cmd_mps2"]),
                    row["geometry"],
                )
                cells[cell][row["estimator"]].append(float(row["min_range_m"]))
                n_rows += 1
    return cells, files, n_rows


def robust(cpas, hit):
    if not cpas:
        return None
    frac = sum(c < hit for c in cpas) / len(cpas)
    if frac >= 0.8:
        return "robust_hit"
    if frac <= 0.2:
        return "robust_miss"
    return "uncertain"


def category_for(cell_data, hit):
    cv = cell_data.get(CV, [])
    orc = cell_data.get("oracle", [])
    cv_cls = robust(cv, hit)
    orc_cls = robust(orc, hit)
    cv_uncertain = cv_cls == "uncertain"
    if cv_cls == "robust_hit":
        return "simple_hit", cv_uncertain
    if orc_cls == "robust_hit":
        return "oracle_hit_ekf_miss", cv_uncertain
    if orc and median(orc) < 2.0:
        return "oracle_near", cv_uncertain
    return "oracle_miss", cv_uncertain


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path, default=Path("fig_boundary_3arm.png"))
    ap.add_argument("--hit", type=float, default=0.5, help="CPA hit threshold [m]")
    ap.add_argument("--title", default="Capability-grid boundary: ego-only vs oracle")
    ap.add_argument("--subtitle", default=(
        "World-Frame CV-EKF is the ego-only arm. Yellow triangles = oracle hit "
        "while ego-only misses."))
    ap.add_argument("--footer", default=(
        "Robust rule = hit >=4/5, miss <=1/5, else uncertain (hollow ring); reps "
        "pooled per cell. CPA vs target GT; settle/config-error trials excluded."))
    a = ap.parse_args()
    cells, files, n_rows = load_all(a.data_dir)
    print(f"loaded {n_rows} valid trial rows from {len(files)} boot files")

    fig, axes = plt.subplots(1, 2, figsize=(14.5, 7.4))
    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.26, top=0.82, wspace=0.14)
    fig.suptitle(a.title, fontsize=16, fontweight="bold", y=0.96)
    fig.text(0.5, 0.895, a.subtitle, ha="center", fontsize=9.5, color="#444444")

    counts_all = defaultdict(int)
    dp_hits = {"crossing": 0, "tail_chase": 0}
    for ax, geometry, title in [
        (axes[0], "crossing", "Crossing"),
        (axes[1], "tail_chase", "Tail chase"),
    ]:
        # regime shading: benign block (fwd 2-4) vs challenging (fwd 4.5-8)
        ax.axhspan(1.75, 4.15, color="#238B45", alpha=0.05, zorder=0)
        ax.axhspan(4.35, 8.35, color="#C14D4D", alpha=0.05, zorder=0)

        geo_cells = {c: d for c, d in cells.items() if c[2] == geometry}
        buckets = defaultdict(list)
        uncertain_pts = []
        dp_pts = []
        for cell, d in geo_cells.items():
            cat, cv_unc = category_for(d, a.hit)
            buckets[cat].append(cell)
            counts_all[(geometry, cat)] += 1
            if cv_unc:
                uncertain_pts.append(cell)
            if robust(d.get("direct_projection", []), a.hit) == "robust_hit":
                dp_pts.append(cell)
                dp_hits[geometry] += 1

        for cat, style in CATEGORIES.items():
            sel = buckets.get(cat, [])
            if not sel:
                continue
            opts = {"s": style["size"], "marker": style["marker"], "color": style["color"],
                    "linewidth": 1.8 if style["marker"] == "x" else 0.8,
                    "label": style["label"], "zorder": 3}
            if style["marker"] != "x":
                opts["edgecolor"] = style.get("edgecolor", "white")
            ax.scatter([c[1] for c in sel], [c[0] for c in sel], **opts)

        if uncertain_pts:
            ax.scatter([c[1] for c in uncertain_pts], [c[0] for c in uncertain_pts],
                       s=230, marker="o", facecolors="none", edgecolors="#111111",
                       linewidths=1.6, label="CV repeats straddle threshold (uncertain)",
                       zorder=4)
        if dp_pts:
            ax.scatter([c[1] for c in dp_pts], [c[0] for c in dp_pts],
                       s=46, marker="D", color="#C0392B", edgecolor="white",
                       linewidths=0.6, label="Direct Projection hit", zorder=5)

        ax.set_title(f"{title}  (DP robust hits: {dp_hits[geometry]})", pad=10)
        ax.set_xlabel("Target lateral acceleration command [m/s^2]")
        ax.set_ylabel("Target forward speed command [m/s]")
        ax.set_xlim(-0.45, 7.55)
        ax.set_ylim(1.6, 8.35)
        ax.set_xticks([0.0, 0.75, 1.5, 3.0, 4.5, 7.0])
        ax.set_yticks([2.0, 3.0, 4.0, 4.5, 6.0, 7.0, 8.0])
        ax.tick_params(axis="x", labelrotation=30)
        ax.grid(color="#DDDDDD", linewidth=0.7, zorder=0)

    handles, labels = axes[0].get_legend_handles_labels()
    h1, l1 = axes[1].get_legend_handles_labels()
    for h, l in zip(h1, l1):
        if l not in labels:
            handles.append(h)
            labels.append(l)
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=9,
               frameon=True, bbox_to_anchor=(0.5, 0.09))
    fig.text(0.5, 0.02, a.footer, ha="center", fontsize=8, color="#666666",
             style="italic")

    a.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.output, dpi=150)
    plt.close(fig)

    print("category counts (geometry, category): count")
    for (geo, cat), n in sorted(counts_all.items()):
        print(f"  {geo:11s} {cat:20s} {n}")
    print(f"DP robust hits: {dp_hits}")
    print(f"wrote {a.output}")


if __name__ == "__main__":
    main()
