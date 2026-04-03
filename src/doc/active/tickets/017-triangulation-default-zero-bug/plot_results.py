#!/usr/bin/env python3
"""Plot AoI MC validation results (Ticket 017, Exp 1 & 2).

Generates illustrative figures from the 100k-sample MC validation run.
No IsaacLab dependency — just matplotlib + numpy.

Usage:
    python3 plot_results.py [--save-dir DIR]
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--save-dir", type=str, default=os.path.dirname(__file__))
args = parser.parse_args()

SAVE_DIR = args.save_dir
os.makedirs(SAVE_DIR, exist_ok=True)

# =============================================================================
# Data from 100k MC validation (2026-04-02)
# =============================================================================

ages_exp1 = np.array([0, 50, 100, 200, 300, 500])  # ms

# NEES values (target = 3.0 for chi²(3))
nees = {
    "oracle (no inflation)":  [3.01, 3.20, 3.78, 6.06, 9.81, 21.94],
    "naive reuse":            [3.02, 3.20, 3.77, 6.04, 9.84, 21.90],
    "const. velocity":        [3.03, 2.75, 2.23, 1.61, 1.36, 1.19],
    "random walk":            [3.02, 0.76, 0.50, 0.42, 0.46, 0.61],
    "OU":                     [3.02, 0.79, 0.55, 0.50, 0.60, 0.94],
}

# 3σ coverage (target = 97.1%)
cov3s = {
    "oracle (no inflation)":  [97.0, 96.2, 93.3, 78.4, 56.5, 23.4],
    "naive reuse":            [97.1, 96.2, 93.4, 78.6, 56.4, 23.5],
    "const. velocity":        [96.9, 98.0, 99.3, 100., 100., 100.],
    "random walk":            [97.0, 100., 100., 100., 100., 100.],
    "OU":                     [97.1, 100., 100., 100., 100., 100.],
}

# Trace ratio (analytical / empirical, target = 1.0)
trace_ratio = {
    "oracle (no inflation)":  [0.998, 0.937, 0.790, 0.489, 0.301, 0.134],
    "naive reuse":            [0.993, 0.934, 0.792, 0.490, 0.299, 0.134],
    "const. velocity":        [0.989, 1.092, 1.350, 1.872, 2.185, 2.479],
    "random walk":            [0.993, 4.234, 6.347, 7.352, 6.621, 4.846],
    "OU":                     [0.994, 4.062, 5.831, 6.171, 5.057, 3.123],
}

# Exp 2: fine-grained age sweep (constant_velocity focus)
ages_exp2 = np.array([0, 50, 100, 150, 200, 300, 400, 500, 750, 1000])  # ms

nees_exp2 = {
    "random walk":       [3.02, 0.76, 0.50, 0.43, 0.42, 0.46, 0.53, 0.61, 0.84, 1.08],
    "const. velocity":   [3.01, 2.75, 2.23, 1.86, 1.61, 1.35, 1.24, 1.18, 1.11, 1.08],
    "OU":                [3.02, 0.78, 0.55, 0.49, 0.50, 0.60, 0.75, 0.95, 1.60, 2.46],
}

trace_ratio_exp2 = {
    "random walk":       [0.994, 4.209, 6.347, 7.272, 7.409, 6.599, 5.663, 4.846, 3.466, 2.704],
    "const. velocity":   [0.997, 1.092, 1.349, 1.629, 1.869, 2.206, 2.384, 2.490, 2.630, 2.694],
    "OU":                [0.994, 4.075, 5.829, 6.373, 6.131, 5.047, 3.947, 3.112, 1.826, 1.183],
}

# =============================================================================
# Styling
# =============================================================================

COLORS = {
    "oracle (no inflation)": "#ef4444",  # red
    "naive reuse":           "#f97316",  # orange
    "const. velocity":       "#3b82f6",  # blue
    "random walk":           "#8b5cf6",  # purple
    "OU":                    "#10b981",  # green
}

MARKERS = {
    "oracle (no inflation)": "x",
    "naive reuse":           "d",
    "const. velocity":       "o",
    "random walk":           "s",
    "OU":                    "^",
}

plt.rcParams.update({
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'legend.fontsize': 9.5,
    'figure.dpi': 150,
})


# =============================================================================
# Figure 1: Exp 1 — NEES vs Age (all conditions)
# =============================================================================

fig, ax = plt.subplots(figsize=(9, 5.5))

for name, vals in nees.items():
    ax.plot(ages_exp1, vals, marker=MARKERS[name], color=COLORS[name],
            linewidth=2, markersize=7, label=name, zorder=3)

# Target NEES = 3.0 band
ax.axhline(3.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax.axhspan(2.0, 4.5, color='gray', alpha=0.08)
ax.text(510, 3.15, 'target NEES = 3.0', fontsize=9, color='gray', va='bottom')

# Annotation for the key insight
ax.annotate('overconfident\n(NEES >> 3)',
            xy=(400, 15), fontsize=9, color='#ef4444', ha='center',
            fontstyle='italic')
ax.annotate('conservative\n(NEES << 3)',
            xy=(350, 0.35), fontsize=9, color='#8b5cf6', ha='center',
            fontstyle='italic')

ax.set_xlabel('Observation Age of Stale Cameras (ms)')
ax.set_ylabel('NEES  (Normalized Estimation Error Squared)')
ax.set_title('Exp 1 — Covariance Calibration: Which Model Tracks True Error?')
ax.set_yscale('log')
ax.set_ylim(0.2, 35)
ax.set_xlim(-15, 530)
ax.legend(loc='upper left', framealpha=0.9)
ax.grid(True, alpha=0.25, which='both')

fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp1_nees_vs_age.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


# =============================================================================
# Figure 2: Exp 1 — 3σ Coverage vs Age
# =============================================================================

fig, ax = plt.subplots(figsize=(9, 5))

for name, vals in cov3s.items():
    ax.plot(ages_exp1, vals, marker=MARKERS[name], color=COLORS[name],
            linewidth=2, markersize=7, label=name, zorder=3)

ax.axhline(97.1, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax.text(510, 97.8, 'expected 97.1%', fontsize=9, color='gray', va='bottom')

# Danger zone
ax.axhspan(0, 80, color='#fee2e2', alpha=0.3)
ax.text(450, 45, 'danger zone:\noverconfident', fontsize=9, color='#dc2626',
        ha='center', fontstyle='italic', alpha=0.7)

ax.set_xlabel('Observation Age of Stale Cameras (ms)')
ax.set_ylabel('3σ Coverage (%)')
ax.set_title('Exp 1 — 3σ Ellipsoid Coverage: Does the Covariance Contain the True Position?')
ax.set_ylim(15, 103)
ax.set_xlim(-15, 530)
ax.legend(loc='center left', framealpha=0.9)
ax.grid(True, alpha=0.25)

fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp1_coverage_vs_age.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


# =============================================================================
# Figure 3: Exp 2 — Process Model Comparison (NEES sweep)
# =============================================================================

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), gridspec_kw={'width_ratios': [1.1, 1]})

# Left: NEES
model_colors = {"random walk": "#8b5cf6", "const. velocity": "#3b82f6", "OU": "#10b981"}
model_markers = {"random walk": "s", "const. velocity": "o", "OU": "^"}

for name, vals in nees_exp2.items():
    ax1.plot(ages_exp2, vals, marker=model_markers[name], color=model_colors[name],
             linewidth=2.5, markersize=7, label=name, zorder=3)

ax1.axhline(3.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax1.axhspan(2.0, 4.5, color='gray', alpha=0.08)
ax1.text(1020, 3.15, 'NEES = 3', fontsize=9, color='gray', va='bottom')

ax1.set_xlabel('Observation Age (ms)')
ax1.set_ylabel('NEES')
ax1.set_title('NEES vs Observation Age')
ax1.legend(loc='upper right', framealpha=0.9)
ax1.grid(True, alpha=0.25)
ax1.set_xlim(-20, 1050)
ax1.set_ylim(0, 4.0)

# Right: Trace ratio
for name, vals in trace_ratio_exp2.items():
    ax2.plot(ages_exp2, vals, marker=model_markers[name], color=model_colors[name],
             linewidth=2.5, markersize=7, label=name, zorder=3)

ax2.axhline(1.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax2.text(1020, 1.15, 'ratio = 1', fontsize=9, color='gray', va='bottom')

ax2.set_xlabel('Observation Age (ms)')
ax2.set_ylabel('Trace Ratio  (analytical / empirical)')
ax2.set_title('Covariance Inflation Factor')
ax2.legend(loc='upper right', framealpha=0.9)
ax2.grid(True, alpha=0.25)
ax2.set_xlim(-20, 1050)
ax2.set_ylim(0, 8)

fig.suptitle('Exp 2 — Process Model Comparison (100k MC samples, v=2 m/s)',
             fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp2_model_comparison.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


# =============================================================================
# Figure 4: The "why" diagram — what happens at 300ms staleness
# =============================================================================

fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))

# Data at 300ms
age_idx = 4  # index for 300ms in exp1 arrays
conditions_300 = ["oracle\n(no inflation)", "naive\nreuse", "const.\nvelocity",
                  "random\nwalk", "OU"]
nees_300 = [nees["oracle (no inflation)"][age_idx],
            nees["naive reuse"][age_idx],
            nees["const. velocity"][age_idx],
            nees["random walk"][age_idx],
            nees["OU"][age_idx]]
cov3_300 = [cov3s["oracle (no inflation)"][age_idx],
            cov3s["naive reuse"][age_idx],
            cov3s["const. velocity"][age_idx],
            cov3s["random walk"][age_idx],
            cov3s["OU"][age_idx]]
tr_300 =   [trace_ratio["oracle (no inflation)"][age_idx],
            trace_ratio["naive reuse"][age_idx],
            trace_ratio["const. velocity"][age_idx],
            trace_ratio["random walk"][age_idx],
            trace_ratio["OU"][age_idx]]

bar_colors = ["#ef4444", "#f97316", "#3b82f6", "#8b5cf6", "#10b981"]
x = np.arange(5)

# NEES
ax = axes[0]
bars = ax.bar(x, nees_300, color=bar_colors, edgecolor='black', linewidth=0.8, alpha=0.85)
ax.axhline(3.0, color='gray', linestyle='--', linewidth=1.5)
ax.axhspan(2.0, 4.5, color='gray', alpha=0.08)
ax.set_ylabel('NEES')
ax.set_title('Calibration\n(target = 3.0)')
ax.set_xticks(x)
ax.set_xticklabels(conditions_300, fontsize=8)
ax.set_ylim(0, 12)
# Value labels
for bar, val in zip(bars, nees_300):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.2,
            f'{val:.1f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

# 3σ Coverage
ax = axes[1]
bars = ax.bar(x, cov3_300, color=bar_colors, edgecolor='black', linewidth=0.8, alpha=0.85)
ax.axhline(97.1, color='gray', linestyle='--', linewidth=1.5)
ax.set_ylabel('3σ Coverage (%)')
ax.set_title('Coverage\n(target = 97.1%)')
ax.set_xticks(x)
ax.set_xticklabels(conditions_300, fontsize=8)
ax.set_ylim(0, 110)
for bar, val in zip(bars, cov3_300):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1.5,
            f'{val:.0f}%', ha='center', va='bottom', fontsize=9, fontweight='bold')

# Trace ratio
ax = axes[2]
bars = ax.bar(x, tr_300, color=bar_colors, edgecolor='black', linewidth=0.8, alpha=0.85)
ax.axhline(1.0, color='gray', linestyle='--', linewidth=1.5)
ax.set_ylabel('Trace Ratio (ana/emp)')
ax.set_title('Inflation\n(target = 1.0)')
ax.set_xticks(x)
ax.set_xticklabels(conditions_300, fontsize=8)
ax.set_ylim(0, 8)
for bar, val in zip(bars, tr_300):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.15,
            f'{val:.1f}x', ha='center', va='bottom', fontsize=9, fontweight='bold')

fig.suptitle('Snapshot at 300 ms Staleness — Why Constant Velocity Wins',
             fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp1_snapshot_300ms.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)

# =============================================================================
# Extended Experiment Data (100k MC, 2026-04-02)
# =============================================================================

# Exp 2b: All-3-stale vs 1-fresh-2-stale
ages_2b = np.array([0, 50, 100, 200, 300, 500, 750, 1000])

nees_all_stale =    [3.03, 2.60, 2.02, 1.45, 1.27, 1.15, 1.11, 1.10]
rmse_all_stale =    [0.2553, 0.2660, 0.2995, 0.4071, 0.5415, 0.8355, 1.2199, 1.6114]
nees_1fresh2stale = [3.02, 2.73, 2.24, 1.61, 1.36, 1.18, 1.11, 1.09]
rmse_1fresh2stale = [0.2547, 0.2627, 0.2855, 0.3629, 0.4651, 0.6966, 1.0013, 1.3173]

# Velocity mismatch (age=300ms, true v=2 m/s)
v_assumed = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0])
nees_vmismatch = [3.88, 3.47, 2.64, 1.90, 1.35, 1.00, 0.75, 0.46, 0.31]
cov3_vmismatch = [93.5, 95.7, 98.8, 99.9, 100., 100., 100., 100., 100.]

# Moving target (age=300ms, camera v=2 m/s)
v_target = np.array([0.0, 0.5, 1.0, 2.0, 3.0, 5.0])
nees_cam_only =     [1.36, 1.42, 1.58, 2.23, 3.32, 6.82]
cov3_cam_only =     [100., 100., 100., 99.7, 98.1, 75.3]
nees_cam_plus_tgt = [1.36, 1.36, 1.36, 1.35, 1.34, 1.34]
cov3_cam_plus_tgt = [100., 100., 100., 100., 100., 100.]
rmse_moving =       [0.4651, 0.4746, 0.5032, 0.6081, 0.7569, 1.1169]


# =============================================================================
# Figure 5: Exp 2b — All-stale vs 1-fresh-2-stale
# =============================================================================

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Left: NEES comparison
ax1.plot(ages_2b, nees_all_stale, 'o-', color='#ef4444', linewidth=2.5, markersize=7,
         label='All 3 cameras stale')
ax1.plot(ages_2b, nees_1fresh2stale, 's-', color='#3b82f6', linewidth=2.5, markersize=7,
         label='1 fresh + 2 stale')
ax1.axhline(3.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax1.axhspan(2.0, 4.5, color='gray', alpha=0.08)
ax1.text(1020, 3.15, 'NEES = 3', fontsize=9, color='gray')
ax1.set_xlabel('Observation Age (ms)')
ax1.set_ylabel('NEES')
ax1.set_title('Covariance Calibration')
ax1.legend(loc='upper right', framealpha=0.9)
ax1.grid(True, alpha=0.25)
ax1.set_ylim(0, 4.0)

# Right: RMSE comparison
ax2.plot(ages_2b, rmse_all_stale, 'o-', color='#ef4444', linewidth=2.5, markersize=7,
         label='All 3 cameras stale')
ax2.plot(ages_2b, rmse_1fresh2stale, 's-', color='#3b82f6', linewidth=2.5, markersize=7,
         label='1 fresh + 2 stale')
ax2.set_xlabel('Observation Age (ms)')
ax2.set_ylabel('RMSE (m)')
ax2.set_title('Triangulation Accuracy')
ax2.legend(loc='upper left', framealpha=0.9)
ax2.grid(True, alpha=0.25)

# Annotate the gap
ax2.annotate('~20% RMSE\npenalty', xy=(600, 0.77), fontsize=9, color='#7f1d1d',
             ha='center', fontstyle='italic',
             arrowprops=dict(arrowstyle='->', color='#7f1d1d', lw=1.5),
             xytext=(750, 1.1))

fig.suptitle('Exp 2b — All Cameras Stale: Calibrated but Noisier',
             fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp2b_all_stale.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


# =============================================================================
# Figure 6: Velocity Mismatch Sensitivity
# =============================================================================

fig, ax = plt.subplots(figsize=(9, 5.5))

# NEES on left axis
color_nees = '#3b82f6'
ax.plot(v_assumed, nees_vmismatch, 'o-', color=color_nees, linewidth=2.5, markersize=8,
        label='NEES', zorder=3)
ax.axhline(3.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax.axhspan(2.0, 4.5, color='gray', alpha=0.06)

# Mark the true velocity
ax.axvline(2.0, color='#10b981', linestyle=':', linewidth=2, alpha=0.8)
ax.text(2.05, 0.15, 'true v = 2 m/s', fontsize=10, color='#10b981',
        rotation=90, va='bottom')

# Shade danger zones
ax.fill_between(v_assumed, 3.0, [max(n, 3.0) for n in nees_vmismatch],
                color='#fecaca', alpha=0.3, label='overconfident zone')
ax.fill_between(v_assumed, [min(n, 1.0) for n in nees_vmismatch], 1.0,
                color='#dbeafe', alpha=0.3, label='wasteful zone')

# 3σ coverage on right axis
ax2 = ax.twinx()
color_cov = '#f97316'
ax2.plot(v_assumed, cov3_vmismatch, 's--', color=color_cov, linewidth=2, markersize=7,
         alpha=0.8, label='3σ coverage')
ax2.set_ylabel('3σ Coverage (%)', color=color_cov, fontweight='bold')
ax2.tick_params(axis='y', labelcolor=color_cov)
ax2.set_ylim(88, 101)

ax.set_xlabel('Velocity Assumed in Σ_drift (m/s)', fontweight='bold')
ax.set_ylabel('NEES', fontweight='bold', color=color_nees)
ax.set_title('Velocity Mismatch Sensitivity (age=300ms, true v=2 m/s)')
ax.tick_params(axis='y', labelcolor=color_nees)
ax.set_ylim(0, 4.5)
ax.grid(True, alpha=0.2)

# Combined legend
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.9)

fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp_velocity_mismatch.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


# =============================================================================
# Figure 7: Moving Target — Camera-only vs Camera+Target drift
# =============================================================================

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

# Left: NEES
ax1.plot(v_target, nees_cam_only, 'x-', color='#ef4444', linewidth=2.5, markersize=9,
         label='Camera drift only', markeredgewidth=2)
ax1.plot(v_target, nees_cam_plus_tgt, 'o-', color='#3b82f6', linewidth=2.5, markersize=7,
         label='Camera + target drift')
ax1.axhline(3.0, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax1.axhspan(2.0, 4.5, color='gray', alpha=0.06)
ax1.text(5.1, 3.15, 'NEES = 3', fontsize=9, color='gray')

# Annotate breakdown
ax1.annotate('overconfident!\ncovariance too small',
             xy=(5.0, 6.82), fontsize=9, color='#dc2626', ha='right',
             fontstyle='italic',
             arrowprops=dict(arrowstyle='->', color='#dc2626', lw=1.5),
             xytext=(3.8, 5.8))

ax1.set_xlabel('Target Velocity (m/s)')
ax1.set_ylabel('NEES')
ax1.set_title('Covariance Calibration')
ax1.legend(loc='upper left', framealpha=0.9)
ax1.grid(True, alpha=0.25)
ax1.set_ylim(0, 8)

# Right: 3σ Coverage
ax2.plot(v_target, cov3_cam_only, 'x-', color='#ef4444', linewidth=2.5, markersize=9,
         label='Camera drift only', markeredgewidth=2)
ax2.plot(v_target, cov3_cam_plus_tgt, 'o-', color='#3b82f6', linewidth=2.5, markersize=7,
         label='Camera + target drift')
ax2.axhline(97.1, color='gray', linestyle='--', linewidth=1.5, alpha=0.7)
ax2.text(5.1, 97.8, '97.1%', fontsize=9, color='gray')

# Shade the RMSE on secondary axis
ax2r = ax2.twinx()
ax2r.fill_between(v_target, 0, rmse_moving, color='#e5e7eb', alpha=0.4)
ax2r.plot(v_target, rmse_moving, ':', color='#6b7280', linewidth=1.5, label='RMSE (m)')
ax2r.set_ylabel('RMSE (m)', color='#6b7280')
ax2r.tick_params(axis='y', labelcolor='#6b7280')
ax2r.set_ylim(0, 1.5)

ax2.set_xlabel('Target Velocity (m/s)')
ax2.set_ylabel('3σ Coverage (%)')
ax2.set_title('Coverage & Accuracy')
lines1, labels1 = ax2.get_legend_handles_labels()
lines2, labels2 = ax2r.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc='lower left', framealpha=0.9)
ax2.grid(True, alpha=0.25)
ax2.set_ylim(70, 103)

fig.suptitle('Moving Target — Adding v_target²·Δt² to Σ_drift Fixes Calibration',
             fontweight='bold', fontsize=14, y=1.02)
fig.tight_layout()
path = os.path.join(SAVE_DIR, "exp_moving_target.png")
fig.savefig(path, dpi=200, bbox_inches='tight')
print(f"Saved: {path}")
plt.close(fig)


print("\nAll plots saved.")
