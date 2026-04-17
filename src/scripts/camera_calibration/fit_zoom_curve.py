#!/usr/bin/env python3
"""Fit Z_effective(zoom_cmd) from intrinsics_summary.csv.

The SIYI A8 mini has a fixed-focal-length lens and pure digital zoom
(see doc/A8 mini User Manual v1.10.pdf). So every per-zoom calibration
is physically the same lens, and fx scales linearly with the *realised*
digital zoom factor Z:

    fx(cmd) = fx_1x · Z(cmd)

The command integer exposed over the SDK (`zoom_cmd` topic) is NOT equal
to Z — measurements show the mapping is nonlinear and resolution-capped
(5.5x max at 1080p). This script fits Z(cmd) using the per-zoom
calibration results, weighted by the fx covariance. Three functional
forms are available (all constrained to Z(1)=1):

    polynomial    Z = 1 + sum_{k=1..degree} a_k · (cmd-1)^k
    exponential   Z = 1 + a · (exp(b · (cmd-1)) - 1)
    power         Z = 1 + a · (cmd-1)^b

Input:  datasets/camera_calibration/<date>/intrinsics_summary.csv
Output: fit printed to stdout, optional JSON lookup table, optional plot.
"""

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.optimize import curve_fit


# --- model definitions ------------------------------------------------------

@dataclass
class FitResult:
    name: str
    params: np.ndarray
    param_names: list[str]
    formula: str
    n_params: int
    evaluate: Callable[[np.ndarray], np.ndarray]


def fit_polynomial(cmds: np.ndarray, z: np.ndarray, sz: np.ndarray, degree: int) -> FitResult:
    # Constrained through (1, 1): fit y = sum_{k=1..degree} a_k · u^k
    # with u = cmd - 1, y = Z - 1. Closed-form weighted LS.
    u = cmds - 1.0
    y = z - 1.0
    design = np.vstack([u**k for k in range(1, degree + 1)]).T
    inv_sigma = 1.0 / np.maximum(sz, 1e-12)
    coeffs, *_ = np.linalg.lstsq(design * inv_sigma[:, None], y * inv_sigma, rcond=None)

    def evaluate(c: np.ndarray) -> np.ndarray:
        uu = c - 1.0
        return 1.0 + sum(coeffs[k] * uu ** (k + 1) for k in range(degree))

    names = [f"a{k+1}" for k in range(degree)]
    formula = "Z = 1 + " + " + ".join(f"{coeffs[k]:+.5g}·(cmd-1)^{k+1}" for k in range(degree))
    return FitResult("polynomial", coeffs, names, formula, degree, evaluate)


def fit_exponential(cmds: np.ndarray, z: np.ndarray, sz: np.ndarray) -> FitResult:
    # Z - 1 = a · (exp(b · (cmd-1)) - 1), through (1, 1).
    def model(c, a, b):
        return 1.0 + a * (np.exp(b * (c - 1.0)) - 1.0)

    p0 = (0.5, 0.4)
    (a, b), _ = curve_fit(model, cmds, z, p0=p0, sigma=sz, absolute_sigma=True, maxfev=10000)

    def evaluate(c: np.ndarray) -> np.ndarray:
        return model(c, a, b)

    formula = f"Z = 1 + {a:+.5g}·(exp({b:+.5g}·(cmd-1)) - 1)"
    return FitResult("exponential", np.array([a, b]), ["a", "b"], formula, 2, evaluate)


def fit_power(cmds: np.ndarray, z: np.ndarray, sz: np.ndarray) -> FitResult:
    # Z - 1 = a · (cmd - 1)^b, through (1, 1). Points where cmd==1 get (cmd-1)^b = 0.
    def model(c, a, b):
        u = np.maximum(c - 1.0, 0.0)
        return 1.0 + a * np.where(u > 0, u**b, 0.0)

    p0 = (0.2, 1.2)
    (a, b), _ = curve_fit(model, cmds, z, p0=p0, sigma=sz, absolute_sigma=True, maxfev=10000)

    def evaluate(c: np.ndarray) -> np.ndarray:
        return model(c, a, b)

    formula = f"Z = 1 + {a:+.5g}·(cmd-1)^{b:+.5g}"
    return FitResult("power", np.array([a, b]), ["a", "b"], formula, 2, evaluate)


MODELS = {
    "polynomial": fit_polynomial,
    "exponential": fit_exponential,
    "power": fit_power,
}


# --- plumbing --------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("csv_path", type=Path)
    p.add_argument("--model", choices=["polynomial", "exponential", "power", "all"],
                   default="all",
                   help="Model form (default: all — fit and compare every model).")
    p.add_argument("--degree", type=int, default=2,
                   help="Polynomial degree (ignored for other models, default 2).")
    p.add_argument("--max-cmd", type=float, default=None,
                   help="Drop points with zoom_cmd > this value (e.g. exclude noisy 6x).")
    p.add_argument("--output", type=Path, default=None,
                   help="Optional JSON output path (only written for the best / selected model).")
    p.add_argument("--samples", type=int, default=51)
    p.add_argument("--plot", type=Path, default=None)
    return p.parse_args()


def load_points(csv_path: Path) -> list[tuple[float, float, float]]:
    rows: list[tuple[float, float, float]] = []
    with csv_path.open() as f:
        for r in csv.DictReader(f):
            cmd = float(r["zoom_level"].rstrip("x"))
            rows.append((cmd, float(r["fx"]), float(r["fx_std"])))
    rows.sort(key=lambda t: t[0])
    return rows


def run_fit(name: str, cmds: np.ndarray, z: np.ndarray, sz: np.ndarray, degree: int) -> FitResult:
    fn = MODELS[name]
    return fn(cmds, z, sz, degree) if name == "polynomial" else fn(cmds, z, sz)


def chi2_dof(fit: FitResult, cmds: np.ndarray, z: np.ndarray, sz: np.ndarray) -> tuple[float, int]:
    z_fit = fit.evaluate(cmds)
    chi2 = float(np.sum(((z - z_fit) / sz) ** 2))
    dof = max(len(cmds) - fit.n_params, 1)
    return chi2, dof


def print_fit(fit: FitResult, cmds: np.ndarray, z: np.ndarray, sz: np.ndarray) -> None:
    chi2, dof = chi2_dof(fit, cmds, z, sz)
    print(f"\n=== {fit.name} ===")
    print(fit.formula)
    print(f"χ² = {chi2:.3f}  dof = {dof}  reduced χ² = {chi2/dof:.3f}")
    z_fit = fit.evaluate(cmds)
    print(f"{'cmd':>6} {'Z_meas':>8} {'σZ':>8} {'Z_fit':>8} {'resid/σ':>10}")
    for i in range(len(cmds)):
        print(f"{cmds[i]:>6.1f} {z[i]:>8.3f} {sz[i]:>8.3f} {z_fit[i]:>8.3f} "
              f"{(z[i]-z_fit[i])/sz[i]:>+10.2f}σ")


def save_plot(
    path: Path,
    fits: list[FitResult],
    used: list[tuple[float, float, float]],
    z_used: np.ndarray,
    sz_used: np.ndarray,
    excluded: list[tuple[float, float, float]],
    fx_1: float,
    s_fx_1: float,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cmd_used = np.array([r[0] for r in used])
    cmd_all = np.array([r[0] for r in used + excluded])
    cmd_min, cmd_max = float(cmd_all.min()), float(cmd_all.max())
    cmd_fine = np.linspace(cmd_min, cmd_max, 400)

    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(8.5, 7.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )

    ax_top.errorbar(cmd_used, z_used, yerr=sz_used, fmt="o", color="black",
                    label="measured (used)", capsize=3, zorder=5, markersize=6)
    if excluded:
        z_excl = np.array([r[1] / fx_1 for r in excluded])
        sz_excl = np.sqrt(
            (np.array([r[2] for r in excluded]) / fx_1) ** 2
            + (z_excl * s_fx_1 / fx_1) ** 2
        )
        ax_top.errorbar([r[0] for r in excluded], z_excl, yerr=sz_excl,
                        fmt="x", color="red", label="excluded", capsize=3, zorder=5)
    ax_top.plot([cmd_min, cmd_max], [cmd_min, cmd_max], "--", color="gray",
                alpha=0.5, label="Z = cmd (ideal)")

    for i, fit in enumerate(fits):
        chi2, dof = chi2_dof(fit, cmd_used, z_used, sz_used)
        ax_top.plot(cmd_fine, fit.evaluate(cmd_fine), "-",
                    label=f"{fit.name}  χ²/dof={chi2/dof:.2f}")
        resid = (z_used - fit.evaluate(cmd_used)) / sz_used
        ax_bot.plot(cmd_used, resid, "o-", label=fit.name, alpha=0.8, markersize=5)

    ax_top.set_title(f"SIYI zoom_cmd → Z_effective  (fx_1x = {fx_1:.1f} ± {s_fx_1:.1f})")
    ax_top.set_ylabel("Z_effective = fx(cmd) / fx(1x)")
    ax_top.grid(alpha=0.3)
    ax_top.legend(loc="upper left")

    ax_bot.axhline(0, color="gray", linewidth=0.8)
    ax_bot.axhspan(-1, 1, color="gray", alpha=0.12)
    ax_bot.set_ylabel("residual / σ")
    ax_bot.set_xlabel("zoom_cmd")
    ax_bot.grid(alpha=0.3)
    ax_bot.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_json(
    output: Path,
    fit: FitResult,
    used: list[tuple[float, float, float]],
    excluded: list[tuple[float, float, float]],
    fx_1: float,
    s_fx_1: float,
    cmds: np.ndarray,
    z: np.ndarray,
    sz: np.ndarray,
    n_samples: int,
) -> None:
    chi2, dof = chi2_dof(fit, cmds, z, sz)
    cmd_grid = np.linspace(cmds.min(), cmds.max(), n_samples)
    z_grid = fit.evaluate(cmd_grid)
    payload = {
        "fx_at_1x": fx_1,
        "fx_at_1x_std": s_fx_1,
        "model": fit.name,
        "formula": fit.formula,
        "parameters": {name: float(v) for name, v in zip(fit.param_names, fit.params)},
        "chi2": chi2,
        "dof": dof,
        "reduced_chi2": chi2 / dof,
        "points_used": [
            {"cmd": float(c), "fx": float(f), "fx_std": float(s)}
            for (c, f, s) in used
        ],
        "points_excluded": [
            {"cmd": float(c), "fx": float(f), "fx_std": float(s)}
            for (c, f, s) in excluded
        ],
        "lookup_table": [
            {"cmd": float(cmd_grid[i]), "z_effective": float(z_grid[i])}
            for i in range(len(cmd_grid))
        ],
    }
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    rows = load_points(args.csv_path)
    if not rows or rows[0][0] != 1.0:
        print("need a 1x baseline row in the CSV", file=sys.stderr)
        return 1

    fx_1, s_fx_1 = rows[0][1], rows[0][2]
    used = [r for r in rows if args.max_cmd is None or r[0] <= args.max_cmd]
    excluded = [r for r in rows if r not in used]
    cmds = np.array([r[0] for r in used])
    fxs = np.array([r[1] for r in used])
    sfxs = np.array([r[2] for r in used])

    z = fxs / fx_1
    sz = np.sqrt((sfxs / fx_1) ** 2 + (z * s_fx_1 / fx_1) ** 2)

    names = ["polynomial", "exponential", "power"] if args.model == "all" else [args.model]
    fits: list[FitResult] = []
    for name in names:
        try:
            fits.append(run_fit(name, cmds, z, sz, args.degree))
        except (RuntimeError, ValueError) as err:
            print(f"[{name}] fit failed: {err}", file=sys.stderr)

    if not fits:
        return 1

    for fit in fits:
        print_fit(fit, cmds, z, sz)

    best = min(fits, key=lambda f: chi2_dof(f, cmds, z, sz)[0] / chi2_dof(f, cmds, z, sz)[1])
    print(f"\nbest by reduced χ²: {best.name}")

    if args.output is not None:
        write_json(args.output, best, used, excluded, fx_1, s_fx_1, cmds, z, sz, args.samples)
        print(f"wrote {args.output} (model={best.name})")

    if args.plot is not None:
        save_plot(args.plot, fits, used, z, sz, excluded, fx_1, s_fx_1)
        print(f"wrote {args.plot}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
