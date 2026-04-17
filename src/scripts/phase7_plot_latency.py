#!/usr/bin/env python3
"""Plot Phase 7 latency distributions vs fitted Gaussians.

Loads the three `phase7_<preset>_latency.csv` files and produces a
single figure with:
- A histogram (density) of measured `age_ms` per preset.
- An overlaid Gaussian PDF fitted from the empirical mean + std.
- A K-S distance in the legend as a rough Gaussian-fit score.

Output: /tmp/032/phase7_latency_dist.png
"""

import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV_DIR = "/tmp/032"
OUT = os.path.join(CSV_DIR, "phase7_latency_dist.png")

PRESETS = [
    ("low",  "dronecop9-2 384×640 @ rtsp 640×360",    "tab:blue"),
    ("mid",  "dronecop9-2 544×960 @ rtsp 960×540",    "tab:orange"),
    ("high", "dronecop9-2 1088×1920 @ rtsp 1920×1080", "tab:red"),
]


def load(preset):
    path = os.path.join(CSV_DIR, f"phase7_{preset}_latency.csv")
    rows = list(csv.DictReader(open(path)))
    return np.asarray([float(r["age_ms"]) for r in rows], dtype=np.float64)


def ks_stat(samples, mu, sigma):
    """One-sample K-S distance against N(mu, sigma)."""
    from math import erf, sqrt
    x = np.sort(samples)
    n = len(x)
    emp = np.arange(1, n + 1) / n
    # Normal CDF via erf (no scipy dependency).
    cdf = 0.5 * (1.0 + np.vectorize(lambda v: erf((v - mu) / (sigma * sqrt(2.0))))(x))
    return float(np.max(np.abs(emp - cdf)))


def main():
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=False)
    for ax, (preset, label, color) in zip(axes, PRESETS):
        samples = load(preset)
        mu = float(samples.mean())
        sigma = float(samples.std(ddof=1))
        p50 = float(np.median(samples))
        p95 = float(np.quantile(samples, 0.95))
        ks = ks_stat(samples, mu, sigma)

        # Histogram (density) — 2 ms bins, adaptive range.
        lo = int(samples.min() // 2) * 2
        hi = int(samples.max() // 2) * 2 + 2
        bins = np.arange(lo, hi + 2, 2)
        ax.hist(
            samples, bins=bins, density=True,
            color=color, alpha=0.55, edgecolor="black", linewidth=0.3,
            label=f"data (N={len(samples)})",
        )

        # Fitted Gaussian PDF.
        xs = np.linspace(mu - 4 * sigma, mu + 4 * sigma, 500)
        pdf = (1.0 / (sigma * np.sqrt(2.0 * np.pi))) * np.exp(
            -0.5 * ((xs - mu) / sigma) ** 2
        )
        ax.plot(xs, pdf, color="black", linewidth=1.8,
                label=f"N({mu:.1f}, {sigma:.1f}²)\nK-S = {ks:.3f}")

        # p50 / p95 guides.
        ax.axvline(p50, color="green", linestyle="--", linewidth=1, alpha=0.7)
        ax.axvline(p95, color="red",   linestyle="--", linewidth=1, alpha=0.7)
        ymax = ax.get_ylim()[1]
        ax.text(p50, ymax * 0.95, f"p50={p50:.1f}", color="green",
                ha="center", va="top", fontsize=8, rotation=90)
        ax.text(p95, ymax * 0.95, f"p95={p95:.1f}", color="red",
                ha="center", va="top", fontsize=8, rotation=90)

        ax.set_title(f"{preset}  —  {label}", fontsize=10)
        ax.set_xlabel("age (ms)")
        ax.set_ylabel("density")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.3)

    fig.suptitle("Phase 7 latency distributions vs fitted Gaussians "
                 "(dronecop9-2, camera-in-the-loop)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=140, bbox_inches="tight")
    print(f"wrote {OUT}")

    # Also print the K-S stats to stdout for the ticket.
    print("\nGoodness-of-fit (K-S distance, lower = more Gaussian):")
    for preset, _label, _c in PRESETS:
        samples = load(preset)
        mu = float(samples.mean())
        sigma = float(samples.std(ddof=1))
        ks = ks_stat(samples, mu, sigma)
        print(f"  {preset:5s}: μ={mu:6.1f}  σ={sigma:5.2f}  K-S={ks:.3f}")


if __name__ == "__main__":
    main()
