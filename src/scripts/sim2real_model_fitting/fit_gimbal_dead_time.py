#!/usr/bin/env python3
"""Fit gimbal command-to-first-move dead time from the rate-step bench.

Input artifact (referenced by mas/036):
    /home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/
        rate_step_summary.csv

The `latency_s` column is the per-step time from `t_cmd_ros` (the moment a
rate command is published) to the first measurable motion at the gimbal
motors. This is hardware dead time — packet transit, microcontroller
polling, motor commutation startup. Distinct from the first-order lag τ
that mas/035 already models.

Output JSON has per-axis stats, a union (yaw + pitch combined) Gaussian
fit, and percentile breakdown. The union distribution is what mas/036
samples from per-env at episode reset.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, stdev


REPO_ROOT = Path("/home/usrg/mas")
INPUT_CSV = REPO_ROOT / (
    "src/gimbal_controller/scripts/"
    "gimbal_rate_step_followspeed_tune/rate_step_summary.csv"
)
OUTPUT_PATH = Path(__file__).parent / "output" / "gimbal_dead_time_fit.json"


def _stats(samples: list[float]) -> dict:
    samples_sorted = sorted(samples)
    n = len(samples_sorted)
    return {
        "n_samples": n,
        "mean_s": mean(samples_sorted),
        "std_s": stdev(samples_sorted) if n > 1 else 0.0,
        "median_s": samples_sorted[n // 2],
        "p05_s":  samples_sorted[int(0.05 * n)],
        "p95_s":  samples_sorted[min(n - 1, int(0.95 * n))],
        "min_s":  samples_sorted[0],
        "max_s":  samples_sorted[-1],
    }


def main() -> None:
    if not INPUT_CSV.exists():
        raise FileNotFoundError(f"Missing input artifact: {INPUT_CSV}")

    per_axis: dict[str, list[float]] = {"yaw": [], "pitch": []}
    with INPUT_CSV.open() as f:
        for row in csv.DictReader(f):
            axis = row["axis"]
            latency = float(row["latency_s"])
            if axis in per_axis:
                per_axis[axis].append(latency)

    per_axis_stats = {axis: _stats(samples) for axis, samples in per_axis.items()}

    union_samples = per_axis["yaw"] + per_axis["pitch"]
    union_stats = _stats(union_samples)

    # Round mean/std to the precision the iris_ma6 cfg defaults use.
    recommended_mean = round(union_stats["mean_s"], 3)
    recommended_std = round(union_stats["std_s"], 3)

    # max bound: ~3σ above mean OR observed max, whichever is larger,
    # rounded up to the next 10 ms tick. Used as the buffer-depth cap.
    max_bound = max(
        union_stats["max_s"],
        union_stats["mean_s"] + 3.0 * union_stats["std_s"],
    )
    recommended_max = round(max_bound + 0.005, 2)

    out = {
        "description": (
            "Gimbal command-to-first-move dead time (rate command published "
            "-> first measurable motor motion). Source: rate-step bench "
            "(mas/026). Aggregated over yaw and pitch, all amplitudes."
        ),
        "units": "seconds",
        "input_csv": str(INPUT_CSV),
        "per_axis": per_axis_stats,
        "union": union_stats,
        "recommended_defaults": {
            "dead_time_mean_s": recommended_mean,
            "dead_time_std_s": recommended_std,
            "dead_time_max_s": recommended_max,
            "note": (
                "iris_ma6 GimbalRateLoopCfg sources its mas/036 defaults "
                "from these values. mean_s and std_s feed the per-env "
                "Gaussian sampler at episode reset; max_s caps the "
                "circular-buffer depth (clip and pre-allocation bound)."
            ),
        },
        "schema_version": 1,
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(out, indent=2) + "\n")
    print(f"Wrote {OUTPUT_PATH}")
    for axis, stats in per_axis_stats.items():
        print(
            f"  {axis}: n={stats['n_samples']}, "
            f"mean={stats['mean_s']*1000:.2f} ms, "
            f"std={stats['std_s']*1000:.2f} ms, "
            f"range=[{stats['min_s']*1000:.1f}, {stats['max_s']*1000:.1f}] ms"
        )
    print(
        f"  union: n={union_stats['n_samples']}, "
        f"mean={union_stats['mean_s']*1000:.2f} ms, "
        f"std={union_stats['std_s']*1000:.2f} ms"
    )
    print(
        f"  recommended: mean={recommended_mean*1000:.0f} ms, "
        f"std={recommended_std*1000:.0f} ms, "
        f"max={recommended_max*1000:.0f} ms"
    )


if __name__ == "__main__":
    main()
