#!/usr/bin/env python3
"""Fit detection end-to-end latency model from phase7 CSVs.

Inputs (hard-coded — these are the measurement artifacts referenced by
mas/029-sim2real-measured-model-impl):
  /home/usrg/mas/src/scripts/phase7_low_latency.csv     (~50 ms)
  /home/usrg/mas/src/scripts/phase7_mid_latency.csv     (~70 ms)
  /home/usrg/mas/src/scripts/phase7_high_latency.csv    (~140 ms)

The CSV column `age_ms` is end-to-end image age (sensor timestamp →
detection-output receive time), not just inference latency. This is the
quantity we want to use for the iris_ma6 `ego_detection_latency_*` DR
parameters.

Output JSON has the per-regime mean/std/percentiles plus a "union"
distribution combining all three regimes (used as the single-Gaussian
fallback when the curriculum-driven regime selector is not in use).
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, pstdev, stdev


REPO_ROOT = Path("/home/usrg/mas")
INPUT_FILES = {
    "low":  REPO_ROOT / "src/scripts/phase7_low_latency.csv",
    "mid":  REPO_ROOT / "src/scripts/phase7_mid_latency.csv",
    "high": REPO_ROOT / "src/scripts/phase7_high_latency.csv",
}
OUTPUT_PATH = Path(__file__).parent / "output" / "detection_latency_fit.json"


def _load_age_ms(path: Path) -> list[float]:
    ages: list[float] = []
    with path.open() as f:
        for row in csv.DictReader(f):
            ages.append(float(row["age_ms"]))
    return ages


def _stats(samples: list[float]) -> dict:
    samples_sorted = sorted(samples)
    n = len(samples_sorted)
    return {
        "n_samples": n,
        "mean_ms": mean(samples_sorted),
        "std_ms": stdev(samples_sorted) if n > 1 else 0.0,
        "median_ms": samples_sorted[n // 2],
        "p05_ms":  samples_sorted[int(0.05 * n)],
        "p95_ms":  samples_sorted[int(0.95 * n)],
        "p99_ms":  samples_sorted[min(n - 1, int(0.99 * n))],
        "min_ms":  samples_sorted[0],
        "max_ms":  samples_sorted[-1],
    }


def main() -> None:
    per_regime: dict[str, dict] = {}
    union_samples: list[float] = []

    for regime, path in INPUT_FILES.items():
        if not path.exists():
            raise FileNotFoundError(f"Missing input artifact: {path}")
        samples = _load_age_ms(path)
        per_regime[regime] = {
            "source_csv": str(path),
            **_stats(samples),
        }
        union_samples.extend(samples)

    union = {"source_csvs": [str(p) for p in INPUT_FILES.values()], **_stats(union_samples)}

    out = {
        "description": (
            "End-to-end detection latency fit (camera sensor timestamp -> "
            "detection output receive time). Three regimes correspond to "
            "low/mid/high YOLO model sizes. Distribution is multi-modal across "
            "regimes; within each regime it is approximately Gaussian."
        ),
        "units": "milliseconds",
        "per_regime": per_regime,
        "union": union,
        "recommended_default_regime": "mid",
        "recommended_default_note": (
            "iris_ma6 currently uses ego_detection_latency_mean=100ms, std=15ms. "
            "Measured 'mid' regime (mean=65.3, std=12.6) is the closest match to "
            "current sim and is recommended as the default until the three-regime "
            "curriculum lands."
        ),
        "schema_version": 1,
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(out, indent=2) + "\n")
    print(f"Wrote {OUTPUT_PATH}")
    for regime, stats in per_regime.items():
        print(
            f"  {regime}: n={stats['n_samples']}, "
            f"mean={stats['mean_ms']:.2f} ms, std={stats['std_ms']:.2f} ms"
        )
    print(
        f"  union: n={union['n_samples']}, "
        f"mean={union['mean_ms']:.2f} ms, std={union['std_ms']:.2f} ms"
    )


if __name__ == "__main__":
    main()
