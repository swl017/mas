#!/usr/bin/env python3
"""Populate `metrics.json` from a solved mrcal cameramodel.

Extracts the three scalar acceptance metrics plus per-parameter 1σ for
the intrinsics, so ticket 028's summarize step can aggregate across
zoom levels without hand-editing metrics.json.

- rms / worst reprojection error: pulled from the cameramodel comment
  header (mrcal-calibrate-cameras writes them there at solve time).
- max projection uncertainty: sampled on a grid across the imager via
  `mrcal.projection_uncertainty(..., what='worstdirection-stdev')`.
- parameter stddev: diagonal of (JtJ)^-1 for the intrinsic state block,
  obtained via `optimizer_callback` + the Cholesky factorization, then
  converted from packed to real units via `unpack_state`.

Usage:
    python3 src/scripts/camera_calibration/compute_metrics.py \
      <zoom_dir>
    # e.g. datasets/camera_calibration/2026-04-17/1x

Writes `<zoom_dir>/metrics.json`.
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

import mrcal


INTRINSIC_NAMES = (
    "fx", "fy", "cx", "cy",
    "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument(
        "zoom_dir",
        type=Path,
        help="Path to a <date>/<zoom>x session directory.",
    )
    p.add_argument(
        "--grid",
        type=int,
        default=21,
        help="NxN grid of sample pixels for projection uncertainty (default 21).",
    )
    p.add_argument(
        "--distance-m",
        type=float,
        default=10.0,
        help="Scene distance used when evaluating projection uncertainty (default 10 m).",
    )
    p.add_argument(
        "--observed-pixel-uncertainty",
        type=float,
        default=1.0,
        help="Assumed 1σ of observed corner noise, in pixels (default 1.0). "
             "Used only for parameter_stddev scaling.",
    )
    return p.parse_args()


def residuals_from_header(model_path: Path) -> tuple[float | None, float | None]:
    text = model_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    rms = worst = None
    for line in text:
        if not line.startswith("#"):
            break
        m = re.search(r"RMS reprojection error:\s*([0-9.eE+-]+)", line)
        if m:
            rms = float(m.group(1))
        m = re.search(r"Worst residual \(by measurement\):\s*([0-9.eE+-]+)", line)
        if m:
            worst = float(m.group(1))
    return rms, worst


def max_projection_uncertainty(model: "mrcal.cameramodel", grid: int, distance_m: float) -> float:
    W, H = model.imagersize()
    xs = np.linspace(0, W - 1, grid)
    ys = np.linspace(0, H - 1, grid)
    qs = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2).astype(float)
    pcam = distance_m * mrcal.unproject(qs, *model.intrinsics(), normalize=True)
    stddev = mrcal.projection_uncertainty(
        pcam, model=model, what="worstdirection-stdev"
    )
    return float(np.max(stddev))


def parameter_stddev(model: "mrcal.cameramodel", obs_unc: float) -> dict[str, float | None]:
    oi = model.optimization_inputs()
    p_packed, _x, _J, fact = mrcal.optimizer_callback(**oi)
    n_state = p_packed.size
    n_intr = mrcal.num_states_intrinsics(**oi)
    i_start = mrcal.state_index_intrinsics(0, **oi)

    bt = np.zeros((n_intr, n_state), dtype=float)
    for k in range(n_intr):
        bt[k, i_start + k] = 1.0
    xt = fact.solve_xt_JtJ_bt(bt)
    var_packed = np.array([xt[k, i_start + k] for k in range(n_intr)])

    scales = np.ones(n_state)
    mrcal.unpack_state(scales, **oi)
    pack_scales = scales[i_start : i_start + n_intr]

    stddev = obs_unc * pack_scales * np.sqrt(np.maximum(var_packed, 0.0))

    result: dict[str, float | None] = {name: None for name in INTRINSIC_NAMES}
    for k, name in enumerate(INTRINSIC_NAMES):
        if k < n_intr:
            result[name] = float(stddev[k])
    return result


def main() -> int:
    args = parse_args()
    zoom_dir: Path = args.zoom_dir.resolve()
    calib_dir = zoom_dir / "calibration"
    candidates = sorted(calib_dir.glob("*.cameramodel"))
    if not candidates:
        print(f"no .cameramodel in {calib_dir}", file=sys.stderr)
        return 1
    model_path = candidates[0]
    model = mrcal.cameramodel(str(model_path))

    rms, worst = residuals_from_header(model_path)
    max_unc = max_projection_uncertainty(model, args.grid, args.distance_m)
    stddev = parameter_stddev(model, args.observed_pixel_uncertainty)

    metrics = {
        "rms_reprojection_error_px": rms,
        "worst_reprojection_error_px": worst,
        "max_projection_uncertainty_px": max_unc,
        "parameter_stddev": stddev,
    }

    metrics_path = zoom_dir / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {metrics_path}")
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
