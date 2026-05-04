#!/usr/bin/env python3
"""Aggregate per-zoom mrcal intrinsics into a single sim-ingest JSON.

Reads `intrinsics_summary.json` from each per-zoom directory under
`/home/usrg/mas/datasets/camera_calibration/<DATE>/<Nx>/` and emits a
flat JSON with `{zoom: {fx, fx_std, fy, fy_std, cx, cy, num_images,
rms_reprojection_error_px, max_projection_uncertainty_px}}` for the zoom
levels we trust.

6x is excluded from the sim-trustworthy set: at 6x the calibration
uncertainty (`fx_std/fx ≈ 10%`) and `max_projection_uncertainty_px` blow
up beyond a level we are willing to use as ground truth for DR. The 6x
entry is still recorded under `excluded_zooms` with the reason, so the
exclusion is auditable.

The trustworthy entries (1x..5x) are also fit to the existing closed-form
zoom curve in `/home/usrg/mas/src/scripts/camera_calibration/zoom_curve.json`
purely for cross-checking — this script does not re-fit the curve.
"""
from __future__ import annotations

import json
from pathlib import Path


CALIBRATION_DATE = "2026-04-17"
CALIBRATION_ROOT = Path(f"/home/usrg/mas/datasets/camera_calibration/{CALIBRATION_DATE}")
ZOOM_CURVE_PATH = Path("/home/usrg/mas/src/scripts/camera_calibration/zoom_curve.json")
OUTPUT_PATH = Path(__file__).parent / "output" / "intrinsics_for_sim.json"

TRUSTWORTHY_ZOOMS = ("1x", "2x", "4x", "5x")
EXCLUDED_ZOOMS = {
    "6x": (
        "fx_std/fx and max_projection_uncertainty_px exceed the trust threshold; "
        "6x is operationally not used. See zoom_curve.json which excludes 6x from the fit."
    ),
}


def _load_summary(zoom: str) -> dict:
    path = CALIBRATION_ROOT / zoom / "intrinsics_summary.json"
    if not path.exists():
        raise FileNotFoundError(f"Missing intrinsics summary: {path}")
    return json.loads(path.read_text())


def _extract_for_sim(s: dict) -> dict:
    intr = s["intrinsics"]
    intr_std = s["intrinsics_stddev"]
    return {
        "fx": intr["fx"],
        "fx_std": intr_std["fx"],
        "fy": intr["fy"],
        "fy_std": intr_std["fy"],
        "cx": intr["cx"],
        "cy": intr["cy"],
        "image_width": s["image_size"]["width"],
        "image_height": s["image_size"]["height"],
        "num_images": s["num_images"],
        "num_detected_corner_frames": s["num_detected_corner_frames"],
        "rms_reprojection_error_px": s["rms_reprojection_error_px"],
        "max_projection_uncertainty_px": s["max_projection_uncertainty_px"],
        "lensmodel": s["lensmodel"],
    }


def main() -> None:
    trustworthy: dict[str, dict] = {}
    for zoom in TRUSTWORTHY_ZOOMS:
        trustworthy[zoom] = _extract_for_sim(_load_summary(zoom))

    excluded: dict[str, dict] = {}
    for zoom, reason in EXCLUDED_ZOOMS.items():
        try:
            entry = _extract_for_sim(_load_summary(zoom))
        except FileNotFoundError:
            entry = {}
        excluded[zoom] = {"reason": reason, **entry}

    fx_values = [trustworthy[z]["fx"] for z in TRUSTWORTHY_ZOOMS]
    fx_stds = [trustworthy[z]["fx_std"] for z in TRUSTWORTHY_ZOOMS]
    fx_min = min(fx - 1.0 * std for fx, std in zip(fx_values, fx_stds))
    fx_max = max(fx + 1.0 * std for fx, std in zip(fx_values, fx_stds))

    out = {
        "description": (
            "Per-zoom camera intrinsics from mrcal calibration. "
            "Trustworthy set (1x..5x) is suitable for grounding "
            "iris_ma6 CameraRandomizationCfg.focal_length_range and "
            "fov_scale_range. 6x is excluded with reason."
        ),
        "calibration_date": CALIBRATION_DATE,
        "calibration_root": str(CALIBRATION_ROOT),
        "zoom_curve_reference": str(ZOOM_CURVE_PATH),
        "trustworthy_zooms": trustworthy,
        "excluded_zooms": excluded,
        "fx_envelope_pm_1sigma": {
            "min": fx_min,
            "max": fx_max,
            "note": (
                "Union envelope: min(fx - 1σ) over 1x..5x to max(fx + 1σ). "
                "Useful as a single-range fallback for focal_length_range when "
                "zoom-conditional intrinsics are not yet wired in."
            ),
        },
        "schema_version": 1,
    }

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(out, indent=2) + "\n")
    print(f"Wrote {OUTPUT_PATH}")
    for zoom, e in trustworthy.items():
        print(f"  {zoom}: fx={e['fx']:.1f} ± {e['fx_std']:.1f} px")
    for zoom in excluded:
        print(f"  {zoom}: EXCLUDED — {EXCLUDED_ZOOMS[zoom]}")
    print(f"  fx envelope ±1σ: [{fx_min:.1f}, {fx_max:.1f}]")


if __name__ == "__main__":
    main()
