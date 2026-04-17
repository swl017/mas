# Camera Intrinsics Calibration (`mrcal`)

This directory contains the bench-test workflow for ticket 028: calibrating the
SIYI zoom camera intrinsics per discrete zoom level with `mrcal`, then
collecting machine-readable summaries for downstream DR tuning.

## Scope

This workflow does all repo-local work for D1:

- creates a dated dataset/session layout under `datasets/camera_calibration/`
- runs monocular `mrcal` calibration per zoom level
- stores `.cameramodel`, logs, and summary sidecars per zoom level
- writes per-zoom JSON summaries and one aggregate CSV across zoom levels

This workflow does not:

- capture images from ROS2 directly
- modify any MAS runtime package
- derive the final `iris_ma6` DR ranges automatically

## Prerequisites

- `mrcal-calibrate-cameras`
- `mrcal-show-residuals`
- `mrcal-show-projection-uncertainty`
- `mrgingham`
- Python 3
- raw full-resolution images for each zoom level

The current dev environment in this workspace does not have `mrcal` installed,
so these scripts were written for bench execution once the package is available.

## Dataset Layout

Each session lives at:

```text
datasets/camera_calibration/<date>/
```

Each zoom level gets its own directory:

```text
datasets/camera_calibration/<date>/<zoom>x/
├── images/
├── analysis/
├── calibration/
├── logs/
├── notes.md
└── metrics.json
```

- `images/`: raw captured frames for one zoom level
- `analysis/`: corner detections, optional plots, uncertainty exports
- `calibration/`: `.cameramodel` outputs
- `logs/`: command logs from `mrcal`
- `notes.md`: operator notes for capture conditions and rejected frames
- `metrics.json`: sidecar consumed by the summary script

## 1. Initialize a Session

Example:

```bash
python3 src/scripts/calibration/init_camera_calibration_session.py \
  --date 2026-04-15 \
  --zoom-levels 1 2 4
```

This creates the directory tree and a session manifest:

```text
datasets/camera_calibration/2026-04-15/session_manifest.json
```

## 2. Capture Images

For each zoom level:

- set the camera to the target discrete zoom
- capture 30+ raw full-resolution images
- vary board position across center, edges, and corners
- vary range and board tilt
- reject blurred frames and frames with incomplete board visibility

Place the raw images into:

```text
datasets/camera_calibration/<date>/<zoom>x/images/
```

Recommended capture rules, adapted from the `mrcal` docs:

- keep the board mostly axis-aligned in the image; `mrgingham` expects clear
  horizontal and vertical corner sequences
- get deliberate coverage near all image borders
- keep the full board in frame for `mrgingham`; partial boards are discarded

Reference:

- `mrcal` calibration guide: https://mrcal.secretsauce.net/how-to-calibrate.html
- `mrcal` camera model docs: https://mrcal.secretsauce.net/cameramodels.html
- projection uncertainty docs: https://mrcal.secretsauce.net/uncertainty.html

## 3. Run Calibration Per Zoom Level

Example for `1x`:

```bash
src/scripts/calibration/run_mrcal_intrinsics.sh \
  --dataset-root datasets/camera_calibration/2026-04-15 \
  --zoom-level 1 \
  --object-spacing-m 0.03 \
  --object-width-n 10 \
  --focal-px 1100 \
  --lensmodel LENSMODEL_OPENCV8
```

The runner:

- checks for required executables
- reuses `analysis/corners.vnl` if already present
- writes the calibration log to `logs/mrcal-calibrate.log`
- writes the camera model to `calibration/`
- creates a `metrics.json` template if one does not yet exist

Optional arguments:

- `--image-glob '*.jpg'`
- `--observed-pixel-uncertainty 1.5`
- `--skip-calobject-warp-solve`
- `--extra-arg <arg>` repeated for any extra `mrcal-calibrate-cameras` flag

## 4. Residual and Uncertainty Review

Typical review commands:

```bash
mrcal-show-residuals \
  datasets/camera_calibration/2026-04-15/1x/calibration/camera-0.cameramodel

mrcal-show-projection-uncertainty \
  datasets/camera_calibration/2026-04-15/1x/calibration/camera-0.cameramodel
```

Record the accepted metrics in:

```text
datasets/camera_calibration/<date>/<zoom>x/metrics.json
```

The summary tooling expects at least:

```json
{
  "rms_reprojection_error_px": 0.32,
  "worst_reprojection_error_px": 1.21,
  "max_projection_uncertainty_px": 1.48,
  "parameter_stddev": {
    "fx": 4.8,
    "fy": 4.6,
    "cx": 1.1,
    "cy": 1.0,
    "k1": 0.003,
    "k2": 0.006,
    "p1": 0.0002,
    "p2": 0.0002,
    "k3": 0.008,
    "k4": null,
    "k5": null,
    "k6": null
  }
}
```

Notes:

- The `.cameramodel` file contains the solved intrinsics directly.
- The `metrics.json` sidecar carries the acceptance metrics and parameter 1 sigma
  values needed by ticket 028.
- The exact extraction workflow for per-parameter sigma can vary by installed
  `mrcal` version; the repo tooling intentionally keeps that version-sensitive
  step outside the parser.

## 5. Generate JSON and Aggregate CSV

After each calibrated zoom level has both a `.cameramodel` and `metrics.json`,
run:

```bash
python3 src/scripts/calibration/summarize_mrcal_intrinsics.py \
  --dataset-root datasets/camera_calibration/2026-04-15
```

Outputs:

- per zoom: `intrinsics_summary.json`
- session aggregate: `intrinsics_summary.csv`

## Current Gap

This slice sets up the reproducible workflow and the machine-readable outputs.
The actual bench capture, `mrcal` execution, and DR re-derivation still require:

- hardware access
- local `mrcal` installation `sudo apt install mrcal libmrcal-dev python3-mrcal`
- accepted `metrics.json` values from the bench session
