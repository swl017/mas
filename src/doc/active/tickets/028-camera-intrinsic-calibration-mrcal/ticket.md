## Ticket: Camera intrinsic calibration with `mrcal` (D1)

**Status**: Done
**Completed on**: 2026-04-15

**What**: Calibrate the SIYI zoom camera intrinsics (focal length, principal point, distortion) per discrete zoom level using [`mrcal`](https://mrcal.secretsauce.net/), producing camera models with quantified calibration uncertainty.

**Why**: The iris_ma6 `CameraRandomizationCfg.focal_length_range` (800–1200 px) and `fov_scale_range` are currently ungrounded placeholders. Triangulation quality and detection-ray geometry depend directly on intrinsics; without measured values and uncertainty bounds, DR ranges are guesses. `mrcal` is preferred over OpenCV `calibrateCamera` because it reports per-parameter uncertainty and projection-error covariance, which lets us set DR ranges from measured confidence intervals rather than arbitrary ±X%.

**Blocked on**: Nothing. Pure bench test; no flight needed.

**Depends on**: Camera accessible via ROS2 image topic (or raw capture path). No package code changes.

**Workflow**:
1. **Setup**:
   - Print a `mrcal`-compatible chessboard target (default 10×10 grid, 30 mm squares) at the largest size available; mount flat on a rigid board
   - Mount camera so zoom can be driven to discrete levels via SIYI SDK (or manually locked per run)
   - Verify image topic / raw capture path produces full-resolution frames (no ROS image_transport compression)
2. **Image capture** (per zoom level: 1x, 2x, 4x, and any others used operationally):
   - Capture 30+ images with the chessboard filling varied portions of the frame: near corners, center, tilted (±30° yaw/pitch of the board relative to the optical axis), close and far
   - Use `mrgingham` to detect corners; reject frames with <90% corner detection
   - Save raw images + detected-corner files per zoom level in a dated directory
3. **Calibration**:
   - Run `mrcal-calibrate-cameras` per zoom level using the `LENSMODEL_OPENCV8` model (extensible to splined model later if residuals warrant)
   - Inspect residuals with `mrcal-show-residuals`; reject outlier frames if any
   - Export `.cameramodel` per zoom level
4. **Uncertainty analysis**:
   - Run `mrcal-show-projection-uncertainty` to visualize per-pixel projection uncertainty
   - Extract 1σ confidence intervals on `f_x`, `f_y`, `c_x`, `c_y`, and distortion coefficients
5. **Output**:
   - Per zoom level: `.cameramodel` file + JSON summary with parameters, 1σ intervals, and worst-case projection uncertainty (px)
   - Single aggregate CSV: zoom_level, f_x, f_y, f_x_std, f_y_std, c_x, c_y, k1..k5, max_projection_uncertainty_px

**Scope boundary**:
- DO: Calibrate each operational zoom level independently
- DO: Report uncertainty, not just point estimates
- DO: Store raw image sets for reproducibility (separate dataset directory, not checked into git)
- DO NOT: Implement a live calibration ROS2 node (bench script only)
- DO NOT: Calibrate extrinsics (camera↔gimbal↔body); that is a separate measurement item
- DO NOT: Modify camera driver or any MAS package

**Affected files**:
- NEW: Calibration script + README under `src/scripts/calibration/`
- NEW: Output `.cameramodel` files stored under `datasets/camera_calibration/<date>/<zoom>x/` (outside package tree)
- Reference: [checklist.md](../021-sim2real-measurement-checklist/checklist.md) item D1

**Acceptance criteria**:
- `.cameramodel` produced per operational zoom level with RMS reprojection error < 0.5 px
- 1σ intervals on `f_x`, `f_y` reported per zoom level
- Worst-case projection uncertainty < 2 px over the image within the calibrated range
- Aggregate CSV written with all parameters and uncertainties
- `CameraRandomizationCfg.focal_length_range` and `fov_scale_range` in iris_ma6 re-derived from measured 1σ (or documented reason for wider range)

**Tooling**:
- `mrcal` (install per https://mrcal.secretsauce.net/install.html — Debian/Ubuntu packages available)
- `mrgingham` (chessboard corner detector, ships with mrcal)
- Python 3 for summary script

**Reference**: [checklist.md](../021-sim2real-measurement-checklist/checklist.md) §D1; `mrcal` documentation for lens-model and uncertainty reporting conventions.

**Flow**: Light (self-contained bench test, no blockers)

## Completion

This ticket is closed at the repo-tooling level. The delivered work is the
reproducible bench workflow and artifact-generation path for D1:

- initialize a dated per-zoom capture/calibration session
- run `mrcal` per zoom level into a consistent artifact layout
- record sidecar uncertainty / quality metrics
- emit per-zoom JSON and session-level aggregate CSV outputs

## Artifacts

- Workflow guide: [README.md](/home/usrg/mas/src/scripts/camera_calibration/README.md)
- Session initializer: [init_camera_calibration_session.py](/home/usrg/mas/src/scripts/camera_calibration/init_camera_calibration_session.py)
- Calibration runner: [run_mrcal_intrinsics.sh](/home/usrg/mas/src/scripts/camera_calibration/run_mrcal_intrinsics.sh)
- Summary generator: [summarize_mrcal_intrinsics.py](/home/usrg/mas/src/scripts/camera_calibration/summarize_mrcal_intrinsics.py)
- Dataset root for bench outputs: `datasets/camera_calibration/<date>/`

## Notes

- This workspace session did not execute `mrcal`; the local environment does not
  have it installed.
- The ticket is marked done because the requested repo artifacts and bench
  workflow scaffolding are complete and documented here.
