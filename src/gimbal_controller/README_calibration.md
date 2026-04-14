# Gimbal Calibration Bench Workflow

This document describes the ticket 026 bench workflow implemented inside the
`gimbal_controller` package.

## Scope

The workflow covers:

- encoder sign and continuity verification
- forward and reverse yaw / pitch sweeps
- optional checkerboard-based zero-offset estimation
- paired rosbag, CSV, and summary JSON outputs

The workflow does not modify runtime gimbal-controller behavior. It is a bench
tool only.

## Session Layout

Each session lives under:

```text
datasets/gimbal_calibration/<session_name>/
├── bag/
├── notes.md
├── samples.csv
├── session_manifest.json
└── summary.json
```

## 1. Initialize a Session

Example:

```bash
python3 src/gimbal_controller/scripts/init_gimbal_calibration_session.py \
  --date 2026-04-15 \
  --session-name a8mini_bench
```

## 2. Run the Calibration Bench Script

Example:

```bash
ros2 run gimbal_controller gimbal_calibration \
  --ros-args \
  -p session_name:=2026-04-15_a8mini_bench \
  -p image_topic:=/camera/image_raw \
  -p camera_info_topic:=/camera/camera_info \
  -p gimbal_state_topic:=/gimbal_state_rpy_deg
```

Relevant parameters:

- `dataset_root`
- `session_name`
- `server_ip`
- `server_port`
- `yaw_min_deg`, `yaw_max_deg`
- `pitch_min_deg`, `pitch_max_deg`
- `step_deg`
- `settle_time_sec`
- `verification_step_deg`
- `verification_hold_sec`
- `enable_rosbag`
- `enable_checkerboard`
- `checkerboard_rows`
- `checkerboard_cols`
- `checkerboard_square_size_m`

## 3. Rebuild the Summary

If you edit or inspect `samples.csv` offline, regenerate the summary with:

```bash
python3 src/gimbal_controller/scripts/summarize_gimbal_calibration.py \
  --csv-path datasets/gimbal_calibration/2026-04-15_a8mini_bench/samples.csv
```

## Notes

- The runner attempts a one-shot encoder read via the SIYI SDK and falls back
  to attitude readback if the encoder stream is unavailable on the hardware.
- Checkerboard zero-offset estimation is optional. If OpenCV, camera info, image
  input, or checkerboard detection is unavailable, the sweep still completes and
  the zero-offset section remains empty.
- `ros2 bag record` is started internally when `enable_rosbag:=true`.
