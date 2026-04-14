## Design Document: gimbal calibration script

### Problem statement

The current MAS stack can command and read SIYI gimbal state, but it does not have a bench script that turns those interfaces into measured encoder-validation and calibration data. Ticket 026 needs a single operator-facing workflow that verifies encoder behavior, performs a commanded sweep, and records enough data to estimate zero offsets and hysteresis before the results are fed back into sim randomization and controller assumptions.

This ticket is blocked on hardware validation from `mas/005-gimbal-encoder-hwtest`, so the design must separate code that can be implemented now from checks that require a live gimbal, camera, and checkerboard target.

### Proposed approach

Implement the calibration tool as a new executable in `gimbal_controller`, not as a change inside `siyi_gimbal_node`. The package already owns the SIYI SDK wrapper and the gimbal ROS conventions, so the script can reuse the existing dependency boundary without modifying runtime control behavior.

The script will be a standalone `rclpy` node that directly owns one SIYI SDK session for commanding and reading the gimbal during a bench session. It will also subscribe to the existing ROS topics needed for supporting measurements, specifically the gimbal state topic already published by `siyi_gimbal_node` conventions and the camera image / camera info topics needed for checkerboard-based angle estimation when available. This keeps the measurement workflow outside the flight-time node while still using the same message conventions and bagging workflow as the rest of the system.

The bench session is structured as three sequential phases inside one process. First, encoder verification performs small signed yaw and pitch commands and records the observed encoder response so the operator can confirm sign convention, continuity, and basic rate-controller compatibility. Second, the calibration sweep steps yaw and pitch through configurable angle grids, first in the forward direction and then in reverse, while logging commanded angle, raw encoder angle, and timestamps so hysteresis can be computed offline and summarized immediately after the run. Third, if image and camera calibration inputs are available, the script captures checkerboard observations at selected poses, solves camera pose against the board, converts that to camera-referenced gimbal angle estimates, and computes per-axis encoder zero-offset statistics.

All bench data is recorded to two outputs at once. A ROS 2 bag captures the time-series topics for later inspection, and a CSV summary captures one row per calibration sample with derived metrics needed by the ticket acceptance criteria. The script is responsible for deterministic session naming so bags and CSVs from a single run stay paired.

### Key interfaces and data flow

```
operator
  |
  v
gimbal_calibration.py
  |
  +-- SIYISDK session
  |     - command step angles / small verification motions
  |     - read raw attitude / encoder-facing state
  |
  +-- ROS subscriptions
  |     - camera/color/camera_info
  |     - image_raw or camera image topic
  |     - optional gimbal_state_rpy_deg mirror for comparison
  |
  +-- phase runner
  |     1. encoder verification
  |     2. forward / reverse sweep
  |     3. checkerboard offset estimation
  |
  +-- data recorder
        - publishes calibration sample topics for rosbag
        - writes per-sample CSV rows
        - writes end-of-run summary statistics
```

The script boundary is intentionally narrow:

- It reuses the existing SIYI SDK API for command and readback.
- It does not change `siyi_ros_node` publisher semantics or controller topics.
- It may publish dedicated calibration-status and calibration-sample topics solely to make rosbag capture structured and self-describing.
- It computes ticket outputs from the recorded samples instead of introducing calibration state into runtime nodes.

### What this does NOT include

- Any modification to `gimbal_controller/gimbal_controller/siyi_ros_node.py` behavior or SIYI SDK internals
- Dynamic-response or acceleration-compensation characterization from ticket 023 / flight data
- A promise to complete hardware validation in this session; hardware-dependent phases remain blocked until `mas/005` is confirmed
- Automatic application of measured offsets back into runtime control parameters

### Open risks

1. The current hardware path in `gimbal_controller` derives some joint values from 0x0D attitude plus aircraft IMU rather than 0x26 encoder stream which is not available anyways. The script must take this inaccuracy into account.
2. Checkerboard-based angle truth depends on already-correct camera intrinsics and a stable board mounting geometry. If those prerequisites are missing, the script should still complete verification and sweep phases and mark zero-offset estimation as skipped rather than fabricating results.
3. ROS 2 bag capture can be managed internally or by an external `ros2 bag record` process. Internal orchestration is more convenient for operators, but it adds process-management complexity to the script. But it closes the loop tightly automated, so it should be implemented.
