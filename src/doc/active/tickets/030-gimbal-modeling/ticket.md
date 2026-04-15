## Ticket: Gimbal rate-command response modeling

**What**: Extend the SIYI ROS node and the gimbal calibration bench test to capture
rate-command step response data, then fit a dynamics model of the gimbal's
closed-loop response to normalized rate commands (0x07).

**Why**: Existing bench data (ticket 026, [step_trajectories.png](../../../../gimbal_controller/scripts/gimbal_calibration/plots/step_trajectories.png))
characterizes the gimbal's response to **angle** setpoints (0x0E), which include
the SIYI internal position controller. The runtime pointing path
([point_to_region_node](../../../../gimbal_controller/gimbal_controller/point_to_region_node.py))
and LOS-rate stabilization instead emit **rate** commands (`gimbal_cmd_los_rate`, 0x07),
for which no measured dynamics model exists. Without this, sim (iris_ma6) gimbal
randomization ranges and control-loop tuning are ungrounded.

**Scope**:

### Part 1 — Improve SIYI ROS node

The SDK already exposes angular-rate feedback via `getAttitudeSpeed()`
([siyi_sdk.py:971](../../../../gimbal_controller/gimbal_controller/siyi_sdk/siyi_sdk.py#L971),
parsed from 0x0D at [:800-802](../../../../gimbal_controller/gimbal_controller/siyi_sdk/siyi_sdk.py#L800-L802)),
but [siyi_ros_node.py](../../../../gimbal_controller/gimbal_controller/siyi_ros_node.py)
does not publish it. Current consumers rely on finite-differenced angles, which
are noisy (visible in rate panels of step_trajectories.png).

- DO: Publish SDK-reported angular rate as a new topic (e.g.
  `siyi_gimbal_angles/state_rate_rpy_deg`, `geometry_msgs/Vector3`) at the
  100 Hz state loop cadence, heading-frame, with the same direction multipliers
  applied as the angle topic.
- DO: Echo last-commanded rate on a debug topic (e.g.
  `siyi_gimbal_angles/cmd_rate_rpy_norm`) so bag-based identification can align
  command and response without relying on subscriber-side timestamps.
- DO NOT: Change existing topic contracts or the 0x07/0x0E command paths.

### Part 2 — Improve gimbal calibration test

Add a **rate-step phase** to [gimbal_calibration.py](../../../../gimbal_controller/scripts/gimbal_calibration.py),
analogous to the existing `phase_step_response` but publishing to
`gimbal_cmd_los_rate` instead of `command_rpy_deg`.

- DO: Sweep normalized rate magnitudes per axis (e.g. ±10, ±25, ±50, ±75, ±100),
  each held for a fixed duration from a known starting angle, with a return-home
  between steps. Starting angle chosen to stay within mechanical rails given
  the max possible travel.
- DO: Log commanded rate, SDK-reported angular rate, and encoder angle into a
  `rate_step_trace.csv` + `rate_step_index.csv`, mirroring the existing step
  artifacts.
- DO: Record a scoped ros2 bag `bag_rate_step/` covering the rate-step phase
  only, reusing `_start_scoped_bag` ([gimbal_calibration.py:358](../../../../gimbal_controller/scripts/gimbal_calibration.py#L358))
  with a new `RATE_STEP_BAG_TOPICS` list. Topics must include the new
  angular-rate state topic, the new rate-command echo topic, the existing
  angle state/encoder topics, and `mavros/imu/data` for aircraft motion
  context. The whole-session `--record-bag` path and the existing angle-step
  `bag_step/` recorder both remain unchanged.
- DO: Per-step metrics — steady-state rate (mean over last N samples), rise
  time to 90% of steady state, command-to-first-motion latency, and peak rate.
  Written to `rate_step_summary.csv`.
- DO: Plot rate trajectories and a **normalized-command → steady-state deg/s
  mapping** curve (this is the calibration of the 0x07 unit).
- DO NOT: Modify the existing angle-step phase.

### Part 3 — Fit dynamics model

From `rate_step_summary.csv`, fit per-axis parameters for a rate-command model.
Candidate form:
```
θ̈ = (k · u_norm − θ̇) / τ_rate
```
where `u_norm ∈ [-1, 1]` is the normalized command, `k` is the steady-state
gain (deg/s per unit command, from the mapping curve), and `τ_rate` is the
rate-loop time constant. Revisit model order if the data shows clear
second-order behavior (overshoot, oscillation).

- DO: Write fit parameters to `datasets/gimbal_calibration/<session>/rate_model.json`
- DO: Record whether the k-curve is linear or saturating; note deadband if any.

**Affected files**:
- MOD: [src/gimbal_controller/gimbal_controller/siyi_ros_node.py](../../../../gimbal_controller/gimbal_controller/siyi_ros_node.py) — add rate publisher, cmd-echo publisher
- MOD: [src/gimbal_controller/CONTEXT.md](../../../../gimbal_controller/CONTEXT.md) — document new topics
- MOD: [src/gimbal_controller/scripts/gimbal_calibration.py](../../../../gimbal_controller/scripts/gimbal_calibration.py) — add `phase_rate_step_response`, CLI flag, metrics fitter
- NEW: `RATE_STEP_BAG_TOPICS` list + `phase_rate_step_response` in [gimbal_calibration.py](../../../../gimbal_controller/scripts/gimbal_calibration.py)
- NEW: Plot script additions in the calibration plotting path for rate traces and k-curve
- NEW: `datasets/gimbal_calibration/<session>/bag_rate_step/` (scoped bag), `rate_step_trace.csv`, `rate_step_index.csv`, `rate_step_summary.csv`, `rate_model.json`

**Acceptance criteria**:
- SIYI node publishes a heading-frame angular-rate topic backed by SDK 0x0D at 100 Hz; verified against finite-differenced angle on bench.
- Calibration script has a `--phase rate_step` mode that runs the rate sweep and emits the new artifacts.
- `rate_model.json` contains per-axis `k` (deg/s per unit), `τ_rate`, and deadband; covers at least yaw and pitch.
- Plot shows rate trajectories overlaid per magnitude, and a normalized-command → deg/s curve.

**Depends on**: ticket 026 calibration infrastructure (already merged).

**Flow**: Medium (hardware-in-the-loop, two-package change).
