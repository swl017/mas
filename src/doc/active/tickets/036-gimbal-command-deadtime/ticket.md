## Ticket: Gimbal command-to-first-move dead time (iris_ma6)

**Status**: Open
**Created**: 2026-04-28

**What**: Model the measured ~70 ms (40–100 ms range) dead time between a commanded rate change at the policy interface and the first observable motion at the gimbal motors. Implement as an integer-step FIFO buffer at the **input** of the rate loop introduced in mas/035, sampled per episode. Add a curriculum knob so training can start at zero delay and ramp toward the measured distribution.

**CRITICAL ARCHITECTURE INVARIANT** (inherited from mas/035): the dead-time buffer applies **only** to the user-LOS-rate command path. The body-motion compensation path (LOS stabilization — converting world-frame target to joint angles using current body orientation) is NOT subject to this delay. Body roll/pitch entering the IK at this same step continues to be reflected in the joint command instantly. If the buffer is incorrectly placed on the IK path, the gimbal will lag during body maneuvers exactly the wrong way.

**Why**: mas/026's rate-step bench data shows a consistent dead time of 40–100 ms between issuing a step command and seeing the motor move. This is real hardware behavior — packet transit, microcontroller polling, motor commutation startup — and the policy needs exposure to it to be sim2real-robust. mas/029 explicitly deferred this to a follow-up; mas/035 left a `dead_time_steps` placeholder at the rate-loop input specifically to drop this in.

**Blocked on**: mas/035 (rate loop must exist; this ticket is incremental on top of it).

**Depends on**: mas/035 architecture (in particular the rate-loop input boundary). No new external artifacts.

### Truth values (already measured)

`latency_s` column in [rate_step_summary.csv](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_step_summary.csv): per-step time from `t_cmd_ros` to first measurable motion. Ranges 40–100 ms across yaw and pitch, all amplitudes. Empirical distribution is roughly Gaussian; mean ≈ 70 ms, std ≈ 20 ms (re-fit from the CSV during workflow step 1).

### Workflow

1. **Re-fit** the `latency_s` column to a single Gaussian and emit a small JSON next to the existing aggregator outputs at `src/scripts/sim2real_model_fitting/output/gimbal_dead_time_fit.json` so the sim-side default has a stable provenance pointer. (The other rate-loop parameters already live in `rate_model.json`; this column was not consumed there.)
2. **Extend `GimbalRateLoopCfg`** (introduced in mas/035) with:
   ```python
   dead_time_mean_s: float = 0.07   # 70 ms — mas/036 fit
   dead_time_std_s: float  = 0.02   # 20 ms
   dead_time_max_s: float  = 0.15   # hard cap on buffer depth
   curriculum_scale: float = 0.0    # 0 = no delay, 1 = full measured
   ```
3. **Extend `GimbalRateLoop`** (mas/035) with a per-env circular buffer over the user-command tensor:
   - At reset, draw `dead_time_steps[i] = round(N(mean, std) * curriculum_scale / dt)`, clipped to `[0, max_buffer_steps]` where `max_buffer_steps = ceil(dead_time_max_s / dt)`.
   - At each `step(omega_cmd, dt)`: push `omega_cmd[i]` onto env i's buffer; pop the entry from `dead_time_steps[i]` ago (or the oldest available); feed the popped value into the existing first-order lag stage. New envs (buffer not full yet) should hold zero rate, matching real cold-start behavior.
4. **Curriculum hook**: register `gimbal_dead_time_curriculum_scale` in the existing curriculum manager, ramping `0 → 1` over `curriculum_steps_to_full_dead_time = 5e6` (matches the cadence in mas/034). Initial value 0 means existing-ticket-035 behavior is preserved bit-exactly when the curriculum hasn't advanced.
5. **Tests** under [controller/tests/](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/):
   - `test_gimbal_dead_time_buffer.py`: feed a step command at t=0 with `dead_time_mean_s=0.07, std=0`. Sim `omega_actual` must be zero for `≥ round(0.07 / dt) − 1` steps, then begin tracking. Asserts both lower bound (no motion before delay) and upper bound (motion starts no later than `round(...) + 1`).
   - `test_gimbal_dead_time_distribution.py`: 10k env-resets at full curriculum scale; empirical mean of per-env dead-time matches measured 70 ms within ±5 ms; distribution covers 40–100 ms.
   - `test_los_stabilization_unaffected.py` (extends mas/035's bypass test): with `dead_time_mean_s = 0.10`, body rotated at 60 deg/s for 0.5 s, policy zero-rate — camera world-frame azimuth still within ±2° of starting value. Failing this test means the buffer is placed on the IK path instead of the rate-loop input.
6. **Doc** the new default and curriculum hook in [domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md).

### Scope boundary

- DO: Add the dead-time buffer at the **input** of the rate loop (mas/035 boundary).
- DO: Sample per-env dead time at episode reset; keep it constant within an episode (matches hardware — the dead time is a structural property of the firmware/motor, not a per-step random variable).
- DO: Default `curriculum_scale = 0` so this ticket merging does not change behavior until the curriculum advances.
- DO: Cover the buffer-cold-start case (initial outputs are zero) — mirrors real hardware.
- DO NOT: Place the buffer downstream of the rate-loop lag stage (would conflate dead time with τ).
- DO NOT: Place the buffer on the body-motion-compensation / IK path. (See `test_los_stabilization_unaffected.py`.)
- DO NOT: Resample dead time per step within an episode.
- DO NOT: Add separate buffers for stabilization vs user paths — the stabilization path already bypasses the rate loop entirely (mas/035), so it inherits no buffer.

### Affected files

- NEW: [src/scripts/sim2real_model_fitting/fit_gimbal_dead_time.py](/home/usrg/mas/src/scripts/sim2real_model_fitting/fit_gimbal_dead_time.py)
- NEW: `src/scripts/sim2real_model_fitting/output/gimbal_dead_time_fit.json`
- EDIT: [controller/gimbal_rate_loop_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop_cfg.py) (introduced in mas/035) — add four dead-time fields.
- EDIT: [controller/gimbal_rate_loop.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop.py) (introduced in mas/035) — add per-env circular buffer.
- EDIT: curriculum manager — register `gimbal_dead_time_curriculum_scale`.
- NEW: [controller/tests/test_gimbal_dead_time_buffer.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_gimbal_dead_time_buffer.py)
- NEW: [controller/tests/test_gimbal_dead_time_distribution.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_gimbal_dead_time_distribution.py)
- NEW: [controller/tests/test_los_stabilization_unaffected.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_los_stabilization_unaffected.py)
- DOC: append to [domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md).

### Acceptance criteria

- Step-input dead-time test: at `dead_time_mean_s = 0.07, std = 0`, `omega_actual` is exactly zero for `round(0.07 / dt) − 1` consecutive sim steps and nonzero by step `round(0.07 / dt) + 1`.
- Distribution test (10k env-resets at `curriculum_scale = 1.0`): empirical mean within ±5 ms of measured 70 ms; range covers 40–100 ms.
- LOS-stabilization-unaffected test passes (body rotated at 60 deg/s, policy zero rate, camera world-frame azimuth stays within ±2° regardless of `dead_time_mean_s`).
- mas/035's rate-loop bench-trace test still passes when `curriculum_scale = 0` (regression).
- Sim-to-sim regression on [compare_gimbal.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/sysid_output/gimbal/compare_gimbal.py): with `curriculum_scale = 1.0`, deviation vs deployed ROS2 stays ≤ 1° in all six scenarios (this is a tighter check that the dead-time path matches reality, not just that it doesn't break things).

### Risk

Low (incremental on mas/035). The cold-start zero-output behavior of the buffer is the only subtle bit; the test in step 5 covers it.

### Coupling

- mas/034: independent.
- mas/035: required prerequisite. Do not merge mas/036 ahead of mas/035.

**Reference**: mas/026 (rate-step measurements, latency_s column), mas/029 ticket.md "Deferred to follow-ups" entry, mas/035 (rate-loop architecture).

**Flow**: Light (incremental on mas/035, well-scoped tests).
