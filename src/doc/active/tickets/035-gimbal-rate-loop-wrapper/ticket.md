## Ticket: Gimbal rate-loop wrapper with LOS-stabilization bypass (iris_ma6)

**Status**: Open
**Created**: 2026-04-28

**What**: Replace the coarse joint-PD `c/k = 0.10 s` match shipped in mas/029 with an explicit first-order **rate loop** that lives between the policy-emitted gimbal yaw/pitch rate command and the joint position target. The rate loop models the SIYI A8 mini's measured rate-command dynamics: τ ≈ 0.10 s, saturation at ±73 deg/s, no deadband. Once the rate loop owns the dynamics, the joint PD reverts to stiff position tracking.

**CRITICAL ARCHITECTURE INVARIANT**: The rate loop applies **only** to the policy-emitted user-LOS-rate command. The body-motion compensation path (LOS stabilization — converting a world-frame pointing target to joint angles using the current body orientation) MUST remain instantaneous and bypass the rate loop entirely. On the real SIYI camera the inner stabilization loop runs at a much higher bandwidth than the user-command path; failing to preserve this split in sim collapses the whole point of having a stabilized gimbal.

**Why**: mas/029's c/k = 0.10 s is a single-number match against the open-loop PD time constant — it does not reproduce the saturation nonlinearity that mas/026 documented (τ scaling with step magnitude on real hardware), nor does it model the rate-loop architecture the deployed ROS2 controller actually uses. The existing sim-to-sim diagnostic ([compare_gimbal.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/sysid_output/gimbal/compare_gimbal.py)) already shows the gap; closing it is the load-bearing fix the user has been gating graduation on.

**Blocked on**: Nothing. Inputs (rate_model.json, ros2_los_math.py port) are already in tree.

**Depends on**: mas/026 (rate-step measurements). Scopes the architecture so mas/036 (command dead-time) can drop in as one extension at the rate-loop **input** boundary.

### Truth values (already fitted)

From [rate_model.json](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_model.json):

| Axis | k_deg_s_per_u | τ_rate (s) | dead time mean (s) |
|---|---:|---:|---:|
| yaw | 73.31 | 0.0995 | ~0.07 (mas/036) |
| pitch | 73.40 | 0.0954 | ~0.07 (mas/036) |

### Architecture

```
                    ┌─── policy command path (subject to rate-loop dynamics) ──┐
                    │                                                          │
policy_action[4:6] ──▶ scale by max_gimbal_rate ──▶ GimbalRateLoop.step ──▶ ω_user_actual ──┐
                                                    (dead time, lag,           │            │
                                                     saturation)                            ▼
                                                                                     ω_world_desired
                                                                                            │
                                                                              integrate via dt
                                                                                            ▼
                                                                              (az_target, el_target)
                                                                                  in WORLD frame
                                                                                            │
                                                                                            ▼
                              ┌─── LOS stabilization path (instant, bypasses rate loop) ──┐
                              │                                                            │
body_orientation_w (NOW) ──▶ analytical IK / Jacobian (az, el, body_quat_NOW) ──▶ joint_pos_target
                              │                                                            │
                              └────────────────────────────────────────────────────────────┘
                                                                                            │
                                                                                            ▼
                                                                            set_joint_position_target
                                                                                  (asset PD tracks
                                                                                  position stiffly)
```

The world-frame target `(az_target, el_target)` is updated by `ω_user_actual` (slow) but the conversion to joint angles always uses the **current** body orientation (fast). Body roll/pitch/yaw enter the joint command this same step with no delay — that's the inner LOS-stabilization loop staying instant.

### Workflow

1. **Profile the deployed controller** at [controller/sysid_output/gimbal/ros2_los_math.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/sysid_output/gimbal/ros2_los_math.py) and the diagnostic notes at [doc/sim-to-sim/gimbal_diagnostic.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/doc/sim-to-sim/gimbal_diagnostic.md). Confirm the deployed loop is pure first-order (rate command → first-order lag) rather than P+I, that saturation is hard-clip at `k * 1.0`, and that the loop runs per-axis (yaw and pitch independent at the rate-loop level — coupling enters through the IK that comes after).
2. **Add `GimbalRateLoopCfg`** under [controller/gimbal_rate_loop_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop_cfg.py): `tau_yaw=0.0995, tau_pitch=0.0954, max_rate_per_axis=1.28 rad/s, deadband=0.0, dead_time_steps=0` (placeholder for mas/036).
3. **Implement `GimbalRateLoop`** under [controller/gimbal_rate_loop.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop.py):
   ```python
   class GimbalRateLoop:
       def __init__(self, num_envs: int, device, cfg: GimbalRateLoopCfg)
       def step(self, omega_cmd: torch.Tensor, dt: float) -> torch.Tensor:
           """omega_cmd: [N, 2] (yaw_rate, pitch_rate) in rad/s.
              Saturates, applies first-order lag with τ_axis from cfg.
              Returns omega_actual: [N, 2]."""
       def reset(self, env_ids: torch.Tensor | None = None)
   ```
   Saturation first (clip to `±max_rate_per_axis`), then lag: `ω_dot = (ω_cmd_clipped - ω_actual) / τ`, exact discretization `α = 1 - exp(-dt / τ)`.
4. **Insert into the gimbal controller path** in [iris_ma_env6_test.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test.py): the policy emits `gimbal_yaw_rate_cmd` / `gimbal_pitch_rate_cmd` (action[4], [5]). Currently these are scaled by `max_gimbal_rate` and integrated to `(az_target, el_target)`. The new pipeline:
   - `omega_user_cmd = action[4:6] * max_gimbal_rate`  (in rad/s, body-frame world-yaw rate)
   - `omega_user_actual = rate_loop.step(omega_user_cmd, dt)`
   - integrate `(az_target, el_target) += omega_user_actual * dt`
   - **no change** below the integration step — `(az_target, el_target)` → joint angles via the existing IK that uses the current `body_quat` (this is the LOS-stabilization path, instant)
5. **Revert the joint-PD coarse fit** in [iris_gimbal3.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py) for the active gimbal axes. With the rate loop owning the dynamics, the joint PD should be stiff-and-fast position tracking (e.g. `stiffness=2e3, damping=1e2`, the iris_gimbal2 values) so the joint follows `set_joint_position_target` without adding a second time constant on top of the rate loop. Update the test in [tests/test_measured_model_defaults.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/test_measured_model_defaults.py) to assert rate-loop τ instead of c/k.
6. **Validate against bench data** with a new test [controller/tests/test_gimbal_rate_loop.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_gimbal_rate_loop.py): for each `u ∈ {0.1, 0.25, 0.5, 0.75, 1.0}` step input, run the rate loop and compare `(rise_time_s, w_ss_deg_s, w_peak_deg_s)` against [rate_step_summary.csv](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_step_summary.csv).
7. **Sim-to-sim regression**: re-run [controller/sysid_output/gimbal/compare_gimbal.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/sysid_output/gimbal/compare_gimbal.py) with the rate loop enabled. The joint-frame Δ between IL `jacobian` mode and the deployed ROS2 controller should drop to ≤ 1° across all six diagnostic scenarios (the bar set in [progress.txt](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/doc/active/progress.txt) for graduation).

### Scope boundary

- DO: Apply the rate loop **only** to the user-LOS-rate command path.
- DO: Preserve the existing `(az, el) → joint` IK as instantaneous body-motion compensation. Body roll/pitch entering the IK at this same step is the LOS stabilization staying fast.
- DO: Revert joint PD to stiff position tracking once dynamics live in the rate loop — do not double-count.
- DO: Match the measured τ and saturation per-axis from `rate_model.json`.
- DO NOT: Apply the rate loop to the body-motion compensation. (Scenario test: at large body roll rate, sim camera should track world-frame target as well as the real SIYI does, even when the policy is sending zero rate command.)
- DO NOT: Add I-term unless step 1 confirms the deployed loop has one (it doesn't — the math port is pure first-order).
- DO NOT: Touch `max_gimbal_rate` (already set in mas/029) — it stays as the user-command saturation; the rate loop respects this same limit internally.
- DEFERRED: Command dead-time (mas/036 — folds in as `dead_time_steps > 0` at the rate-loop input).

### Affected files

- NEW: [controller/gimbal_rate_loop.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop.py)
- NEW: [controller/gimbal_rate_loop_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_rate_loop_cfg.py)
- EDIT: [iris_ma_env6_test.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test.py) — insert rate loop on the `omega_user_cmd` path, leave the IK below it unchanged
- EDIT: [iris_gimbal3.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py) — revert stiffness/damping to stiff position-tracking values (≈ 2e3 / 1e2)
- EDIT: [tests/test_measured_model_defaults.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/test_measured_model_defaults.py) — replace c/k assertion with rate-loop τ assertion
- NEW: [controller/tests/test_gimbal_rate_loop.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_gimbal_rate_loop.py) — bench-trace match
- NEW: [controller/tests/test_los_stabilization_bypass.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/tests/test_los_stabilization_bypass.py) — explicit invariant test (see Acceptance)
- DOC: provenance row in [domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md) updated with the rate-loop replacement.

### Acceptance criteria

- **Bench-trace match**: across all amplitudes in `rate_step_summary.csv`, |τ_sim − τ_measured| / τ_measured ≤ 10 % and |w_ss_sim − w_ss_measured| / w_ss_measured ≤ 5 % per axis. Saturation at `u = ±1.0` lands within 5 % of ±73 deg/s.
- **LOS-stabilization-bypass invariant** (the load-bearing test for this ticket): policy emits zero rate command (`action[4:6] = 0`); body is rotated about its yaw axis at 60 deg/s for 0.5 s; the camera's world-frame azimuth must stay within ±2° of its starting value at every step. If the rate loop is incorrectly inserted on the body-compensation path, this test fails by lagging up to ~30°.
- **Sim-to-sim regression**: [compare_gimbal.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/sysid_output/gimbal/compare_gimbal.py) joint-frame Δ ≤ 1° across all six scenarios.
- All new/edited tests pass on CPU and GPU.
- mas/029's measured-model-defaults test still passes after the c/k → τ assertion swap.

### Risk

**Medium-high**. Inserts a new dynamic element on the gimbal's inner control path. Currently-trained checkpoints will see a domain shift at action→state level and need retraining. The user's progress notes already plan a retrain after the controller is reconciled, so the timing is right; coordinate before merge.

### Coupling

- mas/034: independent.
- mas/036: this ticket scopes the rate loop with `dead_time_steps=0`; mas/036 lifts that to a per-env distribution. Land mas/035 first.

**Reference**: mas/026 (rate-step measurements), mas/029 ticket.md decision #3 (rate-loop wrapper preferred over equivalent joint PD), [progress.txt](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/doc/active/progress.txt) "Reframe (PhD timeline)" entry.

**Flow**: Heavy (architectural change on the gimbal control path; needs careful sim-to-sim regression).
