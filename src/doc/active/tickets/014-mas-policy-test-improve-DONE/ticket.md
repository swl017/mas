## Ticket #014: mas_policy — testing and hardening

### What
Add unit tests and fix fragile patterns in the mas_policy package. Currently zero test coverage.

### Why
Policy is the safety-critical decision layer — it produces velocity, gimbal, and zoom commands consumed by actuators. Silent failures (wrong obs_dim, mismatched checkpoint, angle wrapping spikes) can produce dangerous commands. Several fragile patterns were identified that would be caught by basic tests.

### Scope boundary
Tests + targeted code fixes in mas_policy only. Do not change the observation layout, action space, or policy architecture. Do not add integration/sim tests (future ticket).

### Affected modules
`mas_policy/`

### Tasks

#### A. Hardening fixes (code)

1. **obs_dim consistency assertion** — At startup, assert `assembler.obs_dim == self._obs_dim` before first forward pass. Currently computed independently in `policy_node.py` (formula) and `observation_assembler.py` (property), with no cross-check.

2. **Named constants for observation layout** — Replace magic numbers `30` (ego dims), `16` (inter-agent dims), `6` (triangulation tail) with named constants in one place.

3. **Gimbal rate angle wrapping** — `_gimbal_state_callback` finite-difference yaw rate has no `+-pi` wrapping. A 179 to -179 deg jump injects a ~2pi/dt spike. Add `wrap_to_pi` before differencing.

4. **Checkpoint loading validation** — `load_checkpoint` uses `strict=False` and only warns on missing keys. Add a post-load assertion that zero keys are missing (or allow a configurable allowlist). Fail loudly on architecture mismatch.

5. **Scaler dimension check** — Assert `scaler.running_mean.shape[0] == obs_dim` at load time, before first normalize call.

6. **CBF peer staleness guard** — Before building the CBF position/velocity arrays, skip peers whose odom is older than `stale_timeout`. Currently stale peers are treated as stationary at last known position.

7. **bbox_aoi clipping** — Clip `bbox_aoi` to a reasonable max (e.g., training episode length) so unbounded age doesn't produce out-of-distribution observations.

8. **yaw_joint_offset default** — CONTEXT.md says `-1.5708` but code says `+1.5708`. Consult to the `gimbal_controller` for how it handles the offsets. Fix CONTEXT.md or the code based on this.

#### B. Unit tests

| Priority | Test | What it validates |
|----------|------|-------------------|
| High | `test_obs_dim_consistency` | `assembler.obs_dim == expected` for all (num_agents, enable_tri) combos |
| High | `test_assemble_output_shape` | Assembled vector length matches `obs_dim` with mock cached state |
| High | `test_load_checkpoint_missing_keys` | Missing keys in checkpoint raise error, not silent warning |
| Medium | `test_gimbal_rate_wraparound` | `+-pi` angle jumps produce bounded rate |
| Medium | `test_cbf_filter_basic` | Agents closer than D_s get velocity corrected; far agents unmodified |
| Medium | `test_scaler_dimension_mismatch` | Wrong scaler dim raises at load time |
| Low | `test_action_publisher_scaling` | Published cmd_vel matches `action * max_lin_vel` |
| Low | `test_wrap_to_pi` | Pure math utility correctness |
| Low | `test_bbox_aoi_clipping` | Age-of-information stays bounded |

### Acceptance criteria
- [x] `python3 -m pytest src/mas_policy/test/ -v` passes with all 28 tests green
- [x] obs_dim mismatch between node and assembler is caught at startup (assertion added)
- [x] Checkpoint with wrong architecture fails loudly instead of silently loading zeros (RuntimeError on missing keys)
- [x] Scaler dimension validated at load time
- [x] Ego combined_ang_vel_w from subscription, not finite-differenced
- [x] yaw_joint_offset double-subtraction bug fixed (removed entirely)
- [x] bbox_aoi clipped to max_bbox_aoi parameter (default 20s)
- [x] CBF zeros stale peer velocity instead of using last known (most conservative)
- [x] GRU hidden state warmup verified over 50 ticks (no NaN/Inf)
- [x] CONTEXT.md updated

### Flow
Full QRISPY (Q → R → I → S → P → Y)

### Status
Done (2026-04-02)
