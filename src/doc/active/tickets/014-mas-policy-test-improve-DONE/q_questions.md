## Open Questions — Ticket #014

### Assumptions requiring confirmation

1. **Test framework** — pytest + colcon test, unittest/colcon, or standalone pytest?
2. **Scope of testing** — unit tests only, or also lightweight integration smoke tests?
3. **Checkpoint validation strictness** — fail on missing policy keys only, or require exact match? Value network loading?
4. **bbox_aoi clipping value** — hardcoded or parameter? Source for the value?
5. **CBF staleness behavior** — skip stale peers entirely, or treat as worst-case obstacle?
6. **Gimbal rate computation** — fix finite-diff wrapping, or switch to subscribing the rate topic? What do the gimbal-related observation values mean exactly?
7. **Named constants location** — where to put `30`, `16`, `6` constants? What is the authoritative observation layout?
8. **yaw_joint_offset** — which value is correct, `-1.5708` or `+1.5708`?

### Answers (from engineer)

1. **pytest + colcon test** — matches existing workspace convention (gimbal_controller, vision_opencv all use it). Add `tests_require=['pytest']` to setup.py, `[tool:pytest]` stanza to setup.cfg. Pure unit tests mock rclpy via `sys.modules`; integration smoke tests use real `rclpy.init()`.

2. **Yes, add integration smoke tests** — spin up policy_node with mock checkpoint, verify no crash.

3. **Load value network too** — the value network could evaluate state distributions and performance. Validate that all expected keys (policy, value, state_preprocessor) are present. SKRL checkpoints contain per-agent dicts with `policy`, `value`, `optimizer`, `state_preprocessor`, `value_preprocessor` keys.

4. **Parameter, loaded from training config** — training episode is 20.0s / 500 steps (from `env.yaml` in model dir: `episode_length_s: 20.0`, `decimation: 4`, `sim.dt: 0.01`). Model dir example: `/home/usrg/mas/src/mas_policy/models/2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning/`. Prefer reading from the training run's config rather than hardcoding.

5. **Treat stale peers as worst-case obstacle** — expand safety margin rather than skipping. Skipping means no avoidance at all.

6. **Subscribe, don't diff** — gimbal angular velocity should come from a subscription (like `combined_ang_vel_w` for peers), not finite-differenced in policy. Wrapping at +-180 deg where still needed. Key semantic distinctions for gimbal observations:
   - `gimbal_yaw_body` / `gimbal_pitch_body`: raw joint angles in body-frame (0=forward), used as ego obs [25:27]
   - `combined_ang_vel_w`: world-frame camera sweep rate (body angular vel + gimbal rate), ego obs [27:30] and peer obs
   - `ego_ray_w` / `other_ray_w`: world-frame ray from camera origin to target through bbox center (NOT a LOS direction, NOT a gimbal angle)
   - `yaw_joint_offset`: sim mesh offset (`-pi/2`), subtracted from raw joint to get body-frame angle

7. **Authoritative layout is the training env** — `iris_ma_env6_test_cfg.py` lines 547-569 and `iris_ma_env6_test.py` lines 1517-1573 define the ground truth observation layout. `num_agents` is the parameter; `30 + 16*(num_agents-1)` (+6 optional triangulation). Constants should be derived from this, not independently hardcoded.

8. **Bug found via cross-check.** The offset chain:
   - **Sim mesh**: `yaw_joint=0` points camera along body `-Y` (not forward)
   - **los_rate_controller** (sim): defines `YAW_JOINT_OFFSET = +pi/2`. Publishes `gimbal_state_rpy_deg` with offset **already subtracted** (line 440: `joint - offset`, line 448-449: publishes in degrees). So the topic value is body-frame with yaw=0 → forward.
   - **siyi_ros_node** (real): publishes encoder angles directly — also body-frame with yaw=0 → forward (no offset needed for real hardware).
   - **Training env** (`iris_ma_env6_test.py`): reads raw `joint_positions_b` directly, so it needs `joint - YAW_JOINT_OFFSET` to get body-frame (line 1509).
   - **Deployment** (`observation_assembler.py` line 399): subscribes to `gimbal_state_rpy_deg` (already offset-corrected), then does `gimbal_yaw_body - yaw_joint_offset` with default `+pi/2`. **This double-subtracts the offset.**
   - **Fix**: `yaw_joint_offset` param should default to `0.0` in deployment, since the ROS topic already provides corrected body-frame angles. The parameter exists for the training env which reads raw joints; deployment doesn't need it. Alternatively, remove the subtraction entirely in the assembler and document that the topic contract is "body-frame, 0=forward."
