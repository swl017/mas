## Implementation Plan — Ticket #014

### Slice 1: Hardening fixes (A1-A7)
All code fixes in mas_policy, no tests yet.

- Step 1.1: Remove `yaw_joint_offset` — delete constant, parameter, and subtraction in `observation_assembler.py`. Remove from `policy_node.py` param declarations. Remove from `policy_deploy.yaml`. Remove from `launch/policy_deploy.launch.py` passthrough.
- Step 1.2: Add ego `combined_ang_vel_w` subscription in `_create_ego_subscriptions()`. Use `ego.combined_ang_vel_w` in `assemble()` instead of `compute_combined_angular_velocity_world()`. Remove finite-diff from `_gimbal_state_callback()`. Remove `gimbal_yaw_rate`/`gimbal_pitch_rate` from `VehicleState`. Remove `compute_combined_angular_velocity_world` from `utils.py`.
- Step 1.3: Add `max_bbox_aoi` param to assembler `__init__()` and `policy_node.py`. Clip `bbox_aoi` in `assemble()`.
- Step 1.4: Add obs_dim assertion in `policy_node.py` after assembler construction.
- Step 1.5: In `policy_loader.py`, raise `RuntimeError` on missing keys after `load_state_dict`. Add scaler dimension assertion.
- Step 1.6: In `policy_node.py` `_control_loop()`, zero velocity for stale peers before CBF call. Log warning.
- Test checkpoint: `colcon build --packages-select mas_policy` succeeds. Manual review of diffs.

### Slice 2: Test infrastructure + unit tests (B1-B2)
- Step 2.1: Add `tests_require=['pytest']` to `setup.py`, `[tool:pytest]` to `setup.cfg`, create `test/__init__.py`.
- Step 2.2: Write `test/test_utils.py` — 7 tests for math utilities.
- Step 2.3: Write `test/test_cbf_filter.py` — 4 tests for CBF filter.
- Step 2.4: Write `test/test_policy_loader.py` — 5 tests using real + broken checkpoints.
- Step 2.5: Write `test/test_observation_assembler.py` — 7 tests with mocked rclpy node.
- Test checkpoint: `cd ~/mas && python -m pytest src/mas_policy/test/ -v` all green.

### Slice 3: Integration smoke test + CONTEXT.md (B3 + A8)
- Step 3.1: Write `test/test_integration.py` — 2 tests with real checkpoint and rclpy.
- Step 3.2: Update `CONTEXT.md` — remove yaw_joint_offset, add max_bbox_aoi, add ego combined_ang_vel_w subscription, remove gimbal rate fields.
- Test checkpoint: Integration tests pass with sourced ROS2 env. CONTEXT.md review.
