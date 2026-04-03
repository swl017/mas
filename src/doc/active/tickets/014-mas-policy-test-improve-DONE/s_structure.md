## Structure Outline — Ticket #014

### Modified files

#### `mas_policy/mas_policy/observation_assembler.py`
- [remove] `YAW_JOINT_OFFSET` module-level constant
- [remove] `yaw_joint_offset` parameter from `__init__()` and `self._yaw_joint_offset`
- [remove] `self._prev_gimbal` dict and finite-diff rate estimation in `_gimbal_state_callback()`
- [remove] `gimbal_yaw_rate`, `gimbal_pitch_rate` fields from `VehicleState`
- [add] ego subscription to `combined_ang_vel_w` (Vector3Stamped) in `_create_ego_subscriptions()`
- [modify] `_gimbal_state_callback()` — cache yaw/pitch only, no rate estimation
- [modify] `assemble()` — remove `gimbal_yaw_obs = ... - self._yaw_joint_offset`, use `ego.gimbal_yaw_body` directly. Use `ego.combined_ang_vel_w` instead of `compute_combined_angular_velocity_world()`. Clip `bbox_aoi` to `self._max_bbox_aoi`.
- [modify] peer ray fallback (line 444) — remove offset subtraction, use `other.gimbal_yaw_body` directly
- [add] `max_bbox_aoi` parameter to `__init__()`, stored as `self._max_bbox_aoi`

#### `mas_policy/mas_policy/policy_node.py`
- [remove] `yaw_joint_offset` parameter declaration and reading
- [add] `max_bbox_aoi` parameter declaration (default: `20.0`)
- [add] assertion after assembler construction: `assert self._assembler.obs_dim == self._obs_dim`
- [modify] `_control_loop()` CBF section — zero out velocity for stale peers, log warning

#### `mas_policy/mas_policy/policy_loader.py`
- [modify] `load_checkpoint()` — after `load_state_dict`, raise `RuntimeError` if `missing` keys is non-empty
- [add] scaler dimension assertion: `assert scaler.running_mean.shape[0] == obs_dim`

#### `mas_policy/mas_policy/utils.py`
- [remove] `compute_combined_angular_velocity_world()` — no longer called

#### `mas_policy/config/policy_deploy.yaml`
- [remove] `yaw_joint_offset` entry
- [add] `max_bbox_aoi: 20.0`

#### `mas_policy/setup.py`
- [add] `tests_require=['pytest']`

#### `mas_policy/setup.cfg`
- [add] `[tool:pytest]` section with `junit_family = xunit2`

#### `mas_policy/CONTEXT.md`
- [modify] remove `yaw_joint_offset` from parameters
- [modify] add `max_bbox_aoi` to parameters
- [modify] add ego `combined_ang_vel_w` subscription
- [modify] remove `gimbal_yaw_rate` / `gimbal_pitch_rate` mentions
- [modify] fix docstring reference to yaw_joint_offset default

#### `mas_policy/launch/policy_deploy.launch.py`
- [remove] `yaw_joint_offset` from assembler passthrough (already handled by param removal)

### New files

#### `mas_policy/test/__init__.py`
- (empty)

#### `mas_policy/test/test_utils.py`
- `test_wrap_to_pi_basic()` — known values at 0, pi, -pi, 2pi, -3pi
- `test_wrap_to_pi_array()` — vector input
- `test_euler_xyz_from_quat_identity()` — identity quaternion → (0,0,0)
- `test_euler_xyz_from_quat_90deg_yaw()` — known rotation
- `test_quat_rotate_identity()` — identity quat preserves vector
- `test_quat_rotate_90deg()` — known rotation
- `test_gimbal_ray_direction_world_forward()` — yaw=0, pitch=0, identity quat → +X

#### `mas_policy/test/test_cbf_filter.py`
- `test_cbf_no_constraint_far_agents()` — agents far apart → v_safe == v_nom
- `test_cbf_corrects_close_agents()` — agents within D_deploy → velocity modified
- `test_cbf_single_agent_noop()` — N=1 → no filtering
- `test_cbf_stale_peer_zero_velocity()` — verify zeroed peer velocity produces correct constraint

#### `mas_policy/test/test_policy_loader.py`
- `REAL_CHECKPOINT_DIR` — path to `models/2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning/`
- `_make_broken_checkpoint(tmp_path, ...)` — helper: creates deliberately broken SKRL-format checkpoint for negative tests
- `test_load_real_checkpoint()` — load `best_agent.pt` from real model dir, verify policy forward pass produces correct shape (obs_dim=52, action_dim=7)
- `test_load_real_checkpoint_scaler()` — verify scaler dim matches obs_dim, scaler mean/var are finite
- `test_load_checkpoint_missing_keys_raises()` — broken checkpoint with wrong obs_dim → RuntimeError
- `test_load_checkpoint_scaler_dim_mismatch_raises()` — broken checkpoint with wrong scaler dim → AssertionError
- `test_load_checkpoint_no_scaler_fallback()` — broken checkpoint without state_preprocessor → identity scaler with warning

#### `mas_policy/test/test_observation_assembler.py`
- `_mock_node()` — fixture: creates a minimal mock of rclpy.Node (enough for subscription creation)
- `test_obs_dim_2_agents_tri()` — obs_dim == 52 for 2 agents + tri (matches real checkpoint)
- `test_obs_dim_3_agents_tri()` — obs_dim == 68 for 3 agents + tri
- `test_assemble_output_shape_2_agents()` — populate VehicleState, call assemble(), verify len == obs_dim
- `test_assemble_gimbal_yaw_no_offset()` — verify obs[15] == gimbal_yaw_body (no offset subtraction)
- `test_assemble_bbox_aoi_clipped()` — set detection_timestamp far in past, verify bbox_aoi <= max_bbox_aoi

#### `mas_policy/test/test_integration.py`
- `REAL_CHECKPOINT` — path to `models/.../best_agent.pt`
- `test_node_starts_and_infers()` — rclpy.init(), create PolicyDeployNode with real checkpoint, manually populate ego odom, call `_control_loop()` once, verify no crash
- `test_gru_hidden_warmup()` — run 50 consecutive `_control_loop()` ticks with real checkpoint and synthetic odom updates, verify all actions bounded [-1,1] and hidden state contains no NaN/Inf
