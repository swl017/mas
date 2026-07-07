# mas_policy

## Purpose
Per-vehicle policy deployment for iris_ma6 trained MARL policies. One instance per vehicle, launched inside the vehicle's namespace. Runs inference at 25 Hz: assembles observations from ego + peer cached ROS2 topics, normalizes with the training scaler, runs forward pass through the policy network (MAPPO-RNN or PPO-MLP), applies CBF safety filter, and publishes velocity/gimbal/zoom commands.

## Nodes

### policy_node
**File:** `mas_policy/policy_node.py`
**Executable:** `policy_node`
**Pattern:** Decoupled (subscribers → cache sensor data, timer at 25 Hz → infer + publish)
**Deployment:** Per-vehicle (one instance per namespace, launched via `vehicles.yaml`)

#### Subscriptions

Ego (relative topics, resolved by node namespace):
- `common_frame/odom` (`nav_msgs/Odometry`) — ENU odometry from mas_common_frame (when `use_common_frame=true`)
- `mavros/local_position/odom` (`nav_msgs/Odometry`) — MAVROS odometry fallback (when `use_common_frame=false`)
- `mavros/imu/data` (`sensor_msgs/Imu`) — body-frame linear acceleration
- `gimbal_state_rpy_deg` (`geometry_msgs/Vector3`) — gimbal body-frame RPY (deg, converted to rad internally). 0=forward, no offset subtraction needed.
- `yolo_result_vision` (`vision_msgs/Detection2DArray`) — YOLO detections for bbox observation
- `combined_ang_vel_w` (`geometry_msgs/Vector3Stamped`) — ego combined angular velocity in world frame (from los_rate_controller / siyi_ros_node)
- `chosen_target_ray_w` (`geometry_msgs/Vector3Stamped`) — ego bearing ray to chosen target (from mas_tracker)

Ego additional:
- `camera/zoom_level` (`std_msgs/Float64`) — current zoom level from gimbal controller
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`) — camera resolution for bbox normalization (auto-adapts to actual camera)

Peers (absolute topics, `/{peer_name}/` prefix):
- `/{peer}/common_frame/odom` (`nav_msgs/Odometry`) — peer position + velocity
- `/{peer}/combined_ang_vel_w` (`geometry_msgs/Vector3Stamped`) — peer pre-computed combined angular velocity (body + gimbal) in world frame
- `/{peer}/yolo_result_active` (`std_msgs/Bool`) — peer detection active (compact, replaces full Detection2DArray cross-agent)
- `/{peer}/camera/zoom_level` (`std_msgs/Float64`) — peer zoom level
- `/{peer}/policy/observation` (`std_msgs/Float32MultiArray`) — peer assembled observation vector for value network shared state (BEST_EFFORT, depth=1)

Ego triangulation (relative — resolves under vehicle namespace):
- `chosen_target_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — sort3d's selected target with covariance (when `enable_triangulation=true`). Published per-vehicle by `mas_tracker/sort3d_node`, so deploy subscribes relatively to pick up `/{ego}/chosen_target_pose`.

#### Publishers

Relative topics (resolved by node namespace):
- `policy/value` (`std_msgs/Float32`) — value function V(s) scalar at 25 Hz, consumed by mas_operator for monitoring. Uses true shared state (concatenated ego + peer observations via cross-agent topic).
- `policy/observation` (`std_msgs/Float32MultiArray`) — ego assembled observation vector at 25 Hz (BEST_EFFORT, depth=1), consumed by peer policy_nodes for value network shared state construction.
- `cmd_vel` (`geometry_msgs/TwistStamped`) — velocity + yaw rate setpoint in ENU at 25 Hz, consumed by offboard_py
- `gimbal_cmd_los_rate` (`geometry_msgs/Vector3`) — LOS azimuth/elevation rate in rad/s at 25 Hz (denormalized from policy output × `max_gimbal_rate`), consumed by los_rate_controller
- `zoom_rate_cmd` (`std_msgs/Float32`) — zoom rate in zoom-levels/s at 25 Hz (denormalized from policy output × `max_zoom_rate`)

#### Services
- `~/reset_hidden_state` (`std_srvs/Trigger`) — reset GRU hidden state for this agent

#### Parameters
- `vehicle_name` (`string`, default: from namespace) — this vehicle's namespace
- `peer_names` (`string[]`, default: `[]`) — other vehicles' namespaces (set by launch file)
- `checkpoint_path` (`string`, default: `""`) — path to SKRL .pt checkpoint
- `obs_dim` (`int`, computed) — observation vector dimension: 31 ego + 7 prev-action (if `enable_prev_action_obs`) + 16·(N-1) inter-agent [+6 triangulation]. For N=2 with prev-action on: 54.
- `action_dim` (`int`, default: `7`) — action dimension (3 vel + 1 yaw + 2 gimbal + 1 zoom)
- `architecture` (`string`, default: `"mappo_rnn"`) — `"mappo_rnn"` or `"ppo_mlp"`
- `hidden_size` (`int`, default: `64`) — MLP hidden layer size
- `gru_hidden_size` (`int`, default: `64`) — GRU hidden size
- `gru_num_layers` (`int`, default: `1`) — number of GRU layers
- `control_frequency` (`double`, default: `25.0`) — inference loop rate (Hz)
- `max_lin_vel` (`double`, default: `5.0`) — max linear velocity for action scaling (m/s, matches training `IrisMA6TestEnvCfg.max_lin_vel` post-t045)
- `max_yaw_rate` (`double`, default: `0.7854`) — max yaw rate for action scaling (rad/s)
- `enable_asymmetric_z_envelope` (`bool`, default: `true`) — scale vz by `max_vel_z_up` (climb) / `max_vel_z_dn` (descend) instead of `max_lin_vel`, matching training ticket 039. Applies to both the published `cmd_vel` and the prev-action obs tail.
- `max_vel_z_up` (`double`, default: `3.0`) — max climb rate (m/s, action[2] ≥ 0)
- `max_vel_z_dn` (`double`, default: `1.5`) — max descend rate (m/s, action[2] < 0)
- `enable_prev_action_obs` (`bool`, default: `true`) — append the previous `cmd_vel` (7D: vx,vy,vz m/s, yaw_rate rad/s, gimbal_yaw, gimbal_pitch, zoom normalized) to each ego observation, matching training ticket 043. Must match the checkpoint (agent_400000.pt trained with it ON). The tail is built from the slew-clipped (pre-CBF) action; reset to zero on MISSION entry and stale-data.
- `max_gimbal_rate` (`double`, default: `3.14159`) — max gimbal LOS rate for action denormalization (rad/s, must match training)
- `max_zoom_rate` (`double`, default: `2.0`) — max zoom rate for action denormalization (zoom-levels/s, must match training `ZoomControllerCfg`)
- `enable_cbf` (`bool`, default: `true`) — enable CBF inter-agent safety filter
- `enable_triangulation` (`bool`, default: `false`) — append 6D triangulation tail to observations
- `image_width` (`int`, default: `640`) — image width for bbox normalization
- `image_height` (`int`, default: `480`) — image height for bbox normalization
- `max_bbox_aoi` (`double`, default: `20.0`) — max bbox age-of-information (s), clips to training episode length
- `device` (`string`, default: `"cpu"`) — torch device (`"cpu"` or `"cuda"`)
- `dry_run` (`bool`, default: `false`) — log observations/actions without publishing
- `use_common_frame` (`bool`, default: `true`) — use common_frame/odom (ENU) vs MAVROS odom
- `stale_timeout` (`double`, default: `2.0`) — reset hidden state if odom older than this (s)
- `cbf_D_s` (`double`, default: `2.0`) — physical safety distance (m)
- `cbf_v_max` (`double`, default: `15.0`) — max expected velocity for CBF margin (m/s)
- `cbf_tau_delay_max` (`double`, default: `0.2`) — max communication delay (s)
- `cbf_tau_px4` (`double`, default: `0.3`) — PX4 velocity controller time constant (s)
- `cbf_gamma_deploy` (`double`, default: `1.0`) — CBF decay rate
- `cbf_num_iters` (`int`, default: `2`) — Gauss-Seidel projection iterations
- `enable_action_slew_clip` (`bool`, default: `true`) — per-channel slew-rate clip on the raw normalized action, mirroring training ticket 044 (`iris_ma_env6_test._pre_physics_step`). Clips `|Δaction| ≤ δ_max` per control step in `[-1,1]` units, applied before scaling/CBF. `_last_action` resets to zero on MISSION entry and on stale-data zero-publish. Defaults match the `2026-06-04 ticket047_D_curriculum_cold` (`agent_400000.pt`) deployment baseline.
- `action_slew_vel_xy` (`double`, default: `0.040`) — δ_max for vx, vy
- `action_slew_vel_z` (`double`, default: `0.053`) — δ_max for vz
- `action_slew_yaw_rate` (`double`, default: `0.30`) — δ_max for yaw_rate
- `action_slew_gimbal_yaw_rate` (`double`, default: `0.40`) — δ_max for gimbal yaw rate
- `action_slew_gimbal_pitch_rate` (`double`, default: `0.40`) — δ_max for gimbal pitch rate
- `action_slew_zoom_rate` (`double`, default: `0.20`) — δ_max for zoom rate

## Dependencies
- mas_common_frame — provides `common_frame/odom` (ego + peers)
- los_rate_controller / siyi_gimbal_node — provides `gimbal_state_rpy_deg` (ego), `combined_ang_vel_w` (peers), `camera/zoom_level` (ego + peers)
- ultralytics_ros — provides `yolo_result_vision` (ego), `yolo_result_active` (peers)
- mas_tracker — provides `chosen_target_pose`
- offboard_py — consumes `cmd_vel`

## Key Files
- `mas_policy/policy_node.py` — per-vehicle ROS2 node (25 Hz timer loop)
- `mas_policy/policy_loader.py` — standalone PolicyNetRNN/MLP/ValueNetRNN + SKRL checkpoint loading
- `mas_policy/observation_assembler.py` — ego + peer subscribers → 63/69D observation vector
- `mas_policy/action_publisher.py` — 7D actions → cmd_vel + gimbal_cmd_los_rate + zoom_rate_cmd
- `mas_policy/cbf_filter.py` — deployment CBF safety filter (halfspace projection)
- `mas_policy/utils.py` — math utilities (gimbal_ray_direction_world, euler_from_quat, frame conversions)
- `config/policy_deploy.yaml` — shared parameter defaults
- `config/vehicles.yaml` — vehicle roster (namespaces)
- `launch/policy_deploy.launch.py` — multi-agent launch (one node per vehicle)

## Calling Contract

**Pattern**: Decoupled (subscribe → cache, timer → infer + publish)

- `_odom_callback()`: Caches position, velocity, orientation, angular velocity for ego or peer. No publishing.
- `_imu_callback()`: Caches body-frame linear acceleration (ego only). No publishing.
- `_gimbal_state_callback()`: Caches gimbal body-frame angles for ego (deg→rad, 0=forward, no offset). No publishing.
- `_detection_callback()`: Caches normalized bbox and detection timestamp for ego. No publishing.
- `_peer_detection_active_callback()`: Caches bbox_empty from compact Bool for peers. No publishing.
- `_peer_combined_ang_vel_callback()`: Caches pre-computed combined angular velocity for ego and peers. No publishing.
- `_zoom_level_callback()`: Caches zoom level for ego or peer. No publishing.
- `_triangulation_callback()`: Caches triangulation position and std_dev from covariance. No publishing.
- `_peer_obs_callback()`: Caches peer's assembled observation vector (Float32MultiArray) for value network shared state. No publishing.
- `_control_loop()` (25 Hz timer): Reads all cached state, assembles ego observation vector, publishes it on `policy/observation`, runs policy forward pass with GRU hidden state (tanh squashing), constructs true shared state from ego + peer observations for value network, applies CBF filter (with stale peer velocity zeroing), publishes ego actions. Sole periodic mutation point.
- `_reset_hidden_callback()`: On-demand service. Zeros GRU hidden state.

## Spec
None (self-documented via docstrings and plan file at `.claude/plans/idempotent-finding-pearl.md`).
