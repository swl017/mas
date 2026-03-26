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
- `gimbal_state_rpy_rad` (`geometry_msgs/Vector3`) — gimbal body-frame RPY from los_rate_controller (rad)
- `yolo_result_vision` (`vision_msgs/Detection2DArray`) — YOLO detections for bbox observation

Ego additional:
- `zoom_level` (`std_msgs/Float32`) — current zoom level from gimbal controller

Peers (absolute topics, `/{peer_name}/` prefix):
- `/{peer}/common_frame/odom` (`nav_msgs/Odometry`) — peer position + velocity
- `/{peer}/combined_ang_vel_w` (`geometry_msgs/Vector3Stamped`) — peer pre-computed combined angular velocity (body + gimbal) in world frame
- `/{peer}/yolo_result_active` (`std_msgs/Bool`) — peer detection active (compact, replaces full Detection2DArray cross-agent)
- `/{peer}/zoom_level` (`std_msgs/Float32`) — peer zoom level

Global:
- `/chosen_target_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — triangulation result with covariance (when `enable_triangulation=true`)

#### Publishers

Relative topics (resolved by node namespace):
- `cmd_vel` (`geometry_msgs/TwistStamped`) — velocity + yaw rate setpoint in ENU at 25 Hz, consumed by offboard_py
- `gimbal_cmd_los_rate` (`geometry_msgs/Vector3`) — normalized [-1,1] azimuth/elevation rate at 25 Hz, consumed by los_rate_controller
- `zoom_cmd` (`std_msgs/Float32`) — normalized [-1,1] zoom rate at 25 Hz

#### Services
- `~/reset_hidden_state` (`std_srvs/Trigger`) — reset GRU hidden state for this agent

#### Parameters
- `vehicle_name` (`string`, default: from namespace) — this vehicle's namespace
- `peer_names` (`string[]`, default: `[]`) — other vehicles' namespaces (set by launch file)
- `checkpoint_path` (`string`, default: `""`) — path to SKRL .pt checkpoint
- `obs_dim` (`int`, default: `62`) — observation vector dimension
- `action_dim` (`int`, default: `7`) — action dimension (3 vel + 1 yaw + 2 gimbal + 1 zoom)
- `architecture` (`string`, default: `"mappo_rnn"`) — `"mappo_rnn"` or `"ppo_mlp"`
- `hidden_size` (`int`, default: `64`) — MLP hidden layer size
- `gru_hidden_size` (`int`, default: `64`) — GRU hidden size
- `gru_num_layers` (`int`, default: `1`) — number of GRU layers
- `control_frequency` (`double`, default: `25.0`) — inference loop rate (Hz)
- `max_lin_vel` (`double`, default: `10.0`) — max linear velocity for action scaling (m/s)
- `max_yaw_rate` (`double`, default: `0.7854`) — max yaw rate for action scaling (rad/s)
- `enable_cbf` (`bool`, default: `true`) — enable CBF inter-agent safety filter
- `enable_triangulation` (`bool`, default: `false`) — append 6D triangulation tail to observations
- `image_width` (`int`, default: `640`) — image width for bbox normalization
- `image_height` (`int`, default: `480`) — image height for bbox normalization
- `yaw_joint_offset` (`double`, default: `-1.5708`) — gimbal yaw joint offset (rad)
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

## Dependencies
- mas_common_frame — provides `common_frame/odom` (ego + peers)
- los_rate_controller / siyi_gimbal_node — provides `gimbal_state_rpy_rad` (ego), `combined_ang_vel_w` (peers), `zoom_level` (ego + peers)
- ultralytics_ros — provides `yolo_result_vision` (ego), `yolo_result_active` (peers)
- mas_tracker — provides `chosen_target_pose`
- offboard_py — consumes `cmd_vel`

## Key Files
- `mas_policy/policy_node.py` — per-vehicle ROS2 node (25 Hz timer loop)
- `mas_policy/policy_loader.py` — standalone PolicyNetRNN/MLP + SKRL checkpoint loading
- `mas_policy/observation_assembler.py` — ego + peer subscribers → 62/68D observation vector
- `mas_policy/action_publisher.py` — 7D actions → cmd_vel + gimbal_cmd_los_rate + zoom_cmd
- `mas_policy/cbf_filter.py` — deployment CBF safety filter (halfspace projection)
- `mas_policy/utils.py` — math utilities (gimbal_ray_direction_world, euler_from_quat, frame conversions)
- `config/policy_deploy.yaml` — shared parameter defaults
- `config/vehicles.yaml` — vehicle roster (namespaces)
- `launch/policy_deploy.launch.py` — multi-agent launch (one node per vehicle)

## Calling Contract

**Pattern**: Decoupled (subscribe → cache, timer → infer + publish)

- `_odom_callback()`: Caches position, velocity, orientation, angular velocity for ego or peer. No publishing.
- `_imu_callback()`: Caches body-frame linear acceleration (ego only). No publishing.
- `_gimbal_state_callback()`: Caches gimbal angles for ego, estimates rates via finite differences. No publishing.
- `_detection_callback()`: Caches normalized bbox and detection timestamp for ego. No publishing.
- `_peer_detection_active_callback()`: Caches bbox_empty from compact Bool for peers. No publishing.
- `_peer_combined_ang_vel_callback()`: Caches pre-computed combined angular velocity for peers. No publishing.
- `_zoom_level_callback()`: Caches zoom level for ego or peer. No publishing.
- `_triangulation_callback()`: Caches triangulation position and std_dev from covariance. No publishing.
- `_control_loop()` (25 Hz timer): Reads all cached state, assembles ego observation vector, runs policy forward pass with GRU hidden state, applies CBF filter, publishes ego actions. Sole periodic mutation point.
- `_reset_hidden_callback()`: On-demand service. Zeros GRU hidden state.

## Spec
None (self-documented via docstrings and plan file at `.claude/plans/idempotent-finding-pearl.md`).
