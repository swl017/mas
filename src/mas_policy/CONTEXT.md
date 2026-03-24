# mas_policy — Node Interface Contract

## policy_node (mas_policy_node)

Runs trained MARL policy inference at 25 Hz. Orchestrates the full loop: assemble observations from cached ROS2 topics, normalize with the training scaler, forward pass through the policy network (MAPPO-RNN or PPO-MLP), apply CBF safety filter, and publish velocity/gimbal/zoom commands.

### Subscriptions

Per-vehicle (topic prefix `/{vehicle_name}/`):

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `common_frame/odom` | `nav_msgs/Odometry` | 10 | ENU odometry (when `use_common_frame=true`) |
| `mavros/local_position/odom` | `nav_msgs/Odometry` | 10 | MAVROS odometry fallback (when `use_common_frame=false`) |
| `mavros/imu/data` | `sensor_msgs/Imu` | 10 | Body-frame linear acceleration |
| `gimbal_state_rpy_rad` | `geometry_msgs/Vector3` | 10 | Gimbal RPY from los_rate_controller (rad) |
| `yolo_result_vision` | `vision_msgs/Detection2DArray` | 10 | YOLO detections for bbox observation |

Global:

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `chosen_target_pose` | `geometry_msgs/PoseStamped` | 10 | Triangulation result (when `enable_triangulation=true`) |

### Publishers

Per-vehicle (topic prefix `/{vehicle_name}/`):

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `cmd_vel` | `geometry_msgs/TwistStamped` | 25 Hz | Velocity + yaw rate setpoint (ENU) |
| `gimbal_cmd_los_rate` | `geometry_msgs/Vector3` | 25 Hz | Gimbal azimuth/elevation rate (normalized [-1,1]) |
| `zoom_cmd` | `std_msgs/Float32` | 25 Hz | Zoom rate (normalized [-1,1]) |

### Services

| Service | Type | Notes |
|---------|------|-------|
| `~/reset_hidden_state` | `std_srvs/Trigger` | Reset GRU hidden states for all agents |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `checkpoint_path` | string | `''` | Path to SKRL .pt checkpoint |
| `num_agents` | int | `2` | Number of agents |
| `vehicle_names` | string[] | `['px4_1','px4_2']` | Vehicle namespace prefixes |
| `obs_dim` | int | `62` | Observation vector dimension |
| `action_dim` | int | `7` | Action vector dimension (3 vel + 1 yaw + 2 gimbal + 1 zoom) |
| `architecture` | string | `'mappo_rnn'` | `'mappo_rnn'` or `'ppo_mlp'` |
| `hidden_size` | int | `64` | MLP hidden layer size |
| `gru_hidden_size` | int | `64` | GRU hidden size |
| `gru_num_layers` | int | `1` | Number of GRU layers |
| `control_frequency` | float | `25.0` | Inference loop rate (Hz) |
| `max_lin_vel` | float | `10.0` | Max linear velocity for action scaling (m/s) |
| `max_yaw_rate` | float | `0.7854` | Max yaw rate for action scaling (rad/s) |
| `enable_cbf` | bool | `true` | Enable CBF inter-agent safety filter |
| `enable_triangulation` | bool | `false` | Append 6D triangulation tail to observations |
| `image_width` | int | `640` | Image width for bbox normalization |
| `image_height` | int | `480` | Image height for bbox normalization |
| `yaw_joint_offset` | float | `-1.5708` | Gimbal yaw joint offset (rad) |
| `device` | string | `'cpu'` | Torch device (`'cpu'` or `'cuda'`) |
| `dry_run` | bool | `false` | Log actions without publishing |
| `use_common_frame` | bool | `true` | Use common_frame/odom (ENU) vs MAVROS odom |
| `stale_timeout` | float | `2.0` | Reset hidden state if odom older than this (s) |
| `cbf_D_s` | float | `2.0` | Physical safety distance (m) |
| `cbf_v_max` | float | `15.0` | Max expected velocity for CBF margin (m/s) |
| `cbf_tau_delay_max` | float | `0.2` | Max communication delay (s) |
| `cbf_tau_px4` | float | `0.3` | PX4 velocity controller time constant (s) |
| `cbf_gamma_deploy` | float | `1.0` | CBF decay rate |
| `cbf_num_iters` | int | `2` | Gauss-Seidel projection iterations |

### Dependencies

**Upstream:** `mas_common_frame` (common_frame/odom), `gimbal_controller` / `los_rate_controller` (gimbal_state_rpy_rad), `ultralytics_ros` (yolo_result_vision), `mas_multiview`/`mas_multiview_py` (chosen_target_pose)
**Downstream:** `mas_offboard` consumes `cmd_vel`; `gimbal_controller` consumes `gimbal_cmd_los_rate` and `zoom_cmd`
