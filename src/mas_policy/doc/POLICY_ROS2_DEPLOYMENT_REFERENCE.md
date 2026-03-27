# iris_ma6 Policy → ROS2 Deployment Reference

> Single-sheet reference for implementing the iris_ma6 trained MAPPO-RNN policy as a ROS2 inference node.

---

## 1. Observation Vector (52D for 2 agents)

Formula: `obs_dim = 30 + 16 × (num_agents - 1) + 6` (with triangulation enabled)

### Ego Observations (30D) — indices [0:30]

| Index | Quantity | Dim | Unit | Source (ROS2) |
|-------|----------|-----|------|---------------|
| 0-2 | Body position (world) | 3 | m | `common_frame/odom` pose.position (fallback: `mavros/local_position/odom`) |
| 3-5 | Body linear velocity (world) | 3 | m/s | `common_frame/odom` twist.linear (fallback: `mavros/local_position/odom`) |
| 6-8 | Roll, Pitch, Yaw (Euler) | 3 | rad | Derived from `mavros/imu/data` quaternion via `euler_xyz_from_quat`, wrapped to [-pi, pi] |
| 9-11 | Angular velocity (body) | 3 | rad/s | `mavros/imu/data` angular_velocity |
| 12-14 | Linear acceleration (body) | 3 | m/s^2 | `mavros/imu/data` linear_acceleration |
| 15 | Gimbal yaw (body frame) | 1 | rad | Gimbal joint feedback minus YAW_JOINT_OFFSET (-pi/2). 0 = forward. |
| 16 | Gimbal pitch (body frame) | 1 | rad | Gimbal joint feedback (pitch joint) |
| 17-19 | Camera ray direction (world) | 3 | unit vec | Selected from `target_rays_w` via angular-proximity matching, fallback: gimbal LOS (**deferred: ray selection should move to mas_tracker**) |
| 20-22 | Camera sweep rate (world) | 3 | rad/s | Combined body + gimbal angular velocity in world frame |
| 23 | Bbox age-of-information | 1 | s | Time since last valid detection |
| 24 | Zoom level | 1 | - | Current camera zoom factor |
| 25-28 | Bounding box (normalized) | 4 | [0,1] | [cx, cy, w, h] in image coords, divided by image dims |
| 29 | Bbox empty flag | 1 | bool | 1.0 if no detection, 0.0 if valid |

### Inter-Agent Observations (16D per other agent) — indices [30:46]

| Offset | Quantity | Dim | Unit | Source |
|--------|----------|-----|------|--------|
| +0-2 | Other position (world) | 3 | m | `/{peer}/common_frame/odom` pose.position |
| +3-5 | Other linear velocity (world) | 3 | m/s | `/{peer}/common_frame/odom` twist.linear |
| +6-8 | Other camera ray (world) | 3 | unit vec | `/{peer}/target_rays_w` (selected ray, see note below) |
| +9-11 | Other camera sweep rate (world) | 3 | rad/s | `/{peer}/combined_ang_vel_w` (Vector3Stamped, pre-computed) |
| +12 | Other zoom level | 1 | - | `/{peer}/zoom_level` (Float32) |
| +13 | Other bbox empty flag | 1 | bool | `/{peer}/yolo_result_active` (Bool, inverted: 0.0 if active) |
| +14 | Data age-of-information | 1 | s | Time since motion data received from other |
| +15 | Bbox age-of-information | 1 | s | Time since detection data received from other |

For 2 agents: one block of 16D at indices [30:46].
For 3 agents: two blocks at [30:46] and [46:62].

### Triangulation Tail (6D) — indices [46:52]

| Offset | Quantity | Dim | Unit |
|--------|----------|-----|------|
| +0-2 | Triangulated target position | 3 | m | Midpoint method estimate (zeros if invalid) |
| +3-5 | Position uncertainty (std dev) | 3 | m | sqrt(diag(covariance)), -1.0 if invalid |

---

## 2. Action Vector (7D)

All policy outputs are in [-1, 1]. Scaling applied before sending to actuators.

| Index | Action | Scaling | Physical Range | ROS2 Topic | Message Field |
|-------|--------|---------|----------------|------------|---------------|
| 0 | vx (world ENU) | x max_lin_vel | [-10, 10] m/s | `cmd_vel` | twist.linear.x |
| 1 | vy (world ENU) | x max_lin_vel | [-10, 10] m/s | `cmd_vel` | twist.linear.y |
| 2 | vz (world ENU) | x max_lin_vel | [-10, 10] m/s | `cmd_vel` | twist.linear.z |
| 3 | yaw rate | x max_yaw_rate | [-0.785, 0.785] rad/s | `cmd_vel` | twist.angular.z |
| 4 | gimbal az rate | pass-through | [-1, 1] normalized | `gimbal_cmd_los_rate` | x |
| 5 | gimbal el rate | pass-through | [-1, 1] normalized | `gimbal_cmd_los_rate` | y |
| 6 | zoom rate | pass-through | [-1, 1] normalized | `zoom_cmd` | data |

`ActionPublisher` (mas_policy) handles scaling and publishing. See `mas_policy/action_publisher.py`.

---

## 3. Model Architecture

### Policy Network (MAPPORNNPolicy)

```
Input (52D observations)
  |-- RunningStandardScaler: obs_norm = (obs - running_mean) / sqrt(running_variance + 1e-8)
  |
  |-- Linear(52, 64) + ReLU
  |-- Linear(64, 64) + ReLU
  |-- GRU(input=64, hidden=64, layers=1, batch_first=True)
  |-- Linear(64, 7)  -->  action mean
  |
  +-- log_std parameter (7D, learnable, shared across states)
      init=-0.5, clamp to [-5.0, 0.7]
```

### Value Network (MAPPORNNValue) — not needed for deployment

```
Same architecture as policy, but:
  |-- Linear(64, 1)  -->  scalar V(s)
```

### Key Hyperparameters

| Parameter | Value |
|-----------|-------|
| hidden_size | 64 |
| gru_hidden_size | 64 |
| gru_num_layers | 1 |
| initial_log_std | -0.5 |
| sequence_length (training) | 32 |

---

## 4. Inference Pipeline

### 4.1 Checkpoint Contents

Checkpoint file (`agent_*.pt`) contains per-agent state dicts:
- `policy_state_dict` — network weights
- `value_state_dict` — (not needed for deployment)
- `state_preprocessor` — RunningStandardScaler with `running_mean` and `running_variance` tensors (float64)

### 4.2 Model Reconstruction

```python
import torch
import torch.nn as nn

class PolicyNetwork(nn.Module):
    def __init__(self, obs_dim=52, act_dim=7, hidden=64, gru_layers=1):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, hidden)
        self.fc2 = nn.Linear(hidden, hidden)
        self.gru = nn.GRU(hidden, hidden, num_layers=gru_layers, batch_first=True)
        self.policy_head = nn.Linear(hidden, act_dim)
        self.log_std = nn.Parameter(torch.full((act_dim,), -0.5))

    def forward(self, obs, h):
        """
        obs: (1, obs_dim)  -- single observation
        h:   (1, 1, 64)    -- GRU hidden state
        Returns: action_mean (1, 7), new_h (1, 1, 64)
        """
        x = torch.relu(self.fc1(obs))
        x = torch.relu(self.fc2(x))
        x = x.unsqueeze(1)  # (1, 1, 64) for GRU sequence dim
        x, h_new = self.gru(x, h)
        x = x.squeeze(1)    # (1, 64)
        return self.policy_head(x), h_new
```

### 4.3 Loading Weights

```python
checkpoint = torch.load("best_agent.pt", map_location="cpu", weights_only=False)

# SKRL per-agent format: top-level keys are agent UIDs (e.g., "drone_0", "drone_1")
# All agents share the same policy weights; load from first agent.
agent = checkpoint["drone_0"]

# Policy weights — key names from SKRL's MAPPORNNBaseModel
# fc1 → "net.0.weight/bias", fc2 → "net.2.weight/bias"
# gru → "gru.weight_ih_l0", etc.
# policy_head → "policy_layer.weight/bias"
# log_std → "log_std_parameter"
policy.load_state_dict(agent["policy"], strict=False)

# Preprocessor — running statistics
sp = agent["state_preprocessor"]
running_mean = sp["running_mean"]          # (52,) float64
running_variance = sp["running_variance"]  # (52,) float64
# Normalize: (obs - running_mean) / sqrt(running_variance + 1e-8)
```

### 4.4 Inference Loop (25 Hz)

```python
h = torch.zeros(1, 1, 64)  # GRU hidden state

while running:
    obs = build_observation_vector()           # (52,) from ROS2 topics
    obs_tensor = torch.tensor(obs).unsqueeze(0)  # (1, 52)
    obs_norm = (obs_tensor - running_mean) / torch.sqrt(running_variance + 1e-8)

    with torch.no_grad():
        action_mean, h = policy(obs_norm, h)

    action = action_mean.squeeze(0).numpy()    # (7,) deterministic
    action = np.clip(action, -1.0, 1.0)
    # CBF safety filter applied here (see Section 4.5)
    action_publisher.publish(action)

    # Reset h to zeros on:
    #   - Episode boundary (mission restart)
    #   - Stale ego odometry (>2.0s since last update)
    #   - Service call: ~/reset_hidden_state (std_srvs/Trigger)
```

### 4.5 CBF Safety Filter (Deployment)

Applied between policy inference and action publishing. Modifies velocity commands (actions[0:3]) only.

**Barrier function:** `h_ij = ||p_i - p_j||^2 - D_deploy^2`

**CBF constraint per pair (i,j):** `2 * dp_ij^T * (v_i - v_j) + gamma * h_ij >= 0`

If violated, closed-form halfspace projection shifts `v_i` to satisfy the constraint.

| Parameter | Value | Description |
|-----------|-------|-------------|
| `D_s` | 2.0 m | Physical safety distance |
| `v_max` | 15.0 m/s | Max expected velocity |
| `tau_delay_max` | 0.2 s | Max communication delay |
| `tau_px4` | 0.3 s | PX4 velocity controller time constant |
| `D_deploy` | 9.5 m | Inflated distance: `D_s + v_max * (tau_delay_max + tau_px4)` |
| `gamma_deploy` | 1.0 | CBF decay rate |
| `num_iters` | 2 | Gauss-Seidel projection iterations |

Ego nominal velocity = policy output x max_lin_vel. Peer nominal velocities = their current odometry velocities.

See `mas_policy/cbf_filter.py`.

---

## 5. ROS2 Topic Wiring

### Command Flow (MISSION state)

> **Topic prefix note:** policy_node publishes to bare relative topics (`cmd_vel`, etc.).
> When launched with `use_mission_gate:=true`, topics are remapped to `policy/*` for
> mas_mission gating. **Decision on default prefix is deferred.**

```
policy_node
  | publishes (relative, remapped to policy/* when mission-gated):
  |   /{ns}/[policy/]cmd_vel          (TwistStamped)
  |   /{ns}/[policy/]gimbal_cmd_los_rate (Vector3)
  |   /{ns}/[policy/]zoom_cmd         (Float32)
  v
mas_mission (mission_node, state=MISSION)
  | routes to:
  |   /{ns}/cmd_vel                 (TwistStamped)
  |   /{ns}/gimbal_cmd_los_rate     (Vector3)
  |   /{ns}/zoom_cmd                (Float32)
  v
mas_offboard (offboard_control, state=POLICY)
  | publishes:
  |   /{ns}/mavros/setpoint_velocity/cmd_vel
  v
MAVROS --> PX4
```

### Ego State Feedback (Subscribe)

| Topic | Type | Content | Frame |
|-------|------|---------|-------|
| `/{ns}/common_frame/odom` | Odometry | Position + velocity (primary) | ENU-FLU |
| `/{ns}/mavros/local_position/odom` | Odometry | Fallback when `use_common_frame=false` | ENU-FLU |
| `/{ns}/mavros/imu/data` | Imu | Angular vel, linear accel | ENU-FLU body |
| `/{ns}/gimbal_state_rpy_rad` | Vector3 | Gimbal joints (body frame) | FLU body |
| `/{ns}/yolo_result_vision` | Detection2DArray | YOLO detections | image |
| `/{ns}/zoom_level` | Float32 | Current zoom factor | - |
| `/{ns}/target_rays_w` | TargetRayArray | Camera bearing rays (world) | ENU |

### Cross-Agent Feedback (Subscribe, absolute paths)

| Topic | Type | Content | QoS |
|-------|------|---------|-----|
| `/{peer}/common_frame/odom` | Odometry | Peer position + velocity | BEST_EFFORT |
| `/{peer}/combined_ang_vel_w` | Vector3Stamped | Peer body+gimbal angular velocity (world) | BEST_EFFORT |
| `/{peer}/yolo_result_active` | Bool | Peer detection active (True=tracking) | BEST_EFFORT |
| `/{peer}/zoom_level` | Float32 | Peer zoom factor | BEST_EFFORT |
| `/{peer}/target_rays_w` | TargetRayArray | Peer camera bearing rays | RELIABLE |
| `/{peer}/chosen_target_pose` | PoseWithCovarianceStamped | Peer's selected target (for ray matching, **deferred: should move to mas_tracker**) | RELIABLE |

### Mission State Machine

```
Operator: /mission_state_cmd (global Int8, RELIABLE + transient_local)
  0 → IDLE:     no commands forwarded
  1 → TRACKING: gimbal angle commands forwarded
  2 → MISSION:  policy commands forwarded, offboard enters POLICY state
```

Offboard transitions to POLICY state when: `armed` AND `OFFBOARD mode` AND `at waypoint` AND `mission_state == 2`.

---

## 6. Coordinate Frames

| Frame | Convention | Where Used |
|-------|-----------|------------|
| World | ENU (East-North-Up) | All positions, velocities, observations |
| Body | FLU (Forward-Left-Up) | Angular velocity, acceleration, gimbal joints |
| PX4 internal | NED-FRD | MAVROS converts automatically |

### Quaternion Conventions

| System | Format | Example |
|--------|--------|---------|
| ROS2 (geometry_msgs) | (x, y, z, w) scalar-last | Standard ROS |
| IsaacLab / Isaac Sim | (w, x, y, z) scalar-first | `euler_xyz_from_quat()` expects wxyz |
| Policy observation | Euler RPY (rad) | Converted from quaternion, wrapped to [-pi, pi] |

### Euler Extraction (for obs indices 6-8)

```python
from scipy.spatial.transform import Rotation
# ROS2 quaternion (x, y, z, w) → Euler RPY
r = Rotation.from_quat([q.x, q.y, q.z, q.w])
roll, pitch, yaw = r.as_euler('xyz')  # intrinsic XYZ
# Wrap to [-pi, pi]
roll  = (roll + pi) % (2*pi) - pi
pitch = (pitch + pi) % (2*pi) - pi
yaw   = (yaw + pi) % (2*pi) - pi
```

### Gimbal YAW_JOINT_OFFSET

The gimbal mesh is rotated 90 degrees in the USD. Joint readout must be corrected:
```
gimbal_yaw_body = joint_yaw_raw - (-pi/2)  =  joint_yaw_raw + pi/2
```
This makes `gimbal_yaw_body = 0` mean "camera points forward along body X-axis."

---

## 7. Critical Constants

| Constant | Value | Source |
|----------|-------|--------|
| `max_lin_vel` | 10.0 m/s | `iris_ma_env6_test_cfg.py:272` |
| `max_yaw_rate` | pi/4 = 0.7854 rad/s (45 deg/s) | `iris_ma_env6_test_cfg.py:278` |
| `max_gimbal_rate` | 2*pi = 6.2832 rad/s (360 deg/s) | `iris_ma_env6_test_cfg.py:281` (internal to LOS controller) |
| Policy frequency | 25 Hz (dt = 0.04s) | decimation=4 at 100 Hz sim |
| GRU hidden size | 64 | `skrl_mappo_rnn_cfg.yaml:12` |
| GRU layers | 1 | `skrl_mappo_rnn_cfg.yaml:11` |
| FC hidden size | 64 | `skrl_mappo_rnn_cfg.yaml:10` |
| Obs dim (2 agents) | 52 | 30 + 16*(2-1) + 6 |
| Action dim | 7 | `iris_ma_env6_test_cfg.py:115` |
| YAW_JOINT_OFFSET | -pi/2 | Gimbal mesh correction |
| Bbox image size | (480, 640) default | Camera resolution for bbox normalization |
| Shared policy | Yes | All agents share one policy + separate RNN hidden states |

### Delay System Parameters (training defaults)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Ego motion latency | 5ms first-order lag | Fast proprioception |
| Ego detection latency | 100ms +/- 15ms | NN inference time |
| Inter-agent motion latency | 500ms +/- 80ms | Network communication |
| Bbox dropout rate | 5% per step | Random detection failures |
| Position noise | 0.1 m | Gaussian |
| Velocity noise | 0.05 m/s | Gaussian |
| Orientation noise | 0.6 deg | Gaussian |

---

## 8. Key Source Files

| File | Role |
|------|------|
| `iris_ma6/iris_ma_env6_test.py:1431-1668` | Observation composition |
| `iris_ma6/iris_ma_env6_test.py:518-539` | Action scaling |
| `iris_ma6/iris_ma_env6_test_cfg.py` | All environment constants |
| `iris_ma6/agents/skrl_mappo_rnn_cfg.yaml` | Model + training hyperparameters |
| `scripts/.../skrl/mappo_rnn.py` | MAPPORNNPolicy / MAPPORNNValue classes |
| `scripts/.../skrl/play_iris_mappo_rnn.py` | Reference inference loop |
| `mas_policy/policy_node.py` | Per-vehicle inference node (25 Hz loop) |
| `mas_policy/observation_assembler.py` | Ego + peer observation vector assembly |
| `mas_policy/policy_loader.py` | SKRL checkpoint loading + standalone networks |
| `mas_policy/cbf_filter.py` | Deployment CBF safety filter |
| `mas_policy/utils.py` | Coordinate frame math (euler, gimbal ray, quat rotate) |
| `mas_policy/action_publisher.py` | 7D action → ROS2 topics |
| `mas_mission/mission_node.py` | State-gated command routing |
| `mas_offboard/offboard_control.py` | PX4 offboard state machine |
| `gimbal_stabilizer/los_rate_controller.py` | LOS rate → joint angle IK |
