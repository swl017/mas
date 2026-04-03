## Codebase Research Report — mas_policy

### Module inventory

| File | Lines | Role |
|------|-------|------|
| `policy_node.py` | 295 | Per-vehicle ROS2 node: declares params, loads checkpoint, wires assembler/publisher/CBF, runs 25 Hz control loop |
| `observation_assembler.py` | 501 | Ego+peer ROS2 subscriptions → cached `VehicleState` dataclasses → `assemble()` returns flat numpy obs vector |
| `action_publisher.py` | 85 | 7D action → `cmd_vel` (TwistStamped), `gimbal_cmd_los_rate` (Vector3), `zoom_cmd` (Float32) |
| `policy_loader.py` | 255 | Standalone `PolicyNetRNN`/`PolicyNetMLP` nn.Modules + `load_checkpoint()` + `ScalerState` |
| `cbf_filter.py` | 159 | Distance-based CBF with Gauss-Seidel halfspace projection. Config dataclass computes `D_deploy = D_s + v_max*(tau_delay + tau_px4)` |
| `utils.py` | 214 | NumPy ports of Isaac Lab math: `euler_xyz_from_quat`, `quat_rotate`, `gimbal_ray_direction_world`, `compute_combined_angular_velocity_world`, `wrap_to_pi`, frame conversions |
| `setup.py` | 29 | ament_python package. No `tests_require`. |
| `setup.cfg` | 4 | Only `[develop]` and `[install]` sections. No `[tool:pytest]`. |
| `config/policy_deploy.yaml` | 41 | Default params. `yaw_joint_offset: 1.5708` (+pi/2). |
| `launch/policy_deploy.launch.py` | 123 | Multi-vehicle launch: reads `vehicles.yaml`, one Node per vehicle, `use_mission_gate` remapping. |

### Data models

**VehicleState** (dataclass, `observation_assembler.py:47-80`):
- Motion: `position_w(3)`, `velocity_w(3)`, `orientation_w(4, wxyz)`, `angular_velocity_b(3)`, `linear_acceleration_b(3)`, `motion_timestamp`
- Gimbal: `gimbal_yaw_body`, `gimbal_pitch_body`, `gimbal_yaw_rate`, `gimbal_pitch_rate`, `gimbal_timestamp`
- Detection: `bbox_xywh(4)`, `bbox_empty`, `detection_timestamp`
- Cross-agent: `combined_ang_vel_w(3)`, `chosen_target_ray_w(3|None)`, `zoom_level`
- Flag: `odom_received`

**TriangulationState** (dataclass, `observation_assembler.py:82-89`):
- `position(3)`, `std_dev(3)`, `is_valid`, `timestamp`

**ScalerState** (dataclass, `policy_loader.py:22-31`):
- `running_mean(obs_dim,)`, `running_var(obs_dim,)`, `count`
- `normalize(obs)` = `(obs - mean) / sqrt(var + eps)`

**PolicyNetRNN** (`policy_loader.py:34-88`):
- Architecture: `Linear(obs_dim, hidden)→ReLU→Linear(hidden, hidden)→ReLU→GRU(hidden, gru_hidden)→Linear(gru_hidden, action_dim)`
- Forward: `(obs, hidden) → (mean_action, new_hidden)`. Inference only (no sampling, no log_std).
- `init_hidden()` → zeros `(num_layers, 1, gru_hidden_size)`

**PolicyNetMLP** (`policy_loader.py:91-121`):
- Architecture: `Linear→ELU→Linear→ELU→Linear`

**SKRL checkpoint structure** (from `best_agent.pt`):
- Top-level keys: per-agent (`drone_0`, `drone_1`)
- Each agent contains: `policy`, `value`, `optimizer`, `state_preprocessor`, `shared_state_preprocessor`, `value_preprocessor`
- Scaler uses `running_variance` (not `running_var`); loader handles via fallback aliases

### Observation layout

Authoritative source: `iris_ma_env6_test.py:1517-1573` and `iris_ma_env6_test_cfg.py:547-569`.

**Ego (30D):**
| Index | Dim | Field | Source |
|-------|-----|-------|--------|
| 0-2 | 3 | position_w | odom |
| 3-5 | 3 | velocity_w | odom |
| 6-8 | 3 | euler_rpy (wrapped [-pi,pi]) | odom quaternion |
| 9-11 | 3 | angular_velocity_b | odom twist (training: gyro) |
| 12-14 | 3 | linear_acceleration_b | IMU |
| 15 | 1 | gimbal_yaw_body (0=forward) | joint_pos - YAW_JOINT_OFFSET (training) / gimbal_state_rpy_deg (deploy) |
| 16 | 1 | gimbal_pitch_body | joint_pos (training) / gimbal_state_rpy_deg (deploy) |
| 17-19 | 3 | ray_dir_w (camera→target through bbox) | camera_ray_directions_w (training) / chosen_target_ray_w or gimbal LOS fallback (deploy) |
| 20-22 | 3 | combined_ang_vel_w (body+gimbal sweep) | derived_field_computers (training) / compute_combined_angular_velocity_world (deploy) |
| 23 | 1 | bbox_aoi | sim_time - timestamp_detection |
| 24 | 1 | zoom_level | camera_zoom_level |
| 25-28 | 4 | bbox_xywh (normalized [0,1]) | bboxes_2d / YOLO |
| 29 | 1 | bbox_empty (0/1) | sum(bbox) < eps |

**Inter-agent (16D per peer):**
| Index | Dim | Field |
|-------|-----|-------|
| 0-2 | 3 | position_w |
| 3-5 | 3 | velocity_w |
| 6-8 | 3 | ray_dir_w |
| 9-11 | 3 | combined_ang_vel_w |
| 12 | 1 | zoom_level |
| 13 | 1 | bbox_empty |
| 14 | 1 | data_age |
| 15 | 1 | bbox_age |

**Optional tail (6D):** `tri_pos(3)`, `tri_std(3)`

**Total:** `30 + 16*(N-1)` [+ 6 tri]. For N=2: 46. For N=2 + tri: 52.

### obs_dim computation

Computed in two independent locations:
1. `policy_node.py:100`: `self._obs_dim = 30 + 16 * (num_agents - 1) + (6 if enable_tri else 0)` — uses `num_agents` parameter
2. `observation_assembler.py:495-500`: property `obs_dim` = `30 + 16 * len(self._peer_names)` + 6 — uses actual peer list length

No cross-check between these two values at runtime.

### Checkpoint loading path

`load_checkpoint()` (`policy_loader.py:135-254`):
1. `torch.load(path, weights_only=False)` — loads entire checkpoint
2. Navigates 3 structures: top-level `"policy"` key → per-agent dict → `"checkpoint_modules"` nested → flat fallback
3. Extracts `state_preprocessor` with alias handling (`running_var` / `running_variance`)
4. Constructs fresh `PolicyNetRNN` or `PolicyNetMLP`
5. `load_state_dict(policy_state_dict, strict=False)` — **silently accepts missing keys**
6. On RuntimeError, retries with `"module."` prefix stripped
7. Logs matched/missing/unexpected key counts. Missing only logged as `warning`.
8. Value network keys exist in checkpoint (`value`, `value_preprocessor`) but are **not loaded**.
9. Scaler: if not found, creates identity scaler (mean=0, var=1) with warning.

### yaw_joint_offset chain

| System | Constant | Applied when | Result |
|--------|----------|-------------|--------|
| **Sim mesh** | Joint yaw=0 → camera points body -Y | — | Raw joint needs offset |
| **los_rate_controller** | `YAW_JOINT_OFFSET = +pi/2` | Read: `joint - offset`. Write: `joint + offset`. Publishes `gimbal_state_rpy_deg` with offset removed. | Topic = body-frame, 0=forward |
| **siyi_ros_node** (real) | No offset | Encoder → publish | Topic = body-frame, 0=forward |
| **Training env** | `YAW_JOINT_OFFSET = -pi/2` (imported from gimbal_controller module) | `joint_positions_b[:, 1] - YAW_JOINT_OFFSET` = `joint + pi/2` | Observation = body-frame, 0=forward |
| **Deployment** assembler | `self._yaw_joint_offset = +pi/2` (from param, default 1.5708) | `gimbal_yaw_body - offset` (line 399) | **Double-subtracted**: topic already offset-corrected, then subtracts pi/2 again |
| **gimbal_controller/point_to_region_node.py** | No offset applied | Subscribes to `gimbal_state_rpy_deg`, uses values directly | Confirms topic contract: body-frame, 0=forward |

### Gimbal rate computation

**Ego** (`observation_assembler.py:298-317`):
- Finite-differenced from `gimbal_state_rpy_deg` callbacks: `rate = (new - prev) / dt`
- No angle wrapping applied before differencing
- Guard: `dt > 0.001` only
- Rates stored in `VehicleState.gimbal_yaw_rate`, `gimbal_pitch_rate`
- Used in `compute_combined_angular_velocity_world()` for ego obs [20-22]

**Peers**:
- Subscribe to `/{peer}/combined_ang_vel_w` (pre-computed by los_rate_controller or siyi_ros_node)
- Stored directly in `VehicleState.combined_ang_vel_w`
- No finite-differencing for peers

**Training env** (`iris_ma_env6_test.py`):
- `body_combined_angular_velocity_w` computed by `derived_field_computers.py` using actual joint velocities from sim, not finite-differenced
- Ego and peers both use the same computation path

### CBF filter

`DeploymentCBFFilter` (`cbf_filter.py:51-158`):
- Input: `v_nom(N,3)`, `positions(N,3)`, `neighbor_velocities(N,3)`
- Barrier: `h = ||p_i - p_j||^2 - D_deploy^2`
- Constraint: `2*dp^T*(v_i - v_j) + gamma*h >= 0`
- Projection: closed-form halfspace projection per pair
- Iterations: Gauss-Seidel, default 2 iterations
- No staleness check on peer positions/velocities. Uses whatever is cached, no matter how old.

### Testing infrastructure

- No `test/` or `tests/` directory exists
- No `tests_require` in `setup.py`
- No `[tool:pytest]` in `setup.cfg`
- Other workspace packages (e.g., `gimbal_controller`) use `tests_require=['pytest']`
- All workspace packages use pytest for colcon-discovered lint tests

### Conventions observed

- All ROS2 subscriptions use lambda closures with captured vehicle name for routing to correct VehicleState
- QoS: BEST_EFFORT for sensor-rate topics, RELIABLE for services/commands
- Quaternion convention: wxyz throughout (matching Isaac Lab)
- Frame convention: ENU world, FLU body
- Observation vector: ego first, then peers in declaration order, optional triangulation tail
- Action vector: [0-2] lin_vel (normalized by max_lin_vel), [3] yaw_rate (normalized by max_yaw_rate), [4-5] gimbal az/el rate (pass-through [-1,1]), [6] zoom rate

### Gaps and inconsistencies

1. **yaw_joint_offset double-subtraction**: `gimbal_state_rpy_deg` topic already provides offset-corrected body-frame angles. `observation_assembler.py:399` subtracts `+pi/2` again. Affects obs indices 15 (gimbal_yaw_body) and the gimbal LOS fallback ray computation.

2. **obs_dim dual computation**: `policy_node.py:100` and `observation_assembler.py:495-500` compute obs_dim independently. No assertion that they match.

3. **Ego combined_ang_vel_w computed differently from peers**: Ego uses `compute_combined_angular_velocity_world()` with finite-differenced gimbal rates. Peers use pre-computed topic from `los_rate_controller`/`siyi_ros_node`. In training, both use the same derived_field_computers path.

4. **Ego combined_ang_vel_w not subscribed**: Ego could subscribe to its own `combined_ang_vel_w` topic (published by los_rate_controller) instead of computing from finite-differenced gimbal rates. The topic exists in the ego namespace.

5. **Checkpoint loading `strict=False`**: Missing policy keys are logged as warnings, not errors. A mismatched checkpoint silently produces a partially-initialized network.

6. **Value network not loaded**: Checkpoint contains `value` and `value_preprocessor` per agent. Currently discarded.

7. **Scaler dimension not validated**: `ScalerState.normalize()` will broadcast or error if `running_mean.shape[0] != obs_dim`. No assertion at load time.

8. **bbox_aoi unbounded**: `now - detection_timestamp` grows without limit after detections stop. Training clips implicitly via episode length (500 steps = 20s).

9. **CBF uses all peers regardless of staleness**: `policy_node.py:244-248` builds position/velocity arrays from all peers' cached state. A peer with 30s-old odom is treated as stationary at its last known position.

10. **No test infrastructure**: zero tests, no pytest config, no `tests_require` in setup.py.

11. **Docstring says default yaw_joint_offset is `-pi/2`** (`observation_assembler.py:118`) but the actual default constant is `+pi/2` (`observation_assembler.py:44`). The parameter default in `policy_node.py:54` is `1.5708` (+pi/2).

12. **Gimbal/zoom actions not scaled to physical values**: Training env scales gimbal and zoom actions by their max rates before applying to the sim actuators. `action_publisher.py` passes these through as raw [-1,1]. Published commands should be physical values interpretable on their own (matching the convention for `cmd_vel` which is already in m/s and rad/s). The `gimbal_controller` subscriber side also needs to comply. Deferred to integration phase — not in scope for this ticket.
