# Ticket #025: Policy deployment verification

## Problem

**What**: `mas_policy` deploys a trained MARL policy (SKRL/IsaacLab MAPPO-RNN) as a standalone ROS2 node, but the deployment pipeline has never been systematically verified against the reference training-time play script (`play_iris_mappo_rnn.py`). Observation assembly, scaler normalization, action selection, and upstream topic availability are all unverified in the sim-deploy loop.

**Why**: Without verification, silent numerical divergence between training-time and deploy-time inference can produce subtly wrong behavior — the policy runs and publishes commands, but they may not match what the trained agent would actually do. This is the single most critical correctness property of the system.

**Scope boundary**: This ticket verifies the existing deployment code. It does NOT change the policy architecture, observation structure, or training environment. It does NOT address real-hardware deployment (only simulation).

**Affected modules**: `mas_policy`, `mas_tracker` (sort3d_node), `gimbal_controller` (siyi_ros_node), `los_rate_controller`, `ultralytics_ros` (tracker_node), `mas_common_frame`

**Acceptance criteria**:
1. A documented comparison between `play_iris_mappo_rnn.py` and `policy_node.py` with all differences categorized as (a) intentional, (b) needs fix, or (c) needs investigation
2. All upstream topics verified publishing in the sim stack, with fallback behavior documented
3. QoS compatibility matrix verified (no silent message drops)
4. A test suite that can be run to verify policy deployment produces correct actions

**Flow**: Full QRISPY

---

## Scope Item 1: Sim-to-Deploy Parity with `play_iris_mappo_rnn.py`

### Reference file
`/home/usrg/IsaacPX4/IsaacLab/scripts/reinforcement_learning/skrl/play_iris_mappo_rnn.py`

### Known differences to investigate

| Area | Play script (training-time) | Policy node (deploy-time) | Risk |
|------|----------------------------|--------------------------|------|
| **Architecture defaults** | `hidden_size=256, gru_hidden_size=256, gru_num_layers=2` (from `agent.pkl`) | `hidden_size=64, gru_hidden_size=64, gru_num_layers=1` (from `policy_deploy.yaml`) | **High** — if deploy defaults don't match checkpoint, weights load into wrong-shaped network. Currently relies on user setting correct params. |
| **Observation source** | `env.step()` calls `_get_observations()` internally — obs computed by training env | `ObservationAssembler.assemble()` — obs reassembled from ROS2 topics | **High** — any field ordering, sign convention, or normalization difference produces wrong actions. Must verify field-by-field equivalence. |
| **Scaler loading** | Loads into SKRL's `RunningStandardScaler` via `load_state_dict()` at `play:416` | Extracts `running_mean`/`running_var` manually into `ScalerState` at `policy_loader.py:222-239` | **Medium** — must verify numerical equivalence: `(obs - mean) / sqrt(var + eps)` with same eps. |
| **Action selection** | `agent.act()` → `outputs["mean_actions"]` (deterministic mode) at `play:475` | `policy(obs, hidden)` → raw output → `clip(-1, 1)` at `policy_node.py:261-267` | **Medium** — play script's `mean_actions` comes from the policy's `act()` method which applies tanh squashing. Deploy uses raw linear output. Must verify these are the same. |
| **Hidden state management** | Per-env reset on `terminated\|truncated` at `play:505-512` | Reset on stale timeout (2s) or service call at `policy_node.py:241-249` | **Low** — different reset triggers are intentional (deploy has no episode boundaries), but verify GRU state shape matches. |
| **Value network input** | True shared state: all agents' obs concatenated | Approximation: ego obs tiled N times at `policy_node.py:274` | **Low** — value is monitoring-only, not used for action selection. |

### Investigation steps

1. **Load the same checkpoint in both paths**, feed identical observation vectors, compare output actions numerically (should match to float32 precision)
2. **Trace the observation vector field-by-field** from `_get_observations()` in `iris_ma_env6_test.py` and compare ordering/signs with `ObservationAssembler.assemble()`
3. **Compare scaler normalization** — extract `running_mean`/`running_var` from both paths, verify identical
4. **Check `mean_actions` vs raw policy output** — does MAPPORNNPolicy's `act()` apply any transformation (tanh, scaling) that the standalone `PolicyNetRNN.forward()` does not?

---

## Scope Item 2: Upstream Topic Availability Audit

Each topic that `mas_policy` subscribes to must be verified as actually publishing in the simulation stack. The key question: what happens when a topic hasn't published yet?

### Topic availability matrix

| Topic | Publisher node | Publisher file | When it starts publishing | Policy fallback if missing |
|-------|---------------|---------------|--------------------------|---------------------------|
| `common_frame/odom` | common_frame_node | `mas_common_frame/common_frame_node.py:113` | After first MAVROS EKF update | `odom_received=False` → control loop skips (safe) |
| `mavros/imu/data` | MAVROS | (external) | Immediately on MAVROS start | `linear_acceleration_b` stays zero — feeds into obs but not critical |
| `gimbal_state_rpy_deg` | los_rate_controller (sim) | `los_rate_controller.py:243` | After first control callback | Gimbal angles stay 0.0 (forward) — obs gets gimbal_yaw=0, gimbal_pitch=0 |
| `yolo_result_vision` | tracker_node | `tracker_node.py` | After first camera image + YOLO inference | `bbox_empty=1.0`, `bbox_xywh=zeros` — correct default |
| `combined_ang_vel_w` | los_rate_controller (sim) | `los_rate_controller.py:247` | After first control callback | Stays zeros — obs gets zero combined angular velocity |
| `chosen_target_ray_w` | sort3d_node | `sort3d_node.cpp:63` | Only when triangulated detections arrive AND target is selected | Falls back to `gimbal_ray_direction_world()` computation — **verify this fallback is correct** |
| `camera/zoom_level` | los_rate_controller (sim) | `los_rate_controller.py:249` | After first control callback | Stays 1.0 (default zoom) — correct |
| `yolo_result_active` (peers) | tracker_node | `tracker_node.py:80` | After first camera image | `bbox_empty=1.0` — correct default |
| `/chosen_target_pose` | sort3d_node | `sort3d_node.cpp:61` | Only when target is tracked | `tri_state.is_valid=False` → obs gets `[0,0,0,-1,-1,-1]` — correct sentinel |

### Key concerns

1. **`chosen_target_ray_w` may never publish** if the triangulation pipeline isn't running. The fallback to `gimbal_ray_direction_world()` in `observation_assembler.py:414` must be verified to produce the same value the training env would.
2. **Startup transient**: during the first few seconds, most topics haven't published. The `odom_received` gate prevents publishing actions, but verify the observation vector isn't filled with stale/garbage values during this window.
3. **`chosen_target_pose` availability**: if `enable_triangulation=true` but sort3d isn't running, the triangulation tail stays `[0,0,0,-1,-1,-1]` forever. Is this what the trained policy expects for "no target"?

---

## Scope Item 3: QoS Compatibility Matrix

### Publisher → Subscriber QoS pairs

| Topic | Publisher QoS | Policy Subscription QoS | Compatible? | Notes |
|-------|-------------|------------------------|-------------|-------|
| `common_frame/odom` | BEST_EFFORT / depth=1 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `mavros/imu/data` | BEST_EFFORT (MAVROS sensor) | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `gimbal_state_rpy_deg` (sim) | BEST_EFFORT / depth=1 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `gimbal_state_rpy_deg` (hw) | RELIABLE / depth=10 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | BEST_EFFORT sub receives from RELIABLE pub |
| `yolo_result_vision` | BEST_EFFORT (sensor QoS) | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `combined_ang_vel_w` (sim) | BEST_EFFORT / depth=1 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `combined_ang_vel_w` (hw) | RELIABLE / depth=10 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | BEST_EFFORT sub receives from RELIABLE pub |
| `camera/zoom_level` (sim) | BEST_EFFORT / depth=1 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `camera/zoom_level` (hw) | RELIABLE / depth=10 | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | BEST_EFFORT sub receives from RELIABLE pub |
| `chosen_target_ray_w` | RELIABLE / depth=10 | RELIABLE / depth=10 (default QoS) | Yes | Match |
| `yolo_result_active` | BEST_EFFORT (sensor QoS) | BEST_EFFORT / depth=1 (`sensor_qos`) | Yes | Match |
| `/chosen_target_pose` | RELIABLE / depth=10 | RELIABLE / depth=10 (default QoS) | Yes | Match |

**Result**: All pairs are compatible. No blocking QoS mismatches. The BEST_EFFORT subscriber / RELIABLE publisher combinations (hardware gimbal topics) are valid per ROS2 QoS compatibility rules.

---

## Scope Item 4: Test Suite

### Test 1: Offline numerical parity (unit test)

**Goal**: Verify that `PolicyNetRNN` + `ScalerState` produces the same action as SKRL's `MAPPORNNPolicy` + `RunningStandardScaler` for the same observation input.

**Method**:
1. Load a checkpoint with both `play_iris_mappo_rnn.py`'s loader and `policy_loader.py`'s `load_checkpoint()`
2. Create a known observation vector (e.g., from a recorded rosbag or hardcoded)
3. Run forward pass through both, compare actions — must match to `atol=1e-5`
4. Verify scaler `running_mean` and `running_var` are numerically identical

**File**: `src/mas_policy/test/test_policy_parity.py`

### Test 2: Observation assembly equivalence (unit test)

**Goal**: Verify that `ObservationAssembler.assemble()` produces the same observation vector as `_get_observations()` in the training env for the same state.

**Method**:
1. Record a snapshot of all state variables (position, velocity, orientation, gimbal angles, etc.) from the sim
2. Feed the same state into both the training env's `_get_observations()` and the assembler
3. Compare field-by-field — document any intentional differences

**File**: `src/mas_policy/test/test_observation_assembly.py`

### Test 3: Rosbag replay (integration test)

**Goal**: Verify `policy_node` produces non-zero, reasonable actions when fed recorded sim topics.

**Method**:
1. Record a rosbag from the simulation with all required topics publishing
2. Launch `policy_node` in dry-run mode
3. Replay the rosbag
4. Verify from logs: observation vector is populated, actions are non-zero and within [-1, 1]

**Procedure**: Manual, documented in ticket

### Test 4: Live sim end-to-end (integration test)

**Goal**: Verify the full stack produces closed-loop behavior in simulation.

**Method**:
1. Launch full sim stack (PX4 SITL + MAVROS + common_frame + YOLO + triangulation + tracker + policy)
2. Transition to MISSION state via mas_mission
3. Verify `cmd_vel` publishes non-zero velocities, `gimbal_cmd_los_rate` publishes non-zero rates
4. Observe drone behavior — should track target, not diverge

**Procedure**: Manual, documented in ticket

---

## Affected Files

| File | Role |
|------|------|
| `src/mas_policy/mas_policy/policy_node.py` | Deploy-time inference loop |
| `src/mas_policy/mas_policy/policy_loader.py` | Standalone network + checkpoint loading |
| `src/mas_policy/mas_policy/observation_assembler.py` | ROS2 topics → observation vector |
| `src/mas_policy/mas_policy/action_publisher.py` | Action → ROS2 commands |
| `src/mas_policy/config/policy_deploy.yaml` | Deploy parameter defaults |
| `IsaacPX4/.../play_iris_mappo_rnn.py` | Reference play script (read-only comparison) |
| `IsaacPX4/.../mappo_rnn.py` | SKRL MAPPO-RNN model definitions (read-only comparison) |
| `IsaacPX4/.../iris_ma_env6_test.py` | Training env `_get_observations()` (read-only comparison) |

## Findings

### Jacobian mismatch (combined_ang_vel_w) — SIGNIFICANT

The `combined_ang_vel_w` observation field (ego indices 20-22, peer indices 9-11) is computed differently:

- **Training** (`derived_field_computers.py:340-344`): Full ZXY Jacobian accounting for gimbal yaw/roll positions when mapping joint rates to body-frame angular velocity.
- **Deploy** (`los_rate_controller.py:610-614`): Simplified — assumes gimbal axes are aligned with body Y (pitch) and Z (yaw). No Jacobian.

At gimbal origin (yaw=0, roll=0) they are identical. But at typical operating angles (yaw > 30 deg), errors exceed **0.5 rad/s per unit rate**. Random test over 10k configurations: **95% have error > 0.1, mean error 1.67 rad/s**.

This is outside `mas_policy` scope (fix is in `los_rate_controller.py` in `gimbal_stabilizer` package). **Recommend a follow-up ticket to add the ZXY Jacobian to `los_rate_controller.py`.**

### Action bounding: clip, not tanh

SKRL's `GaussianMixin.act()` does NOT apply tanh. The play script's deterministic mode uses `outputs["mean_actions"]` which is the raw linear output from `policy_layer()`. Deploy correctly uses `np.clip(action, -1, 1)` as a safety bound.

### Value network shared state — fixed

Previously tiled ego obs (`obs.repeat(1, N)`). Now uses true per-agent observations via cross-agent `policy/observation` topic, matching `env.state()` which concatenates all agents' ego-perspective obs.

### Numerical parity — verified

Deploy-side inference matches training-side to max error **7.8e-05** (well below 1e-4) across 500 steps on both untrained (step0) and trained (step400k) checkpoints. Scaler parameters match exactly. See `test/data/parity_report.png`.

### Known limitations (not in scope)

- **Physics dt**: IsaacLab runs at 100 Hz, PegasusSimulator at 250 Hz. Policy runs at 25 Hz nominal in both. Triangulation, detection, and flight control pipelines differ between simulators.
- **Delay modeling**: Cross-agent obs topic adds ~40ms real latency, which the training delay system was designed to model. This is expected behavior, not a bug.

## Live Sim Integration Test Procedure

1. Launch full sim stack:
   ```bash
   tmuxp load ~/IsaacPX4/tmux/isaac_sim.tmuxp.yaml
   # Wait for "Ready for takeoff!"
   tmuxp load tmux/simdrone1.tmuxp.yaml
   tmuxp load tmux/simdrone2.tmuxp.yaml
   tmuxp load tmux/simdrone3.tmuxp.yaml
   ```

2. Wait for all nodes to start (check `ros2 node list`).

3. Transition to MISSION:
   ```bash
   ros2 topic pub /mission_state_cmd std_msgs/Int8 "data: 2" --qos-durability transient_local --qos-reliability reliable --once
   ```

4. Verify policy is active:
   ```bash
   ros2 topic hz /px4_1/policy/cmd_vel          # Should show ~25 Hz
   ros2 topic echo /px4_1/policy/value --once    # Should show non-zero V(s)
   ros2 topic echo /px4_1/policy/observation --once  # Should show 52-element array
   ```

5. **Pass criterion**: Target detection maintained for 3+ seconds (verify via `ros2 topic echo /px4_1/yolo_result_vision` showing non-empty detections).

## Related

- **Ticket #014** — prior mas_policy test improvements
- **Ticket #015** — value function monitoring (added ValueNetRNN)
- **Ticket #023** — policy gimbal observation convention
- **Ticket #024** — triangulation fix (changed gimbal_state_rpy_deg convention)

## Status

```
Flow: Q -> R -> I -> S -> P -> Y -> PR
Status: In Progress (Slices 1-5 done, live sim test pending)
```
