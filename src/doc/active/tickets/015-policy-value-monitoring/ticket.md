## Ticket #015: Policy value function monitoring

### What
Load the SKRL value network alongside the policy network and publish V(s) as a real-time monitoring signal during deployment. Surface it in mas_operator as a per-agent metric.

### Why
The policy currently runs as a black box — there's no way to assess whether it's "confident" in the current state or struggling. The value function V(s) is already trained to estimate expected future returns, making it a free diagnostic. Sudden V drops indicate out-of-distribution states, sim-to-real gaps, or degraded observation quality (e.g., the yaw_joint_offset bug from ticket #014 would have shown as persistently low V).

### Scope boundary
Load and publish the value function only. Do not modify the policy inference path. Do not add V(s) to the observation vector. Do not retrain anything.

### Affected modules
- `mas_policy/` — load value network, run V(obs) in control loop, publish
- `mas_operator/` — subscribe to value topic, display in fleet table

### Tasks

1. **Value network loading** (`policy_loader.py`) — extract `value` state dict and `value_preprocessor` from checkpoint alongside policy. Return a `ValueNetRNN` (same architecture as `PolicyNetRNN` but with `value_layer` → scalar output). The value network shares the same preprocessor as the policy (`state_preprocessor`), but also has its own `value_preprocessor` for the output.

2. **Value inference** (`policy_node.py`) — after policy forward pass, run `V(obs_norm)` with the value network's own GRU hidden state. Publish on `policy/value` (`std_msgs/Float32`).

3. **Operator integration** (`mas_operator/`) — subscribe to `/{veh}/policy/value`, display in fleet status table, optionally add alert condition for V below threshold.

4. **Test** — extend `test_policy_loader.py` to verify value network loads from real checkpoint. Extend integration test to verify V(s) is finite over 50 ticks.

### Acceptance criteria
- [x] Value network loads from real checkpoint without error
- [x] `policy/value` topic publishes at control frequency (25 Hz)
- [x] V(s) is finite over 50 ticks (unit + integration tests)
- [x] V(s) within plausible range during sim e2e
- [x] mas_operator displays per-agent value in fleet table

### Additional work done (beyond original scope)
- Camera resolution auto-adaptation: observation_assembler subscribes to camera_info, normalizes bboxes by actual resolution instead of hardcoded 640x480
- IMU QoS fix: mavros/imu/data subscription changed to BEST_EFFORT to match MAVROS publisher
- Rosbag QoS overrides: created bag/rosbag_qos_overrides.yaml for recording BEST_EFFORT topics
- Operator GRU reset: [r] key and auto-reset on [3] MISSION command via reset_hidden_state service
- Sim time support: policy_deploy.launch.py gains use_sim_time argument, simdrone tmuxp files pass use_sim_time:=true

### Flow
Light (I → S → Y → PR)

### Status
Done
