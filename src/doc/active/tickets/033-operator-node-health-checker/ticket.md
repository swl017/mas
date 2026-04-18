## Ticket #033: Node health checker for mas_operator

### What
Extend `mas_operator` to monitor the liveness and health of all MAS nodes across the fleet, displaying node-level status (running/stale/dead) in the terminal UI and as an alert condition. The operator should know when a specific node has crashed, stopped publishing, or is falling behind expected publish rates.

### Why
The current `mas_operator` (ticket #013) monitors topic-level freshness (AoI on odom) but doesn't track **which node** is unhealthy when something goes wrong. If `triangulation_node` crashes, the operator sees "tri_lost" but doesn't know the root cause. If `siyi_gimbal_node` stops publishing, the operator sees stale gimbal data but can't tell if it's the node or the hardware. For remote drone operations, distinguishing node-level failures from topic-level issues is essential for quick diagnosis.

### Scope

**In scope:**
1. **Node liveness detection** — for each expected node per vehicle, track whether it's publishing on its primary topic(s) at the expected rate
2. **Per-node status** — classify as `OK` (publishing at expected rate), `STALE` (slower than expected), `DEAD` (no publications for `dead_timeout_s`)
3. **Display integration** — add a "NODE HEALTH" table in the curses UI showing per-vehicle per-node status
4. **RViz integration** — color-code the agent sphere based on overall node health (green all OK, yellow any stale, red any dead)
5. **New alert condition** — `node_dead_{veh}_{node}` (CRITICAL) when a critical node stops publishing
6. **Config** — declare expected nodes and their heartbeat topics + rates in `operator.yaml`

**Critical nodes to monitor per drone (initial list):**
- `mavros_node` → `/{veh}/mavros/state` at ~1 Hz
- `common_frame_node` → `/{veh}/common_frame/odom` at ~50 Hz
- `siyi_gimbal_node` → `/{veh}/gimbal_state_rpy_deg` at ~25 Hz
- `triangulation_node` → `/{veh}/triangulated_points` at ~0.5 Hz
- `sort3d_tracking_node` → `/{veh}/tracked_objects/class_0` at ~0.5 Hz
- `mission_node` → `/{veh}/mission_state` at ~1 Hz
- `offboard_control` → `/{veh}/mavros/setpoint_velocity/cmd_vel` at ~100 Hz
- `policy_node` → `/{veh}/policy/cmd_vel` at ~25 Hz (only when in MISSION state)
- `ultralytics_tracker_node` → `/{veh}/yolo_result_vision` at variable rate (depends on camera)

**Out of scope:**
- Process-level monitoring (PID, CPU, memory) — requires SSH/systemd integration, post-MVP
- Restarting crashed nodes — not an operator responsibility, just reporting
- ROS2 lifecycle state monitoring — most MAS nodes don't use lifecycle, post-MVP
- Diagnostics messages (`diagnostic_msgs/DiagnosticArray`) — nodes don't currently publish these

### Design sketch

```
NODE HEALTH  (per vehicle)
VEH    | mavros | common | gimbal | tri   | sort3d | mission | offboard | policy
-------+--------+--------+--------+-------+--------+---------+----------+--------
px4_1  |   OK   |   OK   |   OK   |  OK   |   OK   |   OK    |    OK    |   OK
px4_2  |   OK   |  STALE |   OK   |  DEAD |   OK   |   OK    |    OK    |   OK

ALERTS
[CRIT] px4_2: triangulation_node DEAD (no publish for 8.2s)
[WARN] px4_2: common_frame_node STALE (5 Hz vs 50 Hz expected)
```

Config schema (additions to `operator.yaml`):
```yaml
node_health:
  stale_ratio: 0.3        # flag STALE if rate < 30% of expected
  dead_timeout_s: 5.0     # flag DEAD if no publish for 5s
  nodes:
    - name: mavros_node
      topic: mavros/state
      expected_hz: 1.0
      critical: true
    - name: common_frame_node
      topic: common_frame/odom
      expected_hz: 50.0
      critical: true
    # ... etc
```

Implementation structure:
- `mas_operator/node_health.py` — `NodeHealthMonitor` class with rate estimation (sliding window)
- `operator_node.py` — add subscriptions for each configured node topic (with counting callback)
- `display.py` — add `_draw_node_health_table()`
- `alerts.py` — add `node_dead_*` and `node_stale_*` alert evaluation
- `markers.py` — color-code agent sphere based on aggregate health

### Dependencies
- Builds on mas_operator (ticket #013) — same package, no new packages needed
- No changes to other packages (purely a ground-station monitoring extension)

### Acceptance criteria
- [ ] Configuration-driven: adding/removing monitored nodes requires only `operator.yaml` changes
- [ ] Node health table renders in curses UI alongside fleet status table
- [ ] Rate estimation handles variable publish rates (not just fixed)
- [ ] DEAD alert fires within `dead_timeout_s` of node stopping
- [ ] STALE detection distinguishes from expected low-rate nodes (e.g., triangulation at 0.5 Hz is OK)
- [ ] Agent RViz sphere color reflects worst node status (green/yellow/red)
- [ ] Handles node start-up grace period (don't alert for first N seconds before any data arrives)
- [ ] Policy node health check respects mission state (only expect publications in MISSION)
- [ ] Build and launch tested against live sim with all drone nodes running
- [ ] Manually killing a node in tmux is reflected in the UI within `dead_timeout_s`

### Open questions for Q stage
- Should the health check use topic publisher count (`get_publishers_info_by_topic`) as a secondary signal, or is message-arrival timing sufficient? - What is `get_publishers_info_by_topic`? How does it compare to topic timing? Which one works for both the wallclock at real world deploy, and simtime at SITL testing?
- How to handle nodes that legitimately publish at variable rates (e.g., ultralytics depends on camera FPS)? - Even so, we do expect some range of publish rates, so treat it the same as other topics
- Should the health panel be collapsible in the curses display to save terminal space when everything is OK? - No, always show the health panel
- Is the operator the right place for this, or should each drone publish its own aggregated `diagnostic_msgs/DiagnosticArray`? - May be having `diagnostic_msgs` in each node or drone could be more efficient... Which one fits to our needs, given we do need to monitor the inter-agent topic exchange rates and latencies?
