## Ticket #013: mas_operator — unified monitoring and command interface

### What
Build `mas_operator` as a ROS2 Python package that aggregates fleet-wide state into a single operator console and provides mission command authority. The node subscribes to per-vehicle state topics, computes derived metrics (AoI, covariance magnitude, inter-agent distance, fleet consensus), raises alerts, and publishes mission state commands.

### Why
The current operator interface (`src/scripts/operator.py`) is a bare-bones CLI that only publishes mission state commands and reads back `mission_state`. It has no data aggregation, no health monitoring, no alert logic — the operator must mentally integrate information from multiple terminal panes and RViz. As fleet size grows beyond 2 agents, this becomes untenable. The system spec (`mas_operator/doc/system_spec.md`) defines the full information architecture; this ticket implements the MVP subset.

### Spec reference
`src/mas_operator/doc/system_spec.md` — authoritative. This ticket implements the **MVP** scope defined there (§ MVP vs Future). Do not duplicate spec content here.

### Scope

**In scope (MVP):**
1. **Package scaffolding** — `package.xml`, `setup.py`, `setup.cfg`, launch file, `CONTEXT.md`
2. **Data aggregation node** (`operator_node`) — subscribes to all perception-layer topics per vehicle:
   - `/{veh}/mission_state` (Int8, RELIABLE transient_local)
   - `/{veh}/common_frame/odom` (Odometry, BEST_EFFORT)
   - `/{veh}/gimbal_state_rpy_deg` (Vector3, BEST_EFFORT)
   - `/{veh}/mavros/state` (mavros_msgs/State, RELIABLE)
   - `/{veh}/chosen_target_pose` (PoseWithCovarianceStamped)
   - `/{veh}/yolo_result_vision` (Detection2DArray, BEST_EFFORT)
   - `triangulated_points` (mas_msgs/TriangulatedPointArray)
   - `/{veh}/tracked_objects/class_{i}` (Detection3DArray)
3. **Derived metrics** (comprehension layer) — computed at `status_rate_hz`:
   - Data freshness / AoI proxy per agent per topic
   - Covariance magnitude (trace) from triangulation
   - Triangulation validity (message rate + content check)
   - Inter-agent pairwise distances
   - Fleet consensus (all agents same mission_state?)
   - Observation geometry (baseline-to-range ratio)
4. **Alert engine** — evaluate alert conditions (spec § Alert Conditions) and surface them:
   - Fleet state mismatch (Warning)
   - Communication stale / lost (Warning / Critical)
   - Triangulation lost (Critical)
   - Covariance spike (Warning)
   - Collision proximity (Critical)
   - Agent disarmed during TRACKING/MISSION (Critical)
5. **Command interface** — publish:
   - `/mission_state_cmd` (Int8, RELIABLE transient_local) — IDLE(0) / TRACKING(1) / MISSION(2)
   - `/{veh}/set_auto_pick_mode` (Int8) — per-vehicle auto-pick toggle
6. **Text UI** — structured terminal display (curses or rich) showing fleet status table, active alerts, derived metrics, and keyboard-driven command input
7. **Parameters** — all 9 parameters from spec § Parameters

**Out of scope (future tickets):**
- Camera feed subscription / display
- Projection-layer SA (covariance trends, track loss prediction)
- 3D visualization / RViz panel
- Per-agent mission override
- Manual gimbal control
- Policy reset service calls
- Web dashboard

### Design sketch

```
operator_node (Python, rclpy)
  ┌─────────────────────────────────────────────────────┐
  │  Per-vehicle state cache (dict[str, VehicleState])  │
  │    .mission_state     .mavros_state                 │
  │    .odom              .gimbal_rpy                   │
  │    .chosen_target     .detections                   │
  │    .last_heard[topic] (timestamps for AoI)          │
  ├─────────────────────────────────────────────────────┤
  │  Global state cache                                 │
  │    .triangulated_points    .tracked_objects          │
  ├─────────────────────────────────────────────────────┤
  │  Metrics timer (status_rate_hz)                     │
  │    → compute AoI, covariance, distances, consensus  │
  │    → evaluate alert conditions                      │
  │    → update display                                 │
  ├─────────────────────────────────────────────────────┤
  │  Display (curses or rich)                           │
  │    Fleet status table:                              │
  │      VEH  | STATE    | ARMED | MODE   | AoI   |    │
  │      px4_1| TRACKING | Yes   | OFFBRD | 42ms  |    │
  │      px4_2| TRACKING | Yes   | OFFBRD | 38ms  |    │
  │    Triangulation: pos=(1.2, 3.4, 5.6) cov=0.82     │
  │    Alerts: [!] Covariance spike (0.82 > 0.5)        │
  │    Keys: [1]IDLE [2]TRACK [3]MISSION [a]AutoPick    │
  └─────────────────────────────────────────────────────┘

  Params:
    vehicles: ["px4_1", "px4_2"]
    aoi_warn_ms: 500.0
    aoi_critical_ms: 2000.0
    cov_warn_threshold: 5.0
    safety_distance_m: 9.5
    tri_timeout_s: 1.0
    status_rate_hz: 2.0
    enable_camera_feeds: false
    num_object_classes: 1
```

### File plan
```
src/mas_operator/
  package.xml
  setup.py
  setup.cfg
  resource/mas_operator
  CONTEXT.md
  mas_operator/
    __init__.py
    operator_node.py        # Main node: subscriptions, metrics, alerts
    vehicle_state.py        # VehicleState dataclass + per-vehicle cache
    metrics.py              # Derived metric computations
    alerts.py               # Alert condition evaluation
    display.py              # Terminal UI rendering (curses/rich)
  launch/
    operator.launch.py
  config/
    operator.yaml           # Default parameter values
```

### Dependencies
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `vision_msgs`
- `mavros_msgs` (hard depend)
- `mas_msgs` (TriangulatedPointArray)
- `visualization_msgs` (MarkerArray for RViz)

### Acceptance criteria
- [x] `colcon build --packages-select mas_operator` succeeds
- [x] Node launches and subscribes to all listed topics for configured vehicles
- [x] Terminal display shows per-vehicle state table updated at status_rate_hz
- [x] AoI, covariance, inter-agent distance metrics computed correctly
- [x] Alert conditions fire when thresholds are crossed
- [x] Keyboard commands publish correct mission_state_cmd values
- [x] Rosbag playback (`bag/mas_bag_20260402_004927`) produces meaningful display output
- [x] Node handles missing vehicles / missing topics gracefully (no crash, shows stale/disconnected)
- [x] RViz markers: agent spheres, target markers with chosen highlight, inter-agent AoI lines
- [x] Manual target selection: `t` + ID + Enter publishes `set_target_id` to all vehicles

### Changes beyond original scope
- Added `set_target_id` subscription to `mas_tracker/sort3d_node` (Int8, sets target_id_ and disables auto-pick) to support manual target selection from operator
- Added RViz chosen-target highlighting (green sphere + `[SEL]` label) in both curses and markers
- Changed `mavros/state` subscription QoS to BEST_EFFORT for rosbag compatibility
- Increased default `tri_timeout_s` from 1.0 to 3.0 (triangulation publishes at 0.5 Hz)
