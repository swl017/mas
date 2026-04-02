## Design Document: mas_operator MVP

### Problem statement

The MAS system has no unified operator interface. The existing `scripts/operator.py` only publishes mission commands and reads back state — the operator cannot monitor fleet health, triangulation quality, or alert conditions from a single view. The d2 architecture diagram defines operator edges (mission select, target select, tracked target feedback) that have no working implementation. This package fills that gap as a ground-station node running on a separate machine.

### Proposed approach

`mas_operator` is a single ROS2 node with two threads: the main thread runs `rclpy.spin()` processing all subscriptions and timers, while a daemon thread runs a curses-based terminal UI for display and keyboard input. A shared `FleetState` object (protected by a lock) bridges the two threads.

The node subscribes to per-vehicle topics for each vehicle in its config, caches the latest message and reception timestamp into a `VehicleState` dataclass, and runs a periodic timer at `status_rate_hz` that computes derived metrics, evaluates alert conditions, and updates the display. The curses UI renders a fleet status table, a tracked-targets table, active alerts, and a command key legend. The operator selects targets by track ID and issues mission commands via keyboard.

For 3D spatial context, the node publishes a `visualization_msgs/MarkerArray` containing agent position markers, tracked target markers with text ID labels matching the curses table, and inter-agent AoI line markers. The operator runs RViz on the ground station alongside the curses terminal.

### Key interfaces and data flow

```
                         Ground Station
  ┌──────────────────────────────────────────────────┐
  │  operator_node                                   │
  │                                                  │
  │  Subscriptions (per vehicle, over network):      │
  │    /{veh}/mission_state          (Int8)          │
  │    /{veh}/common_frame/odom      (Odometry)      │
  │    /{veh}/gimbal_state_rpy_deg   (Vector3)       │
  │    /{veh}/mavros/state           (State)         │
  │    /{veh}/chosen_target_pose     (PoseWithCov)   │
  │    /{veh}/tracked_objects/class_{i} (Det3DArray) │
  │    /{veh}/triangulated_points    (TriPtArray)    │
  │                                                  │
  │  Publishers:                                     │
  │    /mission_state_cmd            (Int8)          │
  │    /{veh}/set_auto_pick_mode     (Int8)          │
  │    /operator/markers             (MarkerArray)   │
  │                                                  │
  │  ┌────────────┐    FleetState     ┌───────────┐  │
  │  │ Main thread │◄──(lock)────────►│ UI thread │  │
  │  │ rclpy.spin  │   VehicleState[] │ curses    │  │
  │  │ + timer     │   Metrics        │ display + │  │
  │  │             │   Alerts         │ keyboard  │  │
  │  └────────────┘                   └───────────┘  │
  └──────────────────────────────────────────────────┘
          │                              │
          ▼                              ▼
    RViz (markers)              Terminal (status/commands)
```

**Data flow through the node:**

1. **Callbacks** — each subscription callback writes to `FleetState.vehicles[veh]` fields and updates `last_heard[topic_key]` timestamp. Lock held only during the write.
2. **Metrics timer** — fires at `status_rate_hz`. Reads `FleetState` under lock, computes: AoI per agent, covariance trace, triangulation validity, pairwise inter-agent distances, fleet consensus, observation geometry (baseline-to-range). Writes results to `FleetState.metrics`.
3. **Alert evaluation** — runs inside the metrics timer. Compares metrics against parameter thresholds. Active alerts stored in `FleetState.alerts` with expiry timestamp (cleared condition lingers 1 second).
4. **Display update** — the curses thread reads `FleetState` under lock at its own refresh rate and renders the terminal UI.
5. **Keyboard input** — curses `nodelay` mode polls for keypresses. Command keys publish via the node's publishers (thread-safe in rclpy).
6. **Marker publishing** — runs inside the metrics timer. Builds MarkerArray from vehicle positions and tracked targets, with text labels matching the target table IDs.

**Vehicle config loading** — matches the existing `OpaqueFunction` + `yaml.safe_load()` pattern from `mas_mission`. The launch file reads `config/operator.yaml`, extracts the vehicle namespace list, and passes it as a parameter to the single operator node.

**AoI definition and visualization:**

AoI measures cross-agent data latency (not operator-to-agent). Agent 0's AoI = staleness of data received from other agents. The operator approximates this via `now - msg.header.stamp` for each agent's `common_frame/odom` (same DDS network as the agents, so a reasonable proxy).

In RViz, AoI is visualized as 2-way line markers connecting each agent pair:
- Color-coded: green (healthy) → yellow (`> aoi_warn_ms`) → red (`> aoi_critical_ms`)
- Text label on each line showing the AoI value in ms
- For N agents: N*(N-1)/2 lines (2 agents = 1 line, 3 agents = 3 lines)

**QoS matching:**

| Subscription | QoS | Matches |
|---|---|---|
| `/{veh}/mission_state` | RELIABLE, TRANSIENT_LOCAL, depth=1 | mission_node publisher |
| `/{veh}/common_frame/odom` | BEST_EFFORT, VOLATILE, depth=1 | common_frame_node publisher |
| `/{veh}/gimbal_state_rpy_deg` | BEST_EFFORT, VOLATILE, depth=10 | siyi_gimbal_node publisher |
| `/{veh}/mavros/state` | RELIABLE, depth=10 | MAVROS publisher |
| `/{veh}/chosen_target_pose` | RELIABLE, depth=10 | sort3d_node publisher |
| `/{veh}/tracked_objects/class_{i}` | RELIABLE, depth=10 | sort3d_node publisher |
| `/{veh}/triangulated_points` | RELIABLE, depth=10 | triangulation_node publisher |

| Publication | QoS | Matches |
|---|---|---|
| `/mission_state_cmd` | RELIABLE, TRANSIENT_LOCAL, depth=1 | mission_node subscriber |
| `/{veh}/set_auto_pick_mode` | RELIABLE, depth=10 | sort3d_node subscriber |
| `/operator/markers` | RELIABLE, depth=1 | RViz default |

### What this does NOT include

- Camera feed subscription or display (bandwidth-dependent, post-MVP)
- Projection-layer SA: covariance trends, track loss prediction, coverage analysis
- Per-agent mission override (all agents get same command)
- Manual gimbal control or policy reset service calls
- Web dashboard or rqt plugin
- Modifications to any existing package

### Open risks

1. **AoI approximation accuracy** — the operator measures AoI via `now - msg.header.stamp` on the ground station. This is a proxy for cross-agent latency, not a direct measurement. Requires clock synchronization between machines (NTP or PTP). Acceptable for MVP; direct agent-reported AoI could be added later.
2. **Curses + ROS2 logging conflict** — rclpy logger output writes to stdout, which conflicts with curses screen control. Logger output must be redirected or suppressed while curses is active.
3. **Thread safety of rclpy publishers** — calling `publisher.publish()` from the curses thread while the main thread is spinning. This is safe in rclpy (publishers are thread-safe), but worth verifying under load.
