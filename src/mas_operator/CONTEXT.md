# mas_operator

## Purpose
Unified operator monitoring and command interface for multi-agent systems. Aggregates per-vehicle state into a single view, computes derived metrics (AoI, covariance, inter-agent distances), evaluates alert conditions, and provides mission command authority. Runs on the ground station, separate from drone companion computers.

## Nodes

### operator_node
**File:** `mas_operator/operator_node.py`
**Pattern:** Timer-driven (metrics computed at `status_rate_hz`), subscriber-driven (state cached on arrival)
**Deployment:** Single instance on ground station (not per-drone)

#### Subscriptions (per vehicle in `vehicles` list)
- `/{veh}/mission_state` (`std_msgs/Int8`) — mission phase (RELIABLE, transient local)
- `/{veh}/common_frame/odom` (`nav_msgs/Odometry`) — agent position/velocity (BEST_EFFORT)
- `/{veh}/gimbal_state_rpy_deg` (`geometry_msgs/Vector3`) — gimbal pointing (BEST_EFFORT)
- `/{veh}/mavros/state` (`mavros_msgs/State`) — armed/mode status (BEST_EFFORT)
- `/{veh}/chosen_target_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — selected target (RELIABLE)
- `/{veh}/tracked_objects/class_{i}` (`vision_msgs/Detection3DArray`) — tracked objects (RELIABLE)
- `/{veh}/triangulated_points` (`mas_msgs/TriangulatedPointArray`) — triangulation output (RELIABLE)
- `/{veh}/policy/value` (`std_msgs/Float32`) — policy value function V(s) (BEST_EFFORT)

#### Publishers
- `/mission_state_cmd` (`std_msgs/Int8`) — global mission command: IDLE(0) / TRACKING(1) / MISSION(2) (RELIABLE, transient local)
- `/{veh}/set_auto_pick_mode` (`std_msgs/Int8`) — global auto-pick toggle (RELIABLE)
- `/{veh}/set_target_position` (`geometry_msgs/PointStamped`) — manual target selection by position (RELIABLE)
- `/operator/markers` (`visualization_msgs/MarkerArray`) — RViz visualization (RELIABLE)

#### Parameters
- `vehicles` (`string[]`, default: `["px4_1", "px4_2"]`) — vehicle namespaces to monitor
- `aoi_warn_ms` (`double`, default: `500.0`) — AoI warning threshold (ms)
- `aoi_critical_ms` (`double`, default: `2000.0`) — AoI critical threshold (ms)
- `cov_warn_threshold` (`double`, default: `5.0`) — covariance trace warning threshold
- `safety_distance_m` (`double`, default: `9.5`) — collision proximity threshold (m)
- `tri_timeout_s` (`double`, default: `3.0`) — seconds without valid triangulation before alert
- `status_rate_hz` (`double`, default: `2.0`) — metrics/display update rate
- `num_object_classes` (`int`, default: `1`) — number of tracked object classes

#### Derived metrics (Comprehension layer)
- **AoI** — per-agent topic freshness (`now - msg.header.stamp`)
- **Cross-agent AoI** — pairwise max odom AoI between agent pairs
- **Covariance trace** — average trace of triangulation 3x3 covariance
- **Triangulation validity** — points received within `tri_timeout_s` with non-empty content
- **Inter-agent distances** — pairwise Euclidean from `common_frame/odom`
- **Fleet consensus** — all agents reporting same `mission_state`
- **Baseline-to-range** — `||p_i - p_j|| / ||target - midpoint||` per agent pair

#### Alert conditions
- Fleet state mismatch (WARNING)
- Communication stale / lost (WARNING / CRITICAL, based on AoI thresholds)
- Triangulation lost (CRITICAL)
- Covariance spike (WARNING)
- Collision proximity (CRITICAL)
- Agent disarmed during TRACKING or MISSION (CRITICAL)

#### Display (curses terminal)
- Fleet status table: VEH, STATE, ARMED, MODE, AoI, V(s), POS, GIMBAL
- Tracked targets table with [SEL] highlight (position-proximity based)
- Active alerts with severity coloring
- Commands: [1]IDLE [2]TRACK [3]MISSION [a]AutoPick ON [d]AutoPick OFF [t]Select Target [q]Quit
- Target selection mode: `t` → type track ID → Enter (publishes position to all drones)
- Headless mode: set `MAS_OPERATOR_NODISPLAY=1`

#### RViz markers (`/operator/markers`)
- Agent spheres (blue) with text labels
- Target spheres (red, or green when selected) with `T{id}` / `T{id} [SEL]` labels
- Inter-agent AoI lines (color-coded green/yellow/red) with ms text labels

## Dependencies
- mas_msgs — TriangulatedPointArray
- mas_mission — mission_state, /mission_state_cmd interface
- mas_common_frame — common_frame/odom
- mas_tracker — tracked_objects, chosen_target_pose, set_auto_pick_mode, set_target_position
- mavros_msgs — State message type

## Key Files
- `mas_operator/operator_node.py` — Main node: subscriptions, timer, command publishers
- `mas_operator/fleet_state.py` — VehicleState, Alert, FleetState dataclasses
- `mas_operator/metrics.py` — Derived metric computations
- `mas_operator/alerts.py` — Alert condition evaluation
- `mas_operator/display.py` — Curses terminal UI
- `mas_operator/markers.py` — RViz marker construction
- `config/operator.yaml` — Default parameter values
- `launch/operator.launch.py` — Launch file (OpaqueFunction pattern)
- `doc/system_spec.md` — Authoritative specification
