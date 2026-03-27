# mas_operator — System Specification

**Date:** 2026-03-27
**Status:** Draft
**Scope:** System-level (cross-package operator interface)

## Purpose

`mas_operator` is the human-in-the-loop supervisory interface for MAS. It aggregates information from all per-vehicle nodes and the triangulation pipeline into a unified operator view, and provides mission command authority.

**Design principle:** Information aggregator + command authority. Not an autonomy layer. The operator makes all mission-critical decisions; the system organizes information to support those decisions.

**Information design:** Following the Dronetology situational-awareness model [Martín-Lammerding 2025], operator information is organized in three layers — Perception (raw feeds), Comprehension (fused understanding), and Projection (predicted outcomes). This prevents raw-data overload and surfaces the right information at the right abstraction level for each decision.

**Motivation:** The MAS system performs delay-aware multi-agent active triangulation [Lee & Shim 2026] where agents coordinate gimbal-zoom cameras to localize aerial targets. System performance depends on estimation uncertainty (multi-source covariance), observation geometry (baseline-to-range ratio), communication health (Age-of-Information), and inter-agent safety (collision avoidance). The operator must reason about all of these to make well-timed mission decisions.

## Operator Decision Model

The operator faces four key decisions during a mission. Each decision has specific information requirements that drive the interface design.

### Decision 1: When to start tracking? (IDLE → TRACKING)

| Required information | Source |
|----------------------|--------|
| All agents connected and reporting state | `/{veh}/mission_state` liveness |
| MAVROS armed and in correct mode | `/{veh}/mavros/state` |
| Camera feeds active | `/{veh}/camera/color/image_raw` liveness |
| Gimbal responding to commands | `/{veh}/gimbal_state_rpy_deg` liveness |

**Decision confidence:** Binary — either the fleet is ready or it is not.

### Decision 2: When to approve the mission? (TRACKING → MISSION)

| Required information | Source |
|----------------------|--------|
| Triangulation producing valid estimates | `triangulated_points` message rate + content |
| Estimation uncertainty acceptable | Covariance trace from `triangulated_points` |
| Observation geometry adequate | Baseline-to-range ratio from agent + target positions |
| Agents at designated positions | Per-agent `common_frame/odom` vs. waypoints |
| Target correctly identified | Camera feeds with detection overlay |

**Decision confidence:** Continuous — the operator judges when estimation quality is "good enough." Multi-source analytical covariance [Lee & Shim 2026, Eq. 16] captures pixel noise, pose error, gimbal calibration, and intrinsics uncertainties in a single quantity the operator can monitor.

### Decision 3: When to abort? (→ IDLE)

| Required information | Source |
|----------------------|--------|
| Triangulation lost (no valid estimates) | `triangulated_points` message rate |
| Uncertainty growing (covariance trend) | Time series of covariance trace |
| Communication degraded (high AoI) | Per-agent topic timestamp freshness |
| Collision proximity | Inter-agent distances from `common_frame/odom` |
| Agent disarmed or faulted | `/{veh}/mavros/state` |

**Decision urgency:** Collision proximity and agent faults require immediate response. Uncertainty growth and communication degradation allow more deliberate assessment.

**Delay context:** The system degrades gracefully up to ~200 ms cumulative delay and significantly at ~800 ms [Lee & Shim 2026, Fig. 6]. AoI thresholds should reflect these empirical bounds.

### Decision 4: Which target to track? (multi-target scenario)

| Required information | Source |
|----------------------|--------|
| List of tracked 3D objects with position + covariance | `tracked_objects/class_{i}` |
| Currently selected target | `chosen_target_pose` |
| Detection continuity per track | Track age from `mas_tracker` |

## Information Architecture

### Perception — Raw Feeds

Direct sensor data with minimal processing. The operator uses this for visual confirmation and ground-truth intuition.

| Information | Topic | Type | Notes |
|-------------|-------|------|-------|
| Camera feed (per agent) | `/{veh}/camera/color/image_raw` | sensor_msgs/Image | High bandwidth; may use compressed transport |
| Detection overlay (per agent) | `/{veh}/yolo_result_vision` | vision_msgs/Detection2DArray | Bounding boxes on camera feed |
| Gimbal angles (per agent) | `/{veh}/gimbal_state_rpy_deg` | geometry_msgs/Vector3 | Current pointing direction |
| Agent pose (per agent) | `/{veh}/common_frame/odom` | nav_msgs/Odometry | Position, velocity, orientation in mission frame |
| Mission state (per agent) | `/{veh}/mission_state` | std_msgs/Int8 | IDLE(0) / TRACKING(1) / MISSION(2) |
| Flight controller status | `/{veh}/mavros/state` | mavros_msgs/State | Armed, mode, connected |

### Comprehension — Fused Understanding

Derived quantities that integrate multiple data streams into actionable summaries. This is the primary level for operator decision-making.

| Information | Derived from | Description |
|-------------|-------------|-------------|
| Triangulated target position | `triangulated_points` | 3D position estimate in mission frame |
| Estimation uncertainty | `triangulated_points` covariance fields | Covariance magnitude (trace or determinant) — single number summarizing localization quality |
| Triangulation validity | `triangulated_points` message rate + content | Whether ≥ 2 cameras currently contribute valid observations |
| Selected target identity | `chosen_target_pose` | Which tracked object the system is pursuing, with covariance |
| Agent coordination geometry | Per-agent `common_frame/odom` + target position | Baseline-to-range ratio between agent pairs — drives triangulation conditioning |
| Communication health (AoI) | `now - msg.header.stamp` for each topic | Per-agent data freshness; proxy for Age-of-Information |
| Fleet consensus | Per-agent `mission_state` | Whether all agents are in the same mission phase |
| Inter-agent separation | Pairwise distances from `common_frame/odom` | Collision risk indicator; compare against safety distance |

### Projection — Predicted Outcomes (Future)

Computed trends and predictions that anticipate system behavior. These require time-series analysis and are post-MVP.

| Information | Derived from | Description |
|-------------|-------------|-------------|
| Covariance trend | Time series of covariance trace | Improving / stable / degrading estimation quality |
| Track loss risk | Triangulation validity history + detection continuity | Likelihood of losing the target in near future |
| Coverage gap detection | Agent geometry + target bearing analysis | Whether agent repositioning would improve triangulation |

## Operator Actions

### MVP Actions

| Action | ROS2 output | Description |
|--------|-------------|-------------|
| Start Tracking | `/mission_state_cmd` ← Int8(1) | Transition all agents: IDLE → TRACKING |
| Approve Mission | `/mission_state_cmd` ← Int8(2) | Transition all agents: TRACKING → MISSION |
| Abort | `/mission_state_cmd` ← Int8(0) | Return all agents to IDLE |
| Toggle auto-pick | `/{veh}/set_auto_pick_mode` ← Int8(0\|1) | Enable/disable automatic target selection per agent |

### Future Actions

| Action | Description |
|--------|-------------|
| Per-agent mission override | Command individual agents to different states |
| Manual gimbal slew | Direct gimbal pointing override for a specific agent |
| Policy reset | Trigger `/{veh}/policy_node/reset_hidden_state` (std_srvs/Trigger) |
| Target selection from 3D view | Click-to-select from tracked object list |

## ROS2 Interface

### Subscribed Topics

All per-vehicle topics are subscribed for each vehicle in the `vehicles` parameter list.

| Topic | Type | QoS | Source | Purpose |
|-------|------|-----|--------|---------|
| `/{veh}/mission_state` | std_msgs/Int8 | RELIABLE, transient_local | mas_mission | Per-agent mission phase + fleet consensus |
| `/{veh}/common_frame/odom` | nav_msgs/Odometry | BEST_EFFORT | mas_common_frame | Agent position, velocity, covariance |
| `/{veh}/gimbal_state_rpy_deg` | geometry_msgs/Vector3 | BEST_EFFORT | siyi_gimbal_node | Gimbal pointing (liveness + display) |
| `/{veh}/yolo_result_vision` | vision_msgs/Detection2DArray | BEST_EFFORT | ultralytics_ros | Detection bounding boxes |
| `/{veh}/mavros/state` | mavros_msgs/State | RELIABLE | MAVROS | Arm/mode status |
| `triangulated_points` | mas_msgs/TriangulatedPointArray | default | mas_multiview | Target position + covariance |
| `/{veh}/chosen_target_pose` | PoseWithCovarianceStamped | default | mas_tracker | Selected target |
| `/{veh}/tracked_objects/class_{i}` | vision_msgs/Detection3DArray | default | mas_tracker | All tracked objects (target selection) |
| `/{veh}/camera/color/image_raw` | sensor_msgs/Image | default | camera driver | Camera feed (optional, bandwidth-dependent) |

**Bandwidth note:** Camera feeds (`image_raw`) are optional. The operator node must function without them — they enhance perception-level awareness but are not required for comprehension-level decision support.

### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/mission_state_cmd` | std_msgs/Int8 | RELIABLE, transient_local | Global mission state command (multicast to all agents) |
| `/{veh}/set_auto_pick_mode` | std_msgs/Int8 | default (depth 10) | Per-agent auto-pick toggle |

### Service Clients (Future)

| Service | Type | Target | Description |
|---------|------|--------|-------------|
| `/{veh}/policy_node/reset_hidden_state` | std_srvs/Trigger | mas_policy | Reset policy GRU hidden state |

## Derived Metrics

Quantities computed locally by the operator node from subscribed data. These implement the Comprehension layer.

| Metric | Computation | Inputs |
|--------|-------------|--------|
| Covariance magnitude | `trace(cov_3x3)` from triangulation | `triangulated_points[].covariance` |
| Inter-agent distances | Pairwise Euclidean distance | `/{veh}/common_frame/odom` positions |
| Data freshness (AoI proxy) | `ros_now - msg.header.stamp` | All subscribed topic timestamps |
| Triangulation validity rate | Fraction of recent windows with valid points | `triangulated_points` message rate + content |
| Observation geometry | Baseline-to-range ratio: `‖p_i - p_j‖ / ‖X̂ - midpoint‖` | Agent positions + target position |
| Fleet consensus | All agents report same `mission_state`? | `/{veh}/mission_state` |

## Alert Conditions

Conditions that should be highlighted to the operator. These are informational — they do NOT trigger autonomous actions.

| Alert | Condition | Severity |
|-------|-----------|----------|
| Fleet state mismatch | Not all agents in same `mission_state` after command | Warning |
| Communication stale | AoI for any agent > `aoi_warn_ms` | Warning |
| Communication lost | AoI for any agent > `aoi_critical_ms` | Critical |
| Triangulation lost | No valid triangulated points for > `tri_timeout_s` | Critical |
| Covariance spike | Covariance trace > `cov_warn_threshold` | Warning |
| Collision proximity | Inter-agent distance < `safety_distance_m` | Critical |
| Agent disarmed | `mavros/state.armed == false` during TRACKING or MISSION | Critical |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicles` | string[] | `["px4_1", "px4_2"]` | Vehicle namespaces to monitor |
| `aoi_warn_ms` | double | 500.0 | AoI warning threshold (ms) |
| `aoi_critical_ms` | double | 2000.0 | AoI critical threshold (ms) |
| `cov_warn_threshold` | double | 5.0 | Covariance trace warning threshold (m^2) |
| `safety_distance_m` | double | 9.5 | Inter-agent collision proximity threshold (m); should match CBF `D_deploy` |
| `tri_timeout_s` | double | 1.0 | Seconds without valid triangulation before alert |
| `status_rate_hz` | double | 2.0 | Derived metrics computation rate (Hz) |
| `enable_camera_feeds` | bool | false | Subscribe to raw image topics |
| `num_object_classes` | int | 1 | Number of tracked object classes for tracker subscriptions |

## Minimal Operator Flow

From the operator's perspective, showing what information supports each decision.

```
1. System startup
   Operator sees: per-agent mission_state = IDLE, MAVROS connected + armed
   Decision gate: all agents reporting, all MAVROS connected
   Action: —

2. Start tracking (publish TRACKING)
   Operator sees: camera feeds activate, gimbals slew to target region,
                   detection overlays appear, triangulation begins
   Monitors:      triangulated position + covariance magnitude,
                   triangulation validity, observation geometry quality
   Decision gate: triangulation valid, covariance below threshold,
                   agents at designated positions
   Action: publish /mission_state_cmd = 1

3. Approve mission (publish MISSION)
   Operator sees: agents begin autonomous maneuvering under policy control
   Monitors:      covariance trend, inter-agent distances, AoI,
                   triangulation continuity, fleet consensus
   Alerts active: all alert conditions (§ Alert Conditions)
   Action: publish /mission_state_cmd = 2

4. Abort (publish IDLE) — triggered by:
   - Triangulation lost alert
   - Sustained covariance growth
   - Communication lost
   - Collision proximity
   - Operator judgment
   Action: publish /mission_state_cmd = 0
```

## MVP vs Future

**MVP** (implementable now with existing topics):
- Subscribe to all § Subscribed Topics (camera feeds optional)
- Publish § Published Topics (existing interface from `scripts/operator.py`)
- Compute all § Derived Metrics
- Display § Alert Conditions
- Upgrade from current CLI (`scripts/operator.py`) to structured text or minimal GUI

**Future** (post-MVP):
- Projection-level SA: covariance trends, track loss prediction, coverage analysis
- Camera feed integration with detection overlay compositing
- 3D visualization: agent positions, target, covariance ellipsoids, observation geometry
- Per-agent mission override
- Manual gimbal control
- Policy hidden state reset via service call
- Mission recording and post-mission analysis / replay

## Dependencies

| Package | Used for | Required? |
|---------|----------|-----------|
| `mas_msgs` | TriangulatedPointArray message type | Yes |
| `mas_mission` | `mission_state` topic, `/mission_state_cmd` interface | Yes |
| `mas_common_frame` | `common_frame/odom` for agent positions | Yes |
| `mas_tracker` | `chosen_target_pose`, `tracked_objects`, `set_auto_pick_mode` | Yes |
| `mas_multiview` | `triangulated_points` | Yes |
| `ultralytics_ros` | `yolo_result_vision` for detection overlay | Optional |
| `mavros_msgs` | `State` message type for system health | Optional |

## What This Spec Does NOT Cover

- **UI layout or rendering technology.** This spec defines information and controls, not visual design. Implementation may be CLI, web dashboard, RViz panel, or custom GUI.
- **Autonomous decision-making.** Alerts are informational. The operator node does not make or enforce decisions.
- **Per-vehicle node behavior.** Defined in per-package specs (`mas_mission_spec.md`, etc.).
- **Network transport optimization.** Camera feed compression, DDS tuning, etc. are deployment concerns.
- **Target classification or identification.** The operator sees what YOLO detects; identification logic is out of scope.
