# mas_mission — Specification

**Date:** 2026-03-26
**Status:** Draft
**Package:** `mas_mission` (new, `ament_python`)

## Purpose

`mas_mission` owns mission phase management for all agents. It gates gimbal/zoom/velocity commands based on the current mission state, replacing the implicit gating spread across `mas_offboard` and direct topic wiring.

**Design principle:** state publisher + command router. Not a planner.

**Multi-agent coordination:** One `mas_mission` node runs per agent. The operator publishes a single `mission_state_cmd` topic that all agents receive simultaneously (topic-based multicast). Each agent's `mas_mission` transitions independently and publishes its own `mission_state` for confirmation.

## State Machine

```
IDLE ──[mission_state_cmd]──► TRACKING ──[mission_state_cmd]──► MISSION
  ▲                               │                                │
  └────────[mission_state_cmd]────┴────────[mission_state_cmd]─────┘
```

### States

| State | Description | Gimbal command source | cmd_vel source | Zoom source |
|-------|-------------|----------------------|----------------|-------------|
| `IDLE` | System powered, no active tracking or mission. Gimbal holds last position or accepts manual commands. | None (or manual) | None | None |
| `TRACKING` | Gimbals point to a designated region. Operator monitors localization. Drones may be commanded to takeoff and move to designated positions. | `point_to_region` | `mas_offboard` (internal waypoint) | `point_to_region` (if applicable) |
| `MISSION` | Policy takes over all commands. | `mas_policy` (LOS rate) | `mas_policy` | `mas_policy` |

### Transitions

All transitions are triggered by the operator via the `mission_state_cmd` topic. No automatic transitions.

| From | To | Operator action | Preconditions (checked, not enforced) |
|------|----|-----------------|---------------------------------------|
| `IDLE` | `TRACKING` | "Start tracking" | Drones connected, gimbals responsive |
| `TRACKING` | `MISSION` | "Approve mission" | Target localized (triangulation active), drones at designated positions |
| `TRACKING` | `IDLE` | "Stop tracking" | — |
| `MISSION` | `IDLE` | "Abort mission" | — |
| `MISSION` | `TRACKING` | "Return to tracking" | — |

**Preconditions are advisory:** The node logs warnings if preconditions aren't met but still transitions. The operator has final authority. This avoids blocking the operator in edge cases.

## Interface

### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `mission_state` | `std_msgs/Int8` | RELIABLE, latched | Current state enum. Published on change and at 1 Hz heartbeat. |
| `cmd_vel` | `geometry_msgs/TwistStamped` | BEST_EFFORT | Gated velocity command → `mas_offboard` |
| `gimbal_cmd_rpy_deg` | `geometry_msgs/Vector3` | default | Gated gimbal angle command → gimbal node |
| `gimbal_cmd_los_rate` | `geometry_msgs/Vector3` | default | Gated gimbal LOS rate command → gimbal node |
| `zoom_cmd` | `std_msgs/Float32` | default | Gated zoom command → gimbal node |

### Subscribed Topics

| Topic | Type | QoS | Source | Active in state |
|-------|------|-----|--------|-----------------|
| `/mission_state_cmd` | `std_msgs/Int8` | RELIABLE, transient local | Operator | All (triggers transitions) |
| `policy/cmd_vel` | `geometry_msgs/TwistStamped` | BEST_EFFORT | `mas_policy` | MISSION |
| `policy/gimbal_cmd_los_rate` | `geometry_msgs/Vector3` | default | `mas_policy` | MISSION |
| `policy/zoom_cmd` | `std_msgs/Float32` | default | `mas_policy` | MISSION |
| `tracking/gimbal_cmd_rpy_deg` | `geometry_msgs/Vector3` | default | `point_to_region` | TRACKING |
| `tracking/zoom_cmd` | `std_msgs/Float32` | default | `point_to_region` | TRACKING |

**Naming convention:** Upstream nodes publish to `policy/*` or `tracking/*` prefixed topics (via launch remapping). `mas_mission` selects which source to forward based on state.

### Operator Command (Topic-based Multicast)

The operator publishes a single message to command all agents simultaneously. This replaces a per-agent service call.

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/mission_state_cmd` | `std_msgs/Int8` | RELIABLE, transient local (latched) | Operator's requested state. Global topic (absolute path, not namespaced). Latched so late-joining agents pick up current state. |

Each agent's `mas_mission` subscribes to `/mission_state_cmd` and transitions independently. The operator confirms all agents transitioned by monitoring each agent's `/{veh}/mission_state` topic.

**Why topic instead of service:**
- ROS2 services are point-to-point — one call reaches one server. Commanding N agents would require N parallel service calls and an operator relay node.
- A latched topic naturally multicasts to all subscribers and handles late-joining agents.
- Per-agent confirmation is provided by the existing `mission_state` heartbeat topic (already in spec).

**Constants** (shared across packages via a `mas_msgs` package or defined in code):

```python
IDLE = 0
TRACKING = 1
MISSION = 2
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `heartbeat_rate_hz` | double | 1.0 | Rate for mission_state heartbeat publishing |
| `initial_state` | int | 0 | Initial mission state (IDLE) |

## Minimal Operator Flow

This is the minimal viable interaction for a multi-agent mission:

```
1. Power on, all nodes start                              → All agents: IDLE
2. Operator publishes TRACKING to /mission_state_cmd      → All agents: TRACKING
   - Each agent's mas_offboard runs INIT→ARM→TAKEOFF→HOVER
   - Each agent's point_to_region commands gimbals to designated region (x,y,z)
   - mas_multiview attempts triangulation
   - Operator monitors per-agent /{veh}/mission_state for confirmation
   - Operator monitors camera feeds / 3D view for target localization
3. Operator publishes MISSION to /mission_state_cmd       → All agents: MISSION
   - Each agent's mas_offboard transitions HOVER→POLICY (gated by mission_state)
   - mas_policy takes over: cmd_vel, gimbal LOS rate, zoom
4. Operator publishes IDLE to /mission_state_cmd (abort)  → All agents: IDLE
```

**Minimal operator tooling:** A single `ros2 topic pub` command is sufficient to trigger all agents. A proper UI can be built later.

## Interaction with mas_offboard

Currently `mas_offboard` has its own state machine: `INIT→RAMP_UP→ARM→TAKEOFF→HOVER→POLICY`.

**Change required:** `mas_offboard` subscribes to `mission_state`. The `HOVER→POLICY` transition is gated by `mission_state == MISSION` instead of (or in addition to) the current distance+yaw check.

**Decision: Option A.** Keep offboard's internal state machine for flight phases (INIT through HOVER). Add `mission_state == MISSION` as an additional condition for HOVER→POLICY. Simple, minimal change.

## Interaction with mas_tracker

Target selection is currently via `set_auto_pick_mode` (Int8) on mas_tracker. For the minimal flow, this is sufficient:
- In TRACKING state, mas_tracker auto-picks the closest/best target
- The selected target's position feeds into `point_to_region`
- Operator visually confirms localization before approving MISSION

Future: explicit target selection UI (camera feed with bbox overlay, multi-agent 3D view). Out of scope for this spec.

## Sim/Real Parity

`mas_mission` is environment-agnostic. It only routes ROS2 topics based on state — no hardware dependencies. Works identically in sim and real.

## What This Node Does NOT Do

- Flight control (that's `mas_offboard`)
- Path planning or waypoint generation
- Target selection logic (that's `mas_tracker` + operator)
- Gimbal computation (that's `point_to_region` or `mas_policy`)
- Any decision-making — it's a **state-gated command multiplexer**
