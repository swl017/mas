# Ticket 018 — Add Hover and Waypoint Operator Commands

## Problem

The operator currently cannot halt the swarm mid-mission or redirect drones to a new position. Once `MISSION` is active, drones follow policy commands with no way to pause them in place or send them to an arbitrary waypoint. The only escape is to drop all the way back to `IDLE`, which kills tracking and requires a full re-engagement sequence.

## Goal

Add two new operator capabilities:

1. **Hover (hold position)** — immediately stop all drones and hold their current positions.
2. **Go-to-waypoint** — command all drones to fly to their pre-configured waypoints (from `vehicles.yaml`), then hold.

These give the operator a "pause button" and a "return to start" command without cycling through the full state machine.

## Decision: Option A — New Mission States

**Chosen approach:** Add `HOVER_CMD = 3` and `WAYPOINT = 4` to the mission state enum. This is the most intuitive — every node in the system can observe the fleet mode on the `mission_state` topic.

The *implementation* inside `mas_offboard` reuses its existing HOVER flight state logic (position hold at 100Hz), just triggered by the new mission state values. Best of both worlds: clean system-wide semantics with minimal new code in offboard.

### State Enum (all packages)

```
IDLE = 0      # No commands forwarded
TRACKING = 1  # Gimbal tracking commands forwarded
MISSION = 2   # Full policy commands forwarded (cmd_vel, gimbal, zoom)
HOVER_CMD = 3 # Hold current position (operator emergency pause)
WAYPOINT = 4  # Fly to pre-configured waypoint and hold
```

### Transition Rules

```
Any state ──[h]──→ HOVER_CMD    (hold current position)
{MISSION, HOVER_CMD} ──[w]──→ WAYPOINT  (go to configured waypoint)
Normal flow: IDLE ──[1]──→ TRACKING ──[2]──→ MISSION ──[3]──→ ...
HOVER_CMD or WAYPOINT ──[3]──→ MISSION  (resume policy)
HOVER_CMD or WAYPOINT ──[1]──→ IDLE     (full stop)
```

### Fleet-wide Commands

All hover/waypoint commands apply to the entire fleet. One button press controls the herd.

### Waypoint Source

The waypoint is the one already configured per-vehicle in `mas_offboard` (from `vehicles.yaml` position parameters). No operator coordinate input needed — when something goes wrong during policy, the operator presses `[h]` to freeze or `[w]` to send everyone home.

## Per-Package Changes

### mas_operator

| Item | Detail |
|------|--------|
| New keybinds | `[h]` → `publish_mission_cmd(HOVER_CMD)`, `[w]` → `publish_mission_cmd(WAYPOINT)` |
| `operator_node.py` | Add `HOVER_CMD = 3`, `WAYPOINT = 4` to state constants. Update `_STATE_NAMES`. |
| `display.py` | Add `HOVER_CMD`/`WAYPOINT` to `_STATE_NAMES`. Update key legend. Add color for new states. |

Updated keybind legend:
```
[1]IDLE [2]TRACK [3]MISSION [h]HOVER [w]WAYPOINT [a]AutoPick ON [d]AutoPick OFF [t]Target [r]Reset GRU [q]Quit
```

### mas_mission

| Item | Detail |
|------|--------|
| State enum | Add `HOVER_CMD = 3`, `WAYPOINT = 4` to constants and `_STATE_NAMES`. |
| Command gating | `HOVER_CMD`: forward nothing (no cmd_vel, no gimbal, no zoom). `WAYPOINT`: forward nothing (offboard handles waypoint internally). |
| Transitions | Accept any valid state from `/mission_state_cmd`, no restricted transitions — operator has full authority. |

Effectively, `HOVER_CMD` and `WAYPOINT` behave like `IDLE` from the mission node's perspective (nothing forwarded). The distinction matters for `mas_offboard`.

### mas_offboard

| Item | Detail |
|------|--------|
| `mission_state` handling | React to new values in `_mission_state_cb`. |
| `HOVER_CMD` behavior | When `mission_state` transitions to `HOVER_CMD`: capture current position as hold target, set `flight_state = FlightState.HOVER`, publish position setpoint to that captured point. |
| `WAYPOINT` behavior | When `mission_state` transitions to `WAYPOINT`: set `flight_state = FlightState.HOVER` with the original configured waypoint as target (already stored). |
| `POLICY` → `HOVER_CMD` | If in `FlightState.POLICY` and `mission_state` changes to `HOVER_CMD`, immediately switch to position hold at current pose. No drift. |
| Resume `MISSION` | When `mission_state` returns to `MISSION`, re-evaluate HOVER→POLICY transition (waypoint reached + mission approved). If already at waypoint, transitions immediately. |

### mas_msgs

No new message types needed. Reuses `std_msgs/Int8` for mission state.

## Acceptance Criteria

- [ ] `[h]` key immediately halts all drones in position hold (current position)
- [ ] `[w]` key commands all drones to fly to their configured waypoints and hold
- [ ] Display shows HOVER_CMD / WAYPOINT state per vehicle with appropriate color
- [ ] Key legend updated with new commands
- [ ] `HOVER_CMD` → `MISSION` resumes policy control cleanly
- [ ] `WAYPOINT` → `MISSION` resumes policy after reaching waypoint
- [ ] No regressions in IDLE → TRACKING → MISSION flow
- [ ] Mission state enum consistent across mas_operator, mas_mission, mas_offboard
