# mas_mission

## Purpose
Mission state management and command routing for multi-agent systems. State-gated command multiplexer — routes gimbal/zoom/velocity commands from the active source (tracking or policy) based on the current mission state.

## Nodes

### mission_node
**File:** `mas_mission/mission_node.py`
**Pattern:** Subscriber-driven (command passthrough gated by state, heartbeat timer for state publishing)

#### State Machine
`IDLE (0) → TRACKING (1) → MISSION (2)`
All transitions triggered by operator via `/mission_state_cmd` topic.

#### Subscriptions
- `/mission_state_cmd` (`std_msgs/Int8`) — operator command (global topic, RELIABLE + transient local)
- `policy/cmd_vel` (`geometry_msgs/TwistStamped`) — policy velocity command (MISSION state)
- `policy/gimbal_cmd_los_rate` (`geometry_msgs/Vector3`) — policy gimbal rate command (MISSION state)
- `policy/zoom_cmd` (`std_msgs/Float32`) — policy zoom command (MISSION state)
- `tracking/gimbal_cmd_rpy_deg` (`geometry_msgs/Vector3`) — tracking gimbal angle command (TRACKING state)
- `tracking/zoom_cmd` (`std_msgs/Float32`) — tracking zoom command (TRACKING state)

#### Publishers
- `mission_state` (`std_msgs/Int8`) — current state enum (RELIABLE + transient local, heartbeat at 1 Hz)
- `cmd_vel` (`geometry_msgs/TwistStamped`) — gated velocity command → mas_offboard
- `gimbal_cmd_rpy_deg` (`geometry_msgs/Vector3`) — gated gimbal angle command → gimbal node
- `gimbal_cmd_los_rate` (`geometry_msgs/Vector3`) — gated gimbal LOS rate command → gimbal node
- `zoom_cmd` (`std_msgs/Float32`) — gated zoom command → gimbal node

#### Parameters
- `heartbeat_rate_hz` (`double`, default: `1.0`) — mission_state heartbeat publish rate
- `initial_state` (`int`, default: `0`) — initial mission state (IDLE)

#### Dependencies
- mas_policy — provides policy commands (MISSION state)
- gimbal_controller/point_to_region_node — provides tracking commands (TRACKING state)
- mas_offboard — consumes cmd_vel and mission_state

## Key Files
- `mas_mission/mission_node.py` — Mission state machine and command multiplexer
- `launch/mission.launch.py` — Launch file
- `../doc/mas_mission_spec.md` — Authoritative specification
