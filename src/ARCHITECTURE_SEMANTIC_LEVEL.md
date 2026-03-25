# MAS (Multi-Agent System) Semantic-level Architecture

## Objective
**Define system components and interfaces in `MAS` in semantic-level.**
- What kind of system components are needed? 
- What kind of information is exchanged?

## The Semantic-level System Architecture
### Groups
- Agent N, Agent N+1
- Sim Env
- Real-world

### Components
- Simulator, ROS2 nodes, drivers
- Note: make it distinct from existing components to not-yet existing ones.

### Interface 
- Goal: make Sim Env identical to Real-world
- Note: may differ from the current ROS2 node implementation. Decide topic names and message types later in `Implementation-level Architecture`.

## Decisions to be made

### Do we need a dedicated mission planner?(`mas_mission`)
`mas_offboard` already gates cmd_vel via its state machine (HOVER→POLICY). The parallel problem for gimbal/zoom doesn't have an equivalent gate. A mas_mission node could:

- Own a simple state machine: `IDLE → TRACKING → MISSION`
- In `TRACKING`: enable `point_to_region_node`, disable policy gimbal commands
- In `MISSION`: disable `point_to_region_node`, enable policy commands
- Expose a service for the operator to trigger transitions
- Publish a `mission_state` topic that other nodes can check

This keeps it small — it's a state publisher + service server, not a full planner.

## Current problems

### Explicit sim-to-real counterparts
- It is not described explicitly whether an interface is always connected, or replaced when switched between sim to real.
- Solution(WIP): Add a switch that (a) receives both sim and real interface counterparts (b) connects to the sink component

### Interface data efficiency
- Inter-agent interfaces are limited in bandwidth. It is important to compact the data efficiently.(data compactness, non-duplicate inter-agent streams)
- Example 1(Duplicate inter-agent streams), `ray_w`(timestamp, x, y, z) from `agent_n1.ultralytics_n1` is used in both `agent_n.mas_multiview` and `agent_n.mas_policy`.
- Example 2(Data compactness), `bbox_xywh`(timestamp, center x, center y, width, height)(not shown in the diagram) from `agent_n1.ultralytics_n1` can be compacted into `bbox_empty`(shown in the diagram), since `bbox_xywh` isn't needed in downstream component. We could even encode it into `ray_w`(e.g. `(0,0,0)` if empty), but keeping a dedicated, explicit state could be better. We don't know, yet.

### Gimbal state/command ambiguity
Gimbal states can be represented in multiple ways.

- raw gimbal joint angles (required by `mas_policy`)
- yaw-aligned gimbal base frame LOS orientation
- world frame LOS orientation(body orientation needed to be fused) (required by `mas_policy`, `mas_multiview`)
- Their respective rates

Multiple gimbal commands exist.
- Gimbal LOS angle command(`point_to_region`) vs rate command(`mas_policy`)

#### Investigation findings (2026-03-26)

**SIYI SDK protocol (from `SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log V0.1.1.pdf`):**

Three angle sources available:

| Command | Name | Returns | Frame | Sensor |
|---|---|---|---|---|
| `0x0D` | Request Gimbal Attitude Data | yaw, pitch, roll + rates (int16, /10 for deg) | NED world-frame, IMU-stabilized | IMU (gyro+accel), yaw from magnetic encoder |
| `0x26` | Request Gimbal Magnetic Encoder Angle Data | yaw, pitch, roll (int16, /10 for deg) | **Raw joint angles** | Magnetic rotary encoders on motor shafts |
| `0x22` | Send Aircraft Attitude Data to Gimbal | (input, not output) | NED, radians | Receives drone EKF attitude to improve stabilization |

- `0x0D` (currently used): World-frame stabilized angles. Yaw derived from magnetic encoder, pitch/roll from IMU. Subject to centrifugal drift on pitch/roll during aggressive maneuvers.
- **`0x26` (not yet implemented in SDK): Raw magnetic encoder joint angles.** These are true mechanical motor positions, immune to IMU drift and centrifugal force. This is the definitive solution for body-frame joint angles.
- **`0x22` (not yet implemented in SDK): Accepts drone EKF attitude** (roll, pitch, yaw + rates, NED, radians, 20-50 Hz recommended). Feeding this to the gimbal improves its internal stabilization by replacing its IMU-only estimate with the drone's fused EKF. This mitigates the centrifugal force problem for `0x0D` and for absolute angle commands.
- `0x25` can configure continuous streaming of encoder data (`data_type=3`) at up to 100 Hz.
- Two command modes: **absolute angle** (`0x0E`, `requestSetAngles`) and **rate** (`0x07`, `requestGimbalSpeed`, range [-100, +100]).
- Motion modes: LOCK (world-stabilized), FOLLOW (yaw follows body), FPV (body-fixed, no stabilization).

**Sim vs Real mismatch:**
- Sim (`gimbal_stabilizer`) outputs **body-frame** angles.
- Real (SIYI `0x0D`) outputs **world-frame stabilized** angles.
- Real (SIYI `0x26`) outputs **raw joint angles** (body-frame) — matches sim directly.
- `mas_policy` expects **body-frame** joint angles in radians (applies `yaw_joint_offset = -pi/2`, computes world ray via body quaternion).
- `mas_multiview` default modes (`zyx`/`zxy`) treat input as body-relative. The `"zy"` mode handles world-stabilized angles.

#### Decisions

1. **Use `0x26` (magnetic encoder angles) for gimbal state on real hardware.** These are exact body-frame joint angles from motor shaft encoders — no IMU drift, no centrifugal force error, no frame conversion needed. Matches simulation output directly.
2. **Feed drone EKF to gimbal via `0x22`.** This improves the gimbal's own stabilization for absolute angle commands used by `point_to_region`.
3. **Keep dual command modes:** absolute angle for `point_to_region` (pre-mission), rate for `mas_policy` (mission).
4. **`gimbal_switch` simplifies:** with `0x26`, no frame conversion needed for state — just route raw encoder angles. The switch still handles sim/real routing and `0x22` injection.
5. **Implementation needed:** Add `0x26` and `0x22` support to the SIYI SDK (`siyi_sdk.py`, `siyi_message.py`). Configure `0x25` to stream encoder data at 50-100 Hz.

## Rendering architecture diagram
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.svg --layout=elk`
or
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.png --layout=elk`
- `elk`: straight lines
- `dagre`: curved lines