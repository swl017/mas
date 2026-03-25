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
- Action Item: Investigate and match the actual SIYI SDK.

Multiple gimbal commands exist.
- Gimbal LOS angle command(`point_to_region`) vs rate command(`mas_policy`)
- Action Item: Investigate if the actual gimbal hardware can track LOS angle despite drone tilt. If so, keep LOS angle command for `point_to_region`, else use feedback control using LOS angle rate.

## Rendering architecture diagram
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.svg --layout=elk`
or
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.png --layout=elk`
- `elk`: straight lines
- `dagre`: curved lines