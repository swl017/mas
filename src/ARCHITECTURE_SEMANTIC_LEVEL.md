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

**SIYI SDK capabilities (`siyi_sdk.py`):**
- `getAttitude()` returns **(yaw, pitch, roll) in degrees** — IMU-stabilized, **world-frame** orientation. 0.1° precision.
- `getAttitudeSpeed()` returns angular rates in deg/s.
- **Raw motor/encoder joint angles are not available.** The protocol (`ACQUIRE_GIMBAL_ATT`, cmd `0x0d`) is the only angle source. None of the 16 protocol commands expose encoder positions.
- Two command modes: **absolute angle** (`requestSetAngles`) and **rate** (`requestGimbalSpeed`, range [-100, +100]).

**Sim vs Real mismatch:**
- Sim (`gimbal_stabilizer`) likely outputs **body-frame** angles.
- Real (SIYI) outputs **world-frame stabilized** angles.
- `mas_multiview` default modes (`zyx`/`zxy`) treat input as body-relative. The `"zy"` mode handles world-stabilized angles but this is a config flag, not an explicit contract.
- `mas_policy` expects **body-frame** joint angles in radians (applies `yaw_joint_offset = -pi/2`, computes world ray via body quaternion).

**Centrifugal force limitation:**
- SIYI stabilization is purely IMU-based (gyro + accelerometer). It compensates for static tilt and angular disturbances.
- It **cannot** distinguish gravity from centrifugal/linear acceleration (no external aiding from drone EKF). During banked turns or aggressive maneuvers, the gimbal's world-frame estimate drifts.
- For gentle surveillance orbits the error is small (a few degrees). For aggressive policy maneuvers it may be significant.

**Can raw joint angles be derived from the reported world-frame angles?**
- Only approximately. The conversion `R_joint = inverse(R_body_ekf) * R_gimbal_reported` is inexact because the gimbal's internal IMU estimate (`R_body_imu`) diverges from the drone's EKF (`R_body_ekf`) during acceleration.
- The gimbal firmware internally computes `R_joint_actual = inverse(R_body_imu) * R_gimbal_target`. When `R_body_imu` is wrong (centrifugal bias), the gimbal actuates incorrect joint angles to compensate — so both the reported world-frame angle and the actual joint angle are wrong.
- The derived joint angle error is bounded by the difference between the gimbal's internal IMU and the drone EKF: `inverse(R_body_ekf) * R_body_imu`. Small during gentle flight, grows with acceleration.
- Exact joint angles would require either (a) encoder/motor position access (not in SIYI protocol) or (b) knowledge of the gimbal's internal IMU state (not exposed).

**LOS angle tracking:**
- The SIYI gimbal **can** track absolute LOS angles despite drone tilt (internal stabilization). `point_to_region` can safely use absolute angle commands for pre-mission tracking.
- During aggressive maneuvers (policy active), **rate commands are safer** since they don't depend on the gimbal's internal attitude estimate being correct.

#### Decisions

1. **Standardize internal interface to body-frame joint angles.** The `gimbal_switch` converts SIYI world-frame angles to body-frame: `R_joint ≈ inverse(R_body_ekf) * R_gimbal_reported`. This is approximate — accuracy degrades during aggressive maneuvers due to the gimbal's internal IMU drift.
2. **Keep dual command modes:** absolute angle for `point_to_region` (pre-mission, gentle flight), rate for `mas_policy` (mission, aggressive maneuvers).
3. **`gimbal_switch` is load-bearing** — it's not just routing, it performs frame conversion for real hardware.
4. **Accept the approximation for now.** The joint angle derivation error is small for the expected flight regime. If aggressive maneuvers cause issues, future options include: custom gimbal firmware with encoder readout, or training the policy to be robust to gimbal angle noise.

## Rendering architecture diagram
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.svg --layout=elk`
or
`d2 mas_architecture_semantic.d2 mas_architecture_semantic.png --layout=elk`
- `elk`: straight lines
- `dagre`: curved lines