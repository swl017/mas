# Gap Analysis: Semantic Architecture vs. As-Built Implementation

**Date:** 2026-03-26
**Purpose:** Identify mismatches between the semantic-level architecture (d2 diagram + ARCHITECTURE_SEMANTIC_LEVEL.md) and the current ROS2 implementation (ARCHITECTURE.md + CONTEXT.md files). Prioritize refactoring work for implementation-level conformance.

## Legend

| Status | Meaning |
|--------|---------|
| GAP | Component/interface exists in semantic arch but not in implementation |
| MISMATCH | Both exist but differ in behavior, data, or wiring |
| CONFIRMED | Semantic arch matches implementation |
| CONFIRMED (implicit) | Handled via launch-config remapping rather than a runtime node |

## Review Decisions (2026-03-26)

The following architectural decisions were made during review and are reflected throughout this document:

1. **Switches are launch-config boundaries, not runtime nodes.** `camera_switch`, `fc_switch`, `gimbal_switch` in the semantic arch represent sim/real boundary markers. They are handled by launch-time topic remapping, not active ROS2 nodes. The semantic arch should be updated to avoid confusion.
2. **`mas_multiview_py` is removed** — redundant and outdated. Only C++ `mas_multiview` remains.
3. **`mas_mission` owns all mode gating** — state machine (IDLE→TRACKING→MISSION), command routing for gimbal/zoom/cmd_vel, and operator interaction.
4. **Mission state via enum topic + service** — `MissionState.msg` with constants (IDLE=0, TRACKING=1, MISSION=2) + `SetMissionState.srv` for operator transitions.
5. **Cross-agent interfaces need dedicated compact topics** — `ray_w`, `zoom_level`, `combined_ang_vel` as explicit topics; avoid sending large messages (e.g., full Detection2DArray) cross-agent.
6. **Cross-machine bandwidth** — real hardware radio comms hit ~300 KB/s wall. LTE modems (~1 MB/s) expected to help. Cross-agent topic deduplication (cache node) is deferred to real-world deployment phase.
7. **Operator UI** needed for `mas_mission` and `mas_tracker` interaction across all agents.

---

## 1. Component-Level Comparison

| Semantic Component | Implementation | Status | Notes |
|--------------------|---------------|--------|-------|
| `ultralytics_ros` | `ultralytics_ros/tracker_node` | CONFIRMED | |
| `mas_multiview` | `mas_multiview/triangulation_node` (C++) | CONFIRMED | `mas_multiview_py` removed (redundant, outdated) |
| `mas_tracker` | `mas_tracker/sort3d_tracking_node` | CONFIRMED | |
| `mas_policy` | `mas_policy/policy_node` | CONFIRMED | Both marked WIP |
| `mas_common_frame` | `mas_common_frame/common_frame_node` | CONFIRMED | |
| `mas_offboard` | `mas_offboard/offboard_control` | CONFIRMED | |
| `MAVROS` | MAVROS (external) | CONFIRMED | |
| `point_to_region` | `gimbal_controller/point_to_region_node` | CONFIRMED | |
| **`mas_mission`** | — | **GAP** | State machine (IDLE→TRACKING→MISSION) not implemented. Currently, `mas_offboard` partially fills this role with its own state machine (HOVER→POLICY), but only for cmd_vel gating — no gimbal/zoom command routing exists. `mas_mission` should own all mode gating and operator interaction. |
| `camera_switch` | Launch-config remapping | CONFIRMED (implicit) | Not a runtime node — sim/real camera routing handled by launch-time topic remaps. Semantic arch should annotate as "launch-config boundary." |
| `fc_switch` | Launch-config (SITL vs real PX4) | CONFIRMED (implicit) | Not a runtime node — MAVROS connects to whichever PX4 is running. Semantic arch should annotate as "launch-config boundary." |
| `gimbal_switch` | Launch-config remapping | CONFIRMED (implicit) | Not a runtime node — sim uses `gimbal_stabilizer`/`los_rate_controller`, real uses `siyi_gimbal_node`, selected at launch. Semantic arch should annotate as "launch-config boundary." Note: aircraft attitude injection (`0x22`) for sim gimbal_stabilizer is deferred (low priority). |
| **Operator** | — | **GAP** | No explicit operator interface. Need UI for `mas_mission` (mission state transitions) and `mas_tracker` (target selection) across all agents. |

---

## 2. Interface-Level Comparison

### 2.1 Internal Edges (within Agent N)

| Semantic Interface | From → To | Implementation | Status | Notes |
|--------------------|-----------|---------------|--------|-------|
| `bbox_xywh, timestamp` | ultralytics → mas_multiview | `yolo_result_vision` (Detection2DArray) | CONFIRMED | Detection2DArray contains bbox + timestamp |
| `bbox_xywh, timestamp` | ultralytics → mas_policy | `yolo_result_vision` (Detection2DArray) | CONFIRMED | |
| `target pos est/cov (all)` | mas_multiview → mas_tracker | `triangulated_points` (MarkerArray) | **MISMATCH** | Must be a single structured message: array of (position + covariance) pairs, sent to both mas_tracker and mas_policy. Current split (MarkerArray to tracker, separate covariance to policy) is fragile — simultaneous targets reported individually could confuse the tracker. Needs custom message type in ICD. |
| `target pos est/cov (selected)` | mas_tracker → mas_policy | `chosen_target_pose` (PoseWithCovarianceStamped) | CONFIRMED | Operator selects one target from the array; selected target flows to mas_policy. |
| `pose, cov, vel` | mas_common_frame → mas_policy | `common_frame/odom` (Odometry) | CONFIRMED | Odometry contains pose + covariance + twist |
| `pose, cov, vel` | MAVROS → mas_common_frame | multiple MAVROS topics | CONFIRMED | |
| `lin acc` | MAVROS → mas_policy | `mavros/imu/data` (Imu) | CONFIRMED | |
| `cmd vel, cmd gimbal los rate, cmd zoom rate` | mas_policy → mas_mission | **policy → offboard directly** | **MISMATCH** | No mas_mission in the path. Policy publishes `cmd_vel` directly to offboard, `gimbal_cmd_los_rate` and `zoom_cmd` directly to gimbal nodes. Must be routed through `mas_mission`. |
| `cmd gimbal LOS angle, cmd zoom` | point_to_region → mas_mission | **point_to_region → siyi_gimbal_node directly** | **MISMATCH** | `gimbal_command_rpy_deg` goes directly to siyi_gimbal_node, no mission gate. Must be routed through `mas_mission`. |
| `(MISSION) cmd vel` | mas_mission → mas_offboard | **policy → offboard directly** | **MISMATCH** | `mas_mission` should publish `mission_state`; offboard subscribes and uses it instead of (or in addition to) its internal HOVER→POLICY transition. |
| `mission_state` | mas_mission → mas_offboard | — | **GAP** | No mission_state topic exists. Design: `MissionState.msg` enum (IDLE=0, TRACKING=1, MISSION=2) + `SetMissionState.srv` for operator transitions. |
| `camera feed` | camera_switch → ultralytics | Direct remapping in launch | CONFIRMED (implicit) | Launch-config boundary, not runtime switchable. |
| `base intrinsics, zoom level` | camera_switch → mas_multiview | Direct subscription | CONFIRMED (implicit) | Launch-config boundary. |
| `MAVLink` | fc_switch → MAVROS | Direct connection | CONFIRMED (implicit) | Launch-config boundary. |
| `gimbal joint angles, zoom` | gimbal → mas_multiview | `gimbal_state_rpy_deg` (Vector3) | **MISMATCH** | (1) Direct topic — correct, no switch needed (launch-config boundary). (2) Implementation uses world-frame IMU angles (`0x0D`); should use `0x26` encoder joint angles — not yet wired. (3) Zoom level not yet routed to multiview in real hardware path. Semantic arch diagram label is correct ("joint angles"). |
| `gimbal state` | gimbal → point_to_region | `gimbal_state_rpy_deg` | **MISMATCH** | Same as above — wrong angle source (`0x0D` instead of `0x26`). |
| `cmd gimbal, cmd zoom` | mas_mission → gimbal | — | **GAP** | No mission-gated gimbal command routing. `mas_mission` should gate between point_to_region commands (TRACKING) and policy commands (MISSION). Design decision: this gating must work for both sim and real. |
| `aircraft attitude` | mas_common_frame → gimbal | `common_frame/odom` → `siyi_gimbal_node` | CONFIRMED (implicit) | Attitude injection (`0x22`) is implemented in siyi_gimbal_node directly via odom subscription. No switch needed — launch-config boundary. Note: sim `gimbal_stabilizer` should also receive attitude (deferred, low priority). |

### 2.2 Cross-Agent Edges (Agent N+1 → Agent N)

| Semantic Interface | From → To | Implementation | Status | Notes |
|--------------------|-----------|---------------|--------|-------|
| `pos, vel, ray_w, ang vel, zoom` | N+1.common_frame → N.mas_policy | `/{peer}/common_frame/odom` + `/{peer}/gimbal_state_rpy_rad` | **MISMATCH** | (1) `ray_w` needed as a separate topic — can't guarantee source data arrives in sync; each agent should publish its own pre-computed `ray_w`. (2) `zoom_level` not currently received from peers — needs to be added. (3) `combined_ang_vel` (body + gimbal) needs a dedicated topic rather than estimating gimbal rate via finite differences. |
| `ray_w, pose, cov` | N+1.common_frame → N.mas_multiview | `/{peer}/common_frame/odom` + `/{peer}/gimbal_state_rpy_deg` | CONFIRMED | multiview subscribes to per-camera odom + gimbal topics. Note: potential cross-agent topic deduplication (cache node) deferred to real-world deployment phase. Real radio comms measured ~300 KB/s ceiling; LTE modems (~1 MB/s) expected to relieve. |
| `bbox_empty` | N+1.ultralytics → N.mas_policy | `/{peer}/yolo_result_vision` | **MISMATCH** | Full Detection2DArray sent cross-agent but only emptiness is needed. Should use a compact topic (e.g., `std_msgs/Bool`) — avoid exchanging large messages cross-agent. |

### 2.3 Sim/Real Boundary Edges

All switch components are launch-config boundaries (see Review Decision #1). These are not runtime GAPs.

| Semantic Interface | Status | Notes |
|--------------------|--------|-------|
| PegasusSimulator → camera: camera feed, intrinsics, zoom | CONFIRMED (implicit) | Isaac Sim publishes directly to camera topics via launch remapping. |
| PX4 SITL ↔ MAVROS: MAVLink | CONFIRMED (implicit) | MAVROS connects to whichever PX4 is running (SITL or real). |
| gimbal_stabilizer ← gimbal commands: cmd gimbal, cmd zoom, attitude | CONFIRMED (implicit) | In sim, `los_rate_controller` handles gimbal commands. Note: attitude injection for sim gimbal_stabilizer deferred (low priority). |
| PegasusSimulator → gimbal state: gimbal joint angles, zoom | CONFIRMED (implicit) | In sim, `gimbal_stabilizer` publishes gimbal state directly. |
| Real equivalents (usb_cam, siyi_gimbal_node, PX4 real) | CONFIRMED (implicit) | Direct connections, switched by launch config. |

### 2.4 Operator Edges

| Semantic Interface | Status | Notes |
|--------------------|--------|-------|
| Operator → mas_mission: mission select | **GAP** | No service/topic for mission mode transitions. Need `SetMissionState.srv` for operator to trigger IDLE→TRACKING→MISSION. |
| Operator → mas_tracker: target select | **MISMATCH** | `set_auto_pick_mode` (std_msgs/Int8) exists but is a mode toggle, not a target selection command. Minimal mission flow: (1) command drones to takeoff, optionally move to designated coordinates, (2) command gimbals to point to a region (x,y,z), (3) if target can be localized, operator approves to advance to policy state. Full target selection UI (camera feed with bbox overlay, 3D multi-agent view) deferred — focus on minimal case first. |

---

## 3. Prioritized Refactoring Plan

Ordered by **dependency** (downstream items depend on upstream) and **risk** (impact on sim-to-real parity).

### Priority 1: Gimbal State Source (blocks policy accuracy + sim/real parity)

**Gap:** Semantic arch decided on `0x26` encoder angles for body-frame joint angles. SDK support is implemented but not wired to downstream consumers.

**Tasks:**
- [ ] Wire `gimbal_encoder_rpy_deg` as the primary gimbal state for `mas_multiview` and `point_to_region` on real hardware
- [ ] Verify `mas_policy` gimbal input path: currently expects `gimbal_state_rpy_rad` from `los_rate_controller` — confirm this is correct for both sim and real
- [ ] Define the single canonical gimbal state topic name and message convention (degrees vs radians, body-frame vs world-frame) in the ICD

### Priority 2: mas_mission (blocks mission phase management)

**Gap:** No command routing between pre-mission (point_to_region) and mission (policy) phases for gimbal/zoom. Currently only cmd_vel is gated by offboard's state machine.

**Tasks:**
- [ ] Design `mas_mission` node: state machine, service interface, topic routing
- [ ] Write spec: `doc/mas_mission_spec.md`
- [ ] Implement: subscribe to policy + point_to_region commands, publish gated commands downstream
- [ ] Add `mission_state` topic for offboard to consume
- [ ] Update offboard to use `mission_state` instead of internal HOVER→POLICY transition (or keep both — decide)

### Priority 3: Sim/Real Gimbal Parity (launch-config alignment)

**Decision:** Switches are launch-config boundaries, not runtime nodes. The task is to ensure sim and real gimbal outputs use the same convention so downstream nodes don't need to know which environment they're in.

**Tasks:**
- [ ] Ensure sim `gimbal_stabilizer` outputs body-frame joint angles matching `0x26` encoder convention
- [ ] Align topic names and units (degrees vs radians) between sim and real paths via launch remapping
- [ ] Document the canonical gimbal state convention in ARCHITECTURE.md
- [ ] (Deferred) Feed aircraft attitude to sim `gimbal_stabilizer` for higher-fidelity sim

### Priority 4: Interface Conformance

**Tasks:**
- [ ] `triangulated_points`: replace MarkerArray with structured message — array of (position + covariance) pairs, sent to both mas_tracker and mas_policy
- [ ] Cross-agent `bbox_empty`: replace full Detection2DArray with compact topic (e.g., `std_msgs/Bool`)
- [ ] Cross-agent `ray_w`: add as dedicated per-agent published topic (computed at source)
- [ ] Cross-agent `zoom_level`: add peer zoom subscription to mas_policy
- [ ] Cross-agent `combined_ang_vel`: add dedicated topic (body + gimbal angular velocity)
- [ ] Operator → mas_tracker target selection: implement minimal flow (gimbal pointing → localization confirmation → policy approval)

### ~~Priority 5: Sim/Real Switches for Camera and FC~~ (RESOLVED)

**Decision:** Launch-config remapping is sufficient. `camera_switch` and `fc_switch` are not runtime nodes. Semantic arch should annotate them as "launch-config boundary" markers.

**Tasks:**
- [ ] Update semantic arch diagram to mark camera/fc/gimbal switches as launch-config boundaries instead of active nodes

---

## 4. ICD Template (for Priority 1–3)

For each interface being refactored, fill in:

```
Interface: <descriptive name>
Topic:     <full topic path>
Msg Type:  <package/MsgType>
QoS:       <RELIABLE | BEST_EFFORT | default>
Publisher:  <node>
Subscriber: <node(s)>
Frame:     <coordinate frame convention>
Units:     <radians | degrees | m/s | ...>
Rate:      <Hz>
Notes:     <any special conventions>
```

---

## 5. Next Steps

1. ~~Review this gap analysis~~ — **Done** (2026-03-26). Review decisions incorporated above.
2. Start Priority 1 (gimbal state wiring) — smallest scope, highest immediate value
3. Write `doc/mas_mission_spec.md` for Priority 2 — include `MissionState.msg` enum, `SetMissionState.srv`, minimal operator flow
4. Update semantic arch diagram: annotate switches as launch-config boundaries
5. Remove `mas_multiview_py` package
