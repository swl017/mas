# Remaining Work — Post Priority 4

**Date:** 2026-03-27
**Status:** Active
**Source:** gap_analysis.md open items, spec deferred decisions, session progress notes

---

## 1. Tracker Ray Selection (move from mas_policy to mas_tracker)

**Why:** The spec (POLICY_ROS2_DEPLOYMENT_REFERENCE.md lines 22, 268) defers this. Currently mas_policy does angular-proximity matching to select one bearing ray per agent. This belongs in mas_tracker, which already knows the selected target.

**Plan:** [.claude/plans/lively-tinkering-pascal.md](.claude/plans/lively-tinkering-pascal.md) (approved)

**Status:** Partially implemented. User has already edited:
- `TriangulatedPoint.msg` — added `string[] detection_ids`
- `detection.h` — added `detection_ids` to `Detection3D`
- `sort3d_node.h` — added `rays_sub_`, `chosen_ray_pub_`, `latest_target_rays_`, `raysCallback()`
- `observation_assembler.py` — added `chosen_target_ray_w` field, subscriptions to `chosen_target_ray_w` (ego+peers)

**Remaining code changes:**
- [ ] Propagate detection_ids in `multiview_triangulation.cpp` `Triangulate()` — collect from association's contributing camera detections
- [ ] Publish detection_ids in `triangulation_node.cpp` result loop
- [ ] Implement `raysCallback()` and angular matching in `sort3d_node.cpp` `pubChosenTarget()`
- [ ] Clean up observation_assembler: remove `_select_target_ray()`, remove `target_rays_w` and `chosen_target_pose` peer subscriptions (replaced by `chosen_target_ray_w`)
- [ ] Add `target_rays_w` remapping in `sort3d.launch.py`
- [ ] Build and sim e2e test

**Depends on:** Nothing (can do now)

---

## 2. Gimbal State Convention (Priority 1 & 3 from gap_analysis.md)

**Why:** Sim and real gimbal paths used different topic names and units. Canonical convention now defined: `gimbal_state_rpy_deg` (degrees, body-frame).

**Completed tasks:**
- [x] Define canonical gimbal state topic name and message convention (deg, body-frame) — `gimbal_state_rpy_deg` is canonical (2026-03-29)
- [x] Verify mas_policy gimbal input path correct for both sim and real — switched to `gimbal_state_rpy_deg`, deg→rad conversion internal (2026-03-29)
- [x] Align topic names/units between sim and real — both publishers output `gimbal_state_rpy_deg`, mas_policy subscribes to it (2026-03-29)
- [x] Document the canonical gimbal state convention in ARCHITECTURE.md (2026-03-29)

**Hardware verification (blocked on field test):**
- [ ] Encoder sign convention (yaw/pitch direction multipliers)
- [ ] Encoder stream continuity at 50 Hz under maneuvers
- [ ] `los_rate_controller` compatibility with encoder body-frame angles
- [ ] `point_to_region` closed-loop convergence
- [ ] `mas_multiview` triangulation reprojection error with encoder angles

**Depends on:** Hardware access

---

## 3. Semantic Architecture Diagram Update

**Why:** The d2 diagram still uses `ray_w` for cross-agent edges. Should be `target_rays_w`.

**Tasks:**
- [ ] Rename `ray_w` → `target_rays_w` in `doc/diagrams/mas_architecture_semantic.d2` cross-agent edges
- [ ] Add `chosen_target_ray_w` edge from mas_tracker → mas_policy (after item 1 is done)
- [ ] Re-export PNG/SVG

**Depends on:** Item 1 (tracker ray selection)

---

## 4. CONTEXT.md Updates

**Why:** Several packages have changed interfaces not yet reflected in their CONTEXT.md.

**Packages needing update:**
- [ ] `mas_multiview/CONTEXT.md` — new publishers (TriangulatedPointArray, TargetRayArray), `use_precomputed_rays` param, `detection_ids` field
- [ ] `mas_tracker/CONTEXT.md` — new subscriber (TargetRayArray), new publisher (chosen_target_ray_w, PoseWithCovarianceStamped), subscription type change (TriangulatedPointArray)
- [ ] `gimbal_controller/CONTEXT.md` — new publishers (combined_ang_vel_w, zoom_level)
- [ ] `ultralytics_ros/CONTEXT.md` — new publisher (yolo_result_active Bool), Detection2D.id now set

**Depends on:** Item 1 (for tracker CONTEXT.md to be complete)

---

## 5. Policy Topic Prefix Convention

**Why:** Spec defers the decision (line 225). Currently policy publishes to bare relative topics, remapped to `policy/*` when `use_mission_gate:=true`. The default (without mission gate) has no prefix, meaning policy commands go directly to actuators — no safety gate.

**Decision needed:** Should `policy/*` prefix be the default, with mission gate always active? Or keep the current opt-in approach?

**Impact:** Low risk. Only affects launch configuration, not node code.

**Depends on:** Operator workflow maturity

---

## Prioritized Order

| Order | Item | Effort | Blocked? |
|-------|------|--------|----------|
| 1 | Tracker ray selection | Medium (C++ + Python) | No |
| 2 | CONTEXT.md updates | Small (docs only) | After item 1 |
| 3 | D2 diagram update | Small (diagram + export) | After item 1 |
| 4 | Gimbal convention (software) | Medium | No |
| 5 | Gimbal hardware verification | Medium | Hardware access |
| 6 | Policy topic prefix decision | Small (config only) | Workflow maturity |
