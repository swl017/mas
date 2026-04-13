# Ticket #024: Triangulation not computing after ticket #022 gimbal fix

## Problem

After the gimbal LOS control fix in ticket #022, the triangulation node produces no output — `triangulated_points` topic is empty. Frustum visualization may still publish, but no 3D points are computed.

## Impact

Without triangulation, the entire downstream pipeline is broken:
- `mas_tracker` receives no `TriangulatedPointArray` → no 3D tracks
- `sort3d` cannot associate detections → `chosen_target_ray_w` / `chosen_target_pose_w` are stale
- Policy receives stale/zero target observations → cannot coordinate agents

## Likely Root Causes

### A. `gimbal_state_rpy_deg` pitch double-negation in `zxy` angle order

The `zxy` gimbal angle order (used by all sim/deploy launches) applies `-gimbal.y` at [triangulation_node.cpp:401](src/mas_multiview/src/triangulation_node.cpp#L401):

```cpp
// zxy path
q_gimbal = AngleAxisd(gimbal.z, UnitZ())
         * AngleAxisd(gimbal.x, UnitX())
         * AngleAxisd(-gimbal.y, UnitY());  // ← negates pitch
```

Ticket #022 changed `gimbal_state_rpy_deg.y` to `degrees(-actual_pitch)` ([los_rate_controller.py:604](../../../IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py)), which already negates pitch for the `Ry` standard convention. The `zxy` code path negates **again**, producing `Ry(actual_pitch)` instead of `Ry(-actual_pitch)`.

**Pre-#022**: `msg.y = degrees(actual_pitch)` (internal convention, +pitch=down). The `zxy` `-gimbal.y` converted to Ry standard (+pitch=up). Correct.

**Post-#022**: `msg.y = degrees(-actual_pitch)` (Ry standard, +pitch=up). The `zxy` `-gimbal.y` negates again → `Ry(+actual_pitch)` = pitch inverted. Camera rays point wrong direction.

This causes:
- Incorrect camera ray directions → rays don't intersect properly
- Large reprojection errors → all results rejected by `max_reprojection_error`
- Zero valid `results_` after Ceres optimization

### B. `gimbal_angle_order` mismatch between `zxy` and `zyx`

The launch default is `zyx` ([triangulation.launch.py:70](src/mas_multiview/launch/triangulation.launch.py#L70)), but all tmux sessions pass `gimbal_angle_order:=zxy`. Ticket #022's `gimbal_state_rpy_deg` sign convention was documented for `Rz(yaw)*Rx(roll)*Ry(pitch)` composition — which is the `zxy` order. The `zyx` path does **not** negate pitch ([triangulation_node.cpp:392](src/mas_multiview/src/triangulation_node.cpp#L392)), so using `zyx` with the post-#022 convention would be correct. Need to verify which order matches the actual gimbal chain.

### C. Missing data prerequisite (less likely but check)

Triangulation requires `camera_infos_[i] && camera_poses_[i] && gimbals_[i]` plus non-empty detections from ≥2 cameras ([triangulation_node.cpp:361-371](src/mas_multiview/src/triangulation_node.cpp#L361-L371)). If any input topic stopped publishing, triangulation silently waits. The diagnostic log at [line 512-528](src/mas_multiview/src/triangulation_node.cpp#L512-L528) prints which fields are missing — check this first.

## Data Flow

```
isaac_joint_states
    → los_rate_controller
        → actual_pitch = -raw_pitch  (boundary negation for -Y axis)
        → gimbal_state_rpy_deg.y = degrees(-actual_pitch) = degrees(raw_pitch)
            → triangulation_node gimbalCallback: radians(msg.y)
                → zxy path: Ry(-radians(msg.y)) = Ry(-raw_pitch) = Ry(actual_pitch)
                                                                     ^^^ WRONG SIGN
                → should be: Ry(raw_pitch) = Ry(-actual_pitch) for tilt-up = positive Ry
```

## Investigation Steps

1. **Check diagnostic log**: Run triangulation and look for the "Messages:" log output — which fields show "OK" vs "-"?
2. **Check frustum direction**: With gimbal at known angles, does the RVIZ frustum point the correct direction? If pitch is inverted, frustum will point up when camera points down.
3. **Echo `gimbal_state_rpy_deg`**: Verify pitch sign at known gimbal positions (e.g., camera pointing 20° below horizon should give msg.y > 0 if Ry standard, < 0 if internal).
4. **Test with `gimbal_angle_order:=zyx`**: If `zyx` (no pitch negation) works correctly, confirms the double-negation hypothesis.

## Proposed Fix

**Option 1 (preferred)**: Remove the `-gimbal.y` negation from the `zxy` path in `triangulation_node.cpp`, since `gimbal_state_rpy_deg` now publishes in Ry-standard convention after ticket #022:

```cpp
// zxy path — gimbal_state_rpy_deg already in standard convention post-#022
q_gimbal = Eigen::AngleAxisd(gimbal.z, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(gimbal.x, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(gimbal.y, Eigen::Vector3d::UnitY());  // no negation
```

**Option 2**: Switch launches to `gimbal_angle_order:=zyx` which already does not negate pitch. Requires verifying that `Rz*Ry*Rx` matches the actual gimbal chain order.

**Either way**: Document the canonical `gimbal_state_rpy_deg` sign convention in `frame_conventions.md` so future changes maintain consistency.

## Affected Files

| File | Role |
|---|---|
| `src/mas_multiview/src/triangulation_node.cpp` | `zxy` rotation composition at L397-403 |
| `src/mas_multiview/launch/triangulation.launch.py` | `gimbal_angle_order` default |
| `src/tmux/simdrone1.tmuxp.yaml` | Launch with `gimbal_angle_order:=zxy` |
| `src/tmux/simdrone2.tmuxp.yaml` | Launch with `gimbal_angle_order:=zxy` |
| `src/tmux/multiview.tmuxp.yaml` | Launch with `gimbal_angle_order:=zxy` |
| `IsaacPX4/.../los_rate_controller.py` | Publishes `gimbal_state_rpy_deg` with new sign convention |

## Related

- **Ticket #022** — gimbal LOS control fix that changed `gimbal_state_rpy_deg` sign convention
- **Ticket #008** — prior `camera_pose` publisher issue (resolved via odom callback fallback)
- **Ticket #017** — zero-output guard + reprojection error filtering in triangulation

## Status

```
Flow: I -> S -> Y -> PR
Status: Resolved
```
