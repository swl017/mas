## Ticket #001: Sim e2e test — tracker ray selection

### What
Verify `chosen_target_ray_w` publishes with ID-based matching in a 3-agent sim run

### Why
Code for tracker ray selection is done but untested; can't mark feature complete without e2e verification

### Scope boundary
Do not change sort3d or observation_assembler code unless a bug is found during testing. Do not fix unrelated sim issues.

### Affected modules
`mas_tracker`, `mas_multiview`, `mas_policy`

### Acceptance criteria
`ros2 topic echo` shows `chosen_target_ray_w` with valid origin/direction and non-empty `detection_id` while tracking a target. Policy observation vector incorporates the ray.

### Flow
Direct fix (testing only; code changes only if bugs found)

### Status
Done — all acceptance criteria verified (2026-04-02)

### Changes made (2026-04-01)
- **triangulation_node.cpp**: renamed per-camera viz topic `{prefix}{i}/triangulated_points` → `{prefix}{i}/triangulated_points_viz` (type collision when namespaced)
- **triangulation.launch.py**: added `ns` launch argument for per-drone deployment
- **mission_deploy.launch.py, offboard.launch.py, policy_deploy.launch.py**: added `vehicle_filter` argument for single-vehicle launches
- **simdrone1.tmuxp.yaml**: fully self-contained — all per-drone nodes (triangulation, tracker, gimbal, mission, offboard, policy) with `vehicle_filter:=${ROBOT_NAME}`
- **simdrone2.tmuxp.yaml**: mirror of simdrone1 with px4_2 params
- **simdrone3.tmuxp.yaml**: added offboard window for target drone
- **multiview.tmuxp.yaml**: fixed galactic→humble; now standalone alternative (not needed with simdrone sessions)
- **ARCHITECTURE.md**: updated session layout, node categories, deployment steps to reflect per-drone model
- **cv_bridge**: built from source for humble (cmake args: `-DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DBoost_PYTHON_VERSION=3.8`)
- **vehicles.yaml**: moved drones closer (10m separation, 7m altitude) for easier visual verification

### Blockers (all resolved)
- ~~`isaac_joint_states` has 0 publishers — sim OmniGraph not publishing joint feedback. See ticket #008.~~ Done.
- ~~Gimbal stabilizer oscillating violently — cameras can't hold stable pose. See ticket #009.~~ Done.
- ~~`gimbal_controller` setup.py missing subpackages — `point_to_region_node` failed to import.~~ Fixed with `find_packages()`.
- ~~`cv_bridge` not built for humble.~~ Built from source with `-DBoost_PYTHON_VERSION=3.8`.

### E2E verification results (2026-04-02)

| Check | px4_1 | px4_2 |
|-------|-------|-------|
| `triangulated_points` | ✓ target at (0.0, -0.1, 2.7) | ✓ (same data, per-drone triangulation) |
| sort3d tracking | ✓ target_id=1, 1 active track | ✓ target_id=1 |
| `target_rays_w` (ego cam) | ✓ detection_id='1' | ✓ |
| `chosen_target_ray_w` | ✓ dir=(1.0, 0.008, -0.021) | ✓ dir=(-0.70, 0.71, -0.019) |
| `chosen_target_pose` | ✓ with covariance | ✓ |
| Policy observation | ✓ obs_dim=52, ray incorporated | ✓ |

**Ray selection**: ID-based matching confirmed — `target_rays_w` carries `detection_id: '1'`, matched against nearest TriangulatedPoint.
