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
Blocked — deployment/wiring bugs fixed, e2e blocked on sim joint state feedback (ticket #008)

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

### Blockers
- ~~`isaac_joint_states` has 0 publishers — sim OmniGraph not publishing joint feedback. See ticket #008.~~ Resolved.
- Gimbal stabilizer oscillating violently — cameras can't hold stable pose. See ticket #009.

### Next
Resolve ticket #009, then rerun e2e test
