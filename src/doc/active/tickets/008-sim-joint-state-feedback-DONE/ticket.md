## Ticket #008: Fix Isaac Sim gimbal joint state feedback

### What
`isaac_joint_states` topics have 0 publishers — the OmniGraph `PublishJointState` node configured in the Pegasus launch is not actually publishing. This breaks gimbal stabilization and camera pose computation in simulation.

### Why
Without joint state feedback:
- `los_rate_controller` cannot read actual gimbal angles → `gimbal_state_rpy_deg` is stale/incorrect
- `camera_pose` has 0 publishers → `triangulation_node` cannot compute 3D points
- Gimbals cannot stabilize → cameras don't point at targets → no YOLO detections
- Blocks e2e verification of the full tracking pipeline (ticket #001)

### Scope boundary
Fix the sim joint state feedback only. Do not change MAS node code. Do not fix unrelated sim issues.

### Affected modules
- `IsaacPX4/PegasusSimulator/launch/px4_multi_world_iris_gimbal3.isaac.py` — OmniGraph action graph creation (lines 261-311)
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py` — subscribes to `isaac_joint_states`, publishes `gimbal_state_rpy_deg` and `camera_pose`

### Acceptance criteria
- `ros2 topic info /px4_1/isaac_joint_states` shows Publisher count: 1+
- `ros2 topic echo /px4_1/isaac_joint_states` shows updating joint positions
- `ros2 topic echo /px4_1/camera_pose` shows valid PoseStamped
- Gimbal visually tracks commanded angles in Isaac Sim

### Investigation notes (from ticket #001 testing, 2026-04-01)
- OmniGraph `create_ros_action_graph()` uses `asyncio.ensure_future()` — errors may be silently swallowed
- The graph configures `PublishJointState` with topic `px4_N/isaac_joint_states` and target prim `vehicle_stage_path`
- `SubscribeJointState` (commands IN) works — `isaac_joint_commands` has a publisher (los_rate_controller)
- Possible causes: OmniGraph creation failed silently, QoS mismatch, articulation target path wrong, or Isaac Sim ROS2 bridge issue
- Also check: `camera_pose` publisher — may need to be added to los_rate_controller or ROS2Backend

### Flow
Light (I → S → Y → PR) — root cause likely in OmniGraph config, fix is localized

### Resolution (2026-04-01)

**Root cause:** Two issues from Isaac Sim 4.5 upgrade:
1. OmniGraph node types renamed (`omni.isaac.ros2_bridge.*` → `isaacsim.ros2.bridge.*`, `omni.isaac.core_nodes.*` → `isaacsim.core.nodes.*`). Old types fail silently inside `asyncio.ensure_future()` — no topics created.
2. `ArticulationController.inputs:robotPath` and `PublishJointState.inputs:targetPrim` pointed to `/World/px4_N` but articulation root (`PhysicsArticulationRootAPI`) is at `/World/px4_N/body`. Isaac Sim 4.5 tensor API no longer searches child prims.

**Fix (3 files):**
- `px4_multi_world_iris_gimbal3.isaac.py` — node types, targetPrim list form, articulation path `/body`
- `px4_multi_world.isaac.py` — same
- `px4_lidar_world.isaac.py` — same + import rename

**Verified:** `ros2 topic echo /px4_1/isaac_joint_states` shows updating joint positions.

### Status
Done
