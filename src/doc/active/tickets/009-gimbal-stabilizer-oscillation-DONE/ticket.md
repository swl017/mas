## Ticket #009: Gimbal stabilizer oscillating violently in sim

### What
The sim gimbal (los_rate_controller + Isaac Sim articulation) oscillates violently instead of holding a stable pose. Cameras cannot point at targets, blocking all downstream detection/tracking.

### Why
Gimbal oscillation means no stable camera image → YOLO can't detect targets → triangulation/tracking/ray selection pipeline has no input. Blocks e2e verification (ticket #001).

### Scope boundary
Fix the gimbal stabilization loop only. Do not change MAS node code. Likely in `los_rate_controller` gains, joint feedback sign conventions, or Isaac Sim articulation drive parameters. Target matching parameters described in `/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_controller_cfg.py` and `/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py`. Make sure the sim dt matches to 100 Hz

### Deliverables
- Gimbal response characteristics with plots.

### Affected modules
- `IsaacPX4/ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py` — PID/position control loop
- `IsaacPX4/PegasusSimulator/` — Isaac Sim articulation drive config (stiffness, damping) for gimbal joints
- Isaac Sim `IrisGimbal3` robot model — joint axis conventions, limits

### Acceptance criteria
- Gimbal holds a commanded angle within ±2° without oscillation
- Camera image is stable (no jitter visible in Isaac Sim viewport or ROS2 image topic)
- `gimbal_state_rpy_deg` matches commanded angle after settling

### Investigation notes (from ticket #001 testing, 2026-04-01)
- `isaac_joint_states` now has 1 publisher (ticket #008 fix verified)
- `los_rate_controller` is in `position` control mode, sending `isaac_joint_commands`
- Likely causes:
  - Joint axis sign mismatch between los_rate_controller convention and Isaac Sim joint definition
  - PID gains too aggressive for the articulation drive dynamics
  - Feedback delay (joint_states pub rate vs control rate mismatch)
  - Articulation drive stiffness/damping too low, letting the controller overshoot
- `gimbal_state_rpy_deg` showed pitch ~3° when neutral was expected — possible offset or sign issue

### Root cause
The `los_rate_controller` ran with `use_sim_time: false` (wall-clock), executing its 100 Hz timer at wall-clock rate (~100 Hz) while Isaac Sim advanced at ~26 Hz. This caused:
1. Rate integration (azimuth/elevation) to overshoot ~4x — commands ran far ahead of actuator tracking
2. The implicit actuator PD (k=1000, c=50) saturated trying to catch up, causing violent transient swings
3. When commands stopped, the accumulated position error produced large overshoot as the joint snapped to the far-ahead target

### Fix
Set `use_sim_time: True` in `multi_agent_los_rate.launch.py` so the controller timer fires at sim-clock rate, keeping rate integration in sync with the physics step.

### Results (with fix, sim-time aligned measurement)
- **Steady-state**: 0.05° max error across all axes, **100% within ±2° spec**
- **Transient**: 5° yaw / 4° pitch peak during active 0.3-rate slewing (expected actuator lag)
- **No oscillation**: smooth step response, clean settling
- See [gimbal_response.png](gimbal_response.png)

### Flow
Light (I → S → Y → PR)

### Status
Done
