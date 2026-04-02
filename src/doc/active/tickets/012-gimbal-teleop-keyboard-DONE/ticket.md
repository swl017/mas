## Ticket #012: Keyboard teleop for gimbal LOS rate control

### What
Create a ROS2 teleop node that reads keyboard input and publishes normalized LOS rate commands to `gimbal_cmd_los_rate` (geometry_msgs/Vector3), allowing manual gimbal control during sim testing.

### Why
Currently the only way to move the gimbal is via the policy node's action output. For debugging the gimbal pipeline (los_rate_controller, point_to_region, triangulation), we need a way to manually slew the gimbal without running the full RL policy. The Isaac Lab environment already has a keyboard teleop (see reference below) — this brings equivalent functionality to the ROS2 deployment side.

### Reference
`/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/teleop_iris_ma6.py`

Key design points from the reference:
- Gimbal yaw/pitch are **rate commands** normalized to [-1, 1]
- Keyboard keys are Z/X for yaw left/right, T/G for pitch up/down
- Hold-to-move: rate is non-zero only while key is held
- Sensitivity parameter scales the output magnitude (default 0.1 per key)

### Scope boundary
- Single new node in `gimbal_stabilizer` package (or standalone script)
- Publishes only to `gimbal_cmd_los_rate` (Vector3, BEST_EFFORT QoS)
- Does not modify `los_rate_controller`, policy, or any MAS nodes
- Terminal-based input (curses or raw stdin) — no GUI dependency

### Design sketch
```
gimbal_teleop_keyboard
  Params:
    az_sensitivity: 0.3    # normalized rate per keypress [-1, 1]
    el_sensitivity: 0.3
    max_vel: 180.0         # 180.0 deg/s
    publish_rate: 50.0     # Hz
    namespace: ""          # e.g., /px4_1

  Publishes:
    gimbal_cmd_los_rate (Vector3, BEST_EFFORT)
      x = azimuth rate   [-1, 1]
      x *= max_vel
      y = elevation rate  [-1, 1]
      y *= max_vel
      z = 0 (unused)

  Key mapping (matching Isaac Lab reference):
    Z / X  →  azimuth  +/−
    T / G  →  elevation +/−
    Q / ESC → quit
```

Implementation options:
1. **Python curses** — portable, no deps, works in tmux panes
2. **Raw termios** — lighter, same idea
3. Reuse `teleop_twist_keyboard` pattern — familiar to ROS users

### Acceptance criteria
- Node launches, prints key bindings to terminal
- Holding Z/X slews gimbal in azimuth; releasing stops
- Holding T/G slews gimbal in elevation; releasing stops
- `ros2 topic echo gimbal_cmd_los_rate` shows non-zero during keypress, zero on release
- Works inside tmux pane (no X11/GUI required)
- Configurable namespace for multi-drone use (`--ros-args -r __ns:=/px4_1`)

### Integration
Add a `teleop` window to simdrone tmux configs (optional, commented out by default):
```yaml
# - window_name: teleop
#   panes:
#     - ros2 run gimbal_stabilizer gimbal_teleop_keyboard --ros-args -r __ns:=/${ROBOT_NAME}
```

### Flow
Light (I → S → Y → PR)

### Status
Done

### Implementation (2026-04-01)
- `gimbal_stabilizer/gimbal_teleop_keyboard.py` — curses-based ROS2 node, publishes `gimbal_cmd_los_rate` (Vector3, BEST_EFFORT, 50 Hz)
- Entry point added to `gimbal_stabilizer/setup.py`
- Teleop window added to `simdrone1.tmuxp.yaml` and commented in `simdrone2.tmuxp.yaml`
