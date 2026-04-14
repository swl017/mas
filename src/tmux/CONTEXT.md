# tmux

## Purpose
Tmuxp session configs for launching the MAS system. Each YAML file defines a tmux session with windows for the required ROS2 nodes.

## Session Configs

### Per-Drone (Simulation)
- `simdrone1.tmuxp.yaml` — px4_1: mavros, common_frame, ultralytics_ros + multi-vehicle nodes (offboard, policy, tracker)
- `simdrone2.tmuxp.yaml` — px4_2: mavros, common_frame, ultralytics_ros + multi-vehicle nodes (offboard, policy, tracker)
- `simdrone3.tmuxp.yaml` — px4_3: mavros, common_frame, ultralytics_ros + multi-vehicle nodes (offboard, policy, tracker)

Mavros SITL ports follow PX4 multi-instance convention:
| Vehicle | fcu_url | tgt_system |
|---------|---------|------------|
| px4_1 | `udp://:14541@localhost:14551` | 2 |
| px4_2 | `udp://:14542@localhost:14552` | 3 |
| px4_3 | `udp://:14543@localhost:14553` | 4 |

Multi-vehicle nodes (offboard, policy, tracker) run **once** in simdrone1. They manage all vehicles via config files or global topics.

### Per-Drone (Hardware)
- `drone.tmuxp.yaml` — single physical drone session (sources `drone_config/robot.env`)

### Multi-Node / Shared
- `mas_core_multi.tmuxp.yaml` — multi-vehicle core (common_frame, multiview, tracker)
- `multiview.tmuxp.yaml` — triangulation node standalone
- `simulation.tmuxp.yaml` — Isaac Sim launcher
- `camera_yolo.yaml` — standalone YOLO detection
- `monitor_topics.tmuxp.yaml` — topic monitoring utilities

### Micro-XRCE / Bridge
- `bridge.tmux.yaml` — micro-XRCE-DDS agent bridge
- `bridge_topics_px4_1.yaml` / `bridge_topics_px4_2.yaml` — per-drone bridge topic configs

### Other
- `zoom.tmux.yaml` — zoom/gimbal utilities

## Inter-Session Contracts (Simulation)

The simdrone sessions depend on the Isaac Sim session (`IsaacPX4/tmux/isaac_sim.tmuxp.yaml`):

**Isaac Sim → MAS** (simulator publishes, MAS subscribes):
| Topic | Type | Consumer |
|-------|------|----------|
| `/{veh}/camera/color/image_raw` | `sensor_msgs/Image` | ultralytics_ros |
| `/{veh}/camera/color/camera_info` | `sensor_msgs/CameraInfo` | gimbal_controller |

**Isaac Sim gimbal_stabilizer → MAS** (los_rate_controller publishes):
| Topic | Type | Consumer |
|-------|------|----------|
| `/{veh}/gimbal_state_rpy_deg` | `geometry_msgs/Vector3` | mas_policy |

**MAS → Isaac Sim** (MAS publishes, simulator subscribes):
| Topic | Type | Producer |
|-------|------|----------|
| `/{veh}/gimbal_cmd_los_rate` | `geometry_msgs/Vector3` | mas_policy |
| `/{veh}/zoom_rate_cmd` | `std_msgs/Float32` | mas_policy |

## Usage
```bash
# Start simulator first
tmuxp load ~/IsaacPX4/tmux/isaac_sim.tmuxp.yaml

# Then start drone sessions
tmuxp load tmux/simdrone1.tmuxp.yaml
tmuxp load tmux/simdrone2.tmuxp.yaml
tmuxp load tmux/simdrone3.tmuxp.yaml
```
