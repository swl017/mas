# MAS (Multi-Agent System) Architecture

Multi-agent drone observation system: 2D detection, multi-view triangulation, 3D tracking, and gimbal control.

## Node Graph

```
┌──────────────────────────────────────────────────────────────────────┐
│                         Launch / Composition                         │
└──────────────────────────────────────────────────────────────────────┘

Per-vehicle nodes:                        Centralized nodes:
┌─────────────────────┐                  ┌──────────────────────────┐
│ ultralytics_ros     │                  │ mas_multiview_py         │
│  tracker_node       │──yolo_result───→│  multiview_node          │
│  (YOLO + ByteTrack) │  vision_msgs     │  (triangulation)         │
└─────────────────────┘                  └──────────┬───────────────┘
                                                    │ viz markers
┌─────────────────────┐                             ▼
│ mas_common_frame    │                  ┌──────────────────────────┐
│  common_frame_node  │──odom──────────→│ mas_tracker              │
│  (GPS→common frame) │  (also to       │  sort3d_tracking_node    │
└─────────────────────┘   multiview)    │  (Kalman + Hungarian)    │
                                        └──────────┬───────────────┘
                                                   │ chosen_target_pose
                                                   ▼
                                        ┌──────────────────────────┐
                                        │ gimbal_controller        │
                                        │  point_to_region_node    │
                                        │  (gimbal pointing)       │
                                        └──────────┬───────────────┘
                                                   │ gimbal_command
                                                   ▼
                                        ┌──────────────────────────┐
                                        │ gimbal_controller        │
                                        │  siyi_gimbal_node        │
                                        │  (hardware interface)    │
                                        └──────────────────────────┘
```

## Directed Dependencies

```
ultralytics_ros/tracker_node ──[yolo_result_vision]──→ mas_multiview_py/multiview_node
mas_common_frame/common_frame_node ──[common_frame/odom]──→ mas_multiview_py/multiview_node
gimbal_controller/siyi_gimbal_node ──[gimbal_state_rpy_deg]──→ mas_multiview_py/multiview_node
mas_multiview_py/multiview_node ──[visualization markers]──→ mas_tracker/sort3d_tracking_node
mas_tracker/sort3d_tracking_node ──[target_region]──→ gimbal_controller/point_to_region_node
mas_common_frame/common_frame_node ──[common_frame/pose]──→ gimbal_controller/point_to_region_node
gimbal_controller/point_to_region_node ──[gimbal_command_rpy_deg]──→ gimbal_controller/siyi_gimbal_node
```

## Data Flow

```
Camera Images (per vehicle)
  │
  ▼
[ultralytics_ros/tracker_node] ─── 2D detections (Detection2DArray)
  │
  ├─── + Camera Info (per vehicle)
  ├─── + Odometry from [mas_common_frame] (per vehicle)
  ├─── + Gimbal angles from [gimbal_controller/siyi_gimbal_node] (per vehicle)
  │
  ▼
[mas_multiview_py/multiview_node] ─── 3D triangulated points (MarkerArray)
  │
  ▼
[mas_tracker/sort3d_tracking_node] ─── Tracked targets (Detection3DArray, PoseStamped)
  │
  ├─── + Robot pose from [mas_common_frame]
  ├─── + Camera info
  │
  ▼
[gimbal_controller/point_to_region_node] ─── Gimbal commands (Vector3)
  │
  ▼
[gimbal_controller/siyi_gimbal_node] ─── Hardware actuation

Coordinate Transforms:
  GPS/Local (MAVRos) → [mas_common_frame] → common_frame (broadcasts TF2)
```

## Topic/Service Interface

| Topic | Msg Type | Publisher | Subscriber | QoS |
|-------|----------|----------|------------|-----|
| `/{veh}/yolo_result_vision` | vision_msgs/Detection2DArray | ultralytics_ros | mas_multiview_py | default |
| `/{veh}/camera/color/camera_info` | sensor_msgs/CameraInfo | camera driver | mas_multiview_py, gimbal_controller | default |
| `/{veh}/common_frame/odom` | nav_msgs/Odometry | mas_common_frame | mas_multiview_py | BEST_EFFORT |
| `/{veh}/common_frame/pose` | geometry_msgs/PoseStamped | mas_common_frame | gimbal_controller | BEST_EFFORT |
| `/{veh}/gimbal_state_rpy_deg` | geometry_msgs/Vector3 | gimbal_controller | mas_multiview_py, gimbal_controller | BEST_EFFORT |
| `~/visualization` | visualization_msgs/MarkerArray | mas_multiview_py | mas_tracker | default |
| `tracked_objects/class_{i}` | vision_msgs/Detection3DArray | mas_tracker | — | default |
| `chosen_target_pose` | geometry_msgs/PoseStamped | mas_tracker | — | default |
| `target_region` | geometry_msgs/PointStamped | mas_tracker | gimbal_controller | default |
| `gimbal_command_rpy_deg` | geometry_msgs/Vector3 | gimbal_controller | gimbal_controller | default |

## Parameters

| Parameter | Type | Default | Node | Description |
|-----------|------|---------|------|-------------|
| `vehicle_name_prefix` | string | `"px4_"` | common_frame_node | Vehicle namespace prefix |
| `num_vehicles` | int | `2` | common_frame_node | Number of vehicles |
| `common_frame_origin` | float[] | `[37.7749, -122.4194, 0.0]` | common_frame_node | GPS origin [lat, lon, alt] |
| `yolo_model` | string | `"yolov8n.pt"` | tracker_node | YOLO model file |
| `conf_thres` | double | `0.25` | tracker_node | Detection confidence threshold |
| `publish_rate` | double | `1.0` | multiview_node | Triangulation rate (Hz) |
| `detection_topics` | string[] | — | multiview_node | Per-vehicle detection topics |
| `association_distance_threshold` | double | `1.0` | sort3d_tracking_node | Track association threshold |
| `max_track_age` | int | `30` | sort3d_tracking_node | Frames before track deletion |
| `server_ip` | string | `"192.168.144.25"` | siyi_gimbal_node | SIYI gimbal IP |
| `publish_rate_hz` | double | `25.0` | siyi_gimbal_node | Gimbal state publish rate |

## Node Isolation

**Standalone** (no inter-package topic dependencies):
- `mas_common_frame/common_frame_node` — only needs MAVRos topics
- `ultralytics_ros/tracker_node` — only needs camera images
- `gimbal_controller/siyi_gimbal_node` — only needs gimbal command topic

**Has dependencies** (connected via topics):
- `mas_multiview_py/multiview_node` — needs detections, odom, camera info, gimbal state
- `mas_tracker/sort3d_tracking_node` — needs triangulated markers
- `gimbal_controller/point_to_region_node` — needs target, pose, camera info, gimbal state

## Package Summary

| Package | Build Type | Nodes | Role |
|---------|-----------|-------|------|
| `ultralytics_ros` | ament_cmake (hybrid) | tracker_node, tracker_with_cloud_node | 2D/3D YOLO detection |
| `mas_common_frame` | ament_python | common_frame_node, common_frame_node_single | GPS→common frame transforms |
| `mas_multiview_py` | ament_python | multiview_node | Multi-view triangulation |
| `mas_multiview` | ament_cmake | triangulation_node | Multi-view triangulation (C++) |
| `mas_tracker` | ament_cmake | sort3d_tracking_node | 3D multi-object tracking |
| `gimbal_controller` | ament_python | siyi_gimbal_node, point_to_region_node | Gimbal hardware + pointing |

## File Conventions

- `CONTEXT.md` per package — node routing contracts (topics, services, parameters)
- `config/*.yaml` — parameter files
- `launch/*.launch.py` — launch files
- `doc/*_spec.md` — authoritative specifications
