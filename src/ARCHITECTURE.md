# MAS (Multi-Agent System) Architecture

Multi-agent drone observation system: 2D detection, multi-view triangulation, 3D tracking, gimbal control, and learned policy deployment. All ROS2 nodes run per-vehicle.

## Architecture Diagram

```mermaid
flowchart LR
    subgraph MAS["MAS — Per-vehicle (all nodes run in /{veh}/ namespace)"]
        direction TB

        CAM["Camera Driver<br/><i>camera/color/image_raw</i><br/><i>camera/color/camera_info</i>"]
        YOLO["ultralytics_ros<br/><i>tracker_node</i>"]
        MV["mas_multiview<br/><i>triangulation_node</i>"]
        TRK["mas_tracker<br/><i>sort3d_tracking_node</i>"]
        POL["mas_policy<br/><i>policy_node</i>"]
        CF["mas_common_frame<br/><i>common_frame_node</i>"]
        OFF["mas_offboard<br/><i>offboard_control</i>"]
        GC_SIYI["gimbal_controller<br/><i>siyi_gimbal_node</i>"]
        GC_PTR["gimbal_controller<br/><i>point_to_region_node</i>"]
        MAV["MAVROS"]

        CAM -- "image_raw" --> YOLO
        CAM -- "camera_info" --> MV
        CAM -- "camera_info" --> GC_PTR
        YOLO -- "yolo_result_vision" --> MV
        YOLO -- "yolo_result_vision" --> POL
        CF -- "common_frame/odom" --> MV
        CF -- "common_frame/odom" --> POL
        CF -- "common_frame/pose" --> GC_PTR
        GC_SIYI -- "gimbal_state_rpy_deg" --> MV
        GC_SIYI -- "gimbal_state_rpy_deg" --> GC_PTR
        MV -- "triangulated_points" --> TRK
        MV -- "triangulated_covariance" --> POL
        TRK -- "chosen_target_pose" --> POL
        TRK -- "target_region" --> GC_PTR
        MAV -- "pose, vel, imu" --> CF
        MAV -- "imu/data" --> POL
        MAV -- "state, odom, pose" --> OFF
        GC_PTR -- "gimbal_command_rpy_deg" --> GC_SIYI
        POL -- "cmd_vel" --> OFF
        POL -- "gimbal_cmd_los_rate" --> GC_SIYI
        POL -- "zoom_cmd" --> GC_SIYI
        OFF -- "mavros setpoints" --> MAV
    end

    subgraph SIM["Sim Env"]
        direction TB
        ISIM["Isaac Sim"]
        PEG["PegasusSimulator"]
        PX4["PX4 SITL"]
        GSTAB["gimbal_stabilizer"]
        MAV_TGT["MAVROS<br/>Target vehicle"]
        CF_TGT["mas_common_frame<br/>(target)"]

        ISIM --> PEG --> PX4
        PX4 --> PEG --> ISIM
        MAV_TGT --> CF_TGT
    end

    MISSION(["Mission Start"]) -. "triggers POLICY state" .-> OFF
    MAV -- "mavlink" --- PX4
    OFF -- "mavros setpoints" --> GSTAB
    POL -- "gimbal_cmd_los_rate, zoom_cmd" --> GSTAB
    PEG -- "camera feeds" --> CAM

    style MAS fill:#dbeeff,stroke:#4a90d9
    style SIM fill:#ffe0d0,stroke:#d97a4a
    style MISSION fill:#d4edda,stroke:#6abf69
```

### Mission Phases

- **Before mission:** `point_to_region_node` computes gimbal angles from tracked target → `siyi_gimbal_node` actuates. `offboard_control` runs state machine (INIT→ARM→TAKEOFF→HOVER).
- **After mission start:** `offboard_control` enters POLICY state. `mas_policy` takes over: publishes `cmd_vel`, `gimbal_cmd_los_rate`, `zoom_cmd`.

## Topic/Service Interface

All topics below are per-vehicle, resolved within `/{veh}/` namespace unless noted.

| Topic | Msg Type | Publisher | Subscriber | QoS |
|-------|----------|----------|------------|-----|
| `image_raw` | sensor_msgs/Image | camera driver | ultralytics_ros | default |
| `camera/color/camera_info` | sensor_msgs/CameraInfo | camera driver | mas_multiview, point_to_region_node | default |
| `yolo_result_vision` | vision_msgs/Detection2DArray | ultralytics_ros | mas_multiview, mas_policy | BEST_EFFORT |
| `common_frame/odom` | nav_msgs/Odometry | mas_common_frame | mas_multiview, mas_policy | BEST_EFFORT |
| `common_frame/pose` | geometry_msgs/PoseStamped | mas_common_frame | point_to_region_node | BEST_EFFORT |
| `gimbal_state_rpy_deg` | geometry_msgs/Vector3 | siyi_gimbal_node | mas_multiview, point_to_region_node | BEST_EFFORT |
| `camera/zoom` | std_msgs/Float64 | — | mas_multiview | default |
| `camera_pose` | geometry_msgs/PoseStamped | — | mas_multiview | default |
| `triangulated_points` | visualization_msgs/MarkerArray | mas_multiview | mas_tracker | default |
| `tracked_objects/class_{i}` | vision_msgs/Detection3DArray | mas_tracker | — | default |
| `chosen_target_pose` | geometry_msgs/PoseStamped | mas_tracker | mas_policy | default |
| `target_region` | geometry_msgs/PointStamped | mas_tracker | point_to_region_node | default |
| `gimbal_command_rpy_deg` | geometry_msgs/Vector3 | point_to_region_node | siyi_gimbal_node | default |
| `gimbal_state_rpy_rad` | geometry_msgs/Vector3 | los_rate_controller | mas_policy | default |
| `cmd_vel` | geometry_msgs/TwistStamped | mas_policy | offboard_control | BEST_EFFORT |
| `gimbal_cmd_los_rate` | geometry_msgs/Vector3 | mas_policy | los_rate_controller | default |
| `zoom_cmd` | std_msgs/Float32 | mas_policy | — | default |
| `mavros/state` | mavros_msgs/State | MAVROS | offboard_control | RELIABLE |
| `mavros/local_position/pose` | geometry_msgs/PoseStamped | MAVROS | offboard_control | RELIABLE |
| `mavros/local_position/odom` | nav_msgs/Odometry | MAVROS | offboard_control | RELIABLE |
| `mavros/setpoint_velocity/cmd_vel` | geometry_msgs/TwistStamped | offboard_control | MAVROS | default |
| `mavros/setpoint_position/local` | geometry_msgs/PoseStamped | offboard_control | MAVROS | default |
| `mavros/imu/data` | sensor_msgs/Imu | MAVROS | mas_policy | default |

### Services

| Service | Type | Node | Notes |
|---------|------|------|-------|
| `mavros/cmd/arming` | mavros_msgs/CommandBool | offboard_control (client) | Arm/disarm |
| `mavros/set_mode` | mavros_msgs/SetMode | offboard_control (client) | Set OFFBOARD mode |
| `~/reset_hidden_state` | std_srvs/Trigger | mas_policy | Reset GRU hidden states |

## Parameters

| Parameter | Type | Default | Node | Description |
|-----------|------|---------|------|-------------|
| `vehicle_name_prefix` | string | `"px4_"` | common_frame_node | Vehicle namespace prefix |
| `num_vehicles` | int | `2` | common_frame_node | Number of vehicles |
| `common_frame_origin` | float[] | `[37.7749, -122.4194, 0.0]` | common_frame_node | GPS origin [lat, lon, alt] |
| `yolo_model` | string | `"yolov8n.pt"` | tracker_node | YOLO model file |
| `conf_thres` | double | `0.25` | tracker_node | Detection confidence threshold |
| `publish_rate` | double | `10.0` | triangulation_node | Triangulation rate (Hz) |
| `num_camera` | int | `3` | triangulation_node | Number of cameras |
| `camera_name_prefix` | string | `"/px4_"` | triangulation_node | Prefix for per-camera topics |
| `max_reprojection_error` | double | `100.0` | triangulation_node | Max reprojection error (px) |
| `association_distance_threshold` | double | `1.0` | sort3d_tracking_node | Track association threshold |
| `max_track_age` | int | `30` | sort3d_tracking_node | Frames before track deletion |
| `server_ip` | string | `"192.168.144.25"` | siyi_gimbal_node | SIYI gimbal IP |
| `publish_rate_hz` | double | `25.0` | siyi_gimbal_node | Gimbal state publish rate |
| `vehicle_name` | string | `""` | offboard_control | Vehicle namespace prefix |
| `update_rate` | float | `100.0` | offboard_control | Timer callback frequency (Hz) |
| `target_system` | int | `1` | offboard_control | PX4 MAVLink system ID |
| `position.x/y/z` | float | `0.0` | offboard_control | Waypoint position (ENU, m) |
| `position.yaw_deg` | float | `0.0` | offboard_control | Waypoint yaw (degrees) |
| `takeoff_speed` | float | `3.0` | offboard_control | Climb rate (m/s) |
| `checkpoint_path` | string | `""` | policy_node | Path to SKRL .pt checkpoint |
| `num_agents` | int | `2` | policy_node | Number of agents |
| `vehicle_names` | string[] | `["px4_1","px4_2"]` | policy_node | Vehicle namespace prefixes |
| `architecture` | string | `"mappo_rnn"` | policy_node | Policy network type |
| `control_frequency` | double | `25.0` | policy_node | Inference loop rate (Hz) |
| `enable_cbf` | bool | `true` | policy_node | Enable CBF inter-agent safety filter |

## Node Isolation

**Standalone** (no inter-package topic dependencies):
- `mas_common_frame/common_frame_node` — only needs MAVROS topics
- `ultralytics_ros/tracker_node` — only needs camera images
- `gimbal_controller/siyi_gimbal_node` — only needs gimbal command topic
- `mas_offboard/offboard_control` — only needs MAVROS topics + `cmd_vel`

**Has dependencies** (connected via topics):
- `mas_multiview/triangulation_node` — needs detections, odom, camera info, gimbal state, zoom (from all vehicles)
- `mas_tracker/sort3d_tracking_node` — needs triangulated markers
- `gimbal_controller/point_to_region_node` (before mission) — needs target, pose, camera info, gimbal state
- `mas_policy/policy_node` (after mission start) — needs ego odom/IMU/gimbal/detections + peer odom/gimbal/detections + triangulation

## Package Summary

| Package | Build Type | Nodes | Role |
|---------|-----------|-------|------|
| `ultralytics_ros` | ament_cmake (hybrid) | tracker_node, tracker_with_cloud_node | 2D/3D YOLO detection |
| `mas_common_frame` | ament_python | common_frame_node, common_frame_node_single | GPS→common frame transforms |
| `mas_multiview` | ament_cmake | triangulation_node | Multi-view triangulation (C++, Ceres) |
| `mas_tracker` | ament_cmake | sort3d_tracking_node | 3D multi-object tracking (SORT) |
| `gimbal_controller` | ament_python | siyi_gimbal_node, point_to_region_node | Gimbal hardware + pointing |
| `mas_policy` | ament_python | policy_node | MARL policy inference + CBF safety |
| `mas_offboard` | ament_python | offboard_control | Per-vehicle PX4 offboard controller |

## File Conventions

- `CONTEXT.md` per package — node routing contracts (topics, services, parameters)
- `config/*.yaml` — parameter files
- `launch/*.launch.py` — launch files
- `doc/*_spec.md` — authoritative specifications
