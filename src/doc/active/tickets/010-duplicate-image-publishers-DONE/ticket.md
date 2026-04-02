## Ticket #010: Duplicate image publishers on camera topics

### What
Camera image topics (e.g. `/px4_1/camera/color/image_raw`) have 2 publishers instead of 1. This means subscribers receive duplicate frames, which can cause incorrect detection rates, doubled processing load, and potential data races in downstream nodes.

### Why
Duplicate publishers mean YOLO detection runs on duplicate frames, triangulation gets doubled ray inputs, and bandwidth is wasted. May also cause timestamp mismatches if the two publishers run at different rates.

### Observed
```
$ ros2 topic info /px4_1/camera/color/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 2
Subscription count: 4
```

### Likely cause
Two OmniGraph camera render + ROS2 publish pipelines are being created for the same camera prim in the Isaac Sim launch script. Possibly:
- Duplicate `create_ros_action_graph` calls per vehicle
- Camera publisher created both in the ActionGraph and in a separate CameraGraph
- Multiple launch files spawning overlapping OmniGraph nodes

### Root cause (confirmed)
Two sources of duplicate camera publishers:

1. **`px4_multi_world.isaac.py`**: had BOTH `pub_graphical_sensors: True` (replicator writer) AND `create_ros_camera_graph()` active — two separate OmniGraph pipelines publishing to the same topic.

2. **All scripts**: `Camera.initialize()` in `MonocularCamera.start()` was called twice during the `world.reset()` → `timeline.play()` lifecycle (each triggers `sim_start_stop` → `start()`), creating two SDGPipeline render products and thus two `ROS2PublishImage` publishers per camera.

### Fix applied
1. Commented out `create_ros_camera_graph()` in `px4_multi_world.isaac.py` (line 213)
2. Added `_camera_initialized` guard in `MonocularCamera.start()` to prevent `Camera.initialize()` from being called twice

### Scope boundary
Fix in PegasusSimulator launch scripts only. Ensure exactly 1 publisher per camera topic per vehicle.

### Affected modules
- `IsaacPX4/PegasusSimulator/extensions/.../graphical_sensors/monocular_camera.py` (primary fix — init guard)
- `IsaacPX4/PegasusSimulator/launch/px4_multi_world.isaac.py` (removed duplicate camera graph)

### Acceptance criteria
- `ros2 topic info /px4_N/camera/color/image_raw` shows Publisher count: 1
- Camera images still publish at expected rate

### Flow
Light (I → S → Y → PR)

### Status
Done
