# ultralytics_ros

## Purpose
YOLO-based 2D object detection with ByteTrack tracking. Optional 3D projection using LiDAR point clouds.

## Nodes

### tracker_node (Python)
**File:** `script/tracker_node.py`
**Pattern:** Coupled (subscriber receives image → runs YOLO inference → publishes detections immediately)

#### Subscriptions
- `image_raw` or `image_raw/compressed` (`sensor_msgs/Image` or `CompressedImage`) — camera input. Subscription uses `qos_profile_sensor_data` (BEST_EFFORT, KEEP_LAST, depth=5) to match the standard sensor-stream QoS used by `rtsp_camera` and Isaac Sim's camera publisher. Whether the compressed or raw topic is used is decided by whether `input_topic` ends in `compressed`.

#### Publishers
- `yolo_result` (`ultralytics_ros/YoloResult`) — raw YOLO detections with tracker IDs
- `yolo_result_vision` (`vision_msgs/Detection2DArray`) — standard vision format with tracker IDs in Detection2D.id (consumed by mas_multiview)
- `yolo_result_active` (`std_msgs/Bool`) — true when detections are present (compact cross-agent detection status)
- `yolo_image` (`sensor_msgs/Image`) — annotated image with drawn detections

#### Parameters
- `yolo_model` (`string`, default: `"yolov8n.pt"`) — model file path
- `input_topic` (`string`, default: `"image_raw"`) — input image topic
- `result_topic` (`string`, default: `"yolo_result"`) — output topic name
- `result_image_topic` (`string`, default: `"yolo_image"`) — annotated image topic
- `conf_thres` (`double`, default: `0.25`) — detection confidence threshold
- `iou_thres` (`double`, default: `0.45`) — NMS IoU threshold
- `max_det` (`int`, default: `300`) — max detections per frame
- `classes` (`int[]`, default: `0-79`) — YOLO class filter
- `tracker` (`string`, default: `"bytetrack.yaml"`) — tracker config
- `device` (`string`, default: `"cpu"`) — inference device (cpu/cuda)

---

### tracker_with_cloud_node (C++)
**File:** `src/tracker_with_cloud_node.cpp` | **Header:** `include/tracker_with_cloud_node/tracker_with_cloud_node.h`
**Pattern:** Coupled (synchronized subscriber callback processes all inputs and publishes 3D detections)

#### Subscriptions (time-synchronized)
- `camera_info` (`sensor_msgs/CameraInfo`) — camera intrinsics
- `points_raw` (`sensor_msgs/PointCloud2`) — LiDAR point cloud
- `yolo_result` (`ultralytics_ros/YoloResult`) — 2D detections from tracker_node

All three `message_filters::Subscriber` endpoints use `rmw_qos_profile_sensor_data` (BEST_EFFORT, KEEP_LAST, depth=5) so they match tracker_node's BEST_EFFORT publisher and typical camera-info / LiDAR-driver publishers. This path is not built when PCL is unavailable.

#### Publishers
- `yolo_3d_result` (`vision_msgs/Detection3DArray`) — 3D bounding boxes
- `detection_cloud` (`sensor_msgs/PointCloud2`) — points within detections
- `detection_marker` (`visualization_msgs/MarkerArray`) — RViz visualization

#### Parameters
- `camera_info_topic`, `lidar_topic`, `yolo_result_topic`, `yolo_3d_result_topic` (`string`) — topic names
- `cluster_tolerance` (`float`, default: `0.5`) — Euclidean clustering tolerance
- `voxel_leaf_size` (`float`, default: `0.5`) — voxel grid downsampling
- `min_cluster_size` (`int`, default: `100`) — minimum cluster points
- `max_cluster_size` (`int`, default: `25000`) — maximum cluster points

## Custom Messages
- `ultralytics_ros/YoloResult` — Detection2DArray + optional segmentation masks

## Dependencies
None (standalone). Publishes detections consumed by mas_multiview.

## Key Files
- `script/tracker_node.py` — Python YOLO detection node
- `src/tracker_with_cloud_node.cpp` — C++ LiDAR fusion node
- `include/tracker_with_cloud_node/tracker_with_cloud_node.h` — C++ node header
- `msg/YoloResult.msg` — Custom message definition
