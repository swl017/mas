# Ticket #027 — Dronecop YOLO Weight Integration

## Goal
Integrate the `dronecop9-2.pt` drone/bird detection model into the MAS system.
Determine whether `ultralytics_ros` can be reused as-is or if a new node is needed.

## Background

### Weight file
- **Path:** `/home/usrg/mas/resource/dronecop9-2.pt`
- **Architecture:** YOLOv8s-p2 (`yolov8s-p2.yaml`, scale `s`)
- **Classes (2):** `{0: 'drone', 1: 'bird'}`
- **Training input size:** 1920px
- **Also available:** TensorRT engine (`dronecop9-2_fp16_jetson_orin_nx.engine`), `.wts` export

### Reference code (`kari-dronecop-rd-imgproc`)
- Located at `/home/usrg/mas/resource/kari-dronecop-rd-imgproc/`
- **ROS1** package — uses TensorRT C++ engine + pycuda for inference
- Key files:
  - `scripts/sahi_tensorrt.py` — ROS1 TensorRT inference node (pycuda, custom plugin)
  - `src/main_kari_rd_yolov8p2_infer_node.cpp` — C++ TensorRT inference
  - `src/main_kari_rd_pf_track_node.cpp` — particle filter tracker
  - `include/yolov8_p2_infer_engine/` — TRT engine wrapper headers
  - `include/yolov8_p2_lib/` — YOLOv8-p2 specific lib headers
- **Not directly reusable** (ROS1, hardcoded paths, TRT-only pipeline)

### Additional models in `resource/models/`
- YOLOv11 variants (n/s/m/l/x) with `.pt`, `.onnx`, `.engine` — single-class drone detectors
- May be useful as alternatives or for benchmarking

## Analysis: Can `ultralytics_ros` be reused?

**Yes, very likely.** The existing `ultralytics_ros/tracker_node.py`:
- Uses the `ultralytics` Python API (`YOLO(model_path).track(...)`)
- Accepts any ultralytics-compatible `.pt` file via the `yolo_model` parameter
- Already publishes `Detection2DArray` (vision_msgs) consumed by `mas_multiview`
- Already has ByteTrack tracking, confidence/IoU thresholds, device selection

### Required changes to use dronecop9-2.pt

1. **Model path resolution** — currently resolves relative to the package share dir:
   ```python
   path = get_package_share_directory("ultralytics_ros")
   self.model = YOLO(f"{path}/models/{yolo_model}")
   ```
   Need to either:
   - (a) Copy/symlink `dronecop9-2.pt` into `ultralytics_ros/models/`, or
   - (b) Allow absolute paths in `yolo_model` param (if path starts with `/`, use as-is)

2. **Class filter** — default `classes` param is `list(range(80))` (COCO 80 classes).
   For dronecop model with 2 classes, set `classes: [0, 1]` in launch config.

3. **Input resolution** — model was trained at 1920px. The `ultralytics` API handles
   resizing internally via `imgsz`, but for best accuracy the `imgsz` param should
   be exposed or set appropriately. Currently not a ROS parameter — may need to add it
   or pass through model defaults.

4. **Launch config** — create a launch file or param config that sets:
   - `yolo_model: dronecop9-2.pt`
   - `classes: [0, 1]`
   - `device: cuda` (or `0`)
   - `conf_thres: 0.25` (tune as needed)

## Tasks

- [ ] **T1: Verify ultralytics_ros compatibility** — load `dronecop9-2.pt` in tracker_node,
      confirm it runs inference and publishes detections correctly
- [ ] **T2: Fix model path resolution** — allow absolute paths or install the weight into
      the package share directory
- [ ] **T3: Update launch/param config** — create or update launch config for dronecop model
      with correct classes, device, and threshold settings
- [ ] **T4: (Optional) Expose `imgsz` parameter** — add ROS param to control inference
      resolution for optimal accuracy with the 1920px-trained model
- [ ] **T5: (Optional) TensorRT acceleration** — if PyTorch inference is too slow, evaluate
      using the existing `.engine` file or exporting a new one via `model.export()`
- [ ] **T6: (Optional) Benchmark against YOLOv11 models** — compare dronecop9-2 (YOLOv8s-p2)
      vs YOLOv11 variants in `resource/models/` for accuracy and speed

## Decision
Reuse `ultralytics_ros` tracker_node with configuration changes. No new node needed
unless TensorRT-native inference is required for performance.
