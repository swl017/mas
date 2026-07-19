# mas_multiview_fgo — Cooperative factor-graph smoother (ticket 024)

## Purpose

Measurement-time-aware cooperative target-trajectory fusion. A GTSAM (4.2) factor graph over target
states `x_k = (position, velocity)` fuses transmitted bearing rays (ego's own per-camera
`target_rays_w` + peer rays via `ray_delay`) with each measurement placed at its **capture time** and
a constant-velocity motion factor bridging the async gaps. It publishes pose **+ velocity**,
forward-predicted to now — a principled replacement for the ticket-019 `cv_smoother` alpha-beta hack
(closes the velocity blocker; removes the ticket-020 snapshot `v·τ` bias).

Snapshot `mas_multiview` (single-instant triangulation) is left intact — this is a separate,
swappable backend.

## Node: `coop_smoother_node`

**Pattern:** subscribers buffer bearing rays; a timer rebuilds the windowed factor graph
(full-window LM re-solve, Q3=b), forward-predicts, and publishes.

### Subscriptions (choice A — mixed ego pixel + peer bearing)

**EGO (local raw inputs → pixel factor; the interceptor forms + pose-syncs these itself):**

| Topic (param) | Type | Notes |
|---|---|---|
| `ego_detection_topic` | `vision_msgs/Detection2DArray` | bbox center pixel; `header.stamp` = `t_det` |
| `ego_camera_info_topic` | `sensor_msgs/CameraInfo` | intrinsics K |
| `ego_camera_pose_topic` | `geometry_msgs/PoseStamped` | vehicle pose (buffered, interpolated to `t_det` — Q8) |
| `ego_gimbal_topic` | `geometry_msgs/Vector3` | gimbal r/p/y (deg) |
| `ego_zoom_topic` (opt) | `std_msgs/Float64` | zoom (fx/fy scale) |

**PEER (transmitted, remote → bearing factor):**

| Topic (param `peer_ray_topics`) | Type | Notes |
|---|---|---|
| e.g. `/px4_2/target_rays_w` | `mas_msgs/TargetRayArray` | each ray → a bearing factor at `header.stamp`. Stamp = detection capture time (`triangulation_node`, RAL 024 rev3 fix). Observer-side pose *interpolation* to `t_det` is NOT yet implemented at the sources (RAL 024 AC2 amendment 2026-07-20: deferred maneuvering-observer prerequisite; port `pose_interp.h`) |

The ego camera model `(K, R, t)` is assembled from `camera_pose × gimbal[zyx] × zoom × camera_info`
(`ego_camera.h`, ported from `triangulation_node`), with the pose interpolated to `t_det`. Leave the
ego topics empty to run peer-only (bearing) — the S0 core supports both factors.

### Publishers (drop-in for `pn_guidance` `estimate_source=cooperative`, `PREFIX['cooperative']='coop_loc'`)

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `{coop_prefix}/target_pose` | `geometry_msgs/PoseWithCovarianceStamped` | BestEffort(10) | position + 3×3 marginal cov |
| `{coop_prefix}/target_twist` | `geometry_msgs/TwistStamped` | BestEffort(10) | smoother velocity |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ego_detection_topic` / `ego_camera_info_topic` / `ego_camera_pose_topic` / `ego_gimbal_topic` / `ego_zoom_topic` | string | `""` | ego raw-input topics (empty ⇒ ego arm off) |
| `gimbal_angle_order` | string | `zyx` | `zyx` / `zxy` / `zy` (matches triangulation_node) |
| `pixel_sigma_px` | double | `2.0` | ego pixel-reprojection noise |
| `peer_ray_topics` | string[] | `[]` | transmitted peer bearing-ray streams |
| `coop_prefix` | string | `coop_loc` | output namespace (matches pn's cooperative source) |
| `frame_id` | string | `common_frame` | output frame |
| `publish_rate` | double | `50.0` | belief release rate (Hz) |
| `bearing_sigma_deg` | double | `0.5` | peer `R_static` angular sigma (Q9 fallback) |
| `sigma_psi_deg` | double | `0.0` | Q10 Tier-1 azimuthal inflation (relative-heading bias bound) |
| `window_s` | double | `0.6` | fixed-lag window |
| `q_c` | double | `4.0` | white-noise-acceleration spectral density (CV motion factor) |
| `use_robust` | bool | `false` | Q10 Tier-1 robust kernel on peer bearings |

## Design decisions (see ticket 024 `q_questions.md`)

- **GTSAM** (Q1); **new package** downstream of the rays (Q2); **full-window re-solve** v1, marked
  for the `gtsam_unstable` fixed-lag upgrade (Q3); **TwistStamped** drop-in for pn (Q4=a); baseline
  = `mas_tracker/sort3d` CV-KF (Q5).
- **Mixed ego-pixel + peer-bearing (Q7):** the node uses the **`EgoPixelFactor`** for the ego (raw
  local inputs) and the **`PeerBearingFactor`** for peers (transmitted rays). Deployment topology:
  the **interceptor runs this node ONLY** (ego pixel + local pose-sync); **observers run
  `mas_multiview`** ray-formation to transmit their bearings. (Leave the ego topics empty to run
  peer-only bearing — the ARCH-G smoother.)
- **Q8 pose→`t_det` sync** (`pose_interp.h`) and **Q9 transmitted pose-covariance** are source-side
  concerns. STATUS (RAL 024 rev3, 2026-07-20): NOT landed at the sources — interceptor-side Q8 is
  in-node; the transmitted-ray **stamp** now carries detection capture time; observer-side pose
  *interpolation* is a deferred maneuvering-observer prerequisite; Q9 transmission has a defined
  message (`mas_msgs/TargetRayArrayWithCov`) with the pipeline switchover deferred pending
  bandwidth tests. The node uses characterized fallbacks (`R_static`, `peer_att_sigma_deg`,
  `peer_pos_sigma_m`) meanwhile.
- **Q10 Tier-1** frame-misalignment-bias bound (robust + azimuthal inflation + gate) is a bound, not
  a fix (quantified by ticket 025).

## Key files

| File | Role |
|---|---|
| `lib/coop_smoother/include/factors.h` | CV motion / ego pixel / peer bearing (2-DOF) GTSAM factors |
| `lib/coop_smoother/include/coop_smoother.h` | event-keyframed windowed smoother + forward-predict |
| `lib/coop_smoother/include/pose_interp.h` | Q8 pose→`t_det` interpolation |
| `lib/coop_smoother/include/ego_camera.h` | ego camera-model assembly (camera_pose × gimbal × zoom) |
| `lib/coop_smoother/include/meas_noise.h` | Q9 hybrid + Q10 Tier-1 peer-bearing covariance |
| `src/coop_smoother_node.cpp` | ROS2 node |
| `lib/coop_smoother/test/test_coop_smoother.cpp` | S0 offline unit test (GTSAM, GPU-free) |
| `lib/coop_smoother/test/test_coop_node.py` | S1 offline integration test (publishes synthetic rays) |

## Dependencies

GTSAM 4.2 (`find_package(GTSAM)`, `/usr/local`; standalone C++, ROS-distro-independent), Eigen3,
rclcpp, mas_msgs, geometry_msgs.
