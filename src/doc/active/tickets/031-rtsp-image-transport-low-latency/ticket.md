## Ticket: Low-latency RTSP → ROS2 ingest for the SIYI A8 mini

**What**: Patch `src/rtsp_camera/` (mzahana/rtsp_camera — GStreamer-based RTSP
→ ROS2 `sensor_msgs/Image` bridge) to expose the low-latency tuning knobs of
`rtspsrc` as ROS parameters, replace the polling-based sample pull with a
`new-sample` signal callback, and add an optional hardware-decoder path. The
publisher direction (ROS2 → RTSP) is out of scope.

**Why**: The target source is the **SIYI A8 mini** gimbal camera streaming
H.264 over RTSP at `rtsp://192.168.144.25:8554/main.264` through a short
low-loss datalink. Every frame of latency on this path costs observation
quality in the downstream YOLO / tracker / policy loop.

The original plan (ticket scope v1) was to patch `fkie/rtsp_image_transport`.
That package turned out to require Ubuntu 24.04 + g++ 13 + rolling Live555 +
Iron-vintage rclcpp APIs. Back-porting to the project's Ubuntu 20.04 + Humble
+ g++ 9 host would have required reimplementing `std::format` everywhere,
rewriting for the older `Groupsock` ctor, and replacing `EventLoopWatchVariable`.
That is not a low-latency patch, it is a multi-week backport. The partial
Humble shims applied to `src/rtsp_image_transport/` remain in tree but are
parked; building that package is blocked by the toolchain gap, not by code.

`mzahana/rtsp_camera` is 200 lines of GStreamer glue, builds cleanly on the
existing host, and uses the same `rtspsrc` element whose tuning knobs
correspond 1:1 with the research summary (see conversation log). It's the
shortest path to a running subscriber we can measure against the A8.

**Scope**:

### Part 1 — Expose rtspsrc tuning as ROS parameters

Edit [src/rtsp_camera/src/rtsp_camera_node.cpp](../../../../rtsp_camera/src/rtsp_camera_node.cpp)
to declare and apply these new parameters on the `rtspsrc` element before
`gst_element_set_state(..., GST_STATE_PLAYING)`:

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `latency_ms` | int | `10` | `rtspsrc latency` property [ms]. GStreamer default is 2000. Short A8 link tolerates 5–20 ms. |
| `drop_on_latency` | bool | `true` | `rtspsrc drop-on-latency` — discard late packets rather than buffering. |
| `use_tcp` | bool | `false` | Force RTP interleaved over TCP (`rtspsrc protocols=tcp`). Leave default UDP for A8. |
| `do_retransmission` | bool | `false` | `rtspsrc do-retransmission`. Costs a frame of latency for occasional recovery; not worth it on a short link. |
| `decoder` | string | `"avdec_h264"` | Decoder element factory name. Swap to `nvh264dec` / `vaapih264dec` / `v4l2h264dec` for hardware decode. |

Hardcoded pipeline changes that stay internal (no param):

- `appsink sync=false async=false` — render immediately, no clock wait.
- Insert a `queue max-size-buffers=1 leaky=downstream` before `appsink` so a
  stalled consumer drops the oldest frame instead of backing up.
- `h264parse config-interval=1` — keep SPS/PPS in-band so late-joining
  decoders can sync on the next IDR.

### Part 2 — Replace the polling loop with a `new-sample` callback

Current [rtsp_camera_node.cpp:101-176](../../../../rtsp_camera/src/rtsp_camera_node.cpp#L101-L176)
calls `gst_app_sink_try_pull_sample(sink, GST_SECOND/10)` in a loop with a
`sleep_for(10ms)` fallback. That's 10–100 ms of polling jitter per frame.

- DO: Set `appsink emit-signals=TRUE` (already set) and connect a
  `new-sample` signal handler that calls `gst_app_sink_pull_sample` inline.
- DO: Use a dedicated GLib main loop thread or rclcpp executor callback to
  service sample delivery — the important thing is the sample is processed
  the moment GStreamer posts it, not on a 10 ms polling tick.
- DO: Move the bus watch out of the sample loop into a
  `gst_bus_add_watch` registered on the GLib main loop.
- DO NOT: Keep `try_pull_sample` + sleep anywhere in the hot path.

### Part 3 — Avoid the cv::Mat round-trip when possible

The current code builds an OpenCV `Mat` from the GStreamer buffer even when no
resize is needed, then converts to `sensor_msgs/Image` via `cv_bridge`. On
1080p BGR frames that's ~6 MB memcpy per frame.

- DO: If requested `width_`/`height_` match the stream dimensions, populate
  `sensor_msgs::msg::Image::data` directly from the `GstMapInfo` pointer
  (single copy into the message).
- DO: Keep the `cv::resize` path as a fallback when a non-native size is
  requested.

### Part 4 — SIYI A8 mini validation

- NEW: `src/tmux/rtsp_a8_latency.tmuxp.yaml` (or snippet) that launches the
  node on `rtsp://192.168.144.25:8554/main.264` and an `rqt_image_view`.
- Measure end-to-end latency by pointing the camera at a visible millisecond
  clock (phone stopwatch) and photographing the clock next to the
  `rqt_image_view` window.
- Record baseline (unpatched) latency and patched latency (with
  `latency_ms=10`, `drop_on_latency=true`, software decode) into the
  ticket writeup.

**Affected files**:
- MOD: [src/rtsp_camera/src/rtsp_camera_node.cpp](../../../../rtsp_camera/src/rtsp_camera_node.cpp) — param declares, pipeline additions, callback-driven sample flow
- MOD: [src/rtsp_camera/README.md](../../../../rtsp_camera/README.md) — document new parameters under a "Low-latency tuning" section
- NEW: `src/tmux/rtsp_a8_latency.tmuxp.yaml` (or in-line snippet)

**Acceptance criteria**:
- `colcon build --packages-select rtsp_camera` clean on Ubuntu 20.04 +
  Humble with the existing g++ 9 toolchain (already demonstrated for the
  base package).
- `ros2 param list /rtsp_camera_node` shows the 5 new parameters.
- `ros2 run rtsp_camera rtsp_camera_node --ros-args -p rtsp_url:=rtsp://192.168.144.25:8554/main.264`
  publishes `/camera/image_raw` at the A8's native framerate.
- Measured end-to-end latency on the A8 drops vs. the unpatched build.
  Target: ≤ 150 ms glass-to-`sensor_msgs/Image` with software decode;
  ≤ 80 ms with `decoder:=nvh264dec` if GPU is present.

**Out of scope**:
- ROS2 → RTSP direction (no built-in equivalent; separately handled via
  `maladzenkau/image2rtsp` if ever needed).
- Multi-camera / multi-subscriber orchestration.
- Camera-info publishing — this node is raw image only.

**Depends on**: none. `src/rtsp_camera/` builds standalone.

**Parked work**:
- `src/rtsp_image_transport/` has partial Humble-compat shims and a
  `Findlive555.cmake` sitting on disk from the aborted v1 plan. Those
  changes do no harm (package is gated by toolchain, so the partial patch
  stays parked until a newer host is available or the package is removed).
  Revisit only if we move to Ubuntu 22.04+/24.04+.

**Flow**: Medium (C++ edits in one file + hardware verification).

---

## Implementation status (2026-04-16)

**Code-side complete** — all scoped C++ edits landed in
[src/rtsp_camera/src/rtsp_camera_node.cpp](../../../../rtsp_camera/src/rtsp_camera_node.cpp)
and the README has the "Low-latency tuning" section.

- Part 1 — 5 parameters declared and applied on `rtspsrc`:
  `latency_ms` (10), `drop_on_latency` (true), `use_tcp` (false),
  `do_retransmission` (false), `decoder` ("avdec_h264"). Also
  `rtspsrc buffer-mode=4` (GST_RTP_JITTER_BUFFER_MODE_SYNCED),
  `appsink sync=false async=false max-buffers=1 drop=true`,
  `queue max-size-buffers=1 leaky=downstream` inserted before the sink,
  `h264parse config-interval=-1` for in-band SPS/PPS.
- Part 2 — polling loop removed. `new-sample` signal on the appsink
  pulls the sample inline; a dedicated `GMainLoop` thread services the
  callback and the `gst_bus_add_watch` error/EOS watch.
- Part 3 — zero-copy-to-`cv::Mat` path when stream dims match
  requested `width_`/`height_` (single memcpy from `GstMapInfo` into
  `sensor_msgs/Image::data`); `cv::resize` retained as fallback.
- Part 4 — [src/tmux/rtsp_a8_latency.tmuxp.yaml](../../../../tmux/rtsp_a8_latency.tmuxp.yaml)
  added: `rtsp_camera_node` + `rqt_image_view /a8/image_raw` +
  `topic hz`/`topic bw`/`param list` window.

**Build**: `colcon build --packages-select rtsp_camera` clean on the
Jetson (ROS2 humble, g++ 9). 14.8 s.

**Hardware validation**:
- First bring-up on Jetson Orin against a live A8 on 192.168.144.26
  (2026-04-16): the shipped main stream was **H.265**, not the H.264
  the ticket assumed. Added a `codec` parameter (`h264`/`h265`) so
  `depay=rtp{codec}depay` and `parse={codec}parse` switch together;
  the codec toggle on the camera can then be matched from the node.
  Also added automatic NVMM-path handling: when `decoder` starts with
  `nv` an `nvvidconv` element is spliced between decoder and
  `videoconvert` to pull Jetson HW-decode output into host memory.
- Camera flipped to H.264 by the operator. First raw-only measurement
  (`codec:=h264 decoder:=nvv4l2decoder`) showed ~10 Hz with the default
  RELIABLE QoS — misleadingly low because a lagging/reliable subscriber
  was backpressuring the GStreamer streaming thread.

**Bench on the Jetson after the QoS + compressed-publisher patch
(2026-04-16, A8 H.264 1080p25, short low-loss datalink):**

| Scenario                        | Raw rate | Raw BW    | JPEG rate | JPEG BW  |
|---------------------------------|---------:|----------:|----------:|---------:|
| A — compressed-only subscriber  |    —     |     —     | **22.4 Hz** | 4.1 MB/s |
| B — raw-only subscriber         | **13.9 Hz** |  86 MB/s |     —     |    —     |
| C — raw + compressed concurrent | **16.6 Hz** | 103 MB/s | **22.0 Hz** | 4.0 MB/s |

Mean msg size: raw 6075 KB (1920×1080×3), JPEG q=80 ~179 KB — **~34×
smaller on the wire**. Node CPU is ~110% in scenario C (one core
on memcpy+publish, some on `cv::imencode`). The ~22 Hz ceiling on
the compressed topic is within 12% of the 25 fps native stream;
the raw path never reaches native because 6 MB DDS transport per
frame is the bottleneck on this rmw/loopback.

**Side-by-side: `latency_ms` / `drop_on_latency` — ticket v1 vs.
relaxed (modelled on a known-good `gst-launch` pipeline with default
`latency=2000`, `drop_on_latency` unset, `autovideosink sync=false`).**

Test: 10 s compressed-only bench, Python BEST_EFFORT subscriber,
`age = receive_time − header.stamp`.

| Config                         | Steady rate | age p50 | age p95 | Note                |
|--------------------------------|------------:|--------:|--------:|---------------------|
| A: `latency_ms=10  drop=true`  |    ~23 Hz   | 15.3 ms | 23.4 ms | original ticket v1  |
| B: `latency_ms=200 drop=false` |    ~23 Hz   | 16.0 ms | 25.6 ms | ~1 s cold-start     |

Steady-state rate is identical. Age shifts by ≤1 ms at p50, ≤2 ms at
p95 — within run-to-run noise. Raw-path age is actually *higher* than
compressed age (17 ms vs 15 ms) because 6 MB DDS transport dominates —
not the encoder and not `rtspsrc latency`. **→ relaxed defaults are
strictly better for a production node: same rate, same steady-state
latency, survives link jitter without dropping single packets.**

Defaults flipped to `latency_ms=200`, `drop_on_latency=false`
(2026-04-16). Re-bench confirmed steady ~22–23 Hz compressed on
defaults; p50 age 15–20 ms, p95 24–27 ms. The knobs stay exposed
for links with drastically different characteristics.

**Implication for YOLO**: subscribe to `/image_raw/compressed` and
JPEG-decode in the detector. libjpeg-turbo decodes a 1080p frame in
~5–7 ms, well under the 40 ms frame period, and the wire cost drops
from ~86 MB/s to ~4 MB/s freeing the DDS path for everything else.

**Wired to `ultralytics_ros` tracker_node (2026-04-16)**:

The tracker_node image subscription was created with a bare `qos=1`
(rclpy → RELIABLE + KEEP_LAST(1)), which did not match our
SensorDataQoS publisher — DDS refused to connect ("incompatible QoS"
warning on `/a8/image_raw` / `/a8/image_raw/compressed`).

Fixes in `ultralytics_ros` (user-authorized, scope extended):

- [script/tracker_node.py:73,76](../../../../ultralytics_ros/script/tracker_node.py#L73-L76) —
  raw and compressed `create_subscription` now pass
  `qos_profile_sensor_data` (already imported in this file for the
  publishers). Also dropped the dead `#self.get_parameter(
  "use_compressed_input")...` comment.
- [launch/tracker_drone.launch.xml](../../../../ultralytics_ros/launch/tracker_drone.launch.xml) —
  removed the `use_compressed_input` arg and corresponding param;
  tracker_node picks raw vs compressed from
  `input_topic.endswith("compressed")`, so the parameter was never
  read.
- [src/tracker_with_cloud_node.cpp:32-40](../../../../ultralytics_ros/src/tracker_with_cloud_node.cpp#L32-L40) —
  the 3 `message_filters::Subscriber.subscribe()` calls
  (`camera_info`, `points_raw`, `yolo_result`) now pass
  `rmw_qos_profile_sensor_data` instead of relying on the default
  RELIABLE profile. This node is not built on the A8 path (no PCL)
  but is broken whenever PCL is available, so fixed in the same pass.
- [CONTEXT.md](../../../../ultralytics_ros/CONTEXT.md) —
  subscriber QoS now documented.

One-time build hazard worked around: the existing `install/` copy of
`libultralytics_ros__rosidl_generator_py.so` was 0 bytes (corrupted
from a prior interrupted incremental build). Clean-rebuilt
(`rm -rf build/ultralytics_ros install/ultralytics_ros; colcon build
--packages-select ultralytics_ros`) produced a proper 18 KB .so and
the `UnsupportedTypeSupport` error disappeared.

**E2E verification on live A8**:

- `rtsp_camera_node` on defaults → `/a8/image_raw/compressed` at
  ~22 Hz (179–232 KB/frame JPEG q=80).
- `tracker_node.py --ros-args -p input_topic:=/a8/image_raw/compressed
  -p yolo_model:=yolov11n-drone.pt -p device:=cuda` → subscribes
  clean, publishes `/yolo_result_vision` (Detection2DArray) at
  **~12.6 Hz**. The 22 → 12.6 Hz drop is Jetson CUDA YOLO inference
  time on a 1080p frame, not QoS or plumbing.
- No incompatible-QoS warnings between our nodes. A residual
  warning on `/a8/image_raw` came from `rviz` on a separate display
  (its Image subscriber defaults to RELIABLE); fix is a per-display
  setting in rviz, not a code change.

**End-to-end stamp carry-through (2026-04-16)**:

Fixed tracker_node.py so the input image's `header.stamp` and
`frame_id` propagate to every output:
- `YoloResult.header` (was being set before
  `yolo_result_image_msg = self.create_result_image(...)` clobbered
  it; reordered so the assignments happen last)
- `YoloResult.detections.header` — the nested `Detection2DArray`,
  previously left at its default (stamp=0); this is the message on
  `yolo_result_vision` that triangulation/tracker consume
- `yolo_image.header` — same reassignment-clobber bug as the
  YoloResult one

Re-benched the A8 → `/yolo_result_vision` path with real stamps:

| Metric | Value |
|---|---|
| Detection rate | ~13 Hz |
| Shutter → detection-publish age p50 | **297 ms** |
| Age p95 | 318 ms |

Decomposing the 297 ms:

- ~15 ms rtsp_camera publish → tracker subscribe (measured earlier
  on the compressed topic)
- ~80 ms YOLO inference on yolov11n-drone.pt / CUDA (= 1 / 12.6 Hz)
- **~200 ms backlog**: rtsp_camera publishes at ~22 Hz into the
  `qos_profile_sensor_data` (depth=5) subscription at tracker_node,
  but YOLO drains at ~13 Hz — the 9 frame/s surplus fills the
  5-frame queue in steady state. YOLO is inferring on frames that
  are already ~200 ms old.

Next-step options (not blocking this ticket):
- Override tracker's image-subscription depth to 1 via `qos_overrides`
  — always infer on the newest frame, drop the rest.
- Or rate-match: throttle rtsp_camera to publish at the tracker's
  inference rate (~13 Hz).

**Decoder vs. latency trade on Jetson (2026-04-16, HDMI-direct)**:

Once the tmuxp put rviz2 + a cv2-based Python viewer + a known-good
`gst-launch ... ! autovideosink sync=false` pane side-by-side against
a visible ms clock, the operator reported:

| Decoder / size            | age   | visible gap vs gst-launch | CPU      |
|---------------------------|------:|---------------------------|---------:|
| nvv4l2decoder @ 1920×1080 | ~30 ms | yes (accumulating look)  | ~1 %    |
| nvv4l2decoder @ 960×540   | 10-20 ms | still yes              | 0 %     |
| avdec_h264 @ 1920×1080    | ~30 ms | **none**                 | ~90-110 % of one core |
| avdec_h264 @ 960×540      | ~20 ms | **none**                 | ~90 % (decode still 1080p) |

Two mechanisms:

1. `nvv4l2decoder` is fast per frame (HW NVDEC), but the v4l2 capture
   pool has a driver-imposed minimum of 3-6 NVMM surfaces — a floor
   set by stream profile, not picture size. Even with
   `disable-dpb=true` and `enable-max-performance=true` applied
   (both wired in via
   [src/rtsp_camera_node.cpp:149-167](../../../../rtsp_camera/src/rtsp_camera_node.cpp#L149-L167)),
   that pool always holds 3-6 frames → 120-240 ms of pipeline-internal
   lag that happens *before* our `handle_sample()` stamps the message,
   so `header.stamp` age stays low while the on-screen picture lags.
   This is architectural to Tegra's `nv_v4l2_dec` — no knob fixes it.

2. `avdec_h264` has ≤ 2 frames in flight (match the known-good
   `gst-launch` pipeline). No visible gap, but libav software
   decode at 1080p is ~90-110 % of one core on the Orin. Our
   `width`/`height` params only trigger a downstream `cv::resize` —
   libav still decodes at the stream's native size — so 540p
   doesn't reduce decoder cost, only DDS bandwidth and downstream
   CPU.

Chosen default in [src/tmux/rtsp_a8_latency.tmuxp.yaml](../../../../tmux/rtsp_a8_latency.tmuxp.yaml):
`decoder:=avdec_h264`, `width:=960`, `height:=540`. Operator-confirmed:
CPU 110 % of one core, `age` 10–20 ms, no visible gap vs the parallel
`gst-launch` pane. Single-camera per Jetson is the project scope, so
the CPU cost is acceptable. Flip back to 1080p if YOLO needs the extra
pixels — the freshness tradeoff doesn't change.

**Still pending**:
- Glass-to-`/a8/image_raw` latency measurement against a visible ms
  clock, without VNC in the loop.
- Sort3D / mas_multiview wiring to `/yolo_result_vision` on the
  Jetson A8 path.

**Unrelated build failure surfaced during a full `colcon build`**:
`src/image2rtsp/CMakeLists.txt:27` requires
`gstreamer-rtsp-server-1.0` via `pkg_check_modules`, and
`libgstrtspserver-1.0-dev` is not installed on this host
(`apt-cache policy` shows Candidate 1.20.1-1, Installed: none). This
package is the ROS2→RTSP direction explicitly listed as out of scope
for this ticket. To unblock a full-workspace build without touching
it, either install the dev package
(`sudo apt install libgstrtspserver-1.0-dev`) or run
`colcon build --packages-skip image2rtsp` until a decision is made on
whether we actually need the publisher direction.

