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
