#!/usr/bin/env python3
"""Synthetic-frame publisher for isolating display-stack latency.

Publishes a black frame with a BIG millisecond-precision wallclock
reading in the middle, at `rate_hz`, as both `sensor_msgs/Image` and
`sensor_msgs/CompressedImage`. Intended to be consumed by a viewer
(rviz2, cv2.imshow, rqt_image_view, gst autovideosink via
image_transport republish, …) under test.

Measurement procedure
---------------------
1. Start this publisher alongside the viewer(s) under test. Subscribe
   each viewer to `/bench_ts/image_raw` or `/bench_ts/image_raw/compressed`.
2. Take one photo / screenshot of the viewer window(s) PLUS a
   terminal showing `python3 -c 'import time;print(f"{time.time()%100:.3f}")'`
   printed at the moment the photo was taken. (Any wallclock the
   photo can resolve works; just needs to be in the same frame.)
3. Read the overlay value shown in each viewer off the photo:
     viewer_shown_ms = digit readout off the screen
     real_now_ms     = `time.time() % 100` at photo time
   Glass-to-glass for that viewer = real_now_ms − viewer_shown_ms.
4. Difference across viewers isolates display-stack overhead:
     rviz_extra = glass_to_glass_rviz − glass_to_glass_cv2

The overlay text is the SAME wallclock source used in
`header.stamp` (ROS `get_clock().now()`), so the `age` overlay in
`view_compressed.py` should match `real_now − viewer_shown` up to
display refresh + the viewer's internal callback-to-screen cost.

Usage:
    python3 bench_display_latency.py                 # 25 Hz, 960x540
    python3 bench_display_latency.py --ros-args -p rate_hz:=60.0 -p width:=1280 -p height:=720
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class BenchTimestampPublisher(Node):
    def __init__(self):
        super().__init__("bench_display_latency_publisher")
        self.declare_parameter("rate_hz", 25.0)
        self.declare_parameter("width", 960)
        self.declare_parameter("height", 540)
        self.declare_parameter("topic_ns", "/bench_ts")
        self.declare_parameter("jpeg_quality", 80)

        rate = float(self.get_parameter("rate_hz").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.ns = str(self.get_parameter("topic_ns").value).rstrip("/")
        self.jq = int(self.get_parameter("jpeg_quality").value)

        self.raw_pub = self.create_publisher(
            Image, f"{self.ns}/image_raw", qos_profile_sensor_data
        )
        self.cmp_pub = self.create_publisher(
            CompressedImage,
            f"{self.ns}/image_raw/compressed",
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.frame_id = 0
        self._report_count = 0
        self._report_t0 = time.monotonic()
        # Log actual publish rate every 2 s so we can tell if the Python
        # timer + imencode is keeping up with the requested rate.
        self.rate_timer = self.create_timer(2.0, self._rate_report)
        self.get_logger().info(
            f"Publishing synthetic {self.w}x{self.h} at {rate:.1f} Hz → "
            f"{self.ns}/image_raw[/compressed]"
        )

    def _rate_report(self):
        now = time.monotonic()
        dt = now - self._report_t0
        if dt > 0:
            self.get_logger().info(
                f"publish rate {self._report_count / dt:5.1f} Hz"
            )
        self._report_count = 0
        self._report_t0 = now

    def _tick(self):
        # Use time.time() for the in-frame overlay so its format and
        # reference match pane 3's `real_now` print exactly. Header stamp
        # still uses ROS time for downstream compatibility (they are the
        # same clock when use_sim_time is false).
        wall = time.time()
        stamp = self.get_clock().now()
        # Overlay text is "seconds mod 100 with ms" — easy to read off a
        # photo and easy to differ against a same-format reference.
        t_display = wall % 100.0
        label = f"{t_display:07.3f}"

        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        # Center the big timestamp
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 3.5, 6
        )
        org = ((self.w - tw) // 2, (self.h + th) // 2)
        cv2.putText(
            img, label, org,
            cv2.FONT_HERSHEY_SIMPLEX, 3.5, (0, 255, 0), 6, cv2.LINE_AA,
        )
        # Corner helpers — frame counter and rate-hint so the operator
        # can sanity-check that the stream is actually flowing.
        cv2.putText(
            img, f"frame {self.frame_id}", (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 200), 2, cv2.LINE_AA,
        )
        cv2.putText(
            img, "BENCH DISPLAY LATENCY", (20, self.h - 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2, cv2.LINE_AA,
        )
        self.frame_id += 1
        self._report_count += 1

        stamp_msg = stamp.to_msg()

        if self.raw_pub.get_subscription_count() > 0:
            msg = Image()
            msg.header.stamp = stamp_msg
            msg.header.frame_id = "bench_ts"
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.width = self.w
            msg.height = self.h
            msg.step = 3 * self.w
            msg.data = img.tobytes()
            self.raw_pub.publish(msg)

        if self.cmp_pub.get_subscription_count() > 0:
            ok, buf = cv2.imencode(
                ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jq]
            )
            if ok:
                cmsg = CompressedImage()
                cmsg.header.stamp = stamp_msg
                cmsg.header.frame_id = "bench_ts"
                cmsg.format = "jpeg"
                cmsg.data = buf.tobytes()
                self.cmp_pub.publish(cmsg)


def main():
    rclpy.init()
    node = BenchTimestampPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
