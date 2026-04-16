#!/usr/bin/env python3
"""Minimal low-latency viewer for a `sensor_msgs/CompressedImage` topic.

cv2.imshow renders the frame immediately in the callback thread and
holds exactly one frame on screen. No Qt event queue, no Ogre texture
pipeline, no image_transport plugin negotiation. If this viewer shows
no accumulating latency, the bottleneck is inside rviz/rqt/image_view,
not the publisher or transport.

Usage:
    python3 view_compressed.py /a8/image_raw/compressed
"""

import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Viewer(Node):
    def __init__(self, topic: str):
        super().__init__("view_compressed")
        self.window = f"view_compressed {topic}"
        self.count = 0
        self.t_last_report = time.monotonic()
        self.sub = self.create_subscription(
            CompressedImage, topic, self._cb, qos_profile_sensor_data
        )
        self.get_logger().info(f"subscribed to {topic}")

    def _cb(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        # Overlay header age so accumulation is obvious on-screen.
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_ms = (now_ns - stamp_ns) / 1e6 if stamp_ns else 0.0
        cv2.putText(
            img,
            f"age {age_ms:6.1f} ms",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )
        cv2.imshow(self.window, img)
        cv2.waitKey(1)

        self.count += 1
        now = time.monotonic()
        if now - self.t_last_report >= 2.0:
            self.get_logger().info(
                f"rate {self.count / (now - self.t_last_report):5.1f} Hz, "
                f"last age {age_ms:6.1f} ms"
            )
            self.count = 0
            self.t_last_report = now


def main():
    if len(sys.argv) != 2:
        print("usage: view_compressed.py <topic>", file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    node = Viewer(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
