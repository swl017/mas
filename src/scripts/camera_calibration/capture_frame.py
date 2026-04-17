#!/usr/bin/env python3
"""On-demand frame saver for calibration captures.

rtsp_camera publishes `image_raw` with SensorDataQoS (BEST_EFFORT), so
`image_view/image_saver` (which defaults to RELIABLE) never receives
frames. This node subscribes with a matching BEST_EFFORT profile and
writes one JPEG each time its `~/save` `std_srvs/Empty` service is
called.

Usage:
    python3 src/scripts/camera_calibration/capture_frame.py \
      --ros-args \
      -r image:=/px4_2/camera/color/image_raw \
      -p output_dir:=datasets/camera_calibration/2026-04-17/1x/images \
      -p filename_format:=frame_%04d.jpg

Trigger a save:
    ros2 service call /capture_frame/save std_srvs/srv/Empty
"""

import os
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty


class CaptureFrame(Node):
    def __init__(self) -> None:
        super().__init__("capture_frame")
        self.declare_parameter("output_dir", ".")
        self.declare_parameter("filename_format", "frame_%04d.jpg")
        self.declare_parameter("jpeg_quality", 95)

        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self.filename_format = (
            self.get_parameter("filename_format").get_parameter_value().string_value
        )
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Match rtsp_camera's SensorDataQoS so the subscription connects.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.bridge = CvBridge()
        self.latest: Image | None = None
        self.sub = self.create_subscription(Image, "image", self._on_image, qos)
        self.srv = self.create_service(Empty, "~/save", self._on_save)

        self.counter = self._next_index()
        self.get_logger().info(
            f"capture_frame ready — output={self.output_dir} "
            f"next index={self.counter}"
        )

    def _next_index(self) -> int:
        idx = 0
        while (self.output_dir / (self.filename_format % idx)).exists():
            idx += 1
        return idx

    def _on_image(self, msg: Image) -> None:
        self.latest = msg

    def _on_save(self, _req, resp):
        if self.latest is None:
            self.get_logger().warn("no frame received yet — is the camera up?")
            return resp
        frame = self.bridge.imgmsg_to_cv2(self.latest, desired_encoding="bgr8")
        path = self.output_dir / (self.filename_format % self.counter)
        cv2.imwrite(
            str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        self.get_logger().info(f"saved {path} ({frame.shape[1]}x{frame.shape[0]})")
        self.counter += 1
        return resp


def main() -> None:
    rclpy.init()
    node = CaptureFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
