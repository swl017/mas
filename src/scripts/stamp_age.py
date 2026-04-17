#!/usr/bin/env python3
"""Measure `now - header.stamp` distribution on any stamped topic.

Subscribes BEST_EFFORT depth=1 to a topic whose message type carries a
`std_msgs/Header` (either at `.header` or at `.detections.header` — the
two conventions in this workspace), and records per-message age in
milliseconds.

Per-5-seconds the node prints a live distribution summary (N, mean±std,
p50/p95/p99, min/max). Samples are streamed to CSV line-buffered so
partial runs are never lost.

Topic types are resolved by string name so the script doesn't need
every consumer's custom messages at import time. Tested with:

    # Detection2DArray publisher from ultralytics_ros
    python3 stamp_age.py /px4_2/fdf/yolo_result_vision vision_msgs/msg/Detection2DArray

    # Raw image
    python3 stamp_age.py /px4_2/camera/color/color/image_raw sensor_msgs/msg/Image

    # Compressed
    python3 stamp_age.py /px4_2/camera/color/color/image_raw/compressed sensor_msgs/msg/CompressedImage

Caveat: `age = rx_time - header.stamp`. On the tracker's
`yolo_result_vision` the stamp is the rtsp_camera capture time (see
tracker_node.py header propagation), so this metric covers
`rtsp_camera publish -> tracker inference -> detection publish -> here`.
It does *not* include the A8 encoder + RTSP transit (~245 ms, see
ticket 031); combine with latency_measurement.py for end-to-end.
"""

import argparse
import importlib
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def _resolve_msg_type(type_str: str):
    """Resolve 'pkg/msg/TypeName' to the imported message class."""
    parts = type_str.replace("/", ".").split(".")
    if len(parts) == 3:
        pkg, sub, name = parts
    elif len(parts) == 2:
        pkg, name = parts
        sub = "msg"
    else:
        raise ValueError(f"bad type string {type_str!r}, expected 'pkg/msg/Name'")
    module = importlib.import_module(f"{pkg}.{sub}")
    return getattr(module, name)


def _extract_stamp(msg):
    """Return (sec, nanosec) from msg.header or msg.detections.header."""
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp
    elif hasattr(msg, "detections") and hasattr(msg.detections, "header"):
        stamp = msg.detections.header.stamp
    else:
        return None
    return stamp.sec + stamp.nanosec * 1e-9


class StampAgeMeter(Node):
    def __init__(self, topic: str, type_str: str, csv_path: str | None):
        super().__init__("stamp_age_meter")
        msg_type = _resolve_msg_type(type_str)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(msg_type, topic, self._cb, qos)
        self.samples: list[float] = []
        self.zero_stamp_count = 0
        self.t_last_report = time.monotonic()
        self.csv_path = csv_path
        self.csv_f = None
        if self.csv_path:
            self.csv_f = open(self.csv_path, "w", buffering=1)
            self.csv_f.write("sample_index,t_received,t_stamp,age_ms\n")
            self.get_logger().info(f"streaming samples → {self.csv_path}")
        self.get_logger().info(
            f"measuring stamp age on {topic} ({type_str}) BEST_EFFORT depth=1"
        )

    def _cb(self, msg):
        t_now = time.time()
        t_stamp = _extract_stamp(msg)
        if t_stamp is None or t_stamp == 0.0:
            self.zero_stamp_count += 1
            return
        age_ms = (t_now - t_stamp) * 1000.0
        self.samples.append(age_ms)
        if self.csv_f is not None:
            idx = len(self.samples) - 1
            self.csv_f.write(f"{idx},{t_now:.6f},{t_stamp:.6f},{age_ms:.3f}\n")
        if time.monotonic() - self.t_last_report >= 5.0:
            self._report()
            self.t_last_report = time.monotonic()

    def _report(self):
        if not self.samples:
            self.get_logger().warn(
                f"no samples yet — zero_stamp_count={self.zero_stamp_count}"
            )
            return
        window = np.asarray(self.samples[-1000:], dtype=np.float64)
        self.get_logger().info(
            f"N={len(window):4d}  "
            f"p50={np.median(window):6.1f}  "
            f"p95={np.quantile(window, 0.95):6.1f}  "
            f"p99={np.quantile(window, 0.99):6.1f}  "
            f"mean={float(np.mean(window)):6.1f}±{float(np.std(window)):5.1f}  "
            f"min={float(np.min(window)):5.1f}  max={float(np.max(window)):6.1f} ms  "
            f"zero_stamps={self.zero_stamp_count}"
        )

    def close(self):
        if self.csv_f is not None:
            try:
                self.csv_f.flush()
                self.csv_f.close()
            except Exception:
                pass
            self.csv_f = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic", help="topic to subscribe to")
    ap.add_argument("type", help="message type, e.g. vision_msgs/msg/Detection2DArray")
    ap.add_argument("--csv", default=None, help="stream raw samples to CSV")
    args, _ros = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = StampAgeMeter(args.topic, args.type, args.csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._report()
        except Exception as e:
            print(f"final report failed: {e}", file=sys.stderr)
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
