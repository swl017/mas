#!/usr/bin/env python3
"""Measure glass-to-ROS2-topic latency distribution.

Subscribes to a `sensor_msgs/CompressedImage` topic (BEST_EFFORT,
depth=1), decodes each frame, runs `cv2.QRCodeDetector` on it, parses
the decoded text as a millisecond counter (produced by
`latency_clock_display.py`), and differences against its own
`time.time()` to get per-frame latency in milliseconds.

Per-5-seconds the node prints a live distribution summary (N, mean±std,
p50/p95/p99, min/max, decode-failure rate). At shutdown the full raw
sample list is written to a CSV for offline analysis.

Setup assumptions:
- The clock display and this node run on the same Jetson host (so
  `time.time()` is the same clock on both sides — no PTP/NTP needed).
- The camera is framed so the QR fills a reasonable fraction of the
  image. Small / blurry QRs get dropped as decode failures and
  counted, not silently lost.

Bias: the measurement includes a ~one-monitor-refresh bias (~8–17 ms)
because the QR is visible on the screen slightly after `time.time()`
was read. Subtract a nominal 16 ms if you want a "true" lens-to-ROS
number.

Usage:
    python3 latency_measurement.py /a8/image_raw/compressed
    python3 latency_measurement.py /a8/image_raw/compressed --csv /tmp/lat.csv
"""

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class LatencyMeter(Node):
    def __init__(self, topic: str, csv_path: str | None):
        super().__init__("latency_measurement")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(CompressedImage, topic, self._cb, qos)
        self.detector = cv2.QRCodeDetector()
        self.samples: list[float] = []     # per-frame latency in ms (rolling in-memory)
        self.decode_attempts = 0
        self.decode_failures = 0
        self.bad_parses = 0
        self.t_last_report = time.monotonic()
        self.csv_path = csv_path
        # Open the CSV in line-buffered append mode and write each
        # sample as it arrives. This guarantees the file is always
        # current, regardless of how the process terminates
        # (Ctrl-C, SIGTERM from tmux, rclpy ExternalShutdown, …).
        self.csv_f = None
        if self.csv_path:
            self.csv_f = open(self.csv_path, "w", buffering=1)  # line-buffered
            self.csv_f.write("sample_index,t_received,t_display,latency_ms\n")
            self.get_logger().info(f"streaming samples → {self.csv_path}")
        self.get_logger().info(
            f"measuring QR-encoded glass-to-ROS2 latency on {topic} "
            f"(BEST_EFFORT depth=1)"
        )

    def _cb(self, msg: CompressedImage):
        t_now = time.time()
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return
        self.decode_attempts += 1
        decoded, _pts, _ = self.detector.detectAndDecode(img)
        if not decoded:
            self.decode_failures += 1
            return
        try:
            t_display = float(decoded) / 1000.0
        except ValueError:
            self.bad_parses += 1
            return
        latency_ms = (t_now - t_display) * 1000.0
        # Sanity reject: anything past ±10 s is clock / decode nonsense.
        if not (-100.0 <= latency_ms <= 10000.0):
            self.bad_parses += 1
            return
        self.samples.append(latency_ms)
        if self.csv_f is not None:
            idx = len(self.samples) - 1
            self.csv_f.write(
                f"{idx},{t_now:.6f},{t_display:.6f},{latency_ms:.3f}\n"
            )

        if time.monotonic() - self.t_last_report >= 5.0:
            self._report()
            self.t_last_report = time.monotonic()

    def _report(self):
        if not self.samples:
            self.get_logger().warn(
                f"no samples yet — "
                f"decode_attempts={self.decode_attempts} "
                f"decode_failures={self.decode_failures} "
                f"bad_parses={self.bad_parses}"
            )
            return
        window = np.asarray(self.samples[-1000:], dtype=np.float64)
        p50 = np.median(window)
        p95 = np.quantile(window, 0.95)
        p99 = np.quantile(window, 0.99)
        mean = float(np.mean(window))
        std = float(np.std(window))
        mn = float(np.min(window))
        mx = float(np.max(window))
        drop_pct = (
            100.0 * self.decode_failures / max(1, self.decode_attempts)
        )
        self.get_logger().info(
            f"N={len(window):4d}  "
            f"p50={p50:6.1f}  p95={p95:6.1f}  p99={p99:6.1f}  "
            f"mean={mean:6.1f}±{std:5.1f}  "
            f"min={mn:5.1f}  max={mx:6.1f} ms  "
            f"decode_drop={drop_pct:4.1f}%"
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
    ap.add_argument("topic", help="CompressedImage topic to subscribe to")
    ap.add_argument("--csv", default=None, help="stream raw samples to CSV")
    args, _ros = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = LatencyMeter(args.topic, args.csv)

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
