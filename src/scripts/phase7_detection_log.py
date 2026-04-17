#!/usr/bin/env python3
"""Phase 7 detection logger.

Subscribes to:
  <detection-topic>           (vision_msgs/Detection2DArray)  from ultralytics_ros
  /phase7/display_size_px     (std_msgs/UInt32)               optional, phase7_drone_display.py

For every detection message, records one CSV row:
  (t, display_size_px, num_det, max_bbox_px_short_edge)

The display-size publisher is optional — if nothing is publishing
`/phase7/display_size_px`, the column is filled with -1 and the
summary falls back to a plain bbox-size distribution across detected
frames. Use this mode when manipulating display size manually
(image viewer zoom, step back from monitor, etc.).

Prints a rolling 5 s status line. On exit (Ctrl-C) dumps:
  - overall detection rate
  - bbox short-edge distribution (min / p5 / p50 / p95 / max) of
    detected frames — the `min` is the "smallest detected bbox"
    metric for the engine under test
  - if display_size was populated: per-size-bucket recall table
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt32
from vision_msgs.msg import Detection2DArray


class DetectionLogger(Node):
    def __init__(self, det_topic, csv_path, recall_threshold):
        super().__init__("phase7_detection_log")
        self.current_size = None
        self.recall_threshold = recall_threshold
        self.rows: list[tuple[float, int, int, float]] = []

        det_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(UInt32, "/phase7/display_size_px", self._size_cb, 10)
        self.create_subscription(Detection2DArray, det_topic, self._det_cb, det_qos)

        self.csv_f = open(csv_path, "w", buffering=1)
        self.csv_f.write("t,display_size_px,num_det,max_bbox_px\n")
        self.t_last_report = time.monotonic()

        self.get_logger().info(
            f"logging → {csv_path}  "
            f"det_topic={det_topic}  "
            f"recall_thresh={recall_threshold}"
        )

    def _size_cb(self, msg):
        self.current_size = int(msg.data)

    def _det_cb(self, msg):
        t = time.time()
        n = len(msg.detections)
        max_px = 0.0
        for d in msg.detections:
            short = min(float(d.bbox.size_x), float(d.bbox.size_y))
            if short > max_px:
                max_px = short
        size = self.current_size if self.current_size is not None else -1
        self.rows.append((t, size, n, max_px))
        self.csv_f.write(f"{t:.6f},{size},{n},{max_px:.1f}\n")

        if time.monotonic() - self.t_last_report >= 5.0:
            self._report()
            self.t_last_report = time.monotonic()

    def _report(self):
        recent = self.rows[-300:]
        if not recent:
            return
        det = sum(1 for r in recent if r[2] > 0)
        size_str = f"{self.current_size}" if self.current_size is not None else "?"
        pct = 100 * det / len(recent)
        self.get_logger().info(
            f"last {len(recent):4d} frames @ size={size_str}px  "
            f"detected={det:4d}  ({pct:4.0f}%)"
        )

    def summarize(self):
        if not self.rows:
            print("no samples recorded")
            return

        total = len(self.rows)
        det_count = sum(1 for r in self.rows if r[2] > 0)
        print(f"\n## Phase 7 detection summary  (N_total={total}  N_det={det_count})")

        detected_boxes = [r[3] for r in self.rows if r[2] > 0]
        if detected_boxes:
            arr = np.asarray(detected_boxes, dtype=np.float64)
            p5 = float(np.quantile(arr, 0.05))
            p50 = float(np.median(arr))
            # Only p5 (reliable floor) and p50 (typical sustained
            # detect size) are reported. min/max/p95 are single-frame
            # outliers polluted by false positives; detection rate is
            # a session artifact (how long the operator held the drone
            # at marginal size), not a model property.
            print(f"  detected bbox short-edge px:  p5={p5:.1f}  p50={p50:.1f}")
            print(f"  → reliable min detect size (p5) = {p5:.1f} px")
        else:
            print("  no detections in this run")

        # If display_size was populated, also do the correlation analysis.
        has_display = any(r[1] >= 0 for r in self.rows)
        if has_display:
            print("\n## recall vs display_size")
            print(f"{'display_size_px':>16} {'N':>5} {'detected':>8} {'rate':>6} "
                  f"{'median_bbox_px':>15}")
            print("-" * 60)
            by_size: dict[int, list[tuple[int, float]]] = {}
            for (_t, s, n, mpx) in self.rows:
                if s < 0:
                    continue
                by_size.setdefault(s, []).append((n, mpx))

            min_handled = None
            for s in sorted(by_size.keys(), reverse=True):
                rows = by_size[s]
                t = len(rows)
                d = sum(1 for n, _ in rows if n > 0)
                rate = d / t if t else 0.0
                dboxes = [mpx for n, mpx in rows if n > 0]
                med = float(np.median(dboxes)) if dboxes else 0.0
                print(f"{s:>16} {t:>5} {d:>8} {rate:>6.2f} {med:>15.1f}")
                if rate >= self.recall_threshold:
                    min_handled = s
            if min_handled is not None:
                print(f"\n→ min display_size with recall ≥ "
                      f"{self.recall_threshold:.2f} = {min_handled} px")
            else:
                print(f"\n→ no bucket cleared recall ≥ "
                      f"{self.recall_threshold:.2f}")

    def close(self):
        try:
            self.csv_f.flush()
            self.csv_f.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detection-topic", default="/px4_2/yolo_result_vision")
    ap.add_argument("--csv", required=True, help="output CSV path")
    ap.add_argument("--recall-threshold", type=float, default=0.5)
    args, _ros = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = DetectionLogger(args.detection_topic, args.csv, args.recall_threshold)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.summarize()
        except Exception as e:
            print(f"summarize failed: {e}", file=sys.stderr)
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
