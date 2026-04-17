#!/usr/bin/env python3
"""Phase 7 drone-size display for camera-in-the-loop bbox recall test.

Shows a drone image on a full-screen OpenCV window at an adjustable
pixel size. The operator aims the A8 camera at this monitor, runs the
regular pipeline (rtsp_camera + tracker_drone.launch.xml), and uses
the keyboard to scroll the size down until /yolo_result_vision stops
firing detections.

Current size (short-edge pixels) is published on
/phase7/display_size_px (std_msgs/UInt32) so phase7_detection_log.py
can correlate detections with the size that was on screen at capture
time.

Controls:
  + / =   enlarge by 10%
  - / _   shrink by 10%
  [       enlarge by 1 pixel (short edge)
  ]       shrink by 1 pixel
  r       reset to initial size
  q / ESC quit
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

DEFAULT_IMAGE = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "drone_snapshot.png"
)


def fit_drone(img, target_short_edge):
    h, w = img.shape[:2]
    short = min(h, w)
    if short <= 0:
        return None
    scale = target_short_edge / short
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)


class SizePublisher(Node):
    def __init__(self):
        super().__init__("phase7_drone_display")
        self.pub = self.create_publisher(UInt32, "/phase7/display_size_px", 10)

    def publish_size(self, size):
        self.pub.publish(UInt32(data=int(size)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE,
                    help=f"path to a drone PNG/JPG (default: {DEFAULT_IMAGE})")
    ap.add_argument("--initial-size", type=int, default=200,
                    help="short-edge pixels at startup")
    ap.add_argument("--canvas-w", type=int, default=1920)
    ap.add_argument("--canvas-h", type=int, default=1080)
    ap.add_argument("--bg", default="black", choices=["black", "white", "gray"])
    args, _ros = ap.parse_known_args()

    drone = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if drone is None:
        print(f"ERROR: cannot load {args.image}", file=sys.stderr)
        sys.exit(2)

    bg_color = {"black": 0, "white": 255, "gray": 128}[args.bg]

    rclpy.init(args=sys.argv)
    node = SizePublisher()

    size = args.initial_size
    cv2.namedWindow("phase7_drone", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("phase7_drone", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while rclpy.ok():
            canvas = np.full((args.canvas_h, args.canvas_w, 3), bg_color, dtype=np.uint8)
            scaled = fit_drone(drone, size)
            if scaled is not None:
                sh, sw = scaled.shape[:2]
                cy, cx = args.canvas_h // 2, args.canvas_w // 2
                y0 = max(0, cy - sh // 2)
                x0 = max(0, cx - sw // 2)
                y1 = min(args.canvas_h, y0 + sh)
                x1 = min(args.canvas_w, x0 + sw)
                canvas[y0:y1, x0:x1] = scaled[: y1 - y0, : x1 - x0]

            label = f"drone short edge = {size} px  (+/- to scale, q to quit)"
            cv2.putText(canvas, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("phase7_drone", canvas)
            node.publish_size(size)
            rclpy.spin_once(node, timeout_sec=0.0)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key in (ord('+'), ord('=')):
                size = min(2000, max(1, int(round(size * 1.1))))
            elif key in (ord('-'), ord('_')):
                size = max(1, int(round(size / 1.1)))
            elif key == ord('['):
                size = min(2000, size + 1)
            elif key == ord(']'):
                size = max(1, size - 1)
            elif key == ord('r'):
                size = args.initial_size
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
