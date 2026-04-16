#!/usr/bin/env python3
"""Full-screen latency-clock display.

Shows a QR code containing the current wall-clock millisecond
timestamp (seconds-since-epoch × 1000). The camera under test is
pointed at this window; a subscriber decodes the QR from each received
frame and differences the decoded value against its own `time.time()`
to measure glass-to-ROS2 latency.

Because both endpoints read the same `time.time()` clock on the same
host, the measurement has no clock-drift component. The only known
bias is monitor input-to-photon delay (typically ~8–17 ms at 60 Hz —
one refresh period), which inflates the measurement by a fixed amount.

Usage:
    python3 latency_clock_display.py               # 1920x1080, Q-ECL QR
    python3 latency_clock_display.py 1280 720      # custom size

Press `q` in the window to quit.
"""

import sys
import time

import cv2
import numpy as np


def encode_qr(text: str, side_px: int) -> np.ndarray:
    """Return a (side_px, side_px, 3) BGR uint8 image with the QR filling it.

    Uses cv2's built-in QR encoder (OpenCV ≥ 4.5) — no `qrcode` package
    dependency.
    """
    enc = cv2.QRCodeEncoder_create()
    raw = enc.encode(text)  # (N, N) uint8 with 0 (black) / 255 (white)
    big = cv2.resize(raw, (side_px, side_px), interpolation=cv2.INTER_NEAREST)
    return cv2.merge([big, big, big])


def main():
    screen_w = int(sys.argv[1]) if len(sys.argv) > 1 else 1920
    screen_h = int(sys.argv[2]) if len(sys.argv) > 2 else 1080
    qr_side = min(screen_w, screen_h) * 3 // 5  # leaves margin for text

    win = "latency_clock"
    cv2.namedWindow(win, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)
    qr_x0 = (screen_w - qr_side) // 2
    qr_y0 = (screen_h - qr_side) // 2

    while True:
        now = time.time()
        ts_ms = int(now * 1000)
        qr_img = encode_qr(str(ts_ms), qr_side)

        # Wipe + redraw (only regions we change — the QR and the bottom
        # text bar — so we're not blitting the whole frame every loop).
        canvas[qr_y0:qr_y0 + qr_side, qr_x0:qr_x0 + qr_side] = qr_img
        cv2.rectangle(
            canvas, (0, screen_h - 80), (screen_w, screen_h), (0, 0, 0), -1
        )
        label = f"t = {now % 1000:10.3f}   ms_counter = {ts_ms}"
        cv2.putText(
            canvas, label, (40, screen_h - 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3, cv2.LINE_AA,
        )
        cv2.imshow(win, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
