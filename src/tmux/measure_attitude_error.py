#!/usr/bin/env python3
"""Measure the per-boot PX4-vs-Isaac-truth attitude error — SIMULATION ONLY.

Ticket 004 accepts the PX4 SITL attitude bias as realistic sensing (no
calibration; `SENS_BOARD_*_OFF = 0`). The bias is constant within a boot but
varies between boots, so the experiment treats each **boot as a block** and
**reports the boot-level attitude error** as a covariate. This tool measures it:
compare `fmu/out/vehicle_attitude` (raw PX4, NED-FRD → ENU-FLU) to Isaac
`state/pose` over a window, while the vehicle is in steady hover.

Run it once per boot (after the vehicle is hovering and the EKF has settled) and
record the printed mean [roll, pitch, yaw] alongside that boot's results:
    ROBOT_NAME=px4_1 python3 measure_attitude_error.py [seconds]
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, AccelStamped
from px4_msgs.msg import VehicleAttitude
from scipy.spatial.transform import Rotation as R

R_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
R_FRD_FLU = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])


class AttitudeError(Node):
    def __init__(self, robot, window_s):
        super().__init__('attitude_error_monitor')
        self.window_s = window_s
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.truth = None
        self.acc = None
        self.errs = []
        self.accs = []
        self.create_subscription(PoseStamped, f'/{robot}/state/pose', self._truth, qos)
        self.create_subscription(VehicleAttitude, f'/{robot}/fmu/out/vehicle_attitude',
                                 self._att, qos)
        self.create_subscription(AccelStamped, f'/{robot}/state/accel', self._acc, qos)
        self.create_timer(2.0, self._tick)
        self.get_logger().info(
            f"measuring {robot} attitude error vs Isaac truth for {window_s:.0f}s "
            f"(hold steady hover)")

    def _truth(self, m):
        o = m.pose.orientation
        self.truth = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()

    def _acc(self, m):
        a = m.accel.linear
        self.acc = math.hypot(a.x, a.y)

    def _att(self, m):
        if self.truth is None:
            return
        q = m.q  # [w,x,y,z] body-FRD -> world-NED
        Re = R_NED_ENU @ R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix() @ R_FRD_FLU
        self.errs.append(R.from_matrix(self.truth.T @ Re).as_euler('xyz', degrees=True))
        if self.acc is not None:
            self.accs.append(self.acc)

    def _tick(self):
        if not self.errs:
            self.get_logger().info("waiting for attitude + truth ...")
            return
        e = np.array(self.errs)
        ah = float(np.mean(self.accs)) if self.accs else float('nan')
        self.get_logger().info(
            f"N={len(e)} attitude_err[roll,pitch,yaw]="
            f"({e[:,0].mean():+.2f},{e[:,1].mean():+.2f},{e[:,2].mean():+.2f})deg "
            f"std=({e[:,0].std():.2f},{e[:,1].std():.2f},{e[:,2].std():.2f}) "
            f"|horiz_accel|={ah:.2f} (want steady hover)")


def main():
    robot = os.environ.get("ROBOT_NAME", "px4_1")
    window_s = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = AttitudeError(robot, window_s)
    import threading
    threading.Timer(window_s, lambda: rclpy.try_shutdown()).start()
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == "__main__":
    main()
