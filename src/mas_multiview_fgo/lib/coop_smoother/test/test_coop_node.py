#!/usr/bin/env python3
"""Ticket 024 S1 offline integration test (GPU-free, no Isaac) — choice A (mixed).

Drives the running `coop_smoother_node` with the INTERCEPTOR's local raw ego inputs (detection +
camera_info + camera_pose + gimbal) so it forms the ego PIXEL factor itself, plus one peer
transmitted bearing ray. Verifies `coop_loc/target_pose` + `coop_loc/target_twist` publish and
TRACK a constant-velocity target (position + velocity — the 019 blocker), exercising the mixed
ego-pixel + peer-bearing path. Assumes the node is already running (see the harness). Exit 0/1.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from mas_msgs.msg import TargetRayArray, TargetRay
from vision_msgs.msg import Detection2DArray, Detection2D
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped, Vector3


def be(depth=10):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


W2C = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)


class Harness(Node):
    def __init__(self):
        super().__init__('coop_node_test')
        self.p0 = np.array([30.0, -6.0, 3.0])
        self.v0 = np.array([2.0, 4.0, 0.0])            # CV target, in ego FOV over the run
        self.peer = np.array([30.0, 30.0, 0.0])
        # Ego camera: vehicle at origin, identity attitude, gimbal 0, zoom 1
        # -> assembled R = I, t_cam = (0.1, 0, -0.1) (gimbal mount), K below.
        self.fx, self.fy, self.cx, self.cy = 600.0, 600.0, 320.0, 240.0
        self.R = np.eye(3)
        self.t_cam = np.array([0.1, 0.0, -0.1])
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.pub_det = self.create_publisher(Detection2DArray, '/ego/detection', be())
        self.pub_info = self.create_publisher(CameraInfo, '/ego/camera_info', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/ego/camera_pose', 10)
        self.pub_gimbal = self.create_publisher(Vector3, '/ego/gimbal', be())
        self.pub_peer = self.create_publisher(TargetRayArray, '/peer_rays', be())
        self.create_subscription(PoseWithCovarianceStamped, '/coop_loc/target_pose',
                                 self._on_pose, be())
        self.create_subscription(TwistStamped, '/coop_loc/target_twist', self._on_twist, be())
        self.create_timer(1.0 / 15.0, self._tick)
        self.last_pose = None
        self.last_twist = None
        self.n_pose = 0

    def _gt(self, t_abs):
        return self.p0 + self.v0 * (t_abs - self.t0)

    def _project(self, X):
        Xc = W2C @ self.R.T @ (X - self.t_cam)
        return np.array([self.fx * Xc[0] / Xc[2] + self.cx, self.fy * Xc[1] / Xc[2] + self.cy])

    def _tick(self):
        now_msg = self.get_clock().now().to_msg()
        t = self.get_clock().now().nanoseconds * 1e-9
        tgt = self._gt(t)

        info = CameraInfo()
        info.header.stamp = now_msg
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)

        pose = PoseStamped()
        pose.header.stamp = now_msg
        pose.pose.orientation.w = 1.0            # identity; ego at origin
        self.pub_pose.publish(pose)

        self.pub_gimbal.publish(Vector3(x=0.0, y=0.0, z=0.0))

        px = self._project(tgt)
        da = Detection2DArray()
        da.header.stamp = now_msg
        da.header.frame_id = 'ego_cam'
        d = Detection2D()
        d.bbox.center.position.x = float(px[0])
        d.bbox.center.position.y = float(px[1])
        d.bbox.size_x, d.bbox.size_y = 20.0, 20.0
        da.detections = [d]
        self.pub_det.publish(da)

        m = TargetRayArray()
        m.header.stamp = now_msg
        m.header.frame_id = 'common_frame'
        m.origin.x, m.origin.y, m.origin.z = map(float, self.peer)
        dvec = tgt - self.peer
        dvec = dvec / np.linalg.norm(dvec)
        r = TargetRay()
        r.direction.x, r.direction.y, r.direction.z = map(float, dvec)
        r.detection_id = 'tgt0'
        m.rays = [r]
        self.pub_peer.publish(m)

    def _on_pose(self, m):
        self.last_pose = m
        self.n_pose += 1

    def _on_twist(self, m):
        self.last_twist = m


def main():
    rclpy.init()
    h = Harness()
    t_end = time.time() + 6.0
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(h, timeout_sec=0.05)

    fails = 0

    def check(cond, name, detail=''):
        nonlocal fails
        print(('  [PASS] ' if cond else '  [FAIL] ') + name + (('  (' + detail + ')') if detail else ''))
        if not cond:
            fails += 1

    print('=== Ticket 024 S1: coop_smoother_node offline integration (choice A, mixed) ===')
    check(h.n_pose > 10, 'coop_loc/target_pose is published', 'n=%d' % h.n_pose)
    check(h.last_twist is not None, 'coop_loc/target_twist is published')

    if h.last_pose is not None:
        stamp = h.last_pose.header.stamp.sec + h.last_pose.header.stamp.nanosec * 1e-9
        gt = h._gt(stamp)
        p = h.last_pose.pose.pose.position
        est = np.array([p.x, p.y, p.z])
        perr = float(np.linalg.norm(est - gt))
        check(perr < 1.5, 'fused position tracks the CV target (ego pixel + peer bearing)',
              'err=%.3f m' % perr)

    if h.last_twist is not None:
        tw = h.last_twist.twist.linear
        v = np.array([tw.x, tw.y, tw.z])
        verr = float(np.linalg.norm(v - h.v0))
        check(verr < 2.0, 'recovered velocity tracks GT (019 blocker: v is produced)',
              'err=%.3f m/s  v=(%.2f,%.2f,%.2f)' % (verr, v[0], v[1], v[2]))

    print('=== %s ===' % ('ALL CHECKS PASSED' if fails == 0 else 'FAILURES: %d' % fails))
    h.destroy_node()
    rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
