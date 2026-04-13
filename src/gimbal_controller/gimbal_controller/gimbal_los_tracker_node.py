from __future__ import annotations
"""Gimbal LOS tracker: points gimbal at a target pose using LOS rate commands.

Computes world-frame azimuth/elevation error between the current gimbal
LOS direction and the target, then publishes proportional LOS rate commands
(rad/s) on gimbal_cmd_los_rate. The downstream los_rate_controller handles
the Jacobian IK and body-frame stabilization.

LOS convention: positive elevation = up (ENU standard).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class GimbalLOSTrackerNode(Node):
    """Track a target pose by publishing proportional LOS rate commands."""

    def __init__(self):
        super().__init__('gimbal_los_tracker_node')

        # Parameters
        self.declare_parameter('target_topic', '/target_region')
        self.declare_parameter('proportional_gain', 1.5)
        self.declare_parameter('max_gimbal_rate', math.pi)
        self.declare_parameter('update_rate', 25.0)

        self._target_topic = self.get_parameter('target_topic').value
        self._Kp = self.get_parameter('proportional_gain').value
        self._max_gimbal_rate = self.get_parameter('max_gimbal_rate').value
        self._update_rate = float(self.get_parameter('update_rate').value)

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher: LOS rate in rad/s, x=azimuth, y=elevation
        self._los_rate_pub = self.create_publisher(
            Vector3, 'gimbal_cmd_los_rate', sensor_qos)

        # Subscribers
        self.create_subscription(
            PointStamped, self._target_topic,
            self._target_pose_cb, sensor_qos)
        self.create_subscription(
            Odometry, 'common_frame/odom',
            self._odom_cb, sensor_qos)
        self.create_subscription(
            Vector3, 'gimbal_los_state_deg',
            self._los_state_cb, sensor_qos)

        # Cached state
        self._target_pos: np.ndarray | None = None
        self._ego_pos: np.ndarray | None = None
        self._los_az_rad: float | None = None
        self._los_el_rad: float | None = None

        # Timer
        self.create_timer(1.0 / self._update_rate, self._timer_cb)

        self.get_logger().info(
            f'GimbalLOSTracker started: target={self._target_topic}, '
            f'Kp={self._Kp}, max_rate={math.degrees(self._max_gimbal_rate):.0f} deg/s')

    # ── Callbacks ────────────────────────────────────────────────────

    # def _target_pose_cb(self, msg: PoseStamped):
    #     p = msg.pose.position
    #     self._target_pos = np.array([p.x, p.y, p.z])
    def _target_pose_cb(self, msg: PointStamped):
        p = msg.point
        self._target_pos = np.array([p.x, p.y, p.z])

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._ego_pos = np.array([p.x, p.y, p.z])

    def _los_state_cb(self, msg: Vector3):
        self._los_az_rad = math.radians(msg.x)
        self._los_el_rad = math.radians(msg.y)

    # ── Control loop ─────────────────────────────────────────────────

    def _timer_cb(self):
        if (self._target_pos is None or self._ego_pos is None
                or self._los_az_rad is None):
            return

        # 1. Desired world-frame az/el to target
        d = self._target_pos - self._ego_pos
        xy_dist = math.sqrt(d[0] ** 2 + d[1] ** 2)
        if xy_dist < 0.01:
            return
        az_desired = math.atan2(d[1], d[0])
        el_desired = math.atan2(d[2], xy_dist)  # positive = up

        # 2. Error (wrap azimuth to [-pi, pi])
        az_err = math.atan2(
            math.sin(az_desired - self._los_az_rad),
            math.cos(az_desired - self._los_az_rad))
        el_err = el_desired - self._los_el_rad

        # 3. Proportional rate (rad/s), clamped to max_gimbal_rate
        az_rate = self._Kp * az_err
        el_rate = self._Kp * el_err

        # 4. Clamp and publish
        msg = Vector3()
        msg.x = float(np.clip(az_rate, -self._max_gimbal_rate, self._max_gimbal_rate))
        msg.y = float(np.clip(el_rate, -self._max_gimbal_rate, self._max_gimbal_rate))
        msg.z = 0.0
        self._los_rate_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = GimbalLOSTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
