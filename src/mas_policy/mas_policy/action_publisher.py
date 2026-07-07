"""Action publisher: 7D policy actions → ROS2 velocity and gimbal commands.

Per-vehicle design: publishes using relative topics (resolved by node namespace).
- cmd_vel (TwistStamped, ENU) → offboard_py
- gimbal_cmd_los_rate (Vector3, rad/s) → los_rate_controller
- zoom_rate_cmd (Float32, zoom-levels/s) → los_rate_controller

Action mapping from _pre_physics_step() at iris_ma_env6_test.py:
  [0-1] vx,vy     × max_lin_vel → cmd_vel linear (ENU)
  [2]   vz        × max_vel_z_up (≥0, climb) / max_vel_z_dn (<0, descend) when
                    enable_asymmetric_z_envelope, else × max_lin_vel → cmd_vel.linear.z
  [3]   yaw_rate   × max_yaw_rate → cmd_vel angular.z
  [4]   gimbal_az  × max_gimbal_rate → gimbal_cmd_los_rate.x (rad/s)
  [5]   gimbal_el  × max_gimbal_rate → gimbal_cmd_los_rate.y (rad/s)
  [6]   zoom_rate  × max_zoom_rate → zoom_rate_cmd (zoom-levels/s)
"""

from __future__ import annotations

import logging

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Float32

import numpy as np

logger = logging.getLogger(__name__)


class ActionPublisher:
    """Converts 7D policy actions to ROS2 commands.

    Uses relative topic names — resolved by the node's namespace to the
    correct vehicle.
    """

    def __init__(
        self,
        node: Node,
        max_lin_vel: float = 5.0,
        max_yaw_rate: float = 0.7854,
        max_gimbal_rate: float = 3.141592653589793,
        max_zoom_rate: float = 2.0,
        enable_asymmetric_z_envelope: bool = True,
        max_vel_z_up: float = 3.0,
        max_vel_z_dn: float = 1.5,
    ):
        self._node = node
        self._max_lin_vel = max_lin_vel
        self._max_yaw_rate = max_yaw_rate
        self._max_gimbal_rate = max_gimbal_rate
        self._max_zoom_rate = max_zoom_rate
        # Ticket 039 — asymmetric PX4 vertical envelope. When enabled, vz is
        # scaled by max_vel_z_up (action≥0, climb) / max_vel_z_dn (action<0,
        # descend) instead of the symmetric max_lin_vel. Mirrors
        # iris_ma_env6_test._pre_physics_step so the commanded vz matches training.
        self._enable_asymmetric_z_envelope = enable_asymmetric_z_envelope
        self._max_vel_z_up = max_vel_z_up
        self._max_vel_z_dn = max_vel_z_dn

        # Relative topics — namespace provides the vehicle prefix
        self._cmd_vel_pub = node.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._gimbal_rate_pub = node.create_publisher(Vector3, 'gimbal_cmd_los_rate', 10)
        self._zoom_pub = node.create_publisher(Float32, 'zoom_rate_cmd', 10)

    def publish(self, action: np.ndarray):
        """Publish a single 7D action.

        Args:
            action: 7D action array clipped to [-1, 1].
        """
        now = self._node.get_clock().now().to_msg()

        # Velocity command (ENU)
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = now
        cmd_vel.header.frame_id = 'map'
        cmd_vel.twist.linear.x = float(action[0] * self._max_lin_vel)
        cmd_vel.twist.linear.y = float(action[1] * self._max_lin_vel)
        if self._enable_asymmetric_z_envelope:
            z = float(action[2])
            z_scale = self._max_vel_z_up if z >= 0.0 else self._max_vel_z_dn
            cmd_vel.twist.linear.z = z * z_scale
        else:
            cmd_vel.twist.linear.z = float(action[2] * self._max_lin_vel)
        cmd_vel.twist.angular.z = float(action[3] * self._max_yaw_rate)
        self._cmd_vel_pub.publish(cmd_vel)

        # Gimbal LOS rate command (denormalized to rad/s)
        gimbal_msg = Vector3()
        gimbal_msg.x = float(action[4] * self._max_gimbal_rate)  # azimuth rate (rad/s)
        gimbal_msg.y = float(action[5] * self._max_gimbal_rate)  # elevation rate (rad/s)
        gimbal_msg.z = 0.0
        self._gimbal_rate_pub.publish(gimbal_msg)

        # Zoom rate command (denormalized to zoom-levels/s)
        zoom_msg = Float32()
        zoom_msg.data = float(action[6] * self._max_zoom_rate)
        self._zoom_pub.publish(zoom_msg)

    def publish_zero(self):
        """Publish zero action (safe stop)."""
        self.publish(np.zeros(7))
