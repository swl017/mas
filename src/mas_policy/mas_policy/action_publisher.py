"""Action publisher: 7D policy actions → ROS2 velocity and gimbal commands.

Per-vehicle design: publishes using relative topics (resolved by node namespace).
- cmd_vel (TwistStamped, ENU) → offboard_py
- gimbal_cmd_los_rate (Vector3, normalized [-1,1]) → los_rate_controller
- zoom_cmd (Float32, normalized)

Action mapping from _pre_physics_step() at iris_ma_env6_test.py:523-530:
  [0-2] vx,vy,vz  × max_lin_vel → cmd_vel linear (ENU)
  [3]   yaw_rate   × max_yaw_rate → cmd_vel angular.z
  [4]   gimbal_az  pass-through [-1,1] → gimbal_cmd_los_rate.x
  [5]   gimbal_el  pass-through [-1,1] → gimbal_cmd_los_rate.y
  [6]   zoom_rate  pass-through [-1,1] → zoom_cmd
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
        max_lin_vel: float = 10.0,
        max_yaw_rate: float = 0.7854,
    ):
        self._node = node
        self._max_lin_vel = max_lin_vel
        self._max_yaw_rate = max_yaw_rate

        # Relative topics — namespace provides the vehicle prefix
        self._cmd_vel_pub = node.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._gimbal_rate_pub = node.create_publisher(Vector3, 'gimbal_cmd_los_rate', 10)
        self._zoom_pub = node.create_publisher(Float32, 'zoom_cmd', 10)

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
        cmd_vel.twist.linear.z = float(action[2] * self._max_lin_vel)
        cmd_vel.twist.angular.z = float(action[3] * self._max_yaw_rate)
        self._cmd_vel_pub.publish(cmd_vel)

        # Gimbal LOS rate command (normalized [-1, 1])
        gimbal_msg = Vector3()
        gimbal_msg.x = float(action[4])  # azimuth rate
        gimbal_msg.y = float(action[5])  # elevation rate
        gimbal_msg.z = 0.0
        self._gimbal_rate_pub.publish(gimbal_msg)

        # Zoom rate command
        zoom_msg = Float32()
        zoom_msg.data = float(action[6])
        self._zoom_pub.publish(zoom_msg)

    def publish_zero(self):
        """Publish zero action (safe stop)."""
        self.publish(np.zeros(7))
