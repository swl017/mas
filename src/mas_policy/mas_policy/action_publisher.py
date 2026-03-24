"""Action publisher: 7D policy actions → ROS2 velocity and gimbal commands.

Publishes to downstream nodes:
- offboard_py: /{veh}/cmd_vel (TwistStamped, ENU) for velocity + yaw rate
- los_rate_controller: /{veh}/gimbal_cmd_los_rate (Vector3, normalized [-1,1])
- zoom: /{veh}/zoom_cmd (Float32, normalized)

Action mapping from _pre_physics_step() at iris_ma_env6_test.py:523-530:
  [0-2] vx,vy,vz  × max_lin_vel → cmd_vel linear (ENU)
  [3]   yaw_rate   × max_yaw_rate → cmd_vel angular.z
  [4]   gimbal_az  pass-through [-1,1] → gimbal_cmd_los_rate.x
  [5]   gimbal_el  pass-through [-1,1] → gimbal_cmd_los_rate.y
  [6]   zoom_rate  pass-through [-1,1] → zoom_cmd
"""

from __future__ import annotations

import logging

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Float32

import numpy as np

logger = logging.getLogger(__name__)


class ActionPublisher:
    """Converts policy actions to ROS2 commands for offboard_py and los_rate_controller."""

    def __init__(
        self,
        node: Node,
        vehicle_names: list[str],
        max_lin_vel: float = 10.0,
        max_yaw_rate: float = 0.7854,
    ):
        """Initialize action publisher.

        Args:
            node: ROS2 node to create publishers on.
            vehicle_names: List of vehicle namespace prefixes.
            max_lin_vel: Maximum linear velocity (m/s) for action scaling.
            max_yaw_rate: Maximum yaw rate (rad/s) for action scaling.
        """
        self._node = node
        self._vehicle_names = vehicle_names
        self._max_lin_vel = max_lin_vel
        self._max_yaw_rate = max_yaw_rate

        # Create publishers per vehicle
        self._cmd_vel_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._gimbal_rate_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._zoom_pubs: dict[str, rclpy.publisher.Publisher] = {}

        for veh in vehicle_names:
            self._cmd_vel_pubs[veh] = node.create_publisher(
                TwistStamped,
                f'/{veh}/cmd_vel',
                10,
            )
            self._gimbal_rate_pubs[veh] = node.create_publisher(
                Vector3,
                f'/{veh}/gimbal_cmd_los_rate',
                10,
            )
            self._zoom_pubs[veh] = node.create_publisher(
                Float32,
                f'/{veh}/zoom_cmd',
                10,
            )

    def publish(self, actions: dict[str, np.ndarray]):
        """Publish actions for all vehicles.

        Args:
            actions: Dictionary mapping vehicle name to 7D action array (clipped to [-1, 1]).
        """
        now = self._node.get_clock().now().to_msg()

        for veh, action in actions.items():
            if veh not in self._cmd_vel_pubs:
                logger.warning(f"Unknown vehicle: {veh}")
                continue

            # --- Velocity command (ENU) ---
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = now
            cmd_vel.header.frame_id = 'map'

            # Scale velocity from [-1,1] to physical units
            cmd_vel.twist.linear.x = float(action[0] * self._max_lin_vel)
            cmd_vel.twist.linear.y = float(action[1] * self._max_lin_vel)
            cmd_vel.twist.linear.z = float(action[2] * self._max_lin_vel)
            cmd_vel.twist.angular.z = float(action[3] * self._max_yaw_rate)

            self._cmd_vel_pubs[veh].publish(cmd_vel)

            # --- Gimbal LOS rate command (normalized [-1, 1]) ---
            gimbal_msg = Vector3()
            gimbal_msg.x = float(action[4])  # azimuth rate (normalized)
            gimbal_msg.y = float(action[5])  # elevation rate (normalized)
            gimbal_msg.z = 0.0               # unused

            self._gimbal_rate_pubs[veh].publish(gimbal_msg)

            # --- Zoom rate command ---
            zoom_msg = Float32()
            zoom_msg.data = float(action[6])

            self._zoom_pubs[veh].publish(zoom_msg)

    def publish_zero(self):
        """Publish zero actions for all vehicles (safe stop)."""
        zero_actions = {veh: np.zeros(7) for veh in self._vehicle_names}
        self.publish(zero_actions)
