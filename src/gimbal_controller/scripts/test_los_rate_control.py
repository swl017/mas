#!/usr/bin/env python3
"""
Test LOS rate control: hold gimbal pointing forward using 0x07 rate commands
with proportional feedback from 0x0D state.

Target: yaw=0, pitch=0 (forward-looking, body-centered)
Feedback: gimbal_state_rpy_deg (joint-frame derived angles)
Output: gimbal_cmd_los_rate (heading-frame rate, 0x07)

Usage:
  Terminal 1: ros2 run gimbal_controller siyi_ros_node
  Terminal 2: python3 test_los_rate_control.py

  Rotate the drone body — the gimbal should counter-rotate to hold forward.
  Ctrl+C to stop (sends zero rate to stop gimbal).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
import math
import signal
import sys


class LOSRateController(Node):
    def __init__(self):
        super().__init__('los_rate_control_test')

        self.declare_parameter('target_yaw_deg', 0.0)
        self.declare_parameter('target_pitch_deg', 0.0)
        self.declare_parameter('kp_yaw', 10.0)
        self.declare_parameter('kp_pitch', 10.0)
        self.declare_parameter('max_rate', 80)  # max normalized rate (0-100)
        self.declare_parameter('control_hz', 50.0)

        self.target_yaw = self.get_parameter('target_yaw_deg').value
        self.target_pitch = self.get_parameter('target_pitch_deg').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.max_rate = self.get_parameter('max_rate').value
        control_hz = self.get_parameter('control_hz').value

        qos = QoSProfile(depth=10)

        # Subscribe to joint-frame state (x=roll, y=pitch, z=yaw)
        self._state_yaw = 0.0
        self._state_pitch = 0.0
        self._state_received = False
        self.create_subscription(
            Vector3,
            'siyi_gimbal_angles/state_rpy_deg',
            self._state_cb,
            qos)

        # Publish rate command (x=yaw_rate, y=pitch_rate, normalized -1..1)
        self.rate_pub = self.create_publisher(Vector3, 'gimbal_cmd_los_rate', qos)

        # Control loop timer
        self.create_timer(1.0 / control_hz, self._control)
        self._count = 0

        self.get_logger().info(
            f'LOS rate controller: target=({self.target_yaw:.1f}, {self.target_pitch:.1f}) '
            f'Kp=({self.kp_yaw}, {self.kp_pitch}) max_rate={self.max_rate}')

    def _state_cb(self, msg: Vector3):
        # State convention: x=roll, y=pitch, z=yaw
        self._state_yaw = msg.z
        self._state_pitch = msg.y
        self._state_received = True

    def _control(self):
        if not self._state_received:
            return

        # Error (degrees)
        err_yaw = self.target_yaw - self._state_yaw
        err_pitch = self.target_pitch - self._state_pitch

        # Wrap yaw error to [-180, 180]
        err_yaw = (err_yaw + 180) % 360 - 180

        # Proportional control (output: normalized -1..1)
        cmd_yaw = -self.kp_yaw * err_yaw / self.max_rate
        cmd_pitch = self.kp_pitch * err_pitch / self.max_rate

        # Clamp to [-1, 1]
        cmd_yaw = max(-1.0, min(1.0, cmd_yaw))
        cmd_pitch = max(-1.0, min(1.0, cmd_pitch))

        # Publish rate command (x=yaw, y=pitch)
        msg = Vector3()
        msg.x = cmd_yaw
        msg.y = cmd_pitch
        msg.z = 0.0
        self.rate_pub.publish(msg)

        # Print status every 10 ticks
        self._count += 1
        if self._count % 10 == 0:
            print(f'  state: yaw={self._state_yaw:>7.1f} pitch={self._state_pitch:>7.1f}  '
                  f'err: yaw={err_yaw:>7.1f} pitch={err_pitch:>7.1f}  '
                  f'cmd: yaw={cmd_yaw:>6.2f} pitch={cmd_pitch:>6.2f}')

    def stop(self):
        """Send zero rate to stop gimbal."""
        msg = Vector3()
        self.rate_pub.publish(msg)
        self.get_logger().info('Sent zero rate — gimbal stopped.')


def main():
    rclpy.init()
    node = LOSRateController()

    def shutdown(sig, frame):
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    print('LOS rate control active. Rotate the drone body — gimbal should hold forward.')
    print('Ctrl+C to stop.\n')
    print(f'  {"state":^30}  {"error":^24}  {"cmd":^18}')

    rclpy.spin(node)


if __name__ == '__main__':
    main()
