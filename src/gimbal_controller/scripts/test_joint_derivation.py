#!/usr/bin/env python3
"""
Test joint angle derivation by comparing encoder_rpy_deg (derived joint angles)
against state_rpy_deg (raw 0x0D heading-frame) while tilting the base.

Requires:
  - siyi_ros_node running
  - mavros/imu/data publishing (or not — to test fallback)

Test procedure:
  1. Gimbal on bench, level — both topics should match (aircraft attitude = 0)
  2. Tilt the base in pitch — state_rpy_deg pitch should stay ~0 (heading-stabilized),
     encoder_rpy_deg pitch should show the tilt angle (joint compensating)
  3. Tilt in roll — same logic

Usage:
  ros2 run gimbal_controller siyi_ros_node  (in another terminal)
  python3 test_joint_derivation.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import math


class JointDerivationTest(Node):
    def __init__(self):
        super().__init__('joint_derivation_test')

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        default_qos = QoSProfile(depth=10)

        self._state = None   # 0x0D heading-frame (state_rpy_deg)
        self._joint = None   # derived joint-frame (encoder_rpy_deg)
        self._imu = None     # aircraft attitude from IMU

        # Subscribe to both gimbal topics
        self.create_subscription(
            Vector3, 'siyi_gimbal_angles/state_rpy_deg',
            self._state_cb, default_qos)
        self.create_subscription(
            Vector3, 'siyi_gimbal_angles/encoder_rpy_deg',
            self._joint_cb, default_qos)
        self.create_subscription(
            Imu, 'mavros/imu/data',
            self._imu_cb, best_effort)

        # Print at 5 Hz
        self.create_timer(0.2, self._print)
        self._count = 0

    def _state_cb(self, msg):
        self._state = msg

    def _joint_cb(self, msg):
        self._joint = msg

    def _imu_cb(self, msg):
        self._imu = msg

    def _print(self):
        self._count += 1

        # Header every 20 lines
        if (self._count - 1) % 20 == 0:
            imu_hdr = "  IMU_roll  IMU_pitch" if self._imu else ""
            print(f"\n{'':>6}  {'--- state (heading) ---':^24}  {'--- joint (derived) ---':^24}  {'--- delta ---':^16}{imu_hdr}")
            print(f"{'#':>6}  {'roll':>8}{'pitch':>8}{'yaw':>8}  {'roll':>8}{'pitch':>8}{'yaw':>8}  {'d_pitch':>8}{'d_roll':>8}", end="")
            if self._imu:
                print(f"  {'roll':>8}{'pitch':>8}", end="")
            print()

        if self._state is None or self._joint is None:
            print(f"{'--':>6}  waiting for data...")
            return

        s = self._state
        j = self._joint

        d_pitch = j.y - s.y
        d_roll = j.x - s.x

        line = (f"{self._count:>6}  "
                f"{s.x:>8.1f}{s.y:>8.1f}{s.z:>8.1f}  "
                f"{j.x:>8.1f}{j.y:>8.1f}{j.z:>8.1f}  "
                f"{d_pitch:>8.1f}{d_roll:>8.1f}")

        if self._imu:
            from transforms3d.euler import quat2euler
            q = self._imu.orientation
            r, p, _ = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
            line += f"  {math.degrees(r):>8.1f}{math.degrees(p):>8.1f}"

        print(line)


def main():
    rclpy.init()
    print("Joint derivation test")
    print("  state = 0x0D heading-frame (pitch/roll stabilized to world)")
    print("  joint = derived joint-frame (pitch/roll = state - aircraft)")
    print("  delta = joint - state (should equal -aircraft attitude)")
    print()
    print("  Without IMU: both should match (aircraft = 0)")
    print("  With IMU + tilt: joint should show compensation angle")
    print()
    node = JointDerivationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
