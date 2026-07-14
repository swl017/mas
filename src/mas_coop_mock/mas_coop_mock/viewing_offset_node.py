"""ticket 019 mock-cooperative — scripted observer driver (replaces the RL policy).

Deterministic stand-in for the cooperative-sensing CONTROL policy: holds the
observer (px4_3) at a viewing-parallax offset off the interceptor->target LOS so
px4_3 supplies parallax the interceptor alone cannot. Publishes a moving
`goto_position` (PoseStamped, common_frame) that `mas_offboard offboard_control`
chases in HOVER — px4_3 stays out of MISSION/POLICY. `offset_deg` is the parallax
knob (0=degenerate .. ~90=orthogonal/favorable, 018 convention), re-read each tick
so it can be swept live.

    ros2 run mas_coop_mock viewing_offset --ros-args -r __ns:=/px4_3 \
        -p target_ns:=px4_2 -p interceptor_ns:=px4_1 -p offset_deg:=45.0 \
        -p standoff_m:=45.0 -p use_sim_time:=true
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from .core import viewing_pose


def _be(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _reliable(depth: int = 1) -> QoSProfile:
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.VOLATILE)


class ViewingOffset(Node):
    def __init__(self):
        super().__init__('viewing_offset')
        self.declare_parameter('interceptor_ns', 'px4_1')
        self.declare_parameter('target_ns', 'px4_2')
        self.declare_parameter('offset_deg', 45.0)     # parallax knob (re-read each tick)
        self.declare_parameter('standoff_m', 45.0)
        self.declare_parameter('height_m', 0.0)        # <=0 -> hold target altitude
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('goto_topic', 'goto_position')
        self.declare_parameter('frame_id', 'common_frame')

        i_ns = str(self.get_parameter('interceptor_ns').value).strip('/')
        t_ns = str(self.get_parameter('target_ns').value).strip('/')
        self.frame = str(self.get_parameter('frame_id').value)
        self.p_int = None
        self.p_tgt = None
        self.create_subscription(Odometry, f'/{i_ns}/common_frame/odom', self._on_int, _be())
        self.create_subscription(Odometry, f'/{t_ns}/common_frame/odom', self._on_tgt, _be())
        self.pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goto_topic').value), _reliable())
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"viewing_offset: observe /{t_ns} at offset off /{i_ns} LOS -> "
            f"{self.get_parameter('goto_topic').value} (common_frame)")

    def _on_int(self, m: Odometry):
        self.p_int = m.pose.pose.position

    def _on_tgt(self, m: Odometry):
        self.p_tgt = m.pose.pose.position

    def _tick(self):
        if self.p_int is None or self.p_tgt is None:
            return
        h = float(self.get_parameter('height_m').value)
        pos = viewing_pose(
            [self.p_int.x, self.p_int.y, self.p_int.z],
            [self.p_tgt.x, self.p_tgt.y, self.p_tgt.z],
            float(self.get_parameter('offset_deg').value),
            float(self.get_parameter('standoff_m').value),
            None if h <= 0.0 else h)
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ViewingOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
