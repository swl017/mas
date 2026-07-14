"""ticket 019 mock-cooperative — velocity for the fused cooperative belief.

Subscribes the multi-observer fused target pose (`chosen_target_pose`,
PoseWithCovarianceStamped, common_frame) from mas_tracker/sort3d, runs a
constant-velocity alpha-beta tracker to recover target velocity (SORT3D's own KF
velocity is per-frame / event-rate and unpublished), and republishes the drop-in
PN cooperative contract under the interceptor namespace:

    {coop_prefix}/target_pose   PoseWithCovarianceStamped  (passthrough pose+cov)
    {coop_prefix}/target_twist  TwistStamped               (alpha-beta velocity)

so pn_guidance (estimate_source=cooperative, PREFIX['cooperative']='coop_loc')
consumes it exactly like an EKF arm. This is the ticket-018 belief-smoother role,
closed-loop.

    ros2 run mas_coop_mock cv_smoother --ros-args -r __ns:=/px4_1 -p use_sim_time:=true
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped

from .core import AlphaBetaVel


def _be(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _t(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CvSmoother(Node):
    def __init__(self):
        super().__init__('cv_smoother')
        self.declare_parameter('in_topic', 'chosen_target_pose')
        self.declare_parameter('coop_prefix', 'coop_loc')
        self.declare_parameter('alpha', 0.6)
        self.declare_parameter('beta', 0.15)
        self.declare_parameter('v_max', 30.0)

        in_topic = str(self.get_parameter('in_topic').value)
        prefix = str(self.get_parameter('coop_prefix').value).strip('/')
        self.f = AlphaBetaVel(alpha=float(self.get_parameter('alpha').value),
                              beta=float(self.get_parameter('beta').value),
                              v_max=float(self.get_parameter('v_max').value))
        self.create_subscription(PoseWithCovarianceStamped, in_topic, self._on_pose, _be())
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, f'{prefix}/target_pose', _be())
        self.pub_twist = self.create_publisher(TwistStamped, f'{prefix}/target_twist', _be())
        self.get_logger().info(
            f"cv_smoother: {in_topic} -> {prefix}/target_pose + {prefix}/target_twist "
            f"(alpha={self.f.alpha}, beta={self.f.beta})")

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        _, v = self.f.update(_t(msg.header.stamp), [p.x, p.y, p.z])
        self.pub_pose.publish(msg)                      # pose + covariance passthrough
        tw = TwistStamped()
        tw.header = msg.header                          # common_frame, same stamp
        tw.twist.linear.x = float(v[0])
        tw.twist.linear.y = float(v[1])
        tw.twist.linear.z = float(v[2])
        self.pub_twist.publish(tw)


def main():
    rclpy.init()
    node = CvSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
