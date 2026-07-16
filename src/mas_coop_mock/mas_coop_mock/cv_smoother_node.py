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

import numpy as np

from .core import AlphaBetaVel, JitterDropBuffer


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
        # Cooperative-belief age-of-information (ticket 019 S6 latency axis): a realistic
        # peer-comm AoI on the published belief — mean latency (latency_s = the swept
        # variable, live-settable) + Gaussian jitter + optional BURST DROPOUT (drop_p>0
        # -> level 4, cf. ticket-018 latency_real/a2nom + the RL sim's other_latency
        # mean/std). It stales the whole fused belief (a conservative bound on peer-only
        # latency). latency_s=0, jitter=0, drop_p=0 -> passthrough.
        self.declare_parameter('latency_s', 0.0)          # mean AoI [s] (swept)
        self.declare_parameter('latency_jitter_s', 0.08)  # Gaussian std [s] (RL-sim-like)
        self.declare_parameter('drop_p', 0.0)             # per-msg burst-drop prob (0 = off)
        self.declare_parameter('drop_burst', 3)           # dropped run length
        self.declare_parameter('seed', 42)                # reproducible jitter/drop draws

        in_topic = str(self.get_parameter('in_topic').value)
        prefix = str(self.get_parameter('coop_prefix').value).strip('/')
        self.f = AlphaBetaVel(alpha=float(self.get_parameter('alpha').value),
                              beta=float(self.get_parameter('beta').value),
                              v_max=float(self.get_parameter('v_max').value))
        self.buf = JitterDropBuffer(
            mean_s=max(0.0, float(self.get_parameter('latency_s').value)),
            jitter_s=max(0.0, float(self.get_parameter('latency_jitter_s').value)),
            drop_p=float(self.get_parameter('drop_p').value),
            drop_burst=int(self.get_parameter('drop_burst').value),
            rng=np.random.default_rng(int(self.get_parameter('seed').value)))
        self.create_subscription(PoseWithCovarianceStamped, in_topic, self._on_pose, _be())
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, f'{prefix}/target_pose', _be())
        self.pub_twist = self.create_publisher(TwistStamped, f'{prefix}/target_twist', _be())
        self.create_timer(0.02, self._release)          # 50 Hz belief release
        self.get_logger().info(
            f"cv_smoother: {in_topic} -> {prefix}/target_pose + {prefix}/target_twist "
            f"(alpha={self.f.alpha}, beta={self.f.beta}, AoI mean={self.buf.mean}s "
            f"jitter={self.buf.jitter}s drop_p={self.buf.drop_p})")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        _, v = self.f.update(_t(msg.header.stamp), [p.x, p.y, p.z])
        tw = TwistStamped()
        tw.header = msg.header                          # common_frame, same stamp
        tw.twist.linear.x = float(v[0])
        tw.twist.linear.y = float(v[1])
        tw.twist.linear.z = float(v[2])
        self.buf.mean = max(0.0, float(self.get_parameter('latency_s').value))        # live
        self.buf.jitter = max(0.0, float(self.get_parameter('latency_jitter_s').value))
        self.buf.drop_p = float(self.get_parameter('drop_p').value)
        self.buf.push(self._now(), (msg, tw))          # jittered/dropped AoI; see _release

    def _release(self):
        for msg, tw in self.buf.pop_ready(self._now()):
            self.pub_pose.publish(msg)                  # pose + covariance passthrough
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
