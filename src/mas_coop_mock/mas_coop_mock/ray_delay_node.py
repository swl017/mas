"""ticket 019 mock-cooperative — peer-communication (AoI) delay stage.

Models "the peer computes its bearing locally and transmits it late": buffers the
observer's self-contained bearing ray (mas_msgs/TargetRayArray, the triangulation
`use_precomputed_rays` peer path) and republishes it after `tau_s`, so the fusion
sees px4_3's contribution stale by tau. `tau_s` is re-read each message (settable
live via `ros2 param set` for the latency sweep); tau=0 is a passthrough. Delaying
this one topic (not `chosen_target_pose`) staleness-shifts ONLY the peer, not the
whole fused estimate.

    ros2 run mas_coop_mock ray_delay --ros-args \
        -r in_topic:=/px4_3/target_rays_w_raw -r out_topic:=/px4_3/target_rays_w \
        -p tau_s:=0.1 -p use_sim_time:=true
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from mas_msgs.msg import TargetRayArray

import numpy as np

from .core import JitterDropBuffer


def _be(depth: int = 20) -> QoSProfile:
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _reliable(depth: int = 20) -> QoSProfile:
    # the triangulation's precomputed-rays subscription is RELIABLE, so the delayed
    # peer ray must be published RELIABLE to be received (reliable pub -> reliable sub).
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.VOLATILE)


class RayDelay(Node):
    def __init__(self):
        super().__init__('ray_delay')
        self.declare_parameter('in_topic', 'target_rays_w_raw')
        self.declare_parameter('out_topic', 'target_rays_w')
        # Peer comm latency (ticket 019 S6 fair ego/peer model): mean latency_s (swept)
        # + Gaussian jitter [+ burst dropout]. Same realistic model as cv_smoother, but
        # applied ONLY to the peer's ray (ego camera 1 stays fresh in the fusion).
        self.declare_parameter('latency_s', 0.0)          # mean peer latency [s] (swept)
        self.declare_parameter('latency_jitter_s', 0.01)  # comm jitter [s] (018: 100+-10 ms)
        self.declare_parameter('drop_p', 0.0)             # burst-drop prob (0 = off)
        self.declare_parameter('drop_burst', 3)
        self.declare_parameter('seed', 11)
        self.declare_parameter('release_rate_hz', 200.0)

        self.buf = JitterDropBuffer(
            mean_s=max(0.0, float(self.get_parameter('latency_s').value)),
            jitter_s=max(0.0, float(self.get_parameter('latency_jitter_s').value)),
            drop_p=float(self.get_parameter('drop_p').value),
            drop_burst=int(self.get_parameter('drop_burst').value),
            rng=np.random.default_rng(int(self.get_parameter('seed').value)))
        self.create_subscription(
            TargetRayArray, str(self.get_parameter('in_topic').value), self._on, _be())
        self.pub = self.create_publisher(
            TargetRayArray, str(self.get_parameter('out_topic').value), _reliable())
        rate = max(1.0, float(self.get_parameter('release_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ray_delay: {self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value} (peer latency mean={self.buf.mean}s "
            f"jitter={self.buf.jitter}s drop_p={self.buf.drop_p})")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on(self, msg: TargetRayArray):
        self.buf.mean = max(0.0, float(self.get_parameter('latency_s').value))        # live
        self.buf.jitter = max(0.0, float(self.get_parameter('latency_jitter_s').value))
        self.buf.drop_p = float(self.get_parameter('drop_p').value)
        self.buf.push(self._now(), msg)

    def _tick(self):
        for m in self.buf.pop_ready(self._now()):
            self.pub.publish(m)


def main():
    rclpy.init()
    node = RayDelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
