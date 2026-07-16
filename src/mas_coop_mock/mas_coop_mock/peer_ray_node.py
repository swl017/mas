"""ticket 019 S6 — peer bearing ray for the FAIR ego/peer latency decomposition.

Publishes the observer's (px4_2) line-of-sight to the target as a self-contained
`mas_msgs/TargetRayArray` (origin = observer position, direction = unit LOS in
common_frame) so the triangulation can consume it via `use_precomputed_rays` for
camera 2 while camera 1 (px4_1, the interceptor/ego) stays RAW and FRESH. Routed
through `ray_delay` (peer comm latency + jitter) so ONLY the peer's contribution is
staled — the fair model the whole-belief AoI could not express.

The LOS is GT-derived (observer odom → target odom) with σ_θ bearing noise added
(detector-grade). This isolates the peer LATENCY: the peer bearing is realistic in
noise but GT-clean in geometry (no camera-model bias), and stale by the comm delay.
(The parallax sweep used the peer's real camera; this latency arm swaps to the GT
ray to isolate staleness — stated in the deliverable.)

    ros2 run mas_coop_mock peer_ray --ros-args \
        -p observer_ns:=px4_2 -p target_ns:=px4_3 -p sigma_deg:=0.5 -p use_sim_time:=true
"""
from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from mas_msgs.msg import TargetRayArray, TargetRay


def _be(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


class PeerRay(Node):
    def __init__(self):
        super().__init__('peer_ray')
        self.declare_parameter('observer_ns', 'px4_2')
        self.declare_parameter('target_ns', 'px4_3')
        self.declare_parameter('out_topic', 'target_rays_w_raw')  # -> ray_delay -> target_rays_w
        self.declare_parameter('frame_id', 'common_frame')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('sigma_deg', 0.5)                  # detector-grade bearing noise
        self.declare_parameter('detection_id', 'peer_0')
        self.declare_parameter('seed', 7)

        obs = str(self.get_parameter('observer_ns').value).strip('/')
        tgt = str(self.get_parameter('target_ns').value).strip('/')
        self.frame = str(self.get_parameter('frame_id').value)
        self.sigma = np.deg2rad(float(self.get_parameter('sigma_deg').value))
        self.det_id = str(self.get_parameter('detection_id').value)
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))
        self.p_obs = None
        self.p_tgt = None
        self.create_subscription(Odometry, f'/{obs}/common_frame/odom', self._on_obs, _be())
        self.create_subscription(Odometry, f'/{tgt}/common_frame/odom', self._on_tgt, _be())
        self.pub = self.create_publisher(
            TargetRayArray, str(self.get_parameter('out_topic').value), _be())
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"peer_ray: LOS /{obs}->/{tgt} (sigma={np.rad2deg(self.sigma):.2f}deg) -> "
            f"{self.get_parameter('out_topic').value}")

    def _on_obs(self, m: Odometry):
        p = m.pose.pose.position
        self.p_obs = np.array([p.x, p.y, p.z])

    def _on_tgt(self, m: Odometry):
        p = m.pose.pose.position
        self.p_tgt = np.array([p.x, p.y, p.z])

    def _tick(self):
        if self.p_obs is None or self.p_tgt is None:
            return
        d = self.p_tgt - self.p_obs
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return
        u = d / n
        if self.sigma > 0.0:                       # detector-grade angular noise
            u = u + self.rng.normal(0.0, self.sigma, 3)
            u = u / float(np.linalg.norm(u))
        msg = TargetRayArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.origin.x, msg.origin.y, msg.origin.z = map(float, self.p_obs)
        ray = TargetRay()
        ray.direction = Vector3(x=float(u[0]), y=float(u[1]), z=float(u[2]))
        ray.detection_id = self.det_id
        msg.rays = [ray]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PeerRay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
