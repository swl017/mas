"""Deterministic target-maneuver replay — the non-cooperative quarry (Ticket 004).

Drives the target drone through the point-mass regimes (forward speed + lateral
sinusoid) so the ego-only interceptor has a meaningful, observable target. The
velocity profile mirrors the Ticket 003 point-mass `scenario_grid.target_truth`:

    v(t) = target_speed · f̂  +  A·ω·cos(ω·(t−t0)) · l̂      (ENU)
    f̂ = (cos h, sin h, 0)            forward heading h (deg from +x toward +y)
    l̂ = unit((−sin h, cos h, 0) + (0,0, vertical_ratio))   lateral axis ⟂ f̂

Publishes `maneuver/cmd_vel`; the target's `mas_mission` (engagement_source=
maneuver) forwards it as `cmd_vel` in MISSION → `mas_offboard`. Phase t0 resets
on MISSION entry so the maneuver is reproducible per engagement. Deterministic
(no RNG). Regime presets are set by the launch.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)

from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int8

MISSION = 2


def _be(depth=10):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _reliable_tl():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class TargetManeuverNode(Node):
    def __init__(self):
        super().__init__("target_maneuver_node")

        # Regime parameters (launch sets these from the preset; defaults =
        # dynamic_mild). Match scenario_grid AGILITY_LEVELS.
        self.declare_parameter("target_speed_mps", 6.0)
        self.declare_parameter("sinusoid_amplitude_m", 1.0)
        self.declare_parameter("sinusoid_frequency_hz", 0.20)
        self.declare_parameter("forward_heading_deg", 0.0)
        self.declare_parameter("vertical_ratio", 0.25)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("regime", "dynamic_mild")  # label only

        # Derive the velocity-profile vectors; recompute whenever the conductor
        # switches regime at runtime via a parameter set (ticket 004).
        self._apply_params()
        self.add_on_set_parameters_callback(self._on_set_params)

        self.mission_state = 0
        self.t0 = None   # node-clock time of MISSION entry (phase origin)

        self.create_subscription(Int8, "mission_state", self._on_mission, _reliable_tl())
        self.pub = self.create_publisher(TwistStamped, "maneuver/cmd_vel", _be())

        rate = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"target_maneuver '{self.get_parameter('regime').value}': "
            f"speed={self.speed} amp={self.amp} freq={self.freq} "
            f"heading={self.get_parameter('forward_heading_deg').value:.0f}deg "
            f"-> maneuver/cmd_vel")

    def _apply_params(self, override=None):
        """(Re)derive f̂, l̂, ω from the regime params. ``override`` carries the
        not-yet-applied values during a parameter-set callback."""
        override = override or {}

        def g(name):
            return override[name] if name in override else self.get_parameter(name).value

        self.speed = float(g("target_speed_mps"))
        self.amp = float(g("sinusoid_amplitude_m"))
        self.freq = float(g("sinusoid_frequency_hz"))
        h = math.radians(float(g("forward_heading_deg")))
        vr = float(g("vertical_ratio"))
        self.f_hat = np.array([math.cos(h), math.sin(h), 0.0])
        lat = np.array([-math.sin(h), math.cos(h), 0.0]) + np.array([0.0, 0.0, vr])
        self.l_hat = lat / np.linalg.norm(lat)
        self.omega = 2.0 * math.pi * self.freq

    def _on_set_params(self, params):
        vals = {p.name: p.value for p in params}
        relevant = {"target_speed_mps", "sinusoid_amplitude_m", "sinusoid_frequency_hz",
                    "forward_heading_deg", "vertical_ratio"}
        if relevant & set(vals):
            self._apply_params(vals)
            self.get_logger().info(
                f"regime '{vals.get('regime', self.get_parameter('regime').value)}' "
                f"applied: speed={self.speed} amp={self.amp} freq={self.freq}")
        return SetParametersResult(successful=True)

    def _on_mission(self, msg: Int8):
        entering = (msg.data == MISSION and self.mission_state != MISSION)
        self.mission_state = int(msg.data)
        if entering:
            self.t0 = self.get_clock().now()   # reset phase at engagement

    def _tick(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "common_frame"
        if self.mission_state == MISSION:
            if self.t0 is None:
                self.t0 = self.get_clock().now()
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            v = self.speed * self.f_hat + self.amp * self.omega * math.cos(self.omega * t) * self.l_hat
            out.twist.linear.x = float(v[0])
            out.twist.linear.y = float(v[1])
            out.twist.linear.z = float(v[2])
        # else: zero twist (target holds; mas_mission won't forward it anyway)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetManeuverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
