"""PN guidance node — turns a target estimate into an engagement velocity command.

Reuses the exact point-mass guidance law (`pn_law.proportional_navigation`) and
its constant-speed-pursuer dynamics: on engagement the commanded velocity is
seeded as a `v_max` pursuit toward the target, then each tick
`v_cmd = clamp(v_cmd + a_pn·dt, v_max)`. Publishes a dedicated `pn/cmd_vel`
(ENU `TwistStamped`); `mas_mission` (with `engagement_source=pn`) forwards it to
`cmd_vel` in MISSION. Decoupled pattern: subscribers cache, the control timer is
the sole compute/publish point.

`estimate_source` selects the target input:
  oracle             -> /{target_namespace}/common_frame/odom  (ground-truth ceiling)
  simple_ekf         -> simple_loc/target_pose + simple_loc/target_twist
  direct_projection  -> direct_loc/target_pose + direct_loc/target_twist
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)

from geometry_msgs.msg import (
    PoseWithCovarianceStamped, TwistStamped, Vector3,
)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, Int8

from .pn_law import proportional_navigation, limit_norm, unit

MISSION = 2  # mas_mission state enum

PREFIX = {"simple_ekf": "simple_loc", "direct_projection": "direct_loc"}


def _be(depth=10):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _reliable_tl(depth=1):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _t(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PNGuidanceNode(Node):
    def __init__(self):
        super().__init__("pn_guidance_node")

        self.declare_parameter("nav_constant", 3.0)
        self.declare_parameter("v_max", 9.0)
        self.declare_parameter("a_max", 6.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("estimate_source", "oracle")
        self.declare_parameter("target_namespace", "")
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("cov_trace_gate", 0.0)   # 0 = disabled (EKF arms)
        self.declare_parameter("engage_in_mission_only", True)
        self.declare_parameter("guidance_uses_measured_velocity", False)

        self.N = float(self.get_parameter("nav_constant").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.source = str(self.get_parameter("estimate_source").value)
        self.stale = float(self.get_parameter("stale_timeout_s").value)
        self.cov_gate = float(self.get_parameter("cov_trace_gate").value)
        self.mission_only = bool(self.get_parameter("engage_in_mission_only").value)
        self.use_meas_vel = bool(self.get_parameter("guidance_uses_measured_velocity").value)

        # ego state (actual, from odom)
        self.own_p: Optional[np.ndarray] = None
        self.own_v: Optional[np.ndarray] = None
        # Target estimate, buffered per source. We subscribe to ALL sources and
        # select the active one via the `estimate_source` param — switchable at
        # runtime so the conductor can randomize estimator within a boot
        # (ticket 004). `tgt_*` mirror the active buffer, refreshed each control
        # tick by _refresh_active().
        self._buf = {k: {"p": None, "v": None, "cov": 0.0, "rx": None}
                     for k in ("oracle", *PREFIX)}
        self.tgt_p: Optional[np.ndarray] = None
        self.tgt_v: Optional[np.ndarray] = None
        self.tgt_cov_trace: float = 0.0
        self.tgt_rx = None          # node-clock time the target estimate arrived
        self.mission_state: int = 0
        # internal commanded (guidance) velocity — the integrated PN velocity
        self.v_cmd: Optional[np.ndarray] = None

        if self.source not in self._buf:
            raise ValueError(f"unknown estimate_source '{self.source}'")

        be = _be()
        self.create_subscription(Odometry, "common_frame/odom", self._on_own_odom, be)
        self.create_subscription(Int8, "mission_state", self._on_mission, _reliable_tl())

        tns = str(self.get_parameter("target_namespace").value).strip("/")
        if not tns:
            from .roles import Roles
            tns = Roles.load().namespace("target")
        self.create_subscription(
            Odometry, f"/{tns}/common_frame/odom", self._on_target_odom, be)
        for src, p in PREFIX.items():
            self.create_subscription(
                PoseWithCovarianceStamped, f"{p}/target_pose",
                lambda m, s=src: self._on_target_pose(m, s), be)
            self.create_subscription(
                TwistStamped, f"{p}/target_twist",
                lambda m, s=src: self._on_target_twist(m, s), be)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info(
            f"PN guidance: active estimate_source='{self.source}' "
            f"(oracle=/{tns}/common_frame/odom; EKF arms={list(PREFIX)})")

        self.pub_cmd = self.create_publisher(TwistStamped, "pn/cmd_vel", 10)
        self.pub_diag = self.create_publisher(Float64MultiArray, "pn/diagnostics", 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._control)
        self.get_logger().info(
            f"PN guidance ready: N={self.N} v_max={self.v_max} a_max={self.a_max} "
            f"rate={rate:.0f}Hz mission_only={self.mission_only}")

    # ---- subscribers (cache only) ----
    def _on_own_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.own_p = np.array([p.x, p.y, p.z])
        self.own_v = np.array([v.x, v.y, v.z])

    def _on_mission(self, msg: Int8):
        self.mission_state = int(msg.data)

    def _on_target_odom(self, msg: Odometry):  # oracle
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        b = self._buf["oracle"]
        b["p"] = np.array([p.x, p.y, p.z])
        b["v"] = np.array([v.x, v.y, v.z])
        b["cov"] = 0.0
        b["rx"] = self.get_clock().now()

    def _on_target_pose(self, msg: PoseWithCovarianceStamped, src: str):  # EKF
        p = msg.pose.pose.position
        b = self._buf[src]
        b["p"] = np.array([p.x, p.y, p.z])
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        b["cov"] = float(np.trace(cov[:3, :3]))
        b["rx"] = self.get_clock().now()

    def _on_target_twist(self, msg: TwistStamped, src: str):  # EKF
        v = msg.twist.linear
        self._buf[src]["v"] = np.array([v.x, v.y, v.z])

    def _on_set_params(self, params):
        for prm in params:
            if prm.name == "estimate_source":
                if prm.value not in self._buf:
                    return SetParametersResult(
                        successful=False,
                        reason=f"unknown estimate_source '{prm.value}'")
                self.source = str(prm.value)
                self.v_cmd = None  # reseed guidance from the newly selected source
                self.get_logger().info(f"estimate_source -> '{self.source}'")
        return SetParametersResult(successful=True)

    def _refresh_active(self) -> None:
        """Copy the selected source's buffer into the active tgt_* view."""
        b = self._buf[self.source]
        self.tgt_p, self.tgt_v = b["p"], b["v"]
        self.tgt_cov_trace, self.tgt_rx = b["cov"], b["rx"]

    # ---- control loop (sole compute + publish) ----
    def _engaged(self) -> bool:
        return (not self.mission_only) or (self.mission_state == MISSION)

    def _target_ok(self) -> bool:
        if self.tgt_p is None or self.tgt_v is None or self.tgt_rx is None:
            return False
        age = (self.get_clock().now() - self.tgt_rx).nanoseconds * 1e-9
        if age > self.stale:
            return False
        if self.cov_gate > 0.0 and self.tgt_cov_trace > self.cov_gate:
            return False
        return True

    def _control(self):
        self._refresh_active()
        if (not self._engaged()) or self.own_p is None or not self._target_ok():
            self.v_cmd = None            # reset pursuit; will re-seed on re-engage
            self._publish_zero()
            return

        # Seed the commanded velocity as a v_max pursuit (mirrors point-mass init).
        if self.v_cmd is None:
            self.v_cmd = self.v_max * unit(self.tgt_p - self.own_p)

        own_v_for_guidance = self.own_v if self.use_meas_vel else self.v_cmd
        cmd = proportional_navigation(
            self.own_p, own_v_for_guidance, self.tgt_p, self.tgt_v, self.N, self.a_max)
        # Integrate PN accel into the commanded velocity, clamp to v_max.
        self.v_cmd = limit_norm(self.v_cmd + cmd.acceleration_mps2 * self.dt, self.v_max)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "common_frame"
        out.twist.linear.x = float(self.v_cmd[0])
        out.twist.linear.y = float(self.v_cmd[1])
        out.twist.linear.z = float(self.v_cmd[2])
        self.pub_cmd.publish(out)

        diag = Float64MultiArray()
        diag.data = [
            cmd.closing_speed_mps, cmd.los_rate_radps, cmd.range_est_m,
            1.0 if cmd.saturated else 0.0, float(np.linalg.norm(self.v_cmd)),
            self.tgt_cov_trace,
        ]
        self.pub_diag.publish(diag)

    def _publish_zero(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "common_frame"
        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PNGuidanceNode()
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
