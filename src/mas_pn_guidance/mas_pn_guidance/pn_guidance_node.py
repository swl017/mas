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
    PoseWithCovarianceStamped, TwistStamped, Vector3, Vector3Stamped,
)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, Int8

from .pn_law import proportional_navigation, pn_from_los_rate, limit_norm, unit
from .los_rate import StampedLosRateDifferentiator, coast_decay

MISSION = 2  # mas_mission state enum

GUIDANCE_MODES = ("pn", "bearing_pn", "raw_ibvs")

PREFIX = {"simple_ekf": "simple_loc", "direct_projection": "direct_loc",
          "dc_ekf": "bearing_loc",   # legacy 18-D DC-EKF (in-state feature); ticket 011
          "cooperative": "coop_loc"}  # ticket 019: mock-cooperative fusion (mas_coop_mock)


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
        # "pn" = range-sensitive (LOS rate from cross(r,v)/|r|^2); "bearing_pn" =
        # range-TOLERANT (LOS rate from the position-derived bearing history);
        # "raw_ibvs" = range-FREE (LOS rate from the RAW detection bearing,
        # bearing_raw/los, decoupled from the EKF target_pose) — ticket 012.
        self.declare_parameter("guidance_mode", "pn")
        self.declare_parameter("los_rate_ema_alpha", 0.7)
        # raw_ibvs dropout policy (detection-time aged): within los_timeout_s the
        # measured LOS rate is used at full weight; between los_timeout_s and
        # los_lost_s the held rate is coasted to zero; past los_lost_s the target
        # is declared lost (zero command, differentiator reset).
        self.declare_parameter("los_timeout_s", 0.3)
        self.declare_parameter("los_lost_s", 0.8)

        self.N = float(self.get_parameter("nav_constant").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.source = str(self.get_parameter("estimate_source").value)
        self.stale = float(self.get_parameter("stale_timeout_s").value)
        self.cov_gate = float(self.get_parameter("cov_trace_gate").value)
        self.mission_only = bool(self.get_parameter("engage_in_mission_only").value)
        self.use_meas_vel = bool(self.get_parameter("guidance_uses_measured_velocity").value)
        self.guidance_mode = str(self.get_parameter("guidance_mode").value)
        if self.guidance_mode not in GUIDANCE_MODES:
            self.get_logger().warn(f"unknown guidance_mode '{self.guidance_mode}', using 'pn'")
            self.guidance_mode = "pn"
        self.los_ema_alpha = float(np.clip(self.get_parameter("los_rate_ema_alpha").value, 0.0, 1.0))
        self.los_timeout_s = float(self.get_parameter("los_timeout_s").value)
        self.los_lost_s = float(self.get_parameter("los_lost_s").value)

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
        # Stamped LOS-rate differentiators (shared discipline, ticket 011 #2/#3,
        # 012 #6): differentiate the LOS on NEW stamped samples (actual elapsed
        # time), EMA-hold between samples. `bearing_diff` consumes the estimator-
        # position-derived bearing (bearing_pn, keyed on the estimate arrival
        # time); `raw_diff` consumes the RAW detection bearing (raw_ibvs, keyed on
        # the detection header time).
        self.bearing_diff = StampedLosRateDifferentiator(self.los_ema_alpha)
        self.raw_diff = StampedLosRateDifferentiator(self.los_ema_alpha)
        # Latest raw world-LOS (raw_ibvs) — from bearing_raw/los, decoupled from
        # the EKF target_pose. `raw_stamp_ns` is the DETECTION header time.
        self.raw_n: Optional[np.ndarray] = None
        self.raw_stamp_ns: Optional[int] = None
        self._raw_missing_ticks = 0

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
        # Raw image-feature bearing for raw_ibvs — world-ENU unit LOS at the
        # detection rate (mas_bearing_loc raw_los_node), decoupled from the EKF.
        self.create_subscription(Vector3Stamped, "bearing_raw/los", self._on_raw_los, be)
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
            f"PN guidance ready: mode={self.guidance_mode} N={self.N} v_max={self.v_max} "
            f"a_max={self.a_max} rate={rate:.0f}Hz mission_only={self.mission_only}")

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

    def _on_raw_los(self, msg: Vector3Stamped):  # raw_ibvs image feature
        n = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        norm = float(np.linalg.norm(n))
        if not np.isfinite(norm) or norm < 1e-6:
            return
        self.raw_n = n / norm
        self.raw_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _reset_bearing_state(self):
        self.bearing_diff.reset()
        self.raw_diff.reset()

    def _on_set_params(self, params):
        for prm in params:
            if prm.name == "estimate_source":
                if prm.value not in self._buf:
                    return SetParametersResult(
                        successful=False,
                        reason=f"unknown estimate_source '{prm.value}'")
                self.source = str(prm.value)
                self.v_cmd = None            # reseed guidance from the newly selected source
                self._reset_bearing_state()  # never difference bearings across estimators
                self.get_logger().info(f"estimate_source -> '{self.source}'")
            if prm.name == "guidance_mode":
                if prm.value not in GUIDANCE_MODES:
                    return SetParametersResult(
                        successful=False, reason=f"unknown guidance_mode '{prm.value}'")
                self.guidance_mode = str(prm.value)
                self.v_cmd = None            # reseed pursuit for the new steering law
                self._reset_bearing_state()  # never carry a LOS rate across modes
                self.get_logger().info(f"guidance_mode -> '{self.guidance_mode}'")
            if prm.name == "los_rate_ema_alpha":
                self.los_ema_alpha = float(np.clip(prm.value, 0.0, 1.0))
                self.bearing_diff.ema_alpha = self.los_ema_alpha
                self.raw_diff.ema_alpha = self.los_ema_alpha
                self.get_logger().info(f"los_rate_ema_alpha -> {self.los_ema_alpha}")
            if prm.name == "los_timeout_s":
                self.los_timeout_s = float(prm.value)
                self.get_logger().info(f"los_timeout_s -> {self.los_timeout_s}")
            if prm.name == "los_lost_s":
                self.los_lost_s = float(prm.value)
                self.get_logger().info(f"los_lost_s -> {self.los_lost_s}")
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

    def _go_idle(self):
        """Reset pursuit + LOS-rate state and publish a zero command."""
        self.v_cmd = None
        self._reset_bearing_state()
        self._publish_zero()

    def _control(self):
        self._refresh_active()
        if (not self._engaged()) or self.own_p is None:
            self._go_idle()
            return
        # raw_ibvs steers off the RAW detection LOS (bearing_raw/los), fully
        # decoupled from the EKF target estimate — its own gating (below).
        if self.guidance_mode == "raw_ibvs":
            self._control_raw_ibvs()
            return
        # pn / bearing_pn steer off the selected estimator (oracle/EKF) target.
        if not self._target_ok():
            self._go_idle()
            return
        self._control_estimator()

    def _control_estimator(self):
        """pn (range-sensitive) and bearing_pn (range-tolerant, position-derived
        bearing) — both consume the selected estimator's target_pose/twist."""
        # Seed the commanded velocity as a v_max pursuit (mirrors point-mass init).
        if self.v_cmd is None:
            self.v_cmd = self.v_max * unit(self.tgt_p - self.own_p)

        own_v_for_guidance = self.own_v if self.use_meas_vel else self.v_cmd
        if self.guidance_mode == "bearing_pn":
            # Range-tolerant STEERING: PN acceleration from the LOS *rate*, invariant to
            # instantaneous radial scaling of the relative vector about the observer.
            # NOTE: not immune to general range error — a persistent range bias still
            # perturbs the bearing via parallax as the observer translates (this is the
            # residual raw_ibvs removes). The LOS is differentiated only on a NEW stamped
            # sample (the estimate arrival time), held between samples, so a 20-25 Hz
            # estimate is not aliased by the 50 Hz control tick.
            n_hat = unit(self.tgt_p - self.own_p)
            rx_ns = self.tgt_rx.nanoseconds if self.tgt_rx is not None else None
            omega = (self.bearing_diff.update(n_hat, rx_ns)
                     if rx_ns is not None else self.bearing_diff.omega)
            # True closing speed uses relative velocity when the estimate provides it;
            # own-speed-along-LOS is the named fallback (valid for a ~stationary target).
            v_rel = own_v_for_guidance - (self.tgt_v if self.tgt_v is not None else 0.0)
            closing = float(np.dot(v_rel, n_hat))
            cmd = pn_from_los_rate(
                n_hat, omega, closing, self.N, self.a_max,
                range_est_m=float(np.linalg.norm(self.tgt_p - self.own_p)))
        else:
            cmd = proportional_navigation(
                self.own_p, own_v_for_guidance, self.tgt_p, self.tgt_v, self.N, self.a_max)
        # Integrate PN accel into the commanded velocity, clamp to v_max.
        self.v_cmd = limit_norm(self.v_cmd + cmd.acceleration_mps2 * self.dt, self.v_max)
        self._publish_cmd_and_diag(cmd)

    def _control_raw_ibvs(self):
        """Range-FREE STEERING: PN from the raw detection LOS rate (bearing_raw/los),
        decoupled from the EKF target_pose — no target position or range enters the
        steering path, so the parallax coupling of bearing_pn is removed by
        construction (ticket 012). raw_ibvs REQUIRES bearing_raw/los; it does NOT
        silently fall back to pn."""
        if self.raw_n is None or self.raw_stamp_ns is None:
            self._raw_missing_ticks += 1
            if self._raw_missing_ticks % 100 == 1:   # ~2 s at 50 Hz
                self.get_logger().warn(
                    "raw_ibvs active but no bearing_raw/los yet — holding zero "
                    "command (is mas_bearing_loc raw_los_node running?)")
            self._go_idle()
            return
        self._raw_missing_ticks = 0

        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - self.raw_stamp_ns) * 1e-9
        if age > self.los_lost_s:                     # detection dropout -> target lost
            self._go_idle()
            return

        n_hat = self.raw_n
        # Differentiate on the DETECTION stamp; coast the rate to zero across the
        # dropout window (direction is held at the last bearing).
        omega = self.raw_diff.update(n_hat, self.raw_stamp_ns)
        omega = omega * coast_decay(age, self.los_timeout_s, self.los_lost_s)

        # Range-free pursuit seed along the raw bearing (no target position needed).
        if self.v_cmd is None:
            self.v_cmd = self.v_max * n_hat
        own_v_for_guidance = self.own_v if self.use_meas_vel else self.v_cmd
        # Closing speed = own speed along the LOS (range-free). A relative-closing
        # option would require a range estimate, which raw_ibvs deliberately omits.
        closing = float(np.dot(own_v_for_guidance, n_hat))
        cmd = pn_from_los_rate(n_hat, omega, closing, self.N, self.a_max, range_est_m=0.0)
        self.v_cmd = limit_norm(self.v_cmd + cmd.acceleration_mps2 * self.dt, self.v_max)
        self._publish_cmd_and_diag(cmd)

    def _publish_cmd_and_diag(self, cmd):
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
