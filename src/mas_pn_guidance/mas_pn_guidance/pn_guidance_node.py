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
from .ego_weave import ego_weave_velocity
from .active_sensing import (
    ActiveSensingContext, active_sensing_accel, IMPLEMENTED_CLASSES,
)

MISSION = 2  # mas_mission state enum

GUIDANCE_MODES = ("pn", "bearing_pn", "raw_ibvs")

PREFIX = {"simple_ekf": "simple_loc", "direct_projection": "direct_loc",
          "dc_ekf": "bearing_loc",   # legacy 18-D DC-EKF (in-state feature); ticket 011
          "cooperative": "coop_loc",  # ticket 019: mock-cooperative fusion (mas_coop_mock)
          "ego_fgo": "ego_fgo_loc"}   # RAL ticket 028 S3': ego-only FGO arm (ego_smoother)


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
        # Ego active-sensing weave (ticket 019 B1): a lateral sinusoid perpendicular
        # to the LOS the ego-active baseline uses to excite range observability
        # (bearings-only/TMA: a lone pursuer must maneuver to see range). DEFAULT-OFF
        # (amp 0) -> zero contribution, so oracle / passive-ego / cooperative arms are
        # byte-identical. Only the ego_active arm sets amp>0 (via live_params).
        self.declare_parameter("ego_weave_amp_mps", 0.0)
        self.declare_parameter("ego_weave_freq_hz", 0.0)
        self.declare_parameter("ego_weave_taper_range_m", 0.0)
        # Active-sensing ACCELERATION classes (ticket 023): a_obs is added to the PN
        # accel BEFORE the integrate-and-clamp and the SUM is clamped to the SAME a_max
        # (one shared envelope, Q1), so weaving trades collision-course efficiency for
        # range observability. Orthogonal to guidance_mode (the pn harness is held
        # fixed; only the motion differs). DEFAULT 'none' -> the node takes its
        # byte-identical existing path. Distinct from the ego_weave_* VELOCITY weave
        # (ticket 019). Class 1 (oepn) is built in S0; opt_weave/fim_mpc land in S2.
        self.declare_parameter("active_sensing_class", "none")
        self.declare_parameter("as_amp_mps2", 0.0)       # Class 1/2 peak ⊥LOS accel
        self.declare_parameter("as_freq_hz", 0.0)        # Class 1 weave frequency
        self.declare_parameter("as_taper_range_m", 15.0)  # range-gate for Class 1/2
        self.declare_parameter("as_fim_lambda", 0.0)     # Class 3 Pareto weight (swept knob)
        self.declare_parameter("as_fim_horizon_s", 2.0)  # Class 3 horizon
        self.declare_parameter("as_fim_samples", 64)     # Class 3 CEM sample count
        # Class 2/3 replay schedule (⊥LOS accel m/s^2, piecewise-constant every
        # as_schedule_dt_s): the offline-optimized u(t) from precompute_schedules.py.
        # Default [0.0] -> inert. Set per-boot via run_sweep live_params.
        self.declare_parameter("as_schedule_u_mps2", [0.0])
        self.declare_parameter("as_schedule_dt_s", 0.1)
        # Ticket 026 literature-grade laws (all default-inert; F1/F2 land in 026 S1,
        # F3 fim_mpc_online in 026 S2). F1 (aopn): N1 rides the existing nav_constant
        # param; only N2/sign are new. F2 (dev_pursuit): scalar lead angle + washout.
        self.declare_parameter("as_aopn_n2", 0.0)          # F1 second nav constant N2
        self.declare_parameter("as_aopn_sign", 1.0)        # F1 swing sense (offline-chosen)
        self.declare_parameter("as_dev_delta_deg", 0.0)    # F2 lead angle δ* (deg)
        self.declare_parameter("as_dev_wash_range_m", 15.0)  # F2 washout range R_wash
        self.declare_parameter("as_dev_gain", 1.0)         # F2 steering gain k_δ
        # Law A (026 egofix drafts): recoverability-governed excitation (RGE)
        self.declare_parameter("as_rge_beta", 0.5)         # margin fraction β
        self.declare_parameter("as_rge_gamma_exc", 0.4)    # envelope split γ_exc
        self.declare_parameter("as_rge_msoft", 2.0)        # soft-gate width (m)
        self.declare_parameter("as_rge_sign", 1.0)         # ⊥LOS swing sign
        self.declare_parameter("as_fim_replan_ticks", 5)   # F3 online replan cadence
        self.declare_parameter("as_fim_hit_r_m", 1.0)      # F3 hard CPA feasibility radius
        self.declare_parameter("as_fim_bs_kappa", 1.0)     # Law B risk weight κ
        self.declare_parameter("as_fim_bs_cgeo", 0.5)      # Law B miss-projection factor

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
        self.ego_weave_amp = float(self.get_parameter("ego_weave_amp_mps").value)
        self.ego_weave_freq = float(self.get_parameter("ego_weave_freq_hz").value)
        self.ego_weave_taper = float(self.get_parameter("ego_weave_taper_range_m").value)
        self._weave_t0_ns = 0   # phase base, reset when a pursuit is (re)seeded
        # Active-sensing class + knobs (ticket 023). Unimplemented classes fall back
        # to 'none' so a boot can never silently run a no-op weave.
        self.active_sensing_class = str(self.get_parameter("active_sensing_class").value)
        if self.active_sensing_class not in IMPLEMENTED_CLASSES:
            self.get_logger().warn(
                f"active_sensing_class '{self.active_sensing_class}' not implemented "
                f"(supported: {IMPLEMENTED_CLASSES}); using 'none'")
            self.active_sensing_class = "none"
        self.as_amp = float(self.get_parameter("as_amp_mps2").value)
        self.as_freq = float(self.get_parameter("as_freq_hz").value)
        self.as_taper = float(self.get_parameter("as_taper_range_m").value)
        self.as_fim_lambda = float(self.get_parameter("as_fim_lambda").value)
        self.as_fim_horizon_s = float(self.get_parameter("as_fim_horizon_s").value)
        self.as_fim_samples = int(self.get_parameter("as_fim_samples").value)
        self.as_schedule_u = np.asarray(
            self.get_parameter("as_schedule_u_mps2").value, dtype=float)
        self.as_schedule_dt = float(self.get_parameter("as_schedule_dt_s").value)
        # Ticket 026 F1/F2 knobs (default-inert)
        self.as_aopn_n2 = float(self.get_parameter("as_aopn_n2").value)
        self.as_aopn_sign = float(self.get_parameter("as_aopn_sign").value)
        self.as_dev_delta_deg = float(self.get_parameter("as_dev_delta_deg").value)
        self.as_dev_wash_range_m = float(self.get_parameter("as_dev_wash_range_m").value)
        self.as_dev_gain = float(self.get_parameter("as_dev_gain").value)
        self.as_rge_beta = float(self.get_parameter("as_rge_beta").value)
        self.as_rge_gamma_exc = float(self.get_parameter("as_rge_gamma_exc").value)
        self.as_rge_msoft = float(self.get_parameter("as_rge_msoft").value)
        self.as_rge_sign = float(self.get_parameter("as_rge_sign").value)
        self.as_fim_replan_ticks = int(self.get_parameter("as_fim_replan_ticks").value)
        self.as_fim_hit_r_m = float(self.get_parameter("as_fim_hit_r_m").value)
        self._fim_planner = None   # F3 online CEM, built lazily on first use (ticket 026)
        self._warn_empty_schedule()

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
        # Measured-acceleration estimate (ticket 023 Q7): EMA-smoothed finite
        # difference of the odom velocity, logged in pn/diagnostics as a cross-check
        # on the commanded accel. Not a control driver.
        self._last_ownv: Optional[np.ndarray] = None
        self._last_ownv_ns: Optional[int] = None
        self._a_meas = np.zeros(3)

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
        # Ticket 026 rev1 §4: per-tick F3 online-FIM diagnostics (separate topic, additive —
        # the pn/diagnostics 18-field contract is untouched). Fields (fixed order):
        #   0 fkappa_pred | 1 cpa_pred | 2 feasible_frac | 3 plan_feasible | 4 plan_age_s
        #   5 solve_s | 6 deadline_miss | 7 fallback | 8 replans | 9 ctrl_period_miss
        self.pub_fim_diag = self.create_publisher(
            Float64MultiArray, "pn/fim_diagnostics", 10)
        self._ctrl_last_ns = None          # control-callback period monitor (rev1 §3)
        self._ctrl_period_miss = 0

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
            if prm.name == "ego_weave_amp_mps":
                self.ego_weave_amp = float(prm.value)
                self.get_logger().info(f"ego_weave_amp_mps -> {self.ego_weave_amp}")
            if prm.name == "ego_weave_freq_hz":
                self.ego_weave_freq = float(prm.value)
                self.get_logger().info(f"ego_weave_freq_hz -> {self.ego_weave_freq}")
            if prm.name == "ego_weave_taper_range_m":
                self.ego_weave_taper = float(prm.value)
                self.get_logger().info(f"ego_weave_taper_range_m -> {self.ego_weave_taper}")
            if prm.name == "active_sensing_class":
                if prm.value not in IMPLEMENTED_CLASSES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"active_sensing_class '{prm.value}' not implemented "
                               f"(supported: {IMPLEMENTED_CLASSES})")
                self.active_sensing_class = str(prm.value)
                # fim planner mode is class-dependent (Law B) — rebuild on change
                if self._fim_planner is not None:
                    self._fim_planner.shutdown()
                    self._fim_planner = None
                self.get_logger().info(f"active_sensing_class -> '{self.active_sensing_class}'")
                self._warn_empty_schedule()
            if prm.name == "as_amp_mps2":
                self.as_amp = float(prm.value)
                self.get_logger().info(f"as_amp_mps2 -> {self.as_amp}")
            if prm.name == "as_freq_hz":
                self.as_freq = float(prm.value)
                self.get_logger().info(f"as_freq_hz -> {self.as_freq}")
            if prm.name == "as_taper_range_m":
                self.as_taper = float(prm.value)
                self.get_logger().info(f"as_taper_range_m -> {self.as_taper}")
            if prm.name == "as_fim_lambda":
                self.as_fim_lambda = float(prm.value)
                self.get_logger().info(f"as_fim_lambda -> {self.as_fim_lambda}")
            if prm.name == "as_fim_horizon_s":
                self.as_fim_horizon_s = float(prm.value)
                self._fim_planner = None
                self.get_logger().info(f"as_fim_horizon_s -> {self.as_fim_horizon_s}")
            if prm.name == "as_fim_samples":
                self.as_fim_samples = int(prm.value)
                self._fim_planner = None
                self.get_logger().info(f"as_fim_samples -> {self.as_fim_samples}")
            if prm.name == "as_schedule_u_mps2":
                self.as_schedule_u = np.asarray(prm.value, dtype=float)
                self.get_logger().info(
                    f"as_schedule_u_mps2 -> {self.as_schedule_u.size} knots")
                self._warn_empty_schedule()
            if prm.name == "as_schedule_dt_s":
                self.as_schedule_dt = float(prm.value)
                self.get_logger().info(f"as_schedule_dt_s -> {self.as_schedule_dt}")
            if prm.name == "as_aopn_n2":
                self.as_aopn_n2 = float(prm.value)
                self.get_logger().info(f"as_aopn_n2 -> {self.as_aopn_n2}")
            if prm.name == "as_aopn_sign":
                self.as_aopn_sign = float(prm.value)
                self.get_logger().info(f"as_aopn_sign -> {self.as_aopn_sign}")
            if prm.name == "as_dev_delta_deg":
                self.as_dev_delta_deg = float(prm.value)
                self.get_logger().info(f"as_dev_delta_deg -> {self.as_dev_delta_deg}")
            if prm.name == "as_dev_wash_range_m":
                self.as_dev_wash_range_m = float(prm.value)
                self.get_logger().info(f"as_dev_wash_range_m -> {self.as_dev_wash_range_m}")
            if prm.name == "as_dev_gain":
                self.as_dev_gain = float(prm.value)
                self.get_logger().info(f"as_dev_gain -> {self.as_dev_gain}")
            if prm.name in ("as_rge_beta", "as_rge_gamma_exc", "as_rge_msoft", "as_rge_sign"):
                setattr(self, prm.name, float(prm.value))
                self.get_logger().info(f"{prm.name} -> {float(prm.value)}")
            if prm.name == "as_fim_replan_ticks":
                self.as_fim_replan_ticks = int(prm.value)
                self._fim_planner = None    # rebuild with the new cadence on next use
                self.get_logger().info(f"as_fim_replan_ticks -> {self.as_fim_replan_ticks}")
            if prm.name == "as_fim_hit_r_m":
                self.as_fim_hit_r_m = float(prm.value)
                self._fim_planner = None    # rebuild with the new feasibility radius
                self.get_logger().info(f"as_fim_hit_r_m -> {self.as_fim_hit_r_m}")
        return SetParametersResult(successful=True)

    def _warn_empty_schedule(self):
        """Warn if a schedule-replay class (opt_weave/fim_mpc) has an empty/zero
        schedule — the arm would run inert (a silent no-op weave)."""
        if (self.active_sensing_class in ("opt_weave", "fim_mpc")
                and (self.as_schedule_u.size == 0 or not np.any(self.as_schedule_u)
                     or self.as_schedule_dt <= 0.0)):
            self.get_logger().warn(
                f"active_sensing_class '{self.active_sensing_class}' has an empty/zero "
                f"schedule (as_schedule_u_mps2) — the weave will be INERT this boot")

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
        # Control-callback period monitor (rev1.md §3): count ticks whose realized period
        # exceeds 1.5×dt — the symptom the pre-rev1 synchronous F3 planner produced.
        now_ns = self.get_clock().now().nanoseconds
        if self._ctrl_last_ns is not None:
            if (now_ns - self._ctrl_last_ns) * 1e-9 > 1.5 * self.dt:
                self._ctrl_period_miss += 1
        self._ctrl_last_ns = now_ns
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
            self._weave_t0_ns = self.get_clock().now().nanoseconds  # weave phase base

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
        # Active-sensing acceleration (ticket 023): a_obs enters at the ACCELERATION
        # level and shares the SAME a_max as pursuit (Q1) — a_cmd = clamp(a_pn + a_obs).
        # class 'none' (default) reproduces the existing command EXACTLY (byte-identical).
        a_pn = cmd.raw_acceleration_mps2                      # pre-clamp PN accel
        if self.active_sensing_class == "none":
            a_obs = np.zeros(3)
            a_cmd = cmd.acceleration_mps2                     # == limit_norm(a_pn, a_max)
        else:
            n_hat = unit(self.tgt_p - self.own_p)
            ctx = ActiveSensingContext(
                n_hat=n_hat, t_s=self._active_sensing_t(),
                range_m=float(np.linalg.norm(self.tgt_p - self.own_p)),
                range_rate_mps=-float(cmd.closing_speed_mps),   # ego Ṙ (F1/F3, ticket 026)
                a_pn=a_pn, a_max=self.a_max, v_cmd=self.v_cmd,
                own_p=self.own_p, tgt_p=self.tgt_p,
                tgt_v=self.tgt_v if self.tgt_v is not None else np.zeros(3), dt=self.dt)
            a_obs = active_sensing_accel(self.active_sensing_class, self._as_params(), ctx)
            if (self.active_sensing_class in ("fim_mpc_online", "fim_mpc_bs")
                    and self._fim_planner is not None):
                self._publish_fim_diag(self._fim_planner.last_diag())   # rev1 §4
            a_cmd = limit_norm(a_pn + a_obs, self.a_max)      # shared envelope: sum then clamp
        # Integrate the total accel into the commanded velocity, clamp to v_max.
        self.v_cmd = limit_norm(self.v_cmd + a_cmd * self.dt, self.v_max)
        # The ego_weave_* VELOCITY weave (ticket 019 B1) is a SEPARATE output-only path,
        # default-off; it must NOT be written back into self.v_cmd (a same-sign half-period
        # would accumulate into a v_max cross-LOS drift). 023 arms leave it off (amp 0).
        self._publish_cmd_and_diag(cmd, self._apply_ego_weave(self.v_cmd),
                                   a_pn=a_pn, a_obs=a_obs, a_cmd=a_cmd)

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

    def _apply_ego_weave(self, v_cmd):
        """Return v_cmd + a transient lateral weave offset (ego active-sensing, ticket
        019 B1) for the OUTPUT command. The weave excites range observability for a lone
        bearings-only pursuer (ownship maneuver / TMA). Applied to the output only — never
        stored into the pursuit state (that would integrate to a v_max side-drift).
        DEFAULT-OFF (amp<=0) -> returns v_cmd unchanged."""
        if self.ego_weave_amp <= 0.0 or self.ego_weave_freq <= 0.0:
            return v_cmd
        n_hat = self.tgt_p - self.own_p
        t_s = (self.get_clock().now().nanoseconds - self._weave_t0_ns) * 1e-9
        v_weave = ego_weave_velocity(
            n_hat, t_s, self.ego_weave_amp, self.ego_weave_freq,
            self.ego_weave_taper, float(np.linalg.norm(n_hat)))
        return limit_norm(v_cmd + v_weave, self.v_max)

    def _active_sensing_t(self) -> float:
        """Seconds since the pursuit was (re)seeded — the active-sensing phase base
        (shared with the ego_weave phase clock, self._weave_t0_ns)."""
        return (self.get_clock().now().nanoseconds - self._weave_t0_ns) * 1e-9

    def _as_params(self) -> dict:
        """Active-sensing knobs passed to the class dispatcher (ticket 023)."""
        return {"amp_mps2": self.as_amp, "freq_hz": self.as_freq,
                "taper_range_m": self.as_taper, "fim_lambda": self.as_fim_lambda,
                "fim_horizon_s": self.as_fim_horizon_s, "fim_samples": self.as_fim_samples,
                "schedule_u": self.as_schedule_u, "schedule_dt": self.as_schedule_dt,
                # ticket 026 F1 (aopn) / F2 (dev_pursuit)
                "aopn_n2": self.as_aopn_n2, "aopn_sign": self.as_aopn_sign,
                "dev_delta_deg": self.as_dev_delta_deg,
                "dev_wash_range_m": self.as_dev_wash_range_m, "dev_gain": self.as_dev_gain,
                # Law A rge (026 egofix drafts)
                "rge_beta": self.as_rge_beta, "rge_gamma_exc": self.as_rge_gamma_exc,
                "rge_msoft": self.as_rge_msoft, "rge_sign": self.as_rge_sign,
                # ticket 026 F3 (fim_mpc_online): the stateful planner (built lazily)
                "fim_planner": (self._get_fim_planner()
                                if self.active_sensing_class in ("fim_mpc_online",
                                                                 "fim_mpc_bs") else None),
                # Law B: believed range std from the estimator cov trace (isotropic approx)
                "sigma_R0": float(np.sqrt(max(self.tgt_cov_trace, 0.0) / 3.0))}

    def _get_fim_planner(self):
        """Lazily build the F3 online receding-horizon CEM planner (ticket 026). Rebuilt
        when a governing `as_fim_*` param changes (the runtime setter nulls it)."""
        if self._fim_planner is None:
            from .fim_mpc_online import OnlineFimMpc
            self._fim_planner = OnlineFimMpc(
                a_max=self.a_max, v_max=self.v_max, n_nav=self.N,
                horizon_s=self.as_fim_horizon_s, samples=self.as_fim_samples,
                replan_ticks=self.as_fim_replan_ticks, hit_r=self.as_fim_hit_r_m,
                background=True,      # rev1 §3: CEM off the 50 Hz callback (worker thread)
                belief_space=(self.active_sensing_class == "fim_mpc_bs"),   # Law B
                kappa=float(self.get_parameter("as_fim_bs_kappa").value),
                c_geo=float(self.get_parameter("as_fim_bs_cgeo").value))
        return self._fim_planner

    def _update_a_meas(self) -> np.ndarray:
        """EMA-smoothed finite difference of the measured odom velocity -> measured
        acceleration (ticket 023 Q7). A cross-check on the commanded accel, held
        between updates; robust to repeated/near-simultaneous ticks. Never drives
        the command."""
        if self.own_v is None:
            return self._a_meas
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ownv is None or self._last_ownv_ns is None:
            self._last_ownv = self.own_v.copy()
            self._last_ownv_ns = now_ns
            return self._a_meas
        dt = (now_ns - self._last_ownv_ns) * 1e-9
        if dt > 1e-3:
            raw = (self.own_v - self._last_ownv) / dt
            a = self.los_ema_alpha
            self._a_meas = a * self._a_meas + (1.0 - a) * raw
            self._last_ownv = self.own_v.copy()
            self._last_ownv_ns = now_ns
        return self._a_meas

    def _publish_cmd_and_diag(self, cmd, v_out=None, a_pn=None, a_obs=None, a_cmd=None):
        v = self.v_cmd if v_out is None else v_out
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "common_frame"
        out.twist.linear.x = float(v[0])
        out.twist.linear.y = float(v[1])
        out.twist.linear.z = float(v[2])
        self.pub_cmd.publish(out)

        a_pn = np.zeros(3) if a_pn is None else np.asarray(a_pn, dtype=float)
        a_obs = np.zeros(3) if a_obs is None else np.asarray(a_obs, dtype=float)
        a_cmd = np.zeros(3) if a_cmd is None else np.asarray(a_cmd, dtype=float)
        a_meas = self._update_a_meas()
        diag = Float64MultiArray()
        # Indices 0-5 are UNCHANGED (existing consumers). 6-17 appended (ticket 023 Q7):
        # a_PN (pre-clamp) | a_obs (active sensing) | a_cmd (integrated) | a_meas (odom-diff).
        diag.data = [
            cmd.closing_speed_mps, cmd.los_rate_radps, cmd.range_est_m,        # 0-2
            1.0 if cmd.saturated else 0.0, float(np.linalg.norm(self.v_cmd)),  # 3-4
            self.tgt_cov_trace,                                               # 5
            float(a_pn[0]), float(a_pn[1]), float(a_pn[2]),                   # 6-8
            float(a_obs[0]), float(a_obs[1]), float(a_obs[2]),               # 9-11
            float(a_cmd[0]), float(a_cmd[1]), float(a_cmd[2]),              # 12-14
            float(a_meas[0]), float(a_meas[1]), float(a_meas[2]),          # 15-17
            # 18: node sim-time stamp (026 rev2 §3.2 — Float64MultiArray has no header,
            # so analyzers previously had to align by rosbag record time). Additive:
            # indices 0-17 unchanged; consumers index by position.
            self.get_clock().now().nanoseconds * 1e-9,
        ]
        self.pub_diag.publish(diag)

    def _publish_fim_diag(self, d):
        """Publish the F3 online-FIM planner's per-tick diagnostics (ticket 026 rev1 §4)
        on pn/fim_diagnostics — predicted F_κ/CPA, feasibility, plan age/solve time,
        deadline + control-period misses, fallback. Additive; pn/diagnostics unchanged."""
        msg = Float64MultiArray()
        msg.data = [
            float(d.get("fkappa", 0.0)),
            float(d.get("cpa_pred", float("inf"))),
            float(d.get("feasible_frac", 0.0)),
            1.0 if d.get("plan_feasible") else 0.0,
            float(d.get("plan_age_s", float("inf"))),
            float(d.get("solve_s", 0.0)),
            float(d.get("deadline_miss", 0)),
            1.0 if d.get("fallback") else 0.0,
            float(d.get("replans", 0)),
            float(self._ctrl_period_miss),
        ]
        self.pub_fim_diag.publish(msg)

    def _publish_zero(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "common_frame"
        self.pub_cmd.publish(out)

    def destroy_node(self):
        """Stop the F3 CEM worker thread cleanly (ticket 026 rev1) before shutdown."""
        if getattr(self, "_fim_planner", None) is not None:
            self._fim_planner.shutdown()
        super().destroy_node()


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
