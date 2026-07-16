"""Experiment conductor for the PN-interception engagement (Ticket 004).

Runs the condition matrix (estimator × regime × geometry) as a sequence of
reproducible trials within one sim *boot* (PX4 SITL has boot-variable EKF
attitude bias, so a boot is the experimental block — see ticket 004). Per trial
it:

  1. selects the estimator    (runtime param on the PN node)
  2. selects the target regime (runtime params on the target_maneuver node)
  3. drives both drones to the geometry's initial condition  (goto_position)
  4. waits for them to settle
  5. starts a rosbag           (bag/rosbag_record.sh <trial>)
  6. engages                   (/mission_state_cmd = MISSION)
  7. monitors range → hit (< intercept_radius) / CPA-passed / timeout
  8. disengages + stops the bag
  9. logs the trial row (boot, order, condition, result, min_range, t_cpa, …)

Order is randomized within the boot; the per-boot attitude error is reported
once. There is no sim vehicle reset, so repositioning *is* the reset: each
trial's goto separates the drones again after the previous intercept.

Run (interceptor + target stacks already up, both engaged-idle):
    ros2 run mas_pn_guidance experiment_conductor --ros-args \
        -p boot_id:=A -p use_sim_time:=true
"""
from __future__ import annotations

import csv
import json
import math
import os
import random
import signal
import subprocess
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters, GetParameters
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

MISSION, IDLE = 2, 0

# Geometry initial conditions, in common_frame, interceptor = target0 + 50·d̂
# with the point-mass offset directions from i_design.md (ticket 003 faithful).
# Target at (0,0,30); interceptor yaw faces the target. pose = (x, y, z, yaw_deg).
#   tail_chase d̂ = unit(-0.989,-0.124,-0.062) (= point-mass offset (-80,-10,-5)):
#       behind f̂, ~6 m lateral, ~3 m below  -> interceptor (-49.52,-6.19,26.91)
#   crossing   d̂ = unit(0,-1,-0.1): beam (⟂ f̂), ~5 m below -> (0,-49.75,25.02)
GEOMETRIES = {
    "crossing":   {"interceptor": (0.0, -49.75, 25.02, 90.0),
                   "target":      (0.0, 0.0, 30.0, 0.0)},
    "tail_chase": {"interceptor": (-49.52, -6.19, 26.91, 7.1),
                   "target":      (0.0, 0.0, 30.0, 0.0)},
}

# Target maneuver presets — mirror scenario_grid AGILITY_LEVELS / the launch.
#   (speed, amp, freq, heading_deg, vertical_ratio)
REGIMES = {
    "static_wind":  (0.0, 1.0, 0.20, 0.0, 0.20),
    "dynamic_mild": (6.0, 1.0, 0.20, 0.0, 0.20),
    "dynamic_paper": (7.0, 1.5, 0.25, 0.0, 0.25),
    "dynamic_hard": (8.0, 2.0, 0.30, 0.0, 0.30),
}

ESTIMATORS = ["oracle", "simple_ekf", "direct_projection", "cooperative"]  # ticket 019 mock-coop


def build_target_conditions(mode, fwd_speeds, lat_accels, freq):
    """Map condition-id -> (speed, amp, freq, heading_deg, vert_ratio, vfwd_cmd,
    alat_cmd). 'named' = the REGIMES presets (a_lat back-derived). 'capability_grid'
    = forward-speed × lateral-accel at FIXED freq, amplitude A = a_lat/omega^2,
    pure-horizontal weave (vert=0 so a_lat is horizontal: tilt=atan(a_lat/g)).
    See i_target_capability_envelope.md."""
    conds = {}
    if mode == "capability_grid":
        w = 2.0 * math.pi * freq
        for vf in fwd_speeds:
            for al in lat_accels:
                A = al / (w * w)
                cid = f"vf{vf:g}_alat{al:g}_f{freq:g}".replace(".", "p")
                conds[cid] = (float(vf), float(A), float(freq), 0.0, 0.0, float(vf), float(al))
    else:
        for name, (s, a, f, hd, vr) in REGIMES.items():
            w = 2.0 * math.pi * f
            conds[name] = (s, a, f, hd, vr, s, a * w * w)
    return conds

# Safe, separated parking pose per drone for the end-of-boot return.
HOME = {"interceptor": (0.0, -50.0, 30.0, 90.0), "target": (0.0, 0.0, 30.0, 0.0)}


def _yaw_quat(yaw_deg: float):
    h = math.radians(yaw_deg) * 0.5
    return math.sin(h), math.cos(h)   # (qz, qw); qx=qy=0


def _qos_be():
    return QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _qos_cmd():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class ExperimentConductor(Node):
    def __init__(self):
        super().__init__("experiment_conductor")
        gp = self.declare_parameter
        self.interceptor_ns = str(gp("interceptor_ns", "px4_1").value).strip("/")
        self.target_ns = str(gp("target_ns", "px4_2").value).strip("/")
        # Cooperative observer ns (empty for non-coop runs). Only used to tell the
        # recorder to also log the observer truth odom + fused belief (ticket 019 M4).
        self.observer_ns = str(gp("observer_ns", "").value).strip("/")
        self.pn_node = str(gp("pn_node_name", "pn_guidance_node").value)
        self.maneuver_node = str(gp("maneuver_node_name", "target_maneuver_node").value)
        # PRIMARY success radius = 0.5 m to match ticket 003 (was 1.0). The
        # engagement runs to TRUE CPA regardless (never terminates on a crossing),
        # so success is a post-hoc threshold on the uncensored min-range; we also
        # record whether each of hit_radii was crossed (2/1/0.5 m).
        self.intercept_radius_m = float(gp("intercept_radius_m", 0.5).value)
        self.hit_radii = [float(x) for x in gp("hit_radii", [2.0, 1.0, 0.5]).value]
        # engage_timeout is in SIM seconds (physically meaningful, reproducible
        # regardless of the sim's realtime factor); wall_timeout is a hard
        # wall-clock safety so a hung sim can't block a trial forever.
        self.engage_timeout_s = float(gp("engage_timeout_s", 30.0).value)
        self.wall_timeout_s = float(gp("wall_timeout_s", 400.0).value)
        self.settle_tol_m = float(gp("settle_tol_m", 2.0).value)
        self.settle_speed_tol = float(gp("settle_speed_tol", 0.4).value)
        # SIM seconds — the return can be far downrange (the interceptor ends at
        # the intercept point, ~300 m out after a fleeing-target engagement).
        self.settle_timeout_s = float(gp("settle_timeout_s", 60.0).value)
        self.settle_wall_timeout_s = float(gp("settle_wall_timeout_s", 400.0).value)
        self.settle_dwell_s = float(gp("settle_dwell_s", 2.0).value)
        self.cpa_margin_m = float(gp("cpa_margin_m", 3.0).value)
        self.boot_id = str(gp("boot_id", "A").value)
        self.repeats = int(gp("repeats", 1).value)
        # Comma-separated (empty = use all) so the matrix dims pass cleanly as
        # launch args.
        self.estimators = self._csv(gp("estimators", "").value, ESTIMATORS)
        # Target conditions: 'named' presets, or 'capability_grid' (forward-speed ×
        # lateral-accel @ fixed freq) for the target-capability envelope study.
        self.target_mode = str(gp("target_condition_mode", "named").value)
        fwd = [float(x) for x in self._csv(str(gp("target_forward_speeds", "4.5,6.0,7.0,8.0").value), [])]
        lat = [float(x) for x in self._csv(str(gp("target_lateral_accels", "1.5,3.0,4.5,7.1").value), [])]
        self.target_freq = float(gp("target_frequency_hz", 0.25).value)
        self.conditions = build_target_conditions(self.target_mode, fwd, lat, self.target_freq)
        self.regimes = self._csv(gp("regimes", "").value, list(self.conditions))
        self.geometries = self._csv(gp("geometries", "").value, list(GEOMETRIES))
        # Deterministic shuffle: seed param if given, else a fixed per-boot seed
        # derived from boot_id (reproducible order, distinct across boots).
        self.seed = int(gp("seed", 0).value)
        self.seed_eff = self.seed or (sum(ord(c) for c in self.boot_id) + 1000)
        self.bag_script = str(gp("bag_script", "/home/usrg/mas/bag/rosbag_record.sh").value)
        self.results_dir = str(gp("results_dir", "/home/usrg/mas/bag").value)
        self.record = bool(gp("record", True).value)
        self.dry_run = bool(gp("dry_run", False).value)

        # Publishers / subscriptions.
        self.mission_pub = self.create_publisher(Int8, "/mission_state_cmd", _qos_cmd())
        self.goto = {
            "interceptor": self.create_publisher(
                PoseStamped, f"/{self.interceptor_ns}/goto_position", _qos_cmd()),
            "target": self.create_publisher(
                PoseStamped, f"/{self.target_ns}/goto_position", _qos_cmd()),
        }
        self._odom = {"interceptor": None, "target": None}
        self._vel = {"interceptor": None, "target": None}
        self.create_subscription(
            Odometry, f"/{self.interceptor_ns}/common_frame/odom",
            lambda m: self._on_odom("interceptor", m), _qos_be())
        self.create_subscription(
            Odometry, f"/{self.target_ns}/common_frame/odom",
            lambda m: self._on_odom("target", m), _qos_be())

        # Parameter-set service clients (runtime estimator / regime switching).
        self._pn_cli = self.create_client(
            SetParameters, f"/{self.interceptor_ns}/{self.pn_node}/set_parameters")
        self._pn_get_cli = self.create_client(
            GetParameters, f"/{self.interceptor_ns}/{self.pn_node}/get_parameters")
        self.pn_vmax, self.pn_amax = 9.0, 6.0  # filled from the PN node at boot start
        self._mvr_cli = self.create_client(
            SetParameters, f"/{self.target_ns}/{self.maneuver_node}/set_parameters")
        # EKF reset services (re-initialize the filter fresh from the IC bearing
        # each trial; a diverged estimate from a prior trial must not carry over).
        self._ekf_reset = {
            "simple_ekf": self.create_client(
                Trigger, f"/{self.interceptor_ns}/simple_ekf_node/reset"),
            "direct_projection": self.create_client(
                Trigger, f"/{self.interceptor_ns}/direct_projection_ekf_node/reset"),
        }
        self.ekf_settle_s = float(gp("ekf_settle_s", 3.0).value)

        self._bag_proc: Optional[subprocess.Popen] = None
        self.get_logger().info(
            f"conductor boot={self.boot_id}: {self.estimators} × {self.regimes} × "
            f"{self.geometries} ×{self.repeats}  (intercept<{self.intercept_radius_m}m)")

    # ── ROS plumbing ────────────────────────────────────────────────────
    @staticmethod
    def _csv(s, default):
        items = [x.strip() for x in str(s).split(",") if x.strip()]
        return items or default

    def _on_odom(self, role, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._odom[role] = np.array([p.x, p.y, p.z])
        self._vel[role] = np.array([v.x, v.y, v.z])

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep_until(self, predicate, timeout_s):
        """Poll predicate() until true or timeout. Returns success.

        Uses wall-clock (monotonic), not the sim clock: orchestration timeouts
        must not race the /clock startup (sim time reads 0 until the first
        /clock, then jumps to a huge value and would instantly trip a deadline).
        """
        deadline = time.monotonic() + timeout_s
        while rclpy.ok():
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return False

    def _range(self) -> Optional[float]:
        a, b = self._odom["interceptor"], self._odom["target"]
        if a is None or b is None:
            return None
        return float(np.linalg.norm(a - b))

    def _publish_goto(self, role, pose):
        x, y, z, yaw = pose
        qz, qw = _yaw_quat(yaw)
        m = PoseStamped()
        m.header.frame_id = "common_frame"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        self.goto[role].publish(m)

    def _set_params(self, client, params, label):
        if self.dry_run:
            return True
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label}: set_parameters service unavailable")
            return False
        req = SetParameters.Request(parameters=[p.to_parameter_msg() for p in params])
        fut = client.call_async(req)
        if not self._sleep_until(lambda: fut.done(), 5.0):
            self.get_logger().error(f"{label}: set_parameters timed out")
            return False
        res = fut.result()
        ok = bool(res) and all(r.successful for r in res.results)
        if not ok:
            self.get_logger().error(f"{label}: set_parameters rejected: {res}")
        return ok

    def _reset_ekf(self, estimator):
        cli = self._ekf_reset.get(estimator)
        if cli is None or self.dry_run:
            return True
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(f"{estimator}: reset service unavailable")
            return False
        fut = cli.call_async(Trigger.Request())
        self._sleep_until(lambda: fut.done(), 5.0)
        return True

    def _read_pn_limits(self):
        if self.dry_run or not self._pn_get_cli.wait_for_service(timeout_sec=5.0):
            return
        fut = self._pn_get_cli.call_async(GetParameters.Request(names=["v_max", "a_max"]))
        if self._sleep_until(lambda: fut.done(), 5.0) and fut.result():
            v = fut.result().values
            self.pn_vmax, self.pn_amax = float(v[0].double_value), float(v[1].double_value)
            self.get_logger().info(f"PN limits: v_max={self.pn_vmax} a_max={self.pn_amax}")

    def _select_estimator(self, estimator):
        return self._set_params(
            self._pn_cli, [Parameter("estimate_source", Parameter.Type.STRING, estimator)],
            f"PN estimator={estimator}")

    def _select_regime(self, regime):
        s, a, f, hd, vr, vfwd, alat = self.conditions[regime]
        ps = [
            Parameter("regime", Parameter.Type.STRING, regime),
            Parameter("target_speed_mps", Parameter.Type.DOUBLE, float(s)),
            Parameter("sinusoid_amplitude_m", Parameter.Type.DOUBLE, float(a)),
            Parameter("sinusoid_frequency_hz", Parameter.Type.DOUBLE, float(f)),
            Parameter("forward_heading_deg", Parameter.Type.DOUBLE, float(hd)),
            Parameter("vertical_ratio", Parameter.Type.DOUBLE, float(vr)),
        ]
        return self._set_params(self._mvr_cli, ps, f"target={regime}")

    def _engage(self, on):
        self.mission_pub.publish(Int8(data=MISSION if on else IDLE))

    # ── rosbag ──────────────────────────────────────────────────────────
    def _start_bag(self, suffix):
        if not self.record or self.dry_run:
            return
        # Tell the (env-parameterized) recorder which namespaces this run uses, so
        # the target truth odom + observer + cooperative belief are logged under the
        # right names (ticket 019 M4). Defaults keep legacy px4_1/px4_2 behavior.
        env = dict(os.environ, INT_NS=self.interceptor_ns, TGT_NS=self.target_ns)
        if self.observer_ns:
            env["OBS_NS"] = self.observer_ns
        self._bag_proc = subprocess.Popen(
            ["bash", self.bag_script, suffix], start_new_session=True, env=env)
        time.sleep(1.0)  # let recorder discover topics before engage

    def _stop_bag(self):
        if self._bag_proc is None:
            return
        try:
            os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
            self._bag_proc.wait(timeout=10.0)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"bag stop: {exc}")
        self._bag_proc = None

    # ── trial sequence ──────────────────────────────────────────────────
    def _reposition(self, geometry):
        ic = GEOMETRIES[geometry]
        self._publish_goto("interceptor", ic["interceptor"])
        self._publish_goto("target", ic["target"])

        def settled():
            for role in ("interceptor", "target"):
                tgt = np.array(ic[role][:3])
                cur, vel = self._odom[role], self._vel[role]
                if cur is None or vel is None:
                    return False
                if np.linalg.norm(cur - tgt) > self.settle_tol_m:
                    return False
                if np.linalg.norm(vel) > self.settle_speed_tol:  # must be at rest
                    return False
            return True

        # Wait (up to settle_timeout) for settled() to hold *continuously* for
        # settle_dwell — robust to the drone coasting through the IC at speed
        # after the previous intercept.
        sim_deadline = self._now() + self.settle_timeout_s
        wall_deadline = time.monotonic() + self.settle_wall_timeout_s
        stable_since = None
        while self._now() < sim_deadline and time.monotonic() < wall_deadline:
            if settled():
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self.settle_dwell_s:
                    return True
            else:
                stable_since = None
            time.sleep(0.1)
        return False

    def _monitor_engagement(self):
        """Run to TRUE closest-point-of-approach (never terminate on a radius
        crossing), so min_range is the uncensored CPA — scorable at any radius.
        Records the first sim-time each hit_radius is crossed."""
        t0_sim = self._now()        # sim-time engagement clock (reported t_cpa/duration)
        t0_wall = time.monotonic()  # wall-clock safety
        r0 = None
        min_r, t_cpa = float("inf"), 0.0
        rising = 0
        last_log = -5.0
        t_cross = {rad: None for rad in self.hit_radii}
        t, reason = 0.0, "timeout"
        while rclpy.ok():
            t = self._now() - t0_sim
            tw = time.monotonic() - t0_wall
            r = self._range()
            if r is not None:
                if r0 is None:
                    r0 = r
                if r < min_r:
                    min_r, t_cpa = r, t
                    rising = 0
                elif r > min_r + self.cpa_margin_m:
                    rising += 1
                for rad in self.hit_radii:
                    if t_cross[rad] is None and r < rad:
                        t_cross[rad] = t
                if tw - last_log >= 5.0:
                    last_log = tw
                    self.get_logger().info(
                        f"  engaging t={t:4.1f}s(sim) range={r:5.1f}m min={min_r:5.1f}m")
                # End at TRUE CPA: the pursuer has demonstrably closed then the
                # range re-opened for ~0.5 s. (A fleeing target's range rises
                # first during the velocity ramp — not a CPA pass.)
                closed = min_r < r0 - max(self.cpa_margin_m, 5.0)
                if closed and rising >= 25 and t > 2.0:
                    reason = "cpa_passed"; break
            if t >= self.engage_timeout_s:
                reason = "timeout"; break
            if tw >= self.wall_timeout_s:
                reason = "wall_timeout"; break
            time.sleep(0.02)
        primary = self.intercept_radius_m
        success = bool(np.isfinite(min_r) and min_r < primary)
        out = {
            "success": success,
            "result": "hit" if success else ("timeout" if reason != "cpa_passed" else "miss"),
            "min_range_m": float(min_r), "t_cpa_s": t_cpa,
            "time_to_intercept_s": t_cross.get(primary),
            "duration_s": t, "terminal_reason": reason,
        }
        for rad in self.hit_radii:
            out[f"crossed_{str(rad).replace('.', 'p')}m"] = (t_cross[rad] is not None)
        return out

    def run_trial(self, order, estimator, regime, geometry):
        tag = f"{self.boot_id}_{order:02d}_{estimator}_{regime}_{geometry}"
        self.get_logger().info(f"── trial {tag} ──")
        ic = GEOMETRIES[geometry]
        range_m = float(np.linalg.norm(np.array(ic["interceptor"][:3]) - np.array(ic["target"][:3])))
        # Static per-trial covariates (realized dynamics / estimator-error
        # diagnostics are computed post-hoc from bag_tag by analyze_bags.py).
        s, a, f, hd, vr, vfwd, alat = self.conditions[regime]
        row = {"boot_id": self.boot_id, "order": order, "estimator": estimator,
               "regime": regime, "geometry": geometry, "range_m": round(range_m, 2),
               "v_max": self.pn_vmax, "a_max": self.pn_amax, "seed": self.seed_eff,
               "target_condition": regime, "target_forward_speed_cmd_mps": vfwd,
               "target_lat_accel_cmd_mps2": round(alat, 3),
               "target_sinusoid_amplitude_m": round(a, 3), "target_sinusoid_frequency_hz": f,
               "rho_v_forward_nominal": (round(self.pn_vmax / vfwd, 3) if vfwd > 0 else None),
               "rho_a_nominal": (round(self.pn_amax / alat, 3) if alat > 0 else None),
               "bag_tag": f"eng_{tag}", "tag": tag}

        if not self._select_estimator(estimator) or not self._select_regime(regime):
            row.update(result="config_error", success=False, min_range_m=None)
            return row
        if not self._reposition(geometry):
            self.get_logger().warning(f"{tag}: failed to settle at IC")
            row.update(result="settle_error", success=False, min_range_m=self._range())
            return row

        # EKF arms: reset at the IC so the filter re-initializes from the true
        # ~50 m bearing, then let it converge before engaging.
        if estimator in self._ekf_reset:
            self._reset_ekf(estimator)
            time.sleep(self.ekf_settle_s)

        self._start_bag(f"eng_{tag}")
        self._engage(True)
        outcome = self._monitor_engagement()
        self._engage(False)
        self._stop_bag()
        row.update(outcome)
        mr = outcome["min_range_m"]
        self.get_logger().info(
            f"{tag}: {outcome['result']} (success@{self.intercept_radius_m}m="
            f"{outcome['success']}) min_range={mr:.2f}m t_cpa={outcome['t_cpa_s']:.1f}s "
            f"[{outcome['terminal_reason']}]")
        time.sleep(1.0)  # let disengage propagate before the next reposition
        return row

    def run_boot(self):
        rng = random.Random(self.seed_eff)
        trials = [(e, rg, g) for e in self.estimators for rg in self.regimes
                  for g in self.geometries for _ in range(self.repeats)]
        rng.shuffle(trials)
        self.get_logger().info(
            f"boot {self.boot_id}: {len(trials)} trials (seed={self.seed_eff})")

        # Wait until both odom streams are live before driving anything.
        if not self._sleep_until(
                lambda: all(v is not None for v in self._odom.values()), 30.0):
            self.get_logger().error("no odom from one or both drones; aborting boot")
            return []
        self._read_pn_limits()  # record the PN's configured v_max/a_max

        rows = []
        for i, (e, rg, g) in enumerate(trials):
            rows.append(self.run_trial(i, e, rg, g))
            self._write_results(rows)
        self._publish_goto("interceptor", HOME["interceptor"])
        self._publish_goto("target", HOME["target"])
        self.get_logger().info(f"boot {self.boot_id} complete: {len(rows)} trials")
        return rows

    def _write_results(self, rows):
        base = os.path.join(self.results_dir, f"boot_{self.boot_id}_results")
        with open(base + ".jsonl", "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")
        cols = ["boot_id", "order", "estimator", "regime", "geometry", "range_m",
                "v_max", "a_max", "seed",
                "target_condition", "target_forward_speed_cmd_mps", "target_lat_accel_cmd_mps2",
                "target_sinusoid_amplitude_m", "target_sinusoid_frequency_hz",
                "rho_v_forward_nominal", "rho_a_nominal",
                "result", "success", "min_range_m",
                "t_cpa_s", "time_to_intercept_s",
                "crossed_2p0m", "crossed_1p0m", "crossed_0p5m",
                "duration_s", "terminal_reason", "bag_tag", "tag"]
        with open(base + ".csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", restval="")
            w.writeheader()
            w.writerows(rows)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentConductor()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        node.run_boot()
    except KeyboardInterrupt:
        node.get_logger().info("interrupted; stopping bag + disengaging")
        node._engage(False)
        node._stop_bag()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
