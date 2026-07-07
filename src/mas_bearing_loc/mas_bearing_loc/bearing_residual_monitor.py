"""Quantify the systematic bearing bias of the ego sensing chain against GT.

Every EKF issue we hit (DirectProjection collapse, SimpleEKF mean-fragility, the
covariance runaway) traces back to one question: *is the measured line-of-sight
actually pointing where ground truth says the target is?* This node answers it.

It reconstructs the measured world LOS **exactly as the EKFs do** — YOLO bbox
center → normalized (zoom) → camera ray → gimbal ``R_c_b`` → body ``R_b_w`` →
world — reusing ``camera_model`` so any bias it finds is the bias the EKFs see.
It then computes the GT-implied LOS from the observer odom to the target's GT
odom (sim only) and publishes the angular residual. A non-zero *mean* residual is
a systematic bias (gimbal mount/convention, lever arm, timing); zero-mean scatter
is just detector noise.

Run under the observer (interceptor) namespace::

    ros2 run mas_bearing_loc bearing_residual_monitor --ros-args \
        -r __ns:=/px4_1 -p gt_target_topic:=/px4_2/common_frame/odom \
        -p use_sim_time:=true

Publishes (under the namespace):
    bearing_residual/angle_deg   (std_msgs/Float64)  total angular error meas↔GT
    bearing_residual/azel_err_deg(geometry_msgs/Vector3) x=Δaz, y=Δel (deg)
    bearing_residual/measured_world (geometry_msgs/Vector3) measured unit LOS
    bearing_residual/gt_world       (geometry_msgs/Vector3) GT unit LOS
Logs running mean±std of (Δaz, Δel, angle) every ``log_every`` samples.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray

from .camera_model import CameraIntrinsics, gimbal_R_c_b, project_point
from .quaternion import quat_to_rot


def _be_qos(depth=10):
    return QoSProfile(depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _quat_wxyz(q):
    return np.array([q.w, q.x, q.y, q.z])


def _az(u):   # world ENU unit vector -> azimuth (rad, East->North)
    return math.atan2(u[1], u[0])


def _el(u):   # -> elevation (rad, +up)
    return math.atan2(u[2], math.hypot(u[0], u[1]))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class BearingResidualMonitor(Node):
    def __init__(self):
        super().__init__("bearing_residual_monitor")

        self.declare_parameter("target_class_name", "drone")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("gt_target_topic", "/px4_2/common_frame/odom")
        self.declare_parameter("t_cam_in_body", [0.0, 0.0, 0.0])
        self.declare_parameter("log_every", 50)

        self.cls = str(self.get_parameter("target_class_name").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        tc = list(self.get_parameter("t_cam_in_body").value)
        self.t_cam_b = np.array([float(tc[0]), float(tc[1]), float(tc[2])])
        self.log_every = int(self.get_parameter("log_every").value)
        gt_topic = str(self.get_parameter("gt_target_topic").value)

        self.intr: Optional[CameraIntrinsics] = None
        self.zoom = 1.0
        self.gimbal = np.zeros(3)
        self.obs_p: Optional[np.ndarray] = None
        self.obs_q: Optional[np.ndarray] = None
        self.gt_p: Optional[np.ndarray] = None

        be = _be_qos()
        self.create_subscription(Detection2DArray, "yolo_result_vision", self._on_det, be)
        self.create_subscription(Odometry, "common_frame/odom", self._on_obs, be)
        self.create_subscription(CameraInfo, "camera/color/camera_info", self._on_ci, 10)
        self.create_subscription(Vector3, "gimbal_state_rpy_deg", self._on_gim, be)
        self.create_subscription(Float64, "camera/zoom_level", self._on_zoom, be)
        self.create_subscription(Odometry, gt_topic, self._on_gt, be)

        self.pub_angle = self.create_publisher(Float64, "bearing_residual/angle_deg", 10)
        self.pub_azel = self.create_publisher(Vector3, "bearing_residual/azel_err_deg", 10)
        self.pub_meas = self.create_publisher(Vector3, "bearing_residual/measured_world", 10)
        self.pub_gt = self.create_publisher(Vector3, "bearing_residual/gt_world", 10)
        # Pixel-space residual: detected bbox center − projected-GT pixel.
        self.pub_pix = self.create_publisher(Vector3, "bearing_residual/pixel_err", 10)
        # Instantaneous implied delay = residual_angle / LOS_rate.
        self.pub_delay = self.create_publisher(Float64, "bearing_residual/implied_delay_s", 10)

        self._daz = deque(maxlen=4000)
        self._del = deque(maxlen=4000)
        self._ang = deque(maxlen=4000)
        self._dupix = deque(maxlen=4000)
        self._dvpix = deque(maxlen=4000)
        # Delay regression accumulators: Δ(deg) = a + b·ω(deg/s); delay ≈ −b.
        # az and el fit separately; a = static offset, b = −delay signature.
        self._reg = {k: dict(n=0, sw=0.0, sr=0.0, sww=0.0, swr=0.0)
                     for k in ("az", "el")}
        self._wmin = {"az": 1e9, "el": 1e9}
        self._wmax = {"az": -1e9, "el": -1e9}
        self._prev_t = None
        self._prev_azgt = None
        self._prev_elgt = None
        self._n = 0
        self.get_logger().info(
            f"bearing_residual_monitor: comparing measured LOS vs GT '{gt_topic}'")

    def _on_ci(self, m: CameraInfo):
        if self.intr is None:
            self.intr = CameraIntrinsics(fx=float(m.k[0]), fy=float(m.k[4]),
                                         cx=float(m.k[2]), cy=float(m.k[5]),
                                         width=int(m.width), height=int(m.height))

    def _on_zoom(self, m: Float64):
        if m.data > 0:
            self.zoom = float(m.data)

    def _on_gim(self, m: Vector3):
        self.gimbal = np.array([math.radians(m.x), math.radians(m.y), math.radians(m.z)])

    def _on_obs(self, m: Odometry):
        p = m.pose.pose.position
        self.obs_p = np.array([p.x, p.y, p.z])
        self.obs_q = _quat_wxyz(m.pose.pose.orientation)

    def _on_gt(self, m: Odometry):
        p = m.pose.pose.position
        self.gt_p = np.array([p.x, p.y, p.z])

    def _pick_best(self, msg):
        best, best_s = None, -1.0
        for det in msg.detections:
            for r in det.results:
                hyp = getattr(r, "hypothesis", None)
                cls = str(getattr(hyp, "class_id", "")) if hyp else str(getattr(r, "id", ""))
                score = float(getattr(hyp, "score", 0.0)) if hyp else float(getattr(r, "score", 0.0))
                if score < self.min_conf:
                    continue
                if self.cls and cls != self.cls:
                    continue
                if score > best_s:
                    best_s, best = score, det
        return best

    def _on_det(self, msg: Detection2DArray):
        if self.intr is None or self.obs_p is None or self.obs_q is None or self.gt_p is None:
            return
        det = self._pick_best(msg)
        if det is None:
            return
        c = det.bbox.center
        u_pix, v_pix = (float(c.position.x), float(c.position.y)) if hasattr(c, "position") \
            else (float(c.x), float(c.y))
        p_bar = np.array(self.intr.normalize(u_pix, v_pix, self.zoom))

        # Measured world LOS — identical reconstruction to the EKF nodes.
        R_b_w = quat_to_rot(self.obs_q)
        R_c_w = R_b_w @ gimbal_R_c_b(self.gimbal)
        n_cam = np.array([p_bar[0], p_bar[1], 1.0])
        n_cam /= np.linalg.norm(n_cam)
        u_meas = R_c_w @ n_cam

        # GT-implied world LOS (camera optical center -> GT target).
        p_cam = self.obs_p + R_b_w @ self.t_cam_b
        d = self.gt_p - p_cam
        nd = np.linalg.norm(d)
        if nd < 1e-6:
            return
        u_gt = d / nd

        ang = math.degrees(math.acos(float(np.clip(np.dot(u_meas, u_gt), -1.0, 1.0))))
        az_m, az_g = _az(u_meas), _az(u_gt)
        el_m, el_g = _el(u_meas), _el(u_gt)
        daz = math.degrees(_wrap(az_m - az_g))
        dele = math.degrees(el_m - el_g)

        # Pixel-space residual: bbox center vs projected-GT pixel (same geometry).
        pbar_pred = project_point(self.gt_p, self.obs_p, R_b_w,
                                  gimbal_R_c_b(self.gimbal), self.t_cam_b)
        du = dv = float("nan")
        if pbar_pred[2] > 0.0:
            u_pred = self.intr.cx + pbar_pred[0] * self.intr.fx * self.zoom
            v_pred = self.intr.cy + pbar_pred[1] * self.intr.fy * self.zoom
            du, dv = u_pix - u_pred, v_pix - v_pred

        # LOS angular rate (deg/s) from consecutive GT bearings, and the delay
        # regression Δ(deg)=a+b·ω: a=static offset, delay≈−b. A delay shows up
        # as residual ∝ LOS rate; a static offset is rate-independent.
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        implied = float("nan")
        if self._prev_t is not None:
            dt = t - self._prev_t
            if 1e-3 < dt < 1.0:
                waz = math.degrees(_wrap(az_g - self._prev_azgt)) / dt
                wel = math.degrees(el_g - self._prev_elgt) / dt
                self._accum("az", waz, daz)
                self._accum("el", wel, dele)
                wmag = math.hypot(waz, wel)
                if wmag > 2.0:
                    implied = ang / wmag  # s  (|residual| / |ω|)
        self._prev_t, self._prev_azgt, self._prev_elgt = t, az_g, el_g

        self.pub_angle.publish(Float64(data=ang))
        self.pub_azel.publish(Vector3(x=daz, y=dele, z=0.0))
        self.pub_meas.publish(Vector3(x=float(u_meas[0]), y=float(u_meas[1]), z=float(u_meas[2])))
        self.pub_gt.publish(Vector3(x=float(u_gt[0]), y=float(u_gt[1]), z=float(u_gt[2])))
        self.pub_pix.publish(Vector3(x=du, y=dv, z=0.0))
        if implied == implied:
            self.pub_delay.publish(Float64(data=implied))

        self._daz.append(daz); self._del.append(dele); self._ang.append(ang)
        if du == du:
            self._dupix.append(du); self._dvpix.append(dv)
        self._n += 1
        if self._n % self.log_every == 0:
            self._log(nd)

    def _accum(self, key, w, r):
        a = self._reg[key]
        a["n"] += 1; a["sw"] += w; a["sr"] += r; a["sww"] += w * w; a["swr"] += w * r
        self._wmin[key] = min(self._wmin[key], w)
        self._wmax[key] = max(self._wmax[key], w)

    def _fit(self, key):
        a = self._reg[key]
        n = a["n"]
        if n < 8:
            return None
        denom = n * a["sww"] - a["sw"] ** 2
        if abs(denom) < 1e-9:
            return None
        b = (n * a["swr"] - a["sw"] * a["sr"]) / denom   # slope, deg per deg/s = s
        off = (a["sr"] - b * a["sw"]) / n                # intercept = static offset, deg
        return off, b

    def _log(self, nd):
        msg = (f"N={len(self._ang)}: "
               f"Δaz={np.mean(self._daz):+.2f}±{np.std(self._daz):.2f}°  "
               f"Δel={np.mean(self._del):+.2f}±{np.std(self._del):.2f}°  "
               f"|ang|={np.mean(self._ang):.2f}°  "
               f"pix=({np.mean(self._dupix) if self._dupix else float('nan'):+.1f},"
               f"{np.mean(self._dvpix) if self._dvpix else float('nan'):+.1f})  "
               f"r≈{nd:.1f}m")
        faz, fel = self._fit("az"), self._fit("el")
        if faz and fel:
            # delay = −slope (s). az & el agreeing ⇒ real pipeline lag;
            # slope≈0 with nonzero offset ⇒ static geometry/detection bias.
            msg += (f" || FIT az: off={faz[0]:+.2f}° delay={-faz[1]*1e3:+.0f}ms"
                    f" ω∈[{self._wmin['az']:+.0f},{self._wmax['az']:+.0f}]°/s |"
                    f" el: off={fel[0]:+.2f}° delay={-fel[1]*1e3:+.0f}ms"
                    f" ω∈[{self._wmin['el']:+.0f},{self._wmax['el']:+.0f}]°/s")
        self.get_logger().info(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = BearingResidualMonitor()
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
