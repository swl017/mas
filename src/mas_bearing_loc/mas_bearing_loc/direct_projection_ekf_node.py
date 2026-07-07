"""ROS2 node for the direct-projection relative-state bearing-only EKF.

Wires the same input topics as `simple_ekf_node` (yolo + odom + gimbal + zoom +
camera_info), but runs the 6D *relative-state* `DirectProjectionEKF`:

  * State is the observer→target relative geometry `[q, q̇]`, not the absolute
    target — so the observer's own acceleration enters the prediction as a
    known control input (estimated here by differentiating `common_frame/odom`
    velocity; set `use_obs_accel:=false` to fall back to a constant-relative-
    velocity model).
  * The YOLO bbox center is turned into a 3D *world unit bearing* and fed to the
    filter's direct tangent-plane projection update, instead of the 2D pinhole
    image-feature update used by `SimpleEKF` / `DCEKF`.

Pixel noise is converted to bearing noise per update: a `sigma_pix` 1σ at the
effective focal length `fx·zoom` is ≈ `sigma_pix / (fx·zoom)` rad.

Publishes under `{veh}/direct_loc/...` by default; override with the
`publish_prefix` parameter for side-by-side comparisons.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PointStamped, PoseWithCovarianceStamped, TwistStamped, Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from .camera_model import CameraIntrinsics, gimbal_R_c_b
from .direct_projection_ekf import DirectProjectionEKF, DirectProjectionEKFConfig
from .quaternion import quat_to_rot


def _be_qos(depth=10):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _t(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_msg_to_wxyz(q):
    return np.array([q.w, q.x, q.y, q.z])


class DirectProjectionEKFNode(Node):
    def __init__(self):
        super().__init__("direct_projection_ekf_node")

        self.declare_parameter("target_class_name", "drone")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("init_range_guess", 15.0)
        self.declare_parameter("sigma_pix", 3.0)
        self.declare_parameter("sigma_target_acc", 0.5)
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("reject_mahalanobis", 16.0)
        self.declare_parameter("t_cam_in_body", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_prefix", "direct_loc")
        self.declare_parameter("use_obs_accel", True)
        self.declare_parameter("obs_accel_lpf_alpha", 0.3)
        # fix #1: floor the per-update bearing noise so a high zoom can't make
        # σ_bearing = σ_pix/(fx·zoom) unrealistically tiny (default = the
        # point-mass realistic baseline, 0.013 rad).
        self.declare_parameter("sigma_bearing_floor", 0.013)
        # fix #3: 1/r Jacobian + covariance collapse guards.
        self.declare_parameter("range_floor", 2.0)
        self.declare_parameter("pos_var_floor", 1.0)

        cfg = DirectProjectionEKFConfig(
            sigma_target_acc=float(self.get_parameter("sigma_target_acc").value),
            init_range=float(self.get_parameter("init_range_guess").value),
            range_floor=float(self.get_parameter("range_floor").value),
            pos_var_floor=float(self.get_parameter("pos_var_floor").value),
            reject_mahalanobis=float(self.get_parameter("reject_mahalanobis").value),
        )
        self._cfg = cfg
        self.ekf = DirectProjectionEKF(cfg)
        self.sigma_pix = float(self.get_parameter("sigma_pix").value)
        self.sigma_bearing_floor = float(self.get_parameter("sigma_bearing_floor").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.reject_mahal = float(self.get_parameter("reject_mahalanobis").value)
        self._prefix = str(self.get_parameter("publish_prefix").value)
        t_cam = list(self.get_parameter("t_cam_in_body").value)
        self.t_cam_b = np.array([float(t_cam[0]), float(t_cam[1]), float(t_cam[2])])
        self.use_obs_accel = bool(self.get_parameter("use_obs_accel").value)
        self.accel_alpha = float(self.get_parameter("obs_accel_lpf_alpha").value)

        self.intrinsics: Optional[CameraIntrinsics] = None
        self.zoom = 1.0
        self.gimbal_rpy_rad = np.zeros(3)
        self.aircraft_pos: Optional[np.ndarray] = None
        self.aircraft_vel: Optional[np.ndarray] = None
        self.aircraft_quat: Optional[np.ndarray] = None
        self.aircraft_acc = np.zeros(3)          # world ENU, EMA of d(v_odom)/dt
        self._last_odom_t: Optional[float] = None
        self._last_odom_v: Optional[np.ndarray] = None
        # Observer (camera) state at the filter's current time, used to recover
        # the absolute target from the relative state without a frame mismatch.
        self._obs_p_at_update: Optional[np.ndarray] = None
        self._obs_v_at_update: Optional[np.ndarray] = None

        be = _be_qos()
        self.create_subscription(Odometry, "common_frame/odom", self._on_odom, be)
        self.create_subscription(Detection2DArray, "yolo_result_vision",
                                 self._on_detections, be)
        self.create_subscription(CameraInfo, "camera/color/camera_info",
                                 self._on_camera_info, 10)
        self.create_subscription(Vector3, "gimbal_state_rpy_deg",
                                 self._on_gimbal, be)
        self.create_subscription(Float64, "camera/zoom_level", self._on_zoom, be)

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, f"{self._prefix}/target_pose", 10)
        self.pub_twist = self.create_publisher(
            TwistStamped, f"{self._prefix}/target_twist", 10)
        self.pub_diag = self.create_publisher(
            Float64MultiArray, f"{self._prefix}/diagnostics", 10)
        self.pub_pbar_meas = self.create_publisher(
            PointStamped, f"{self._prefix}/image_feature_meas", 10)
        self.pub_pbar_pred = self.create_publisher(
            PointStamped, f"{self._prefix}/image_feature_pred", 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._on_publish)

        # Reset switch: drop filter + observer-accel state so it re-initializes
        # from the next bearing. Clears a diverged/NaN run without restarting the
        # node — the sim has no vehicle reset (ticket 004).
        self.create_service(Trigger, "~/reset", self._on_reset)

        self.get_logger().info(
            "DirectProjectionEKF node ready (6D relative-state, direct-projection "
            f"bearing; use_obs_accel={self.use_obs_accel})")

    def _on_reset(self, request, response):
        self.ekf = DirectProjectionEKF(self._cfg)
        # Clear the observer-acceleration EMA and recovery anchors so the
        # control input restarts cleanly.
        self.aircraft_acc = np.zeros(3)
        self._last_odom_t = None
        self._last_odom_v = None
        self._obs_p_at_update = None
        self._obs_v_at_update = None
        self.get_logger().info("DirectProjectionEKF reset: re-initializing from next detection")
        response.success = True
        response.message = "DirectProjectionEKF reset; re-initializes from next bearing"
        return response

    def _on_camera_info(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = CameraIntrinsics(
                fx=float(msg.k[0]), fy=float(msg.k[4]),
                cx=float(msg.k[2]), cy=float(msg.k[5]),
                width=int(msg.width), height=int(msg.height),
            )
            self.get_logger().info(
                f"CameraInfo: fx={self.intrinsics.fx:.1f} cx={self.intrinsics.cx:.1f} "
                f"cy={self.intrinsics.cy:.1f}")

    def _on_zoom(self, msg: Float64):
        z = float(msg.data)
        if z > 0:
            self.zoom = z

    def _on_gimbal(self, msg: Vector3):
        self.gimbal_rpy_rad = np.array([
            math.radians(msg.x), math.radians(msg.y), math.radians(msg.z)])

    def _on_odom(self, msg: Odometry):
        self.aircraft_pos = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        self.aircraft_quat = _quat_msg_to_wxyz(msg.pose.pose.orientation)

        # Estimate observer acceleration by EMA-filtered velocity differencing.
        t_odom = _t(msg.header.stamp)
        if (self.use_obs_accel and self._last_odom_t is not None
                and self._last_odom_v is not None):
            dt = t_odom - self._last_odom_t
            if dt > 1e-3:
                a_raw = (v - self._last_odom_v) / dt
                a = max(0.0, min(1.0, self.accel_alpha))
                self.aircraft_acc = (1.0 - a) * self.aircraft_acc + a * a_raw
        self._last_odom_t = t_odom
        self._last_odom_v = v
        self.aircraft_vel = v

    def _cam_pos(self, p_aircraft: np.ndarray, R_b_w: np.ndarray) -> np.ndarray:
        return p_aircraft + R_b_w @ self.t_cam_b

    def _pick_best(self, msg):
        best = None
        best_score = -1.0
        for det in msg.detections:
            for r in det.results:
                hyp = getattr(r, "hypothesis", None)
                if hyp is not None:
                    cls = str(getattr(hyp, "class_id", ""))
                    score = float(getattr(hyp, "score", 0.0))
                else:
                    cls = str(getattr(r, "id", ""))
                    score = float(getattr(r, "score", 0.0))
                if score < self.min_conf:
                    continue
                if self.target_class_name and cls != self.target_class_name:
                    continue
                if score > best_score:
                    best_score = score
                    best = det
        return best

    def _on_detections(self, msg: Detection2DArray):
        if (self.intrinsics is None or self.aircraft_pos is None
                or self.aircraft_quat is None):
            return
        det = self._pick_best(msg)
        if det is None:
            return
        center = det.bbox.center
        if hasattr(center, "position"):
            u_pix, v_pix = float(center.position.x), float(center.position.y)
        else:
            u_pix, v_pix = float(center.x), float(center.y)
        p_bar = np.array(self.intrinsics.normalize(u_pix, v_pix, self.zoom))
        t = _t(msg.header.stamp)
        R_b_w = quat_to_rot(self.aircraft_quat)
        R_c_b = gimbal_R_c_b(self.gimbal_rpy_rad)
        R_c_w = R_b_w @ R_c_b

        # Bbox center → 3D world unit bearing (camera→target).
        n_cam = np.array([p_bar[0], p_bar[1], 1.0])
        n_cam /= np.linalg.norm(n_cam)
        bearing_world = R_c_w @ n_cam

        p_cam = self._cam_pos(self.aircraft_pos, R_b_w)

        if not self.ekf.initialized:
            self.ekf.initialize_from_bearing(t, bearing_world, self.aircraft_vel)
            self._obs_p_at_update = p_cam
            self._obs_v_at_update = (self.aircraft_vel.copy()
                                     if self.aircraft_vel is not None
                                     else np.zeros(3))
            self.get_logger().info(
                f"DirectProjectionEKF init at t={t:.3f}; bearing_world={bearing_world}")
            return

        m = PointStamped()
        m.header = msg.header
        m.point.x = float(p_bar[0])
        m.point.y = float(p_bar[1])
        self.pub_pbar_meas.publish(m)

        # Pixel 1σ → bearing 1σ at the effective focal length, floored so high
        # zoom cannot drive it to an unrealistic sub-mrad value (fix #1).
        fx_eff = max(self.intrinsics.fx * self.zoom, 1e-6)
        sigma_bearing = max(self.sigma_pix / fx_eff, self.sigma_bearing_floor)

        obs_a = self.aircraft_acc if self.use_obs_accel else None
        diag = self.ekf.update_bearing(t, bearing_world, obs_a, sigma_bearing)
        if diag is None:
            return
        if diag["mahalanobis"] > self.reject_mahal:
            self.get_logger().warning(
                f"Bearing reject mahal={diag['mahalanobis']:.1f}")
        self._obs_p_at_update = p_cam
        self._obs_v_at_update = (self.aircraft_vel.copy()
                                 if self.aircraft_vel is not None else np.zeros(3))

        # Predicted (filtered) image feature for Fig.10-style reconstruction.
        n_cam_pred = R_c_w.T @ self.ekf.predicted_bearing_world
        if n_cam_pred[2] > 1e-6:
            pp = PointStamped()
            pp.header = msg.header
            pp.point.x = float(n_cam_pred[0] / n_cam_pred[2])
            pp.point.y = float(n_cam_pred[1] / n_cam_pred[2])
            self.pub_pbar_pred.publish(pp)

    def _on_publish(self):
        if not self.ekf.initialized or self._obs_p_at_update is None:
            return
        now = self.get_clock().now().to_msg()

        # Recover the absolute target against the observer state *at the filter's
        # current time* (stored at the last update) to avoid a frame mismatch.
        p = self.ekf.target_position(self._obs_p_at_update)
        v = self.ekf.target_velocity(self._obs_v_at_update)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = "common_frame"
        pose.pose.pose.position.x = float(p[0])
        pose.pose.pose.position.y = float(p[1])
        pose.pose.pose.position.z = float(p[2])
        pose.pose.pose.orientation.w = 1.0
        cov = np.zeros((6, 6))
        cov[:3, :3] = self.ekf.cov_position
        pose.pose.covariance = list(cov.flatten())
        self.pub_pose.publish(pose)

        tw = TwistStamped()
        tw.header.stamp = now
        tw.header.frame_id = "common_frame"
        tw.twist.linear.x = float(v[0])
        tw.twist.linear.y = float(v[1])
        tw.twist.linear.z = float(v[2])
        self.pub_twist.publish(tw)

        diag = Float64MultiArray()
        diag.data = [
            float(np.trace(self.ekf.cov_position)),
            float(np.trace(self.ekf.cov_velocity)),
            float(self.ekf.range),
            float(np.linalg.norm(self.ekf.rel_velocity)),
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = DirectProjectionEKFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
