"""ROS2 node for the minimal 6D bearing-only EKF (`simple_ekf.SimpleEKF`).

Wires the same input topics as `dc_ekf_node` (yolo + odom + gimbal + zoom +
camera_info) but uses the simple constant-velocity target tracker — no IBVS
state, no IMU integration, no quaternion error state, no delay compensation.

Publishes under `{veh}/simple_loc/...` by default; override with the
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
from .quaternion import quat_to_rot
from .simple_ekf import SimpleEKF, SimpleEKFConfig


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


class SimpleEKFNode(Node):
    def __init__(self):
        super().__init__("simple_ekf_node")

        self.declare_parameter("target_class_name", "drone")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("init_range_guess", 15.0)
        self.declare_parameter("sigma_pix", 3.0)
        # Floor on the effective normalized measurement 1σ (rad). sigma_pix/(fx·zoom)
        # under-estimates the true ~0.03 rad angular error at high zoom, over-fitting the
        # filter so the bearing-only range collapses. Flooring fixes it (0.76 m vs 1.19 m
        # on bag_20260622_164718). See research/.../dc_ekf_retune/DC_EKF_RETUNE_001.md.
        # 0 = disabled (legacy behavior).
        self.declare_parameter("sigma_norm_floor", 0.0)
        self.declare_parameter("sigma_target_acc", 0.5)
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("reject_mahalanobis", 16.0)
        self.declare_parameter("t_cam_in_body", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_prefix", "simple_loc")
        # Cap trace(P_pos) (m²) so the weakly-observed LOS/range direction can't
        # run the covariance away. Structural safeguard; 0 = disabled.
        self.declare_parameter("pos_var_ceiling", 300.0)

        t_cam = list(self.get_parameter("t_cam_in_body").value)
        cfg = SimpleEKFConfig(
            sigma_pix=float(self.get_parameter("sigma_pix").value),
            sigma_norm_floor=float(self.get_parameter("sigma_norm_floor").value),
            sigma_target_acc=float(self.get_parameter("sigma_target_acc").value),
            init_range=float(self.get_parameter("init_range_guess").value),
            t_cam_in_body=(float(t_cam[0]), float(t_cam[1]), float(t_cam[2])),
            pos_var_ceiling=float(self.get_parameter("pos_var_ceiling").value),
            reject_mahalanobis=float(self.get_parameter("reject_mahalanobis").value),
        )
        self._cfg = cfg
        self.ekf = SimpleEKF(cfg)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.reject_mahal = float(self.get_parameter("reject_mahalanobis").value)
        self._prefix = str(self.get_parameter("publish_prefix").value)

        self.intrinsics: Optional[CameraIntrinsics] = None
        self.zoom = 1.0
        self.gimbal_rpy_rad = np.zeros(3)
        self.aircraft_pos: Optional[np.ndarray] = None
        self.aircraft_vel: Optional[np.ndarray] = None
        self.aircraft_quat: Optional[np.ndarray] = None

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

        # Reset switch: drop filter state so it re-initializes from the next
        # bearing. Clears a diverged/NaN run without restarting the node — needed
        # because the sim has no vehicle reset (ticket 004).
        self.create_service(Trigger, "~/reset", self._on_reset)

        self.get_logger().info("SimpleEKF node ready (6D constant-velocity bearing-only)")

    def _on_reset(self, request, response):
        self.ekf = SimpleEKF(self._cfg)
        self.get_logger().info("SimpleEKF reset: re-initializing from next detection")
        response.success = True
        response.message = "SimpleEKF reset; re-initializes from next bearing"
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
        self.aircraft_vel = np.array([msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y,
                                      msg.twist.twist.linear.z])
        self.aircraft_quat = _quat_msg_to_wxyz(msg.pose.pose.orientation)

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

        if not self.ekf.initialized:
            n_cam = np.array([p_bar[0], p_bar[1], 1.0])
            n_cam /= np.linalg.norm(n_cam)
            R_c_w = R_b_w @ R_c_b
            bearing_world = R_c_w @ n_cam
            self.ekf.initialize_from_bearing(
                t, bearing_world, self.aircraft_pos, self.aircraft_vel, R_b_w)
            self.get_logger().info(
                f"SimpleEKF init at t={t:.3f}; bearing_world={bearing_world}")
            return

        m = PointStamped()
        m.header = msg.header
        m.point.x = float(p_bar[0])
        m.point.y = float(p_bar[1])
        self.pub_pbar_meas.publish(m)

        # NOTE: SimpleEKF intentionally uses cfg.sigma_pix as the *normalized*-
        # image measurement σ directly (not σ_pix/(fx·zoom)). That makes the
        # update heavily conservative, which is what keeps this pinhole filter
        # stable in sim — feeding the physically-"correct" small radian σ makes
        # it chase systematic bearing/gimbal bias and diverge (regressed once,
        # ticket 004). Do not "unit-correct" this without re-tuning + re-verify.
        diag = self.ekf.update_bearing(
            t, p_bar, self.aircraft_pos, R_b_w, R_c_b)
        if diag is not None and diag["mahalanobis"] > self.reject_mahal:
            self.get_logger().warning(
                f"Bearing reject mahal={diag['mahalanobis']:.1f}")

    def _on_publish(self):
        if not self.ekf.initialized or self.aircraft_pos is None:
            return
        now = self.get_clock().now().to_msg()

        p = self.ekf.target_position
        v = self.ekf.target_velocity

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
            float(np.linalg.norm(p - self.aircraft_pos)),
            float(np.linalg.norm(v)),
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleEKFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
