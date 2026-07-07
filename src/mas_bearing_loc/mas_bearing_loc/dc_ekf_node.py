"""ROS2 node wiring DCEKF to the MAS architecture topics.

Subscribes (within `/{veh}/` namespace):
    yolo_result_vision       vision_msgs/Detection2DArray   (delayed bearing measurement)
    common_frame/odom        nav_msgs/Odometry              (aircraft world pose; REP-147)
    mavros/imu/data          sensor_msgs/Imu                (high-rate predict input)
    camera/color/camera_info sensor_msgs/CameraInfo         (intrinsics)
    gimbal_state_rpy_deg     geometry_msgs/Vector3          (gimbal angles, deg)
    camera/zoom_level        std_msgs/Float64               (zoom factor)

Publishes:
    bearing_loc/target_pose             geometry_msgs/PoseWithCovarianceStamped
    bearing_loc/target_twist            geometry_msgs/TwistStamped
    bearing_loc/image_feature_meas      geometry_msgs/PointStamped (normalized)
    bearing_loc/image_feature_pred      geometry_msgs/PointStamped (normalized)
    bearing_loc/diagnostics             std_msgs/Float64MultiArray
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PointStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Float32, Float64, Float64MultiArray
from vision_msgs.msg import Detection2DArray

from .camera_model import CameraIntrinsics, gimbal_R_c_b
from .dc_ekf import DCEKF, DCEKFConfig, ERR_PR, ERR_VR


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _t_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_msg_to_wxyz(q) -> np.ndarray:
    # geometry_msgs Quaternion uses xyzw layout; we store wxyz
    return np.array([q.w, q.x, q.y, q.z])


class DCEKFNode(Node):
    def __init__(self):
        super().__init__('dc_ekf_node')

        # Class filter: empty string = accept any class.  YOLO classes are
        # strings ('drone', 'person', ...), per vision_msgs/ObjectHypothesis.
        self.declare_parameter('target_class_name', '')
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('camera_focal_baseline', 0.0)   # 0 → use CameraInfo
        self.declare_parameter('use_odom_attitude_seed', True)
        self.declare_parameter('init_range_guess', 30.0)
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('sigma_pix', 2.0)
        self.declare_parameter('sigma_target_acc', 1.5)
        self.declare_parameter('reject_mahalanobis', 16.0)
        # If true, replace the EKF's integrated attitude with `common_frame/odom`
        # on every odom message.  Defeats slow gyro drift in long episodes at the
        # cost of strict paper fidelity.  Recommended for any run > 5 s.
        self.declare_parameter('override_attitude_from_odom', True)
        # If true, pin v_r to the aircraft's odom velocity each odom message.
        # Assumes v_target ≈ 0; defeats IMU-integration drift on v_r in long runs.
        # Set False for genuinely maneuvering-target experiments where you want
        # the EKF to estimate v_target through its own dynamics.
        self.declare_parameter('override_velocity_from_odom', True)
        # Disable the delay-compensation rewind+replay path; run as a
        # vanilla EKF that applies image updates to the current state.
        self.declare_parameter('disable_delay_compensation', False)
        # Topic prefix under the vehicle namespace.  Default 'bearing_loc'
        # matches the DC-EKF stream; set to 'vanilla_loc' (or anything else)
        # when running a second instance side-by-side for comparison.
        self.declare_parameter('publish_prefix', 'bearing_loc')
        self.declare_parameter('gimbal_zero_R_override', '')   # empty → use default
        # Camera-in-body translational offset (body FLU, m).  Default = (0,0,0),
        # matching observed Pegasus sim behavior; override per real platform.
        self.declare_parameter('t_cam_in_body', [0.0, 0.0, 0.0])
        # Stabilization knobs (default off = paper-faithful). See
        # research/.../dc_ekf_retune/DC_EKF_RETUNE_001.md — they slow but do not cure the
        # 18D in-state-image-feature divergence on high-bearing-rate orbits.
        self.declare_parameter('sigma_norm_floor', 0.0)   # rad; floor on effective meas 1σ
        self.declare_parameter('depth_floor', 0.0)        # m; min |Z| in the IBVS L_s (caps 1/Z)
        self.declare_parameter('cov_eig_floor', 0.0)      # covariance PD-projection floor
        self.declare_parameter('cov_eig_ceil', 0.0)       # covariance PD-projection ceiling

        t_cam = list(self.get_parameter('t_cam_in_body').value)
        cfg = DCEKFConfig(
            sigma_pix=float(self.get_parameter('sigma_pix').value),
            sigma_target_acc=float(self.get_parameter('sigma_target_acc').value),
            init_range=float(self.get_parameter('init_range_guess').value),
            t_cam_in_body=(float(t_cam[0]), float(t_cam[1]), float(t_cam[2])),
            disable_delay_compensation=bool(
                self.get_parameter('disable_delay_compensation').value),
            sigma_norm_floor=float(self.get_parameter('sigma_norm_floor').value),
            depth_floor=float(self.get_parameter('depth_floor').value),
            cov_eig_floor=float(self.get_parameter('cov_eig_floor').value),
            cov_eig_ceil=float(self.get_parameter('cov_eig_ceil').value),
        )
        self.ekf = DCEKF(cfg)
        self.reject_mahal = float(self.get_parameter('reject_mahalanobis').value)
        self.target_class_name = str(self.get_parameter('target_class_name').value)
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.use_odom_attitude_seed = bool(self.get_parameter('use_odom_attitude_seed').value)
        self.override_attitude = bool(self.get_parameter('override_attitude_from_odom').value)
        self.override_velocity = bool(self.get_parameter('override_velocity_from_odom').value)
        self._prefix = str(self.get_parameter('publish_prefix').value)

        # Caches
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.zoom: float = 1.0
        self.gimbal_rpy_rad: np.ndarray = np.zeros(3)
        self.aircraft_pos: Optional[np.ndarray] = None
        self.aircraft_vel: Optional[np.ndarray] = None
        self.aircraft_quat: Optional[np.ndarray] = None
        self.last_odom_stamp: Optional[float] = None
        self.last_imu_stamp: Optional[float] = None

        # IO
        be = _best_effort_qos()
        self.create_subscription(Imu, 'mavros/imu/data', self._on_imu, be)
        self.create_subscription(Odometry, 'common_frame/odom', self._on_odom, be)
        self.create_subscription(Detection2DArray, 'yolo_result_vision',
                                 self._on_detections, be)
        self.create_subscription(CameraInfo, 'camera/color/camera_info',
                                 self._on_camera_info, 10)
        self.create_subscription(Vector3, 'gimbal_state_rpy_deg',
                                 self._on_gimbal, be)
        # zoom is published as Float64 in arch
        self.create_subscription(Float64, 'camera/zoom_level',
                                 self._on_zoom, be)

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, f'{self._prefix}/target_pose', 10)
        self.pub_twist = self.create_publisher(
            TwistStamped, f'{self._prefix}/target_twist', 10)
        self.pub_pbar_meas = self.create_publisher(
            PointStamped, f'{self._prefix}/image_feature_meas', 10)
        self.pub_pbar_pred = self.create_publisher(
            PointStamped, f'{self._prefix}/image_feature_pred', 10)
        self.pub_diag = self.create_publisher(
            Float64MultiArray, f'{self._prefix}/diagnostics', 10)

        publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(publish_period, self._on_publish)

        self.get_logger().info('DC-EKF node ready (reproducing Liu et al. 2026 §III-E)')

    # ---------- subscription callbacks ----------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self.intrinsics is None:
            K = msg.k
            self.intrinsics = CameraIntrinsics(
                fx=float(K[0]), fy=float(K[4]),
                cx=float(K[2]), cy=float(K[5]),
                width=int(msg.width), height=int(msg.height),
            )
            self.get_logger().info(
                f'CameraInfo received: fx={self.intrinsics.fx:.1f} '
                f'fy={self.intrinsics.fy:.1f} cx={self.intrinsics.cx:.1f} '
                f'cy={self.intrinsics.cy:.1f}')

    def _on_zoom(self, msg: Float64) -> None:
        z = float(msg.data)
        if z > 0.0:
            self.zoom = z

    def _on_gimbal(self, msg: Vector3) -> None:
        self.gimbal_rpy_rad = np.array([
            math.radians(msg.x), math.radians(msg.y), math.radians(msg.z)])

    def _on_odom(self, msg: Odometry) -> None:
        self.aircraft_pos = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])
        self.aircraft_vel = np.array([msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y,
                                      msg.twist.twist.linear.z])
        self.aircraft_quat = _quat_msg_to_wxyz(msg.pose.pose.orientation)
        self.last_odom_stamp = _t_to_sec(msg.header.stamp)
        # Keep the EKF's internal attitude pinned to odom when requested.
        # This kills the slow gyro-integration drift that otherwise biases
        # the predicted image feature over multi-second episodes.
        if self.override_attitude and self.ekf.initialized:
            self.ekf.override_attitude(self.aircraft_quat)
        if self.override_velocity and self.ekf.initialized and self.aircraft_vel is not None:
            self.ekf.override_relative_velocity(self.aircraft_vel)

    def _on_imu(self, msg: Imu) -> None:
        if not self.ekf.initialized:
            return
        t = _t_to_sec(msg.header.stamp)
        omega = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        R_c_b = gimbal_R_c_b(self.gimbal_rpy_rad)
        self.ekf.predict_imu(t, omega, accel, R_c_b)
        self.last_imu_stamp = t

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self.intrinsics is None or self.aircraft_quat is None:
            return
        det = self._pick_best_detection(msg)
        if det is None:
            return
        # vision_msgs/BoundingBox2D.center is a Pose2D: {position: {x, y}, theta}.
        center = det.bbox.center
        if hasattr(center, 'position'):
            u_pix = float(center.position.x)
            v_pix = float(center.position.y)
        else:  # very old vision_msgs (no Pose2D wrapping)
            u_pix = float(center.x)
            v_pix = float(center.y)
        p_bar = np.array(self.intrinsics.normalize(u_pix, v_pix, self.zoom))
        t_img = _t_to_sec(msg.header.stamp)
        R_c_b = gimbal_R_c_b(self.gimbal_rpy_rad)

        if not self.ekf.initialized:
            self._initialize(t_img, p_bar, R_c_b)
            return

        # Gate against the prior to reject obvious false positives.
        innov = p_bar - self.ekf.p_bar
        if np.linalg.norm(innov) > 0.5:  # > ~30° from prediction on a unit-focal sensor
            self.get_logger().debug(f'Bearing reject: |innov|={np.linalg.norm(innov):.3f}')

        # Publish raw measurement for diagnostics (Fig.10-style plots).
        m = PointStamped()
        m.header = msg.header
        m.point.x = float(p_bar[0])
        m.point.y = float(p_bar[1])
        m.point.z = 0.0
        self.pub_pbar_meas.publish(m)

        diag = self.ekf.update_bearing(t_img, p_bar, R_c_b)
        if diag is not None and diag['mahalanobis'] > self.reject_mahal:
            self.get_logger().warning(
                f'Bearing reject (mahal={diag["mahalanobis"]:.1f}); '
                f'innovation={diag["innovation"]}')

    # ---------- helpers ----------

    def _pick_best_detection(self, msg: Detection2DArray):
        """Pick the highest-score detection that satisfies the class filter.

        vision_msgs/Detection2D.results is a list of ObjectHypothesisWithPose
        whose `hypothesis.class_id` is a *string* (e.g. 'drone').  Older
        builds of vision_msgs put a plain `id` (int) directly on the result;
        we handle both shapes defensively.
        """
        best = None
        best_score = -1.0
        for det in msg.detections:
            for r in det.results:
                hyp = getattr(r, 'hypothesis', None)
                if hyp is not None:
                    cls_name = str(getattr(hyp, 'class_id', ''))
                    score = float(getattr(hyp, 'score', 0.0))
                else:
                    cls_name = str(getattr(r, 'id', ''))
                    score = float(getattr(r, 'score', 0.0))
                if score < self.min_conf:
                    continue
                if self.target_class_name and cls_name != self.target_class_name:
                    continue
                if score > best_score:
                    best_score = score
                    best = det
        return best

    def _initialize(self, t_img: float, p_bar: np.ndarray, R_c_b: np.ndarray) -> None:
        # Bearing in camera frame: through normalized image point at unit depth
        n_cam = np.array([p_bar[0], p_bar[1], 1.0])
        n_cam = n_cam / np.linalg.norm(n_cam)
        # Aircraft attitude seed from common_frame/odom
        q_seed = self.aircraft_quat if self.use_odom_attitude_seed \
            else np.array([1.0, 0.0, 0.0, 0.0])
        # Compute R_b_w from seed quat
        from .quaternion import quat_to_rot
        R_b_w = quat_to_rot(q_seed)
        R_c_w = R_b_w @ R_c_b
        bearing_world = R_c_w @ n_cam
        self.ekf.initialize_from_bearing(t_img, bearing_world, q_seed,
                                          v_aircraft_world=self.aircraft_vel)
        self.get_logger().info(
            f'DC-EKF initialized at t={t_img:.3f}; '
            f'bearing_world={bearing_world}, range_guess={self.ekf.cfg.init_range:.1f} m')

    def _on_publish(self) -> None:
        if not self.ekf.initialized or self.aircraft_pos is None:
            return
        # Target world pose
        p_t = self.ekf.target_position_world(self.aircraft_pos)
        v_t = self.ekf.target_velocity_world(self.aircraft_vel) \
            if self.aircraft_vel is not None else np.zeros(3)

        now = self.get_clock().now().to_msg()

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'common_frame'
        pose.pose.pose.position.x = float(p_t[0])
        pose.pose.pose.position.y = float(p_t[1])
        pose.pose.pose.position.z = float(p_t[2])
        pose.pose.pose.orientation.w = 1.0
        # Map 3x3 position covariance into 6x6 row-major upper-left block.
        cov = np.zeros((6, 6))
        cov[:3, :3] = self.ekf.cov_position
        pose.pose.covariance = list(cov.flatten())
        self.pub_pose.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = 'common_frame'
        twist.twist.linear.x = float(v_t[0])
        twist.twist.linear.y = float(v_t[1])
        twist.twist.linear.z = float(v_t[2])
        self.pub_twist.publish(twist)

        pred = PointStamped()
        pred.header.stamp = now
        pred.header.frame_id = 'camera'
        pred.point.x = float(self.ekf.p_bar[0])
        pred.point.y = float(self.ekf.p_bar[1])
        self.pub_pbar_pred.publish(pred)

        diag = Float64MultiArray()
        diag.data = [
            float(np.trace(self.ekf.cov_position)),
            float(np.trace(self.ekf.cov_velocity)),
            float(np.linalg.norm(self.ekf.relative_position)),
            float(np.linalg.norm(self.ekf.relative_velocity)),
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = DCEKFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
