"""Observation assembler: ROS2 topics → 63/69D observation vector.

Per-vehicle design: each policy_node instance runs in a vehicle namespace and
assembles observations for its own agent (ego). It subscribes to peer vehicles'
topics via absolute paths for inter-agent observations.

Observation structure:
- Ego (31D): pos(3), vel(3), euler_rpy(3), ang_vel_b(3), lin_acc_b(3),
  gimbal_yaw_body(1, 0=forward), gimbal_pitch_body(1), ray_dir_w(3),
  combined_ang_vel_w(3, from topic), bbox_aoi(1, clipped), zoom(1),
  effective_hfov(1, rad), bbox(4), bbox_empty(1)
- Optional prev-action tail (7D, appended to ego): vx,vy,vz (m/s),
  yaw_rate (rad/s), gimbal_yaw_rate, gimbal_pitch_rate, zoom_rate (normalized).
  Ticket 043 — the previously-commanded cmd_vel, set by the node each loop.
- Inter-agent (16D per other): pos(3), vel(3), ray_dir_w(3),
  combined_ang_vel_w(3), zoom(1), bbox_empty(1), data_age(1), bbox_age(1)
- Optional triangulation tail (6D): tri_pos(3), tri_std(3)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu
from std_msgs.msg import Bool, Float64
from vision_msgs.msg import Detection2DArray

from .utils import (
    euler_xyz_from_quat,
    quat_rotate_inverse,
    wrap_to_pi,
)
import random

# ENU gravity vector in world frame (pulls −Z / down).
_GRAVITY_WORLD_ENU = np.array([0.0, 0.0, -9.81])

# SIYI A8 mini measured zoom curve. Mirrors IsaacLab iris_ma6
# controller/zoom_controller.py: z_eff = 1 + a*(exp(b*(cmd-1)) - 1), cmd ∈ [1, 5].
# Source: /home/usrg/mas/src/scripts/camera_calibration/zoom_curve.json
_ZOOM_CURVE_A: float = 0.32489
_ZOOM_CURVE_B: float = 0.4767
_ZOOM_CMD_MIN: float = 1.0
_ZOOM_CMD_MAX: float = 5.0  # trust region of mrcal calibration (mas/028); training zoom_max


def _compute_z_eff(zoom_cmd: float) -> float:
    """Operator-facing zoom command → effective focal-length multiplier.

    Sub-linear: cmd=5 ⇒ z_eff ≈ 2.86. Clamps to [1, 5] for parity with training.
    """
    cmd = max(_ZOOM_CMD_MIN, min(_ZOOM_CMD_MAX, float(zoom_cmd)))
    return 1.0 + _ZOOM_CURVE_A * (math.exp(_ZOOM_CURVE_B * (cmd - 1.0)) - 1.0)


logger = logging.getLogger(__name__)


@dataclass
class VehicleState:
    """Cached state for a single vehicle, updated by subscriber callbacks."""

    # Motion state (from odometry)
    position_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation_w: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # wxyz
    angular_velocity_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    linear_acceleration_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    motion_timestamp: float = 0.0

    # Gimbal state (from los_rate_controller / siyi_ros_node)
    gimbal_yaw_body: float = 0.0   # radians, body-frame, 0=forward
    gimbal_pitch_body: float = 0.0  # radians
    gimbal_timestamp: float = 0.0

    # Detection state (from YOLO)
    bbox_xywh: np.ndarray = field(default_factory=lambda: np.zeros(4))  # normalized [0,1]
    bbox_empty: float = 1.0
    detection_timestamp: float = 0.0

    # Zoom level
    zoom_level: float = 1.0

    # Pre-computed cross-agent state (from dedicated topics)
    combined_ang_vel_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    chosen_target_ray_w: np.ndarray | None = None  # pre-selected bearing ray from tracker

    # Whether we've received any data
    odom_received: bool = False


@dataclass
class TriangulationState:
    """Cached triangulation result."""

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    std_dev: np.ndarray = field(default_factory=lambda: np.full(3, -1.0))
    is_valid: bool = False
    timestamp: float = 0.0


class ObservationAssembler:
    """Assembles the observation vector for a single ego agent.

    Ego topics use relative names (resolved by the node's namespace).
    Peer topics use absolute paths (/{peer_name}/...).
    """

    def __init__(
        self,
        node: Node,
        ego_name: str,
        peer_names: list[str],
        image_width: int = 640,
        image_height: int = 480,
        enable_triangulation: bool = False,
        use_common_frame: bool = True,
        max_bbox_aoi: float = 20.0,
        enable_prev_action_obs: bool = False,
    ):
        """Initialize observation assembler for a single ego vehicle.

        Args:
            node: ROS2 node (running inside the ego vehicle's namespace).
            ego_name: This vehicle's namespace (e.g., "px4_1").
            peer_names: Other vehicles' namespaces (e.g., ["px4_2", "px4_3"]).
            image_width: Camera image width for bbox normalization.
            image_height: Camera image height for bbox normalization.
            enable_triangulation: Whether to include 6D triangulation tail.
            use_common_frame: If True, use common_frame/odom (ENU).
            max_bbox_aoi: Maximum bbox age-of-information (s). Clips to training range.
            enable_prev_action_obs: Whether to append the 7D prev-action tail to
                the ego block (ticket 043). The node feeds the previous cmd_vel via
                ``set_prev_action`` each loop.
        """
        self._node = node
        self._ego_name = ego_name
        self._peer_names = peer_names
        self._all_names = [ego_name] + list(peer_names)
        self._image_width = image_width
        self._image_height = image_height
        self._enable_triangulation = enable_triangulation
        self._use_common_frame = use_common_frame
        self._max_bbox_aoi = max_bbox_aoi
        self._enable_prev_action_obs = enable_prev_action_obs
        # Previous-step cmd_vel for the prev-action obs tail (ticket 043).
        # [vx,vy,vz (m/s), yaw_rate (rad/s), g_yaw, g_pitch, zoom (normalized)].
        # Updated by the node via set_prev_action; reset to zero on episode entry.
        self._prev_action_obs = np.zeros(7, dtype=np.float32)

        # Ego camera fx at current zoom-independent setting (K[0,0] from camera_info).
        # effective_hfov = 2 * atan2(W/2, fx * zoom). Populated by _camera_info_callback.
        self._camera_fx: float = 0.0

        # Cached state for ego + all peers
        self._states: dict[str, VehicleState] = {
            name: VehicleState() for name in self._all_names
        }

        # Triangulation state
        self._tri_state = TriangulationState()

        # QoS: BEST_EFFORT for sensor-rate topics (gimbal, ang_vel, zoom)
        self._sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create subscriptions
        self._subscriptions = []

        # Ego subscriptions — relative topics (resolved by node namespace)
        self._create_ego_subscriptions(use_common_frame)

        # Peer subscriptions — absolute topics
        for peer in peer_names:
            self._create_peer_subscriptions(peer, use_common_frame)

        # Triangulation subscription (per-vehicle: sort3d runs inside each vehicle namespace).
        # Relative topic resolves to /{ego_name}/chosen_target_pose.
        if enable_triangulation:
            sub = node.create_subscription(
                PoseWithCovarianceStamped,
                'chosen_target_pose',
                self._triangulation_callback,
                10,
            )
            self._subscriptions.append(sub)

    def _create_ego_subscriptions(self, use_common_frame: bool):
        """Create subscriptions for the ego vehicle using relative topics."""
        node = self._node
        ego = self._ego_name

        # Odometry
        if use_common_frame:
            topic = 'common_frame/odom'
        else:
            topic = 'mavros/local_position/odom'
        sub = node.create_subscription(
            Odometry, topic,
            lambda msg, v=ego: self._odom_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # IMU (BEST_EFFORT to match MAVROS publisher)
        sub = node.create_subscription(
            Imu, 'mavros/imu/data',
            lambda msg, v=ego: self._imu_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Gimbal state (BEST_EFFORT to match los_rate_controller/siyi publisher)
        sub = node.create_subscription(
            Vector3, 'gimbal_state_rpy_deg',
            lambda msg, v=ego: self._gimbal_state_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # YOLO detections (BEST_EFFORT to match ultralytics_ros sensor QoS)
        sub = node.create_subscription(
            Detection2DArray, 'yolo_result_vision',
            lambda msg, v=ego: self._detection_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Zoom level (BEST_EFFORT to match publisher)
        sub = node.create_subscription(
            Float64, 'camera/zoom_level',
            lambda msg, v=ego: self._zoom_level_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Combined angular velocity (ego, from los_rate_controller / siyi_ros_node)
        sub = node.create_subscription(
            Vector3Stamped, 'combined_ang_vel_w',
            lambda msg, v=ego: self._peer_combined_ang_vel_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Pre-selected target ray (from tracker, for bearing-ray observation)
        sub = node.create_subscription(
            Vector3Stamped, 'chosen_target_ray_w',
            lambda msg, v=ego: self._chosen_target_ray_callback(msg, v), 10,
        )
        self._subscriptions.append(sub)

        # Camera info (for bbox normalization — adapts to actual camera resolution)
        sub = node.create_subscription(
            CameraInfo, 'camera/color/camera_info',
            self._camera_info_callback, 10,
        )
        self._subscriptions.append(sub)

    def _create_peer_subscriptions(self, peer: str, use_common_frame: bool):
        """Create subscriptions for a peer vehicle using absolute topics."""
        node = self._node

        # Odometry (for position, velocity, orientation)
        if use_common_frame:
            topic = f'/{peer}/common_frame/odom'
        else:
            topic = f'/{peer}/mavros/local_position/odom'
        sub = node.create_subscription(
            Odometry, topic,
            lambda msg, v=peer: self._odom_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Combined angular velocity (pre-computed, cross-agent, BEST_EFFORT)
        sub = node.create_subscription(
            Vector3Stamped, f'/{peer}/combined_ang_vel_w',
            lambda msg, v=peer: self._peer_combined_ang_vel_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Detection active (compact Bool for bbox_empty, cross-agent, BEST_EFFORT)
        sub = node.create_subscription(
            Bool, f'/{peer}/yolo_result_active',
            lambda msg, v=peer: self._peer_detection_active_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Zoom level (cross-agent, BEST_EFFORT)
        sub = node.create_subscription(
            Float64, f'/{peer}/camera/zoom_level',
            lambda msg, v=peer: self._zoom_level_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Pre-selected target ray (cross-agent, from peer's tracker)
        sub = node.create_subscription(
            Vector3Stamped, f'/{peer}/chosen_target_ray_w',
            lambda msg, v=peer: self._chosen_target_ray_callback(msg, v), 10,
        )
        self._subscriptions.append(sub)

    # --- Callbacks (shared for ego and peers) ---

    def _odom_callback(self, msg: Odometry, veh: str):
        """Cache odometry data."""
        state = self._states[veh]

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ori = msg.pose.pose.orientation
        ang = msg.twist.twist.angular

        state.position_w = np.array([pos.x, pos.y, pos.z])
        state.velocity_w = np.array([vel.x, vel.y, vel.z])
        # ROS quaternion is (x,y,z,w), convert to (w,x,y,z)
        state.orientation_w = np.array([ori.w, ori.x, ori.y, ori.z])
        state.angular_velocity_b = np.array([ang.x, ang.y, ang.z])
        state.motion_timestamp = self._get_time()
        state.odom_received = True

    def _imu_callback(self, msg: Imu, veh: str):
        """Cache IMU linear acceleration (body frame). Ego only."""
        state = self._states[veh]
        acc = msg.linear_acceleration
        state.linear_acceleration_b = np.array([acc.x, acc.y, acc.z])

    def _gimbal_state_callback(self, msg: Vector3, veh: str):
        """Cache gimbal state (body-frame, deg → rad).

        The gimbal_state_rpy_deg topic provides body-frame angles with
        yaw=0 meaning forward. No offset subtraction needed.
        """
        state = self._states[veh]
        state.gimbal_yaw_body = wrap_to_pi(math.radians(msg.z))
        state.gimbal_pitch_body = math.radians(msg.y)
        state.gimbal_timestamp = self._get_time()

    def _detection_callback(self, msg: Detection2DArray, veh: str):
        """Cache YOLO detection bbox."""
        state = self._states[veh]

        if len(msg.detections) > 0:
            det = msg.detections[0]
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            state.bbox_xywh = np.array([
                cx / self._image_width,
                cy / self._image_height,
                w / self._image_width,
                h / self._image_height,
            ])
            state.bbox_empty = 0.0
        else:
            state.bbox_xywh = np.zeros(4)
            state.bbox_empty = 1.0

        state.detection_timestamp = self._get_time()

    def _zoom_level_callback(self, msg: Float64, veh: str):
        """Cache zoom level, clamped to training domain [1.0, 5.0].

        Training-time `zoom_max=5.0` is the trust region of the SIYI mrcal
        calibration; values above 5 are OOD for the policy.
        """
        self._states[veh].zoom_level = max(_ZOOM_CMD_MIN, min(_ZOOM_CMD_MAX, float(msg.data)))

    def _peer_combined_ang_vel_callback(self, msg: Vector3Stamped, veh: str):
        """Cache peer pre-computed combined angular velocity in world frame."""
        state = self._states[veh]
        state.combined_ang_vel_w = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _peer_detection_active_callback(self, msg: Bool, veh: str):
        """Cache peer detection active state (compact cross-agent topic)."""
        state = self._states[veh]
        state.bbox_empty = 0.0 if msg.data else 1.0
        state.detection_timestamp = self._get_time()

    def _chosen_target_ray_callback(self, msg: Vector3Stamped, veh: str):
        """Cache pre-selected target bearing ray from tracker."""
        self._states[veh].chosen_target_ray_w = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _camera_info_callback(self, msg: CameraInfo):
        """Update bbox normalization dimensions + fx from actual camera_info."""
        if msg.width > 0 and msg.height > 0:
            if msg.width != self._image_width or msg.height != self._image_height:
                logger.info(
                    f"Camera resolution updated: {self._image_width}x{self._image_height} "
                    f"→ {msg.width}x{msg.height}"
                )
                self._image_width = msg.width
                self._image_height = msg.height
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]; fx is the zoom-independent (1x) focal length
        fx = float(msg.k[0])
        if fx > 0.0:
            self._camera_fx = fx

    def _triangulation_callback(self, msg: PoseWithCovarianceStamped):
        """Cache triangulation result. Presence-of-message = validity (matches training).

        Training's triangulation tail is `sqrt(clamp(cov_diag, min=1e-12))` whenever
        `is_valid` is True, so a near-zero covariance is *still* valid and yields a
        near-zero std. mas_tracker only publishes `chosen_target_pose` when a target
        is tracked, so message arrival is the authoritative validity signal. Treat
        NaN/negative variances as the only "invalid" condition.
        """
        pos = msg.pose.pose.position
        self._tri_state.position = np.array([pos.x, pos.y, pos.z])

        cov = msg.pose.covariance
        variances = np.array([cov[0], cov[7], cov[14]])

        if np.all(np.isfinite(variances)) and np.all(variances >= 0.0):
            # Match training: clamp to 1e-12 before sqrt so std is well-defined
            # even when the upstream triangulator reports zero covariance.
            self._tri_state.std_dev = np.sqrt(np.clip(variances, 1e-12, None))
            self._tri_state.is_valid = True
        else:
            self._tri_state.std_dev = np.full(3, -1.0)
            self._tri_state.is_valid = False

        self._tri_state.timestamp = self._get_time()

    def _get_time(self) -> float:
        """Get current ROS time as float seconds."""
        return self._node.get_clock().now().nanoseconds / 1e9

    # --- Assembly ---

    def assemble(self) -> np.ndarray:
        """Assemble the observation vector for the ego agent.

        Returns:
            Observation array of shape (obs_dim,).
        """
        now = self._get_time()
        ego = self._states[self._ego_name]

        # --- Ego observation (31D) ---
        roll, pitch, yaw = euler_xyz_from_quat(ego.orientation_w)
        euler_rpy = wrap_to_pi(np.array([roll, pitch, yaw]))

        # Gimbal angles are body-frame with 0=forward (topic already offset-corrected)
        gimbal_yaw_obs = ego.gimbal_yaw_body
        gimbal_pitch_obs = ego.gimbal_pitch_body

        # Use pre-selected target ray from tracker; zero when unavailable.
        # Gate by bbox_empty so deployment matches IL training: in training
        # the ray is computed from the bbox itself, so bbox_empty=1 ⇒ ray=0.
        # The SORT tracker can keep emitting a non-zero ray after YOLO drops a
        # frame, which would create an OOD (bbox_empty=1, ray≠0) combination
        # the policy never saw at training time.
        if ego.bbox_empty == 0 and ego.chosen_target_ray_w is not None:
            ray_w = ego.chosen_target_ray_w
        else:
            ray_w = np.zeros(3)

        # Combined angular velocity from dedicated topic (same source as peers)
        combined_ang_vel_w = ego.combined_ang_vel_w

        bbox_aoi = now - ego.detection_timestamp if ego.detection_timestamp > 0 else 0.0
        # bbox_aoi = random.uniform(0.0, 0.04)
        # bbox_aoi = 0.0
        bbox_aoi = min(bbox_aoi, self._max_bbox_aoi)

        # effective_hfov uses the SIYI A8 measured zoom curve, NOT a linear
        # fx*zoom multiplier. Matches training: fx_eff = fx_base * z_eff(zoom).
        # (Training: iris_ma_env6_test.py:1071-1078 with dr_scale=1 at deploy.)
        z_eff = _compute_z_eff(ego.zoom_level)
        fx_eff = self._camera_fx * z_eff
        if fx_eff > 0.0:
            effective_hfov = 2.0 * math.atan2(self._image_width * 0.5, fx_eff)
        else:
            effective_hfov = 0.0

        # MAVROS IMU reports specific force (proper accel = a_kinematic − g_world) in body
        # frame, while training uses kinematic body acc (zero at hover). Add g_world→body
        # to recover the kinematic acceleration expected by the policy.
        lin_acc_kinematic_b = ego.linear_acceleration_b + quat_rotate_inverse(
            ego.orientation_w, _GRAVITY_WORLD_ENU
        )

        ego_obs = np.concatenate([
            ego.position_w,                         # 0-2: position (3)
            ego.velocity_w,                         # 3-5: velocity (3)
            euler_rpy,                              # 6-8: euler RPY (3)
            ego.angular_velocity_b,                 # 9-11: angular vel body (3)
            lin_acc_kinematic_b,                    # 12-14: kinematic lin acc body (3)
            np.array([gimbal_yaw_obs]),             # 15: gimbal yaw body (1)
            np.array([gimbal_pitch_obs]),           # 16: gimbal pitch body (1)
            ray_w,                                  # 17-19: ray direction world (3)
            combined_ang_vel_w,                     # 20-22: combined ang vel world (3)
            np.array([bbox_aoi]),                   # 23: bbox AoI (1)
            np.array([ego.zoom_level]),             # 24: zoom level (1)
            np.array([effective_hfov]),             # 25: effective HFOV rad (1)
            ego.bbox_xywh,                          # 26-29: bbox normalized (4)
            np.array([ego.bbox_empty]),             # 30: bbox empty (1)
        ])

        # --- Prev-action tail (7D, ticket 043) ---
        # Appended to the ego block (before peers), matching training where
        # _cmd_vel_filt is cat'd into ego_obs_parts in _get_observations.
        if self._enable_prev_action_obs:
            ego_obs = np.concatenate([ego_obs, self._prev_action_obs])

        # --- Inter-agent observations (16D per peer) ---
        other_obs_parts = []
        for peer in self._peer_names:
            other = self._states[peer]

            # Use pre-selected target ray from peer's tracker; zero when
            # unavailable. Gate by peer bbox_empty so deployment matches IL
            # training (ray is bbox-derived in training, so bbox_empty=1
            # implies ray=0).
            if other.bbox_empty == 0 and other.chosen_target_ray_w is not None:
                other_ray_w = other.chosen_target_ray_w
            else:
                other_ray_w = np.zeros(3)
            # Peer combined_ang_vel: use pre-computed value from dedicated topic
            other_combined_ang_vel_w = other.combined_ang_vel_w
            data_age = now - other.motion_timestamp if other.motion_timestamp > 0 else 0.0
            bbox_age = now - other.detection_timestamp if other.detection_timestamp > 0 else 0.0

            other_obs = np.concatenate([
                other.position_w,                       # 0-2: position (3)
                other.velocity_w,                       # 3-5: velocity (3)
                other_ray_w,                            # 6-8: ray direction (3)
                other_combined_ang_vel_w,               # 9-11: combined ang vel (3)
                np.array([other.zoom_level]),            # 12: zoom (1)
                np.array([other.bbox_empty]),            # 13: bbox empty (1)
                np.array([data_age]),                    # 14: data age (1)
                np.array([bbox_age]),                    # 15: bbox age (1)
            ])
            other_obs_parts.append(other_obs)

        full_obs = np.concatenate([ego_obs] + other_obs_parts)

        # --- Optional triangulation tail (6D) ---
        if self._enable_triangulation:
            tri = self._tri_state
            if tri.is_valid:
                tri_obs = np.concatenate([tri.position, tri.std_dev])
            else:
                tri_obs = np.concatenate([np.zeros(3), np.full(3, -1.0)])
            full_obs = np.concatenate([full_obs, tri_obs])

        return full_obs

    def set_prev_action(self, cmd_vel: np.ndarray):
        """Set the previous-step cmd_vel for the prev-action obs tail (ticket 043).

        Args:
            cmd_vel: 7D array [vx,vy,vz (m/s), yaw_rate (rad/s), g_yaw, g_pitch,
                zoom (normalized)] — the command applied last control step, or
                zeros on episode entry.
        """
        self._prev_action_obs = np.asarray(cmd_vel, dtype=np.float32).copy()

    def get_vehicle_state(self, name: str) -> VehicleState:
        """Get cached state for any vehicle (ego or peer)."""
        return self._states[name]

    @property
    def ego_state(self) -> VehicleState:
        """Get cached state for the ego vehicle."""
        return self._states[self._ego_name]

    @property
    def all_names(self) -> list[str]:
        """All vehicle names: [ego] + peers."""
        return self._all_names

    @property
    def obs_dim(self) -> int:
        """Expected observation dimension."""
        dim = 31 + 16 * len(self._peer_names)
        if self._enable_prev_action_obs:
            dim += 7
        if self._enable_triangulation:
            dim += 6
        return dim
