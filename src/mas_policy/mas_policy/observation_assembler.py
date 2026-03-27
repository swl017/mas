"""Observation assembler: ROS2 topics → 62/68D observation vector.

Per-vehicle design: each policy_node instance runs in a vehicle namespace and
assembles observations for its own agent (ego). It subscribes to peer vehicles'
topics via absolute paths for inter-agent observations.

Observation structure:
- Ego (30D): pos(3), vel(3), euler_rpy(3), ang_vel_b(3), lin_acc_b(3),
  gimbal_yaw_body(1), gimbal_pitch_body(1), ray_dir_w(3),
  combined_ang_vel_w(3), bbox_aoi(1), zoom(1), bbox(4), bbox_empty(1)
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
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from vision_msgs.msg import Detection2DArray

from .utils import (
    euler_xyz_from_quat,
    gimbal_ray_direction_world,
    compute_combined_angular_velocity_world,
    wrap_to_pi,
)

logger = logging.getLogger(__name__)

# Default YAW_JOINT_OFFSET from training (gimbal_controller.py)
YAW_JOINT_OFFSET = -math.pi / 2


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

    # Gimbal state (from los_rate_controller)
    gimbal_yaw_body: float = 0.0   # radians, body-frame
    gimbal_pitch_body: float = 0.0  # radians
    gimbal_yaw_rate: float = 0.0   # estimated rate (rad/s)
    gimbal_pitch_rate: float = 0.0
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
        yaw_joint_offset: float = YAW_JOINT_OFFSET,
        enable_triangulation: bool = False,
        use_common_frame: bool = True,
    ):
        """Initialize observation assembler for a single ego vehicle.

        Args:
            node: ROS2 node (running inside the ego vehicle's namespace).
            ego_name: This vehicle's namespace (e.g., "px4_1").
            peer_names: Other vehicles' namespaces (e.g., ["px4_2", "px4_3"]).
            image_width: Camera image width for bbox normalization.
            image_height: Camera image height for bbox normalization.
            yaw_joint_offset: Gimbal yaw joint offset (default: -pi/2).
            enable_triangulation: Whether to include 6D triangulation tail.
            use_common_frame: If True, use common_frame/odom (ENU).
        """
        self._node = node
        self._ego_name = ego_name
        self._peer_names = peer_names
        self._all_names = [ego_name] + list(peer_names)
        self._image_width = image_width
        self._image_height = image_height
        self._yaw_joint_offset = yaw_joint_offset
        self._enable_triangulation = enable_triangulation
        self._use_common_frame = use_common_frame

        # Cached state for ego + all peers
        self._states: dict[str, VehicleState] = {
            name: VehicleState() for name in self._all_names
        }
        # Previous gimbal angles for rate estimation
        self._prev_gimbal: dict[str, tuple[float, float, float]] = {
            name: (0.0, 0.0, 0.0) for name in self._all_names
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

        # Triangulation subscription (global topic)
        if enable_triangulation:
            sub = node.create_subscription(
                PoseWithCovarianceStamped,
                '/chosen_target_pose',
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

        # IMU
        sub = node.create_subscription(
            Imu, 'mavros/imu/data',
            lambda msg, v=ego: self._imu_callback(msg, v), 10,
        )
        self._subscriptions.append(sub)

        # Gimbal state (BEST_EFFORT to match los_rate_controller/siyi publisher)
        sub = node.create_subscription(
            Vector3, 'gimbal_state_rpy_rad',
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
            Float32, 'zoom_level',
            lambda msg, v=ego: self._zoom_level_callback(msg, v),
            self._sensor_qos,
        )
        self._subscriptions.append(sub)

        # Pre-selected target ray (from tracker, for bearing-ray observation)
        sub = node.create_subscription(
            Vector3Stamped, 'chosen_target_ray_w',
            lambda msg, v=ego: self._chosen_target_ray_callback(msg, v), 10,
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
            Float32, f'/{peer}/zoom_level',
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
        """Cache gimbal state (body-frame, radians)."""
        state = self._states[veh]
        now = self._get_time()

        # msg: x=roll, y=pitch, z=yaw
        new_yaw = msg.z
        new_pitch = msg.y

        # Estimate gimbal rates from finite differences
        prev_yaw, prev_pitch, prev_t = self._prev_gimbal[veh]
        dt = now - prev_t
        if dt > 0.001:
            state.gimbal_yaw_rate = (new_yaw - prev_yaw) / dt
            state.gimbal_pitch_rate = (new_pitch - prev_pitch) / dt

        state.gimbal_yaw_body = new_yaw
        state.gimbal_pitch_body = new_pitch
        state.gimbal_timestamp = now
        self._prev_gimbal[veh] = (new_yaw, new_pitch, now)

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

    def _zoom_level_callback(self, msg: Float32, veh: str):
        """Cache zoom level from dedicated topic."""
        self._states[veh].zoom_level = float(msg.data)

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

    def _triangulation_callback(self, msg: PoseWithCovarianceStamped):
        """Cache triangulation result with covariance."""
        pos = msg.pose.pose.position
        self._tri_state.position = np.array([pos.x, pos.y, pos.z])

        cov = msg.pose.covariance
        var_x, var_y, var_z = cov[0], cov[7], cov[14]

        variances = np.array([var_x, var_y, var_z])
        if np.all(np.isfinite(variances)) and np.all(variances > 0):
            self._tri_state.std_dev = np.sqrt(variances)
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

        # --- Ego observation (30D) ---
        roll, pitch, yaw = euler_xyz_from_quat(ego.orientation_w)
        euler_rpy = wrap_to_pi(np.array([roll, pitch, yaw]))

        gimbal_yaw_obs = ego.gimbal_yaw_body - self._yaw_joint_offset
        gimbal_pitch_obs = ego.gimbal_pitch_body

        # Use pre-selected target ray from tracker, fall back to gimbal LOS
        if ego.chosen_target_ray_w is not None:
            ray_w = ego.chosen_target_ray_w
        else:
            ray_w = gimbal_ray_direction_world(
                np.array(gimbal_yaw_obs),
                np.array(gimbal_pitch_obs),
                ego.orientation_w,
            )
        combined_ang_vel_w = compute_combined_angular_velocity_world(
            ego.angular_velocity_b,
            ego.gimbal_pitch_rate,
            ego.gimbal_yaw_rate,
            ego.orientation_w,
        )
        bbox_aoi = now - ego.detection_timestamp if ego.detection_timestamp > 0 else 0.0

        ego_obs = np.concatenate([
            ego.position_w,                         # 0-2: position (3)
            ego.velocity_w,                         # 3-5: velocity (3)
            euler_rpy,                              # 6-8: euler RPY (3)
            ego.angular_velocity_b,                 # 9-11: angular vel body (3)
            ego.linear_acceleration_b,              # 12-14: linear acc body (3)
            np.array([gimbal_yaw_obs]),             # 15: gimbal yaw body (1)
            np.array([gimbal_pitch_obs]),           # 16: gimbal pitch body (1)
            ray_w,                                  # 17-19: ray direction world (3)
            combined_ang_vel_w,                     # 20-22: combined ang vel world (3)
            np.array([bbox_aoi]),                   # 23: bbox AoI (1)
            np.array([ego.zoom_level]),             # 24: zoom level (1)
            ego.bbox_xywh,                          # 25-28: bbox normalized (4)
            np.array([ego.bbox_empty]),             # 29: bbox empty (1)
        ])

        # --- Inter-agent observations (16D per peer) ---
        other_obs_parts = []
        for peer in self._peer_names:
            other = self._states[peer]

            # Use pre-selected target ray from peer's tracker, fall back to gimbal LOS
            if other.chosen_target_ray_w is not None:
                other_ray_w = other.chosen_target_ray_w
            else:
                other_gimbal_yaw = other.gimbal_yaw_body - self._yaw_joint_offset
                other_ray_w = gimbal_ray_direction_world(
                    np.array(other_gimbal_yaw),
                    np.array(other.gimbal_pitch_body),
                    other.orientation_w,
                )
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
        dim = 30 + 16 * len(self._peer_names)
        if self._enable_triangulation:
            dim += 6
        return dim
