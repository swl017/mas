"""Observation assembler: ROS2 topics → 62/68D observation vector.

Subscribes to per-vehicle ROS2 topics, caches latest data asynchronously,
and assembles the observation vector matching the exact ordering from
iris_ma_env6_test.py _get_observations() (lines 1464-1657).

Observation structure per agent:
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
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection2DArray

from .utils import (
    euler_xyz_from_quat,
    gimbal_ray_direction_world,
    compute_combined_angular_velocity_world,
    wrap_to_pi,
    ned_to_enu_position,
    ned_to_enu_velocity,
    frd_to_flu_angular_velocity,
    quat_ned_frd_to_enu_flu,
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
    gimbal_yaw_body: float = 0.0   # radians, body-frame (no offset applied yet)
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
    """Assembles observation vectors from cached ROS2 topic data.

    Creates subscriptions on the provided ROS2 node and caches data
    asynchronously. The assemble() method reads cached data to build
    the observation vector at policy frequency.
    """

    def __init__(
        self,
        node: Node,
        vehicle_names: list[str],
        image_width: int = 640,
        image_height: int = 480,
        yaw_joint_offset: float = YAW_JOINT_OFFSET,
        enable_triangulation: bool = False,
        use_common_frame: bool = True,
    ):
        """Initialize observation assembler.

        Args:
            node: ROS2 node to create subscriptions on.
            vehicle_names: List of vehicle namespace prefixes (e.g., ["px4_1", "px4_2"]).
            image_width: Camera image width for bbox normalization.
            image_height: Camera image height for bbox normalization.
            yaw_joint_offset: Gimbal yaw joint offset (default: -pi/2).
            enable_triangulation: Whether to include 6D triangulation tail.
            use_common_frame: If True, use common_frame/odom (ENU). If False,
                use fmu/out/vehicle_odometry (NED, converted).
        """
        self._node = node
        self._vehicle_names = vehicle_names
        self._image_width = image_width
        self._image_height = image_height
        self._yaw_joint_offset = yaw_joint_offset
        self._enable_triangulation = enable_triangulation
        self._use_common_frame = use_common_frame

        # Per-vehicle cached state
        self._states: dict[str, VehicleState] = {
            veh: VehicleState() for veh in vehicle_names
        }
        # Previous gimbal angles for rate estimation
        self._prev_gimbal: dict[str, tuple[float, float, float]] = {
            veh: (0.0, 0.0, 0.0) for veh in vehicle_names  # (yaw, pitch, timestamp)
        }

        # Triangulation state (shared across agents)
        self._tri_state = TriangulationState()

        # Create subscriptions for each vehicle
        self._subscriptions = []
        for veh in vehicle_names:
            self._create_vehicle_subscriptions(veh)

        # Triangulation subscription
        if enable_triangulation:
            sub = node.create_subscription(
                PoseStamped,
                'chosen_target_pose',
                self._triangulation_callback,
                10,
            )
            self._subscriptions.append(sub)

    def _create_vehicle_subscriptions(self, veh: str):
        """Create all subscriptions for a single vehicle."""
        node = self._node

        if self._use_common_frame:
            # Common frame odometry (already in ENU)
            sub = node.create_subscription(
                Odometry,
                f'/{veh}/common_frame/odom',
                lambda msg, v=veh: self._odom_callback(msg, v),
                10,
            )
            self._subscriptions.append(sub)
        else:
            # PX4 odometry (NED, needs conversion)
            sub = node.create_subscription(
                Odometry,
                f'/{veh}/mavros/local_position/odom',
                lambda msg, v=veh: self._odom_callback(msg, v),
                10,
            )
            self._subscriptions.append(sub)

        # IMU for body-frame linear acceleration
        sub = node.create_subscription(
            Imu,
            f'/{veh}/mavros/imu/data',
            lambda msg, v=veh: self._imu_callback(msg, v),
            10,
        )
        self._subscriptions.append(sub)

        # Gimbal state from los_rate_controller (body-frame, radians)
        sub = node.create_subscription(
            Vector3,
            f'/{veh}/gimbal_state_rpy_rad',
            lambda msg, v=veh: self._gimbal_state_callback(msg, v),
            10,
        )
        self._subscriptions.append(sub)

        # YOLO detections
        sub = node.create_subscription(
            Detection2DArray,
            f'/{veh}/yolo_result_vision',
            lambda msg, v=veh: self._detection_callback(msg, v),
            10,
        )
        self._subscriptions.append(sub)

    def _odom_callback(self, msg: Odometry, veh: str):
        """Cache odometry data."""
        state = self._states[veh]

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ori = msg.pose.pose.orientation
        ang = msg.twist.twist.angular

        if self._use_common_frame:
            # Already ENU
            state.position_w = np.array([pos.x, pos.y, pos.z])
            state.velocity_w = np.array([vel.x, vel.y, vel.z])
            # ROS quaternion is (x,y,z,w), convert to (w,x,y,z)
            state.orientation_w = np.array([ori.w, ori.x, ori.y, ori.z])
            state.angular_velocity_b = np.array([ang.x, ang.y, ang.z])
        else:
            # MAVROS local_position/odom is in ENU/FLU already
            state.position_w = np.array([pos.x, pos.y, pos.z])
            state.velocity_w = np.array([vel.x, vel.y, vel.z])
            state.orientation_w = np.array([ori.w, ori.x, ori.y, ori.z])
            # Body angular velocity from twist is in body frame
            state.angular_velocity_b = np.array([ang.x, ang.y, ang.z])

        state.motion_timestamp = self._get_time()
        state.odom_received = True

    def _imu_callback(self, msg: Imu, veh: str):
        """Cache IMU linear acceleration (body frame)."""
        state = self._states[veh]
        acc = msg.linear_acceleration
        state.linear_acceleration_b = np.array([acc.x, acc.y, acc.z])

    def _gimbal_state_callback(self, msg: Vector3, veh: str):
        """Cache gimbal state from los_rate_controller (body-frame, radians)."""
        state = self._states[veh]
        now = self._get_time()

        # msg: x=roll, y=pitch, z=yaw (body-frame radians)
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

            # Normalize to [0, 1]
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

    def _triangulation_callback(self, msg: PoseStamped):
        """Cache triangulation result."""
        pos = msg.pose.position
        self._tri_state.position = np.array([pos.x, pos.y, pos.z])
        # Use a fixed uncertainty when not available from the message
        self._tri_state.std_dev = np.array([1.0, 1.0, 1.0])
        self._tri_state.is_valid = True
        self._tri_state.timestamp = self._get_time()

    def _get_time(self) -> float:
        """Get current ROS time as float seconds."""
        return self._node.get_clock().now().nanoseconds / 1e9

    def assemble(self) -> dict[str, np.ndarray]:
        """Assemble observation vectors for all agents.

        Returns:
            Dictionary mapping vehicle name to observation array (obs_dim,).
        """
        now = self._get_time()
        obs = {}

        for veh in self._vehicle_names:
            ego = self._states[veh]

            # --- Ego observation (30D) ---
            # [0-2] position
            # [3-5] velocity
            # [6-8] euler RPY
            roll, pitch, yaw = euler_xyz_from_quat(ego.orientation_w)
            euler_rpy = wrap_to_pi(np.array([roll, pitch, yaw]))

            # [9-11] angular velocity body
            # [12-14] linear acceleration body
            # [15] gimbal yaw body (with offset subtracted, 0=forward)
            gimbal_yaw_obs = ego.gimbal_yaw_body - self._yaw_joint_offset
            # [16] gimbal pitch body
            gimbal_pitch_obs = ego.gimbal_pitch_body
            # [17-19] camera ray direction world
            ray_w = gimbal_ray_direction_world(
                np.array(gimbal_yaw_obs),
                np.array(gimbal_pitch_obs),
                ego.orientation_w,
            )
            # [20-22] combined angular velocity world
            combined_ang_vel_w = compute_combined_angular_velocity_world(
                ego.angular_velocity_b,
                ego.gimbal_pitch_rate,
                ego.gimbal_yaw_rate,
                ego.orientation_w,
            )
            # [23] bbox age of information
            bbox_aoi = now - ego.detection_timestamp if ego.detection_timestamp > 0 else 0.0
            # [24] zoom level
            # [25-28] bbox normalized
            # [29] bbox empty

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

            # --- Inter-agent observations (16D per other agent) ---
            other_obs_parts = []
            for other_veh in self._vehicle_names:
                if other_veh == veh:
                    continue
                other = self._states[other_veh]

                # Other agent's gimbal ray direction
                other_gimbal_yaw = other.gimbal_yaw_body - self._yaw_joint_offset
                other_ray_w = gimbal_ray_direction_world(
                    np.array(other_gimbal_yaw),
                    np.array(other.gimbal_pitch_body),
                    other.orientation_w,
                )

                # Other's combined angular velocity
                other_combined_ang_vel_w = compute_combined_angular_velocity_world(
                    other.angular_velocity_b,
                    other.gimbal_pitch_rate,
                    other.gimbal_yaw_rate,
                    other.orientation_w,
                )

                # Data ages
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

            # Concatenate ego + all others
            full_obs = np.concatenate([ego_obs] + other_obs_parts)

            # --- Optional triangulation tail (6D) ---
            if self._enable_triangulation:
                tri = self._tri_state
                if tri.is_valid:
                    tri_obs = np.concatenate([tri.position, tri.std_dev])
                else:
                    tri_obs = np.concatenate([np.zeros(3), np.full(3, -1.0)])
                full_obs = np.concatenate([full_obs, tri_obs])

            obs[veh] = full_obs

        return obs

    def get_vehicle_state(self, veh: str) -> VehicleState:
        """Get cached state for a vehicle (for CBF filter use)."""
        return self._states[veh]

    @property
    def obs_dim(self) -> int:
        """Expected observation dimension."""
        n_agents = len(self._vehicle_names)
        dim = 30 + 16 * (n_agents - 1)
        if self._enable_triangulation:
            dim += 6
        return dim
