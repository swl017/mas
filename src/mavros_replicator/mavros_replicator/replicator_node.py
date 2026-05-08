"""mavros_replicator: PX4 (px4_msgs, NED/FRD) ↔ MAVROS-shaped (ENU/FLU) translator.

See [src/doc/mavros_replicator_spec.md](../../doc/mavros_replicator_spec.md).
"""
from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    QoSPresetProfiles,
    ReliabilityPolicy,
)

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TwistStamped,
    Vector3,
)
from mavros_msgs.msg import HomePosition as MavrosHomePosition, State as MavrosState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from px4_msgs.msg import (
    HomePosition as Px4HomePosition,
    OffboardControlMode,
    SensorCombined,
    TrajectorySetpoint,
    VehicleControlMode,
    VehicleOdometry,
    VehicleStatus,
)

from mavros_replicator import frames


# Subset of PX4 nav_state values mapped to MAVROS mode strings (see spec §7).
NAV_STATE_TO_MODE = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO.MISSION",
    4: "AUTO.LOITER",
    5: "AUTO.RTL",
    14: "OFFBOARD",
    17: "AUTO.TAKEOFF",
    18: "AUTO.LAND",
}
MANUAL_MODES = {"MANUAL", "ALTCTL", "POSCTL", "STABILIZED"}


def _px4_be_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _mavros_reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _mavros_latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class MavrosReplicator(Node):
    def __init__(self) -> None:
        super().__init__("mavros_replicator")

        env_robot = os.environ.get("ROBOT_NAME", "px4_1")
        self.declare_parameter("robot_name", env_robot)
        self.declare_parameter("frame_id_world", "")
        self.declare_parameter("frame_id_body", "")
        self.declare_parameter("setpoint_timeout_ms", 250)

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        fw = self.get_parameter("frame_id_world").get_parameter_value().string_value
        fb = self.get_parameter("frame_id_body").get_parameter_value().string_value
        self.frame_id_world = fw or f"{self.robot_name}/map"
        self.frame_id_body = fb or f"{self.robot_name}/base_link"

        self.fmu_in = f"/{self.robot_name}/fmu/in"
        self.fmu_out = f"/{self.robot_name}/fmu/out"
        self.mavros_ns = f"/{self.robot_name}/mavros"

        # Cached state for cross-callback composition.
        self._latest_q_enu_flu_xyzw: np.ndarray | None = None
        self._latest_status: VehicleStatus | None = None
        self._latest_status_recv_time: float = 0.0
        self._latest_control_mode: VehicleControlMode | None = None
        self._warned_pose_frame = False
        self._warned_velocity_frame = False
        self._warned_q_invalid = False

        # PX4 subscribers.
        be = _px4_be_qos()
        self.create_subscription(
            VehicleOdometry, f"{self.fmu_out}/vehicle_odometry", self._on_odometry, be
        )
        self.create_subscription(
            SensorCombined, f"{self.fmu_out}/sensor_combined", self._on_sensor_combined, be
        )
        self.create_subscription(
            VehicleStatus, f"{self.fmu_out}/vehicle_status", self._on_vehicle_status, be
        )
        self.create_subscription(
            VehicleControlMode,
            f"{self.fmu_out}/vehicle_control_mode",
            self._on_control_mode,
            be,
        )
        self.create_subscription(
            Px4HomePosition,
            f"{self.fmu_out}/home_position",
            self._on_home_position,
            be,
        )

        # MAVROS-shaped publishers.
        rel = _mavros_reliable_qos()
        self.pub_pose = self.create_publisher(
            PoseStamped, f"{self.mavros_ns}/local_position/pose", rel
        )
        self.pub_pose_cov = self.create_publisher(
            PoseWithCovarianceStamped, f"{self.mavros_ns}/local_position/pose_cov", rel
        )
        self.pub_velocity_local = self.create_publisher(
            TwistStamped, f"{self.mavros_ns}/local_position/velocity_local", rel
        )
        self.pub_odom = self.create_publisher(
            Odometry, f"{self.mavros_ns}/local_position/odom", rel
        )
        self.pub_imu = self.create_publisher(Imu, f"{self.mavros_ns}/imu/data", rel)
        self.pub_state = self.create_publisher(MavrosState, f"{self.mavros_ns}/state", rel)
        self.pub_home = self.create_publisher(
            MavrosHomePosition,
            f"{self.mavros_ns}/home_position/home",
            _mavros_latched_qos(),
        )

        # Setpoint pipeline (ROS → PX4).
        self.create_subscription(
            TwistStamped,
            f"{self.mavros_ns}/setpoint_velocity/cmd_vel",
            self._on_cmd_vel,
            rel,
        )
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, f"{self.fmu_in}/offboard_control_mode", be
        )
        self.pub_trajectory_setpoint = self.create_publisher(
            TrajectorySetpoint, f"{self.fmu_in}/trajectory_setpoint", be
        )

        # State republishing timer (5 Hz) — drives `connected` freshness.
        self.create_timer(0.2, self._publish_state)

        self.get_logger().info(
            f"mavros_replicator up: robot={self.robot_name} "
            f"in={self.fmu_out}/* out={self.mavros_ns}/*"
        )

    # --- helpers -------------------------------------------------------------

    def _now_stamp(self):
        return self.get_clock().now().to_msg()

    @staticmethod
    def _quat_xyzw_to_msg(q: np.ndarray) -> Quaternion:
        m = Quaternion()
        m.x, m.y, m.z, m.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return m

    @staticmethod
    def _vec_to_point(v: np.ndarray) -> Point:
        m = Point()
        m.x, m.y, m.z = float(v[0]), float(v[1]), float(v[2])
        return m

    @staticmethod
    def _vec_to_vector3(v: np.ndarray) -> Vector3:
        m = Vector3()
        m.x, m.y, m.z = float(v[0]), float(v[1]), float(v[2])
        return m

    # --- callbacks -----------------------------------------------------------

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            if not self._warned_pose_frame:
                self.get_logger().warn(
                    f"vehicle_odometry.pose_frame={msg.pose_frame}, expected NED({VehicleOdometry.POSE_FRAME_NED}). "
                    "Treating as NED anyway."
                )
                self._warned_pose_frame = True
        if msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            if not self._warned_velocity_frame:
                self.get_logger().warn(
                    f"vehicle_odometry.velocity_frame={msg.velocity_frame}, expected NED({VehicleOdometry.VELOCITY_FRAME_NED}). "
                    "Treating as NED anyway."
                )
                self._warned_velocity_frame = True

        # PX4 marks invalid pose by setting q[0]=NaN.
        q_wxyz = np.asarray(msg.q, dtype=np.float64)
        if math.isnan(q_wxyz[0]):
            if not self._warned_q_invalid:
                self.get_logger().warn("vehicle_odometry.q invalid (NaN) — skipping pose publish")
                self._warned_q_invalid = True
            return

        q_enu_flu = frames.attitude_px4_to_ros_xyzw(q_wxyz)
        self._latest_q_enu_flu_xyzw = q_enu_flu

        position_enu = frames.position_ned_to_enu(msg.position)
        velocity_world_enu = frames.vector_world_ned_to_enu(msg.velocity)
        # Body-frame angular velocity from PX4 is in FRD (per VehicleOdometry.msg comment).
        angular_velocity_flu = frames.vector_body_frd_to_flu(msg.angular_velocity)
        # World ENU → body FLU for odom.twist.linear.
        R_world_body = frames.quat_to_matrix_xyzw(q_enu_flu)
        velocity_body_flu = R_world_body.T @ velocity_world_enu

        stamp = self._now_stamp()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id_world
        pose.pose.position = self._vec_to_point(position_enu)
        pose.pose.orientation = self._quat_xyzw_to_msg(q_enu_flu)
        self.pub_pose.publish(pose)

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = pose.header
        pose_cov.pose.pose = pose.pose
        pose_cov.pose.covariance = frames.pose_covariance_ned_to_enu(
            msg.position_variance, msg.orientation_variance
        )
        self.pub_pose_cov.publish(pose_cov)

        vel_local = TwistStamped()
        vel_local.header.stamp = stamp
        vel_local.header.frame_id = self.frame_id_world
        vel_local.twist.linear = self._vec_to_vector3(velocity_world_enu)
        vel_local.twist.angular = self._vec_to_vector3(angular_velocity_flu)
        self.pub_velocity_local.publish(vel_local)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id_world
        odom.child_frame_id = self.frame_id_body
        odom.pose.pose = pose.pose
        odom.pose.covariance = pose_cov.pose.covariance
        odom.twist.twist.linear = self._vec_to_vector3(velocity_body_flu)
        odom.twist.twist.angular = self._vec_to_vector3(angular_velocity_flu)
        odom.twist.covariance = frames.odom_twist_covariance(msg.velocity_variance, q_enu_flu)
        self.pub_odom.publish(odom)

    def _on_sensor_combined(self, msg: SensorCombined) -> None:
        if self._latest_q_enu_flu_xyzw is None:
            return  # wait for first attitude before publishing IMU

        gyro_flu = frames.vector_body_frd_to_flu(msg.gyro_rad)
        accel_flu = frames.vector_body_frd_to_flu(msg.accelerometer_m_s2)

        imu = Imu()
        imu.header.stamp = self._now_stamp()
        imu.header.frame_id = self.frame_id_body
        imu.orientation = self._quat_xyzw_to_msg(self._latest_q_enu_flu_xyzw)
        imu.angular_velocity = self._vec_to_vector3(gyro_flu)
        imu.linear_acceleration = self._vec_to_vector3(accel_flu)
        # Covariances are zero — PX4 does not publish per-sample IMU covariance,
        # matching the practical content of MAVROS's imu/data.
        self.pub_imu.publish(imu)

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._latest_status = msg
        self._latest_status_recv_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_control_mode(self, msg: VehicleControlMode) -> None:
        self._latest_control_mode = msg

    def _publish_state(self) -> None:
        if self._latest_status is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        connected = (now - self._latest_status_recv_time) < 1.0
        nav_state = int(self._latest_status.nav_state)
        mode = NAV_STATE_TO_MODE.get(nav_state, f"NAV_STATE_{nav_state}")
        armed = self._latest_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        st = MavrosState()
        st.header.stamp = self._now_stamp()
        st.connected = connected
        st.armed = bool(armed)
        st.guided = mode == "OFFBOARD"
        st.manual_input = mode in MANUAL_MODES
        st.mode = mode
        st.system_status = 0  # PX4 doesn't expose MAVLink MAV_STATE; leave UNINIT
        self.pub_state.publish(st)

    def _on_home_position(self, msg: Px4HomePosition) -> None:
        if not (msg.valid_hpos and msg.valid_lpos):
            return  # do not publish until PX4 gives a valid home (per spec §12)

        out = MavrosHomePosition()
        out.header.stamp = self._now_stamp()
        out.header.frame_id = self.frame_id_world
        out.geo = GeoPoint()
        out.geo.latitude = float(msg.lat)
        out.geo.longitude = float(msg.lon)
        out.geo.altitude = float(msg.alt) if msg.valid_alt else 0.0

        # Local position is NED in PX4 — convert to ENU.
        position_enu = frames.position_ned_to_enu([msg.x, msg.y, msg.z])
        out.position = self._vec_to_point(position_enu)

        # Build orientation from PX4 home yaw (NED) → ENU yaw quaternion (FLU body, level).
        # NED yaw ψ measured CW from north (i.e., about +Z down). ENU yaw ψ' = π/2 - ψ
        # (rotate from "from north CW" to "from east CCW"). Build q from yaw only.
        yaw_enu = math.pi / 2.0 - float(msg.yaw)
        out.orientation = Quaternion()
        out.orientation.z = math.sin(yaw_enu / 2.0)
        out.orientation.w = math.cos(yaw_enu / 2.0)
        # approach: not provided by PX4 home_position → leave zero.
        self.pub_home.publish(out)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        linear_enu = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        yawspeed_flu = msg.twist.angular.z
        linear_ned, yawspeed_ned = frames.velocity_setpoint_enu_flu_to_ned(
            linear_enu, yawspeed_flu
        )

        ts_us = int(self.get_clock().now().nanoseconds // 1000)

        ocm = OffboardControlMode()
        ocm.timestamp = ts_us
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.pub_offboard_mode.publish(ocm)

        nan = float("nan")
        ts = TrajectorySetpoint()
        ts.timestamp = ts_us
        ts.position = [nan, nan, nan]
        ts.velocity = [float(linear_ned[0]), float(linear_ned[1]), float(linear_ned[2])]
        ts.acceleration = [nan, nan, nan]
        ts.jerk = [nan, nan, nan]
        ts.yaw = nan
        ts.yawspeed = float(yawspeed_ned)
        self.pub_trajectory_setpoint.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosReplicator()
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
