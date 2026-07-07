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
    OffboardControlMode,
    SensorCombined,
    TrajectorySetpoint,
    VehicleControlMode,
    VehicleLocalPosition,
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
        # Last EKF origin (ref_lat, ref_lon, ref_alt) we have already published as
        # mavros home_position — used to latch one-shot republishes on origin change.
        self._last_ref_lla: tuple[float, float, float] | None = None
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
        # PX4's stock uXRCE-DDS topic whitelist does not include `home_position`,
        # so we derive the MAVROS home_position from `vehicle_local_position.ref_*`
        # (the EKF local-frame origin in LLH) — that is the same quantity
        # `mas_common_frame` consumes from `home_position.geo.*`.
        self.create_subscription(
            VehicleLocalPosition,
            f"{self.fmu_out}/vehicle_local_position",
            self._on_local_position,
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

        # Setpoint pipeline (ROS → PX4). Both velocity and position MAVROS
        # topics feed the same /fmu/in/trajectory_setpoint + offboard_control_mode
        # pair; the latest message on either topic wins. The OffboardControlMode
        # flags tell PX4 which fields of TrajectorySetpoint are authoritative,
        # so callers can freely alternate between position and velocity
        # commands without dropping out of OFFBOARD.
        self.create_subscription(
            TwistStamped,
            f"{self.mavros_ns}/setpoint_velocity/cmd_vel",
            self._on_cmd_vel,
            rel,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.mavros_ns}/setpoint_position/local",
            self._on_setpoint_position,
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
        # REP-147 aerial convention: linear in world ENU (deviates from nav_msgs/Odometry
        # child_frame doc string), angular in body FLU. Matches PX4 VehicleOdometry.
        odom.twist.twist.linear = self._vec_to_vector3(velocity_world_enu)
        odom.twist.twist.angular = self._vec_to_vector3(angular_velocity_flu)
        odom.twist.covariance = frames.velocity_local_covariance_ned_to_enu(msg.velocity_variance)
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

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        # Wait until the EKF declares both horizontal and vertical global refs valid.
        if not (msg.xy_global and msg.z_global):
            return

        ref_lla = (float(msg.ref_lat), float(msg.ref_lon), float(msg.ref_alt))
        # Latch: republish only when the EKF origin actually changes (e.g. on
        # EKF reset). vehicle_local_position arrives at ~100 Hz; we don't want to
        # spam the TRANSIENT_LOCAL home_position topic at that rate.
        if self._last_ref_lla == ref_lla:
            return
        self._last_ref_lla = ref_lla

        out = MavrosHomePosition()
        out.header.stamp = self._now_stamp()
        out.header.frame_id = self.frame_id_world
        out.geo = GeoPoint()
        out.geo.latitude = ref_lla[0]
        out.geo.longitude = ref_lla[1]
        out.geo.altitude = ref_lla[2]

        # The EKF reference IS the local-frame origin, so the home's local-frame
        # position is identically zero (in either NED or ENU).
        out.position = self._vec_to_point(np.zeros(3))

        # Real MAVROS leaves HOME_POSITION.orientation at identity (the heading
        # at the moment home was set is reported through `approach`, which we
        # also leave zero — PX4's uXRCE-DDS does not export that snapshot).
        # mas_common_frame ignores this field; identity is the safe default.
        out.orientation = Quaternion()
        out.orientation.w = 1.0
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
        # Do not set actuator / thrust_and_torque / direct_actuator: PX4 1.14 has
        # `actuator`, 1.15 split it into `thrust_and_torque` + `direct_actuator`.
        # All default to False in either schema, which is what we want.
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

    def _on_setpoint_position(self, msg: PoseStamped) -> None:
        position_enu = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        q_xyzw = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        position_ned, yaw_ned = frames.position_setpoint_enu_flu_to_ned(
            position_enu, q_xyzw
        )

        ts_us = int(self.get_clock().now().nanoseconds // 1000)

        ocm = OffboardControlMode()
        ocm.timestamp = ts_us
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.pub_offboard_mode.publish(ocm)

        nan = float("nan")
        ts = TrajectorySetpoint()
        ts.timestamp = ts_us
        ts.position = [float(position_ned[0]), float(position_ned[1]), float(position_ned[2])]
        ts.velocity = [nan, nan, nan]
        ts.acceleration = [nan, nan, nan]
        ts.jerk = [nan, nan, nan]
        ts.yaw = float(yaw_ned)
        ts.yawspeed = nan
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
