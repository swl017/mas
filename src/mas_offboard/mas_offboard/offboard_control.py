"""OffboardControl node — per-vehicle offboard controller for PX4 via MAVROS.

State machine: INIT → RAMP_UP → ARM → TAKEOFF → HOVER → POLICY

All coordinates are ENU-FLU (MAVROS handles NED↔ENU conversion).
"""

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class FlightState(Enum):
    INIT = auto()
    RAMP_UP = auto()
    ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    POLICY = auto()


def _yaw_from_quat(q) -> float:
    """Extract yaw (rad) from geometry_msgs Quaternion (ENU-FLU)."""
    # yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Signed shortest angular difference (a - b) in degrees, range [-180, 180]."""
    d = (a_deg - b_deg) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


class OffboardControl(Node):
    """Per-vehicle offboard controller using MAVROS."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # -- Parameters --
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_name', ''),
                ('update_rate', 100.0),
                ('target_system', 1),
                ('position.x', 0.0),
                ('position.y', 0.0),
                ('position.z', 0.0),
                ('position.yaw_deg', 0.0),
                ('takeoff_speed', 3.0),
            ],
        )

        self.vehicle_name: str = self.get_parameter('vehicle_name').value
        self.update_rate: float = self.get_parameter('update_rate').value
        self.target_system: int = self.get_parameter('target_system').value
        self.takeoff_speed: float = self.get_parameter('takeoff_speed').value

        wp_x: float = self.get_parameter('position.x').value
        wp_y: float = self.get_parameter('position.y').value
        wp_z: float = self.get_parameter('position.z').value
        wp_yaw_deg: float = self.get_parameter('position.yaw_deg').value
        wp_yaw_rad: float = math.radians(wp_yaw_deg)

        self.get_logger().info(
            f'vehicle_name={self.vehicle_name}  target_system={self.target_system}  '
            f'waypoint=({wp_x}, {wp_y}, {wp_z}) yaw={wp_yaw_deg}°  '
            f'takeoff_speed={self.takeoff_speed} m/s  rate={self.update_rate} Hz'
        )

        # Pre-compute waypoint PoseStamped (ENU)
        self.waypoint_pose = PoseStamped()
        self.waypoint_pose.header.frame_id = 'map'
        self.waypoint_pose.pose.position.x = wp_x
        self.waypoint_pose.pose.position.y = wp_y
        self.waypoint_pose.pose.position.z = wp_z
        # Quaternion from yaw (ENU: rotation about Z-up)
        self.waypoint_pose.pose.orientation.x = 0.0
        self.waypoint_pose.pose.orientation.y = 0.0
        self.waypoint_pose.pose.orientation.z = math.sin(wp_yaw_rad / 2.0)
        self.waypoint_pose.pose.orientation.w = math.cos(wp_yaw_rad / 2.0)

        self.waypoint_z = wp_z
        self.waypoint_yaw_deg = wp_yaw_deg

        # Pre-compute initial_waypoint Odometry message
        self._initial_waypoint_msg = Odometry()
        self._initial_waypoint_msg.header.frame_id = 'common_frame'
        self._initial_waypoint_msg.child_frame_id = 'waypoint'
        self._initial_waypoint_msg.pose.pose.position.x = wp_x
        self._initial_waypoint_msg.pose.pose.position.y = wp_y
        self._initial_waypoint_msg.pose.pose.position.z = wp_z
        self._initial_waypoint_msg.pose.pose.orientation = (
            self.waypoint_pose.pose.orientation
        )

        # -- State --
        self.flight_state = FlightState.INIT
        self.ramp_counter = 0
        self.arm_request_tick = 0  # throttle service calls

        # Cached subscriber data
        self.mavros_state: State | None = None
        self.current_pose: PoseStamped | None = None
        self.current_odom: Odometry | None = None
        self.cmd_vel: TwistStamped | None = None

        # -- QoS --
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -- Subscribers (cache-only) --
        self.create_subscription(
            State, 'mavros/state', self._state_cb, qos_reliable
        )
        self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self._pose_cb, qos_reliable
        )
        self.create_subscription(
            Odometry, 'mavros/local_position/odom', self._odom_cb, qos_reliable
        )
        self.create_subscription(
            TwistStamped, 'cmd_vel', self._cmd_vel_cb, qos_best_effort
        )

        # -- Publishers --
        self.vel_pub = self.create_publisher(
            TwistStamped, 'mavros/setpoint_velocity/cmd_vel', qos_reliable
        )
        self.pos_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', qos_reliable
        )
        self.waypoint_pub = self.create_publisher(
            Odometry, 'initial_waypoint', qos_reliable
        )

        # -- Service clients --
        self.arming_client = self.create_client(
            CommandBool, 'mavros/cmd/arming'
        )
        self.set_mode_client = self.create_client(
            SetMode, 'mavros/set_mode'
        )

        # -- Timer --
        self.timer = self.create_timer(
            1.0 / self.update_rate, self._timer_cb
        )

    # ── Subscriber callbacks (cache only) ──────────────────────────────

    def _state_cb(self, msg: State) -> None:
        self.mavros_state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_odom = msg

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        self.cmd_vel = msg

    # ── Helpers ─────────────────────────────────────────────────────────

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        self.vel_pub.publish(msg)

    def _publish_zero_velocity(self) -> None:
        self._publish_velocity(0.0, 0.0, 0.0)

    def _publish_waypoint(self) -> None:
        self._initial_waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(self._initial_waypoint_msg)

    def _request_offboard_and_arm(self) -> None:
        """Call MAVROS services to set OFFBOARD mode and arm. Throttled to ~1 Hz."""
        self.arm_request_tick += 1
        ticks_per_second = int(self.update_rate)
        if self.arm_request_tick % ticks_per_second != 0:
            return

        # Request OFFBOARD mode
        if self.mavros_state is not None and self.mavros_state.mode != 'OFFBOARD':
            if self.set_mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self._set_mode_done)
                self.get_logger().info(f'{self.vehicle_name}: Requesting OFFBOARD mode')
            else:
                self.get_logger().warn(
                    f'{self.vehicle_name}: mavros/set_mode service not ready'
                )

        # Request arming
        if self.mavros_state is not None and not self.mavros_state.armed:
            if self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                future.add_done_callback(self._arming_done)
                self.get_logger().info(f'{self.vehicle_name}: Requesting ARM')
            else:
                self.get_logger().warn(
                    f'{self.vehicle_name}: mavros/cmd/arming service not ready'
                )

    def _set_mode_done(self, future) -> None:
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info(f'{self.vehicle_name}: OFFBOARD mode request accepted')
            else:
                self.get_logger().warn(f'{self.vehicle_name}: OFFBOARD mode request rejected')
        except Exception as e:
            self.get_logger().error(f'{self.vehicle_name}: set_mode service call failed: {e}')

    def _arming_done(self, future) -> None:
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'{self.vehicle_name}: ARM command accepted')
            else:
                self.get_logger().warn(
                    f'{self.vehicle_name}: ARM command rejected (result={resp.result})'
                )
        except Exception as e:
            self.get_logger().error(f'{self.vehicle_name}: arming service call failed: {e}')

    def _distance_to_waypoint(self) -> float:
        if self.current_pose is None:
            return float('inf')
        p = self.current_pose.pose.position
        w = self.waypoint_pose.pose.position
        return math.sqrt(
            (p.x - w.x) ** 2 + (p.y - w.y) ** 2 + (p.z - w.z) ** 2
        )

    def _yaw_error_deg(self) -> float:
        if self.current_pose is None:
            return 180.0
        current_yaw_deg = math.degrees(
            _yaw_from_quat(self.current_pose.pose.orientation)
        )
        return _angle_diff_deg(current_yaw_deg, self.waypoint_yaw_deg)

    # ── Timer callback (state machine) ─────────────────────────────────

    def _timer_cb(self) -> None:
        # Always publish initial waypoint
        self._publish_waypoint()

        if self.flight_state == FlightState.INIT:
            self._state_init()
        elif self.flight_state == FlightState.RAMP_UP:
            self._state_ramp_up()
        elif self.flight_state == FlightState.ARM:
            self._state_arm()
        elif self.flight_state == FlightState.TAKEOFF:
            self._state_takeoff()
        elif self.flight_state == FlightState.HOVER:
            self._state_hover()
        elif self.flight_state == FlightState.POLICY:
            self._state_policy()

    def _state_init(self) -> None:
        # Keep streaming zero velocity while waiting
        self._publish_zero_velocity()
        if self.mavros_state is not None and self.current_pose is not None:
            self.get_logger().info(
                f'{self.vehicle_name}: MAVROS topics received, starting ramp-up'
            )
            self.flight_state = FlightState.RAMP_UP

    def _state_ramp_up(self) -> None:
        self._publish_zero_velocity()
        self.ramp_counter += 1
        if self.ramp_counter >= 11:
            self.get_logger().info(
                f'{self.vehicle_name}: Ramp-up complete, requesting offboard + arm'
            )
            self.flight_state = FlightState.ARM

    def _state_arm(self) -> None:
        self._publish_zero_velocity()
        self._request_offboard_and_arm()

        if (
            self.mavros_state is not None
            and self.mavros_state.armed
            and self.mavros_state.mode == 'OFFBOARD'
        ):
            self.get_logger().info(
                f'{self.vehicle_name}: Armed + OFFBOARD confirmed, taking off'
            )
            self.flight_state = FlightState.TAKEOFF

    def _state_takeoff(self) -> None:
        # Climb at takeoff_speed (positive Z = up in ENU)
        self._publish_velocity(0.0, 0.0, self.takeoff_speed)

        if self.current_pose is not None:
            alt = self.current_pose.pose.position.z
            if alt >= self.waypoint_z:
                self.get_logger().info(
                    f'{self.vehicle_name}: Target altitude {self.waypoint_z}m reached '
                    f'(current {alt:.1f}m), hovering'
                )
                self.flight_state = FlightState.HOVER

    def _state_hover(self) -> None:
        # Publish position setpoint
        self.waypoint_pose.header.stamp = self.get_clock().now().to_msg()
        self.pos_pub.publish(self.waypoint_pose)

        dist = self._distance_to_waypoint()
        yaw_err = abs(self._yaw_error_deg())
        if dist < 2.0 and yaw_err < 10.0:
            self.get_logger().info(
                f'{self.vehicle_name}: Waypoint reached (dist={dist:.2f}m, '
                f'yaw_err={yaw_err:.1f}°), entering POLICY mode'
            )
            self.flight_state = FlightState.POLICY

    def _state_policy(self) -> None:
        if self.cmd_vel is not None:
            # Forward policy command with fresh timestamp
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.cmd_vel.header.frame_id
            msg.twist = self.cmd_vel.twist
            self.vel_pub.publish(msg)
        else:
            self._publish_zero_velocity()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
