"""OffboardControl node — per-vehicle offboard controller for PX4 via MAVROS.

State machine: INIT → RAMP_UP → WAIT_OFFBOARD → TAKEOFF → HOVER → POLICY

Arming and OFFBOARD mode are NOT requested by this node — the operator must do
both externally (QGC, RC, or a separate tool) once setpoints are streaming.
While WAIT_OFFBOARD streams zero-velocity setpoints at the timer rate, PX4 will
accept the operator's OFFBOARD switch.

Waypoints are specified in the common frame (shared mission reference frame).
The node subscribes to common_frame/pose and computes a one-time offset to
convert waypoints into the MAVROS local frame for setpoint publishing.

All MAVROS interactions are ENU-FLU (MAVROS handles NED↔ENU conversion).
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

from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from mavros_msgs.msg import State

# Mission state constants (must match mas_mission)
_MISSION_IDLE = 0
_MISSION_TRACKING = 1
_MISSION_MISSION = 2
_MISSION_HOVER_CMD = 3
_MISSION_WAYPOINT = 4


class FlightState(Enum):
    INIT = auto()
    RAMP_UP = auto()
    WAIT_OFFBOARD = auto()  # passive: stream setpoints while operator arms + sets OFFBOARD
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

        # Waypoint orientation quaternion (ENU: rotation about Z-up)
        wp_quat_z = math.sin(wp_yaw_rad / 2.0)
        wp_quat_w = math.cos(wp_yaw_rad / 2.0)

        # Waypoint in common frame (used for distance/yaw checks)
        self.waypoint_pose = PoseStamped()
        self.waypoint_pose.header.frame_id = 'common_frame'
        self.waypoint_pose.pose.position.x = wp_x
        self.waypoint_pose.pose.position.y = wp_y
        self.waypoint_pose.pose.position.z = wp_z
        self.waypoint_pose.pose.orientation.x = 0.0
        self.waypoint_pose.pose.orientation.y = 0.0
        self.waypoint_pose.pose.orientation.z = wp_quat_z
        self.waypoint_pose.pose.orientation.w = wp_quat_w

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
        self._waited_log_tick = 0  # throttle "waiting for arm/OFFBOARD" log

        # Cached subscriber data. All drone state comes from mas_common_frame —
        # this node deliberately does NOT subscribe to mavros/local_position/*.
        self.mavros_state: State | None = None
        self.common_frame_pose: PoseStamped | None = None
        # Constant common→local offset: position of the drone's local-frame
        # origin (EKF home) expressed in common_frame ENU. Published latched
        # by mas_common_frame on `common_frame/local_origin`.
        self.local_origin_offset: Point | None = None
        self.cmd_vel: TwistStamped | None = None
        self.mission_state: int = _MISSION_IDLE

        # Position hold target for HOVER_CMD, captured in common_frame and
        # converted to local frame at publish time.
        self.hover_hold_common_pose: PoseStamped | None = None

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
        # Drone state comes exclusively from mas_common_frame; mavros/state is
        # the only mavros topic consumed (needed to know armed + flight mode).
        self.create_subscription(
            State, 'mavros/state', self._state_cb, qos_reliable
        )
        self.create_subscription(
            PoseStamped, 'common_frame/pose', self._cf_pose_cb, qos_best_effort
        )
        self.create_subscription(
            TwistStamped, 'cmd_vel', self._cmd_vel_cb, qos_best_effort
        )
        qos_reliable_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # mas_common_frame publishes the constant common→local offset latched
        # once the EKF home position is known.
        self.create_subscription(
            PointStamped,
            'common_frame/local_origin',
            self._local_origin_cb,
            qos_reliable_latched,
        )
        self.create_subscription(
            Int8, 'mission_state', self._mission_state_cb, qos_reliable_latched
        )
        # Runtime IC repositioning (ticket 004 conductor): fly to an arbitrary
        # common-frame pose and hold. The sim has no vehicle reset, so each
        # trial's initial condition is set by driving the drones here.
        self.create_subscription(
            PoseStamped, 'goto_position', self._goto_position_cb, qos_reliable
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

        # Arming and OFFBOARD mode change are out-of-band: the operator drives
        # them through QGC / RC / their own tool. This node only streams
        # setpoints, which is the precondition PX4 requires before it will
        # accept an OFFBOARD switch.

        # -- Timer --
        self.timer = self.create_timer(
            1.0 / self.update_rate, self._timer_cb
        )

    # ── Subscriber callbacks (cache only) ──────────────────────────────

    def _state_cb(self, msg: State) -> None:
        self.mavros_state = msg

    def _cf_pose_cb(self, msg: PoseStamped) -> None:
        self.common_frame_pose = msg

    def _local_origin_cb(self, msg: PointStamped) -> None:
        # Constant after the first message; we still accept updates in case
        # mas_common_frame ever re-broadcasts after an EKF origin reset.
        first_time = self.local_origin_offset is None
        self.local_origin_offset = msg.point
        if first_time:
            self.get_logger().info(
                f'{self.vehicle_name}: common→local offset received '
                f'({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})m'
            )

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        self.cmd_vel = msg

    def _mission_state_cb(self, msg: Int8) -> None:
        prev = self.mission_state
        self.mission_state = msg.data

        # Only react when already airborne (HOVER or POLICY)
        if self.flight_state not in (FlightState.HOVER, FlightState.POLICY):
            return

        if msg.data == _MISSION_HOVER_CMD and prev != _MISSION_HOVER_CMD:
            # Capture current common-frame position as hold target. Converted
            # to local frame at publish time via local_origin_offset.
            if self.common_frame_pose is not None:
                self.hover_hold_common_pose = PoseStamped()
                self.hover_hold_common_pose.header.frame_id = 'common_frame'
                p = self.common_frame_pose.pose.position
                self.hover_hold_common_pose.pose.position.x = p.x
                self.hover_hold_common_pose.pose.position.y = p.y
                self.hover_hold_common_pose.pose.position.z = p.z
                self.hover_hold_common_pose.pose.orientation = (
                    self.common_frame_pose.pose.orientation
                )
            self.flight_state = FlightState.HOVER
            self.get_logger().info(
                f'{self.vehicle_name}: HOVER_CMD — holding current position'
            )

        elif msg.data == _MISSION_WAYPOINT:
            # Return to configured waypoint. Idempotent: a repeated `w` press
            # re-centers the drone even if the operator's latched mission_state
            # was already WAYPOINT when this node subscribed.
            self.hover_hold_common_pose = None  # use configured waypoint
            self.flight_state = FlightState.HOVER
            self.get_logger().info(
                f'{self.vehicle_name}: WAYPOINT — returning to configured waypoint'
            )

        elif msg.data == _MISSION_MISSION and prev in (_MISSION_HOVER_CMD, _MISSION_WAYPOINT):
            # Resume mission — clear hold pose so HOVER uses configured waypoint
            # and can auto-transition to POLICY when waypoint reached
            self.hover_hold_common_pose = None
            self.get_logger().info(
                f'{self.vehicle_name}: Resuming MISSION from '
                f'{"HOVER_CMD" if prev == _MISSION_HOVER_CMD else "WAYPOINT"}'
            )

    # ── Helpers ─────────────────────────────────────────────────────────

    def _goto_position_cb(self, msg: PoseStamped) -> None:
        """Retarget the HOVER waypoint to an arbitrary common-frame pose.

        Honored only once airborne. In POLICY this reverts to HOVER, i.e. the
        drone leaves the engagement to go reposition — the conductor uses this
        to set each trial's IC and to return drones home afterwards (no sim
        vehicle reset exists). To then (re)engage, the operator/conductor sets
        mission_state=MISSION once the drone has settled at the new waypoint.
        """
        frame = msg.header.frame_id
        if frame and frame != 'common_frame':
            self.get_logger().warning(
                f'{self.vehicle_name}: goto_position frame "{frame}" != '
                f'common_frame; ignoring')
            return
        if self.flight_state not in (FlightState.HOVER, FlightState.POLICY):
            self.get_logger().warning(
                f'{self.vehicle_name}: goto_position ignored in state '
                f'{self.flight_state.name} (must be airborne)')
            return

        wp = PoseStamped()
        wp.header.frame_id = 'common_frame'
        wp.pose = msg.pose
        self.waypoint_pose = wp
        self.waypoint_z = float(msg.pose.position.z)
        self.waypoint_yaw_deg = math.degrees(_yaw_from_quat(msg.pose.orientation))
        # Use the new waypoint, not a stale HOVER_CMD hold pose.
        self.hover_hold_common_pose = None
        # Keep the 'initial_waypoint' viz in sync with the new target.
        self._initial_waypoint_msg.pose.pose.position.x = msg.pose.position.x
        self._initial_waypoint_msg.pose.pose.position.y = msg.pose.position.y
        self._initial_waypoint_msg.pose.pose.position.z = msg.pose.position.z
        self._initial_waypoint_msg.pose.pose.orientation = msg.pose.orientation
        if self.flight_state == FlightState.POLICY:
            self.flight_state = FlightState.HOVER
        self.get_logger().info(
            f'{self.vehicle_name}: goto_position -> '
            f'({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f}, '
            f'{msg.pose.position.z:.1f}), yaw={self.waypoint_yaw_deg:.0f}°')

    def _common_pose_to_local_setpoint(self, common_pose: PoseStamped) -> PoseStamped:
        """Convert a common-frame PoseStamped into a local-frame setpoint.

        Assumes local_origin_offset has been received; caller must check.
        Orientation passes through unchanged — for short distances (< 10 km)
        the common→local rotation is essentially identity, matching what
        mas_common_frame's own orientation transform does in practice.
        """
        off = self.local_origin_offset  # position of local origin in common frame
        out = PoseStamped()
        out.header.frame_id = 'map'
        out.pose.position.x = common_pose.pose.position.x - off.x
        out.pose.position.y = common_pose.pose.position.y - off.y
        out.pose.position.z = common_pose.pose.position.z - off.z
        out.pose.orientation = common_pose.pose.orientation
        return out

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # linear: world ENU (map); angular.z: body FLU yawrate (REP-147 aerial convention)
        msg.header.frame_id = 'map'
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

    def _distance_to_waypoint(self) -> float:
        if self.common_frame_pose is None:
            return float('inf')
        p = self.common_frame_pose.pose.position
        w = self.waypoint_pose.pose.position
        return math.sqrt(
            (p.x - w.x) ** 2 + (p.y - w.y) ** 2 + (p.z - w.z) ** 2
        )

    def _yaw_error_deg(self) -> float:
        if self.common_frame_pose is None:
            return 180.0
        current_yaw_deg = math.degrees(
            _yaw_from_quat(self.common_frame_pose.pose.orientation)
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
        elif self.flight_state == FlightState.WAIT_OFFBOARD:
            self._state_wait_offboard()
        elif self.flight_state == FlightState.TAKEOFF:
            self._state_takeoff()
        elif self.flight_state == FlightState.HOVER:
            self._state_hover()
        elif self.flight_state == FlightState.POLICY:
            self._state_policy()

    def _state_init(self) -> None:
        # Keep streaming zero velocity while waiting. Block ramp-up until
        # mavros_state is known AND mas_common_frame has reported both the
        # drone's common-frame pose and the constant common→local offset.
        self._publish_zero_velocity()
        if (
            self.mavros_state is not None
            and self.common_frame_pose is not None
            and self.local_origin_offset is not None
        ):
            self.get_logger().info(
                f'{self.vehicle_name}: state + common_frame ready, starting ramp-up'
            )
            self.flight_state = FlightState.RAMP_UP

    def _state_ramp_up(self) -> None:
        self._publish_zero_velocity()
        self.ramp_counter += 1
        if self.ramp_counter >= 11:
            self.get_logger().info(
                f'{self.vehicle_name}: Ramp-up complete — waiting for operator '
                f'to arm and switch to OFFBOARD (zero-velocity setpoints streaming)'
            )
            self.flight_state = FlightState.WAIT_OFFBOARD

    def _state_wait_offboard(self) -> None:
        # Keep streaming zero-velocity setpoints so PX4 accepts the operator's
        # OFFBOARD switch. Do NOT call mavros/set_mode or mavros/cmd/arming —
        # arming and mode change are operator-driven (QGC / RC / external tool).
        self._publish_zero_velocity()

        self._waited_log_tick += 1
        ticks_per_second = max(int(self.update_rate), 1)
        if self._waited_log_tick % (5 * ticks_per_second) == 0:
            armed = self.mavros_state.armed if self.mavros_state else False
            mode = self.mavros_state.mode if self.mavros_state else '<no state>'
            self.get_logger().info(
                f'{self.vehicle_name}: Waiting for operator (armed={armed}, mode={mode})'
            )

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
        # Climb at takeoff_speed (positive Z = up in ENU). Both `alt` and
        # `target_z` are now in common_frame — no mixed-frame comparison.
        self._publish_velocity(0.0, 0.0, self.takeoff_speed)

        if self.common_frame_pose is not None:
            alt = self.common_frame_pose.pose.position.z
            target_z = self.waypoint_z  # common-frame z from vehicles.yaml
            if alt >= target_z:
                self.get_logger().info(
                    f'{self.vehicle_name}: Target altitude {target_z}m reached '
                    f'(current {alt:.1f}m), hovering'
                )
                self.flight_state = FlightState.HOVER

    def _state_hover(self) -> None:
        # local_origin_offset is guaranteed non-None here — INIT blocks until
        # it has been received. Select the common-frame setpoint (hold pose or
        # configured waypoint) and convert to local frame at publish time.
        if self.hover_hold_common_pose is not None:
            setpoint = self._common_pose_to_local_setpoint(self.hover_hold_common_pose)
            setpoint.header.stamp = self.get_clock().now().to_msg()
            self.pos_pub.publish(setpoint)
            # HOVER_CMD never auto-transitions to POLICY — operator must resume
            return

        setpoint = self._common_pose_to_local_setpoint(self.waypoint_pose)
        setpoint.header.stamp = self.get_clock().now().to_msg()
        self.pos_pub.publish(setpoint)

        dist = self._distance_to_waypoint()
        yaw_err = abs(self._yaw_error_deg())
        waypoint_reached = dist < 2.0 and yaw_err < 10.0
        mission_approved = self.mission_state == _MISSION_MISSION

        if waypoint_reached and mission_approved:
            self.get_logger().info(
                f'{self.vehicle_name}: Waypoint reached (dist={dist:.2f}m, '
                f'yaw_err={yaw_err:.1f}°) and mission approved, entering POLICY mode'
            )
            self.flight_state = FlightState.POLICY
        elif waypoint_reached and not mission_approved:
            self.get_logger().info(
                f'{self.vehicle_name}: Waypoint reached, waiting for mission approval '
                f'(mission_state={self.mission_state})',
                throttle_duration_sec=5.0,
            )

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
