"""MissionNode — state-gated command multiplexer for multi-agent missions.

State machine: IDLE → TRACKING → MISSION

All transitions triggered by operator via /mission_state_cmd topic.
Routes gimbal/zoom-rate/velocity commands from the active source based on state.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Float32, Float64, Int8

# Mission state constants
IDLE = 0
TRACKING = 1
MISSION = 2
HOVER_CMD = 3
WAYPOINT = 4

_STATE_NAMES = {
    IDLE: 'IDLE',
    TRACKING: 'TRACKING',
    MISSION: 'MISSION',
    HOVER_CMD: 'HOVER_CMD',
    WAYPOINT: 'WAYPOINT',
}


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        # -- Parameters --
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        self.declare_parameter('initial_state', IDLE)

        heartbeat_rate = self.get_parameter('heartbeat_rate_hz').value
        self.state = self.get_parameter('initial_state').value

        # -- QoS profiles --
        qos_reliable_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Operator command (global topic) --
        self.create_subscription(
            Int8, '/mission_state_cmd', self._mission_cmd_cb,
            qos_reliable_latched,
        )

        # -- Upstream: policy commands (MISSION state) --
        self.create_subscription(
            TwistStamped, 'policy/cmd_vel', self._policy_cmd_vel_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Vector3, 'policy/gimbal_cmd_los_rate', self._policy_gimbal_rate_cb,
            qos_default,
        )
        self.create_subscription(
            Float32, 'policy/zoom_rate_cmd', self._policy_zoom_cb,
            qos_default,
        )

        # -- Upstream: tracking commands (TRACKING state) --
        self.create_subscription(
            Vector3, 'tracking/gimbal_cmd_los_world_deg', self._tracking_gimbal_pos_cb,
            qos_default,
        )
        self.create_subscription(
            Vector3, 'tracking/gimbal_cmd_los_rate', self._tracking_gimbal_rate_cb,
            qos_best_effort,
        )
        self.create_subscription(
            Float32, 'tracking/zoom_rate_cmd', self._tracking_zoom_cb,
            qos_default,
        )

        # -- Downstream publishers --
        self.mission_state_pub = self.create_publisher(
            Int8, 'mission_state', qos_reliable_latched,
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 'cmd_vel', qos_best_effort,
        )
        self.gimbal_cmd_los_world_deg_pub = self.create_publisher(
            Vector3, 'gimbal_cmd_los_world_deg', qos_default,
        )
        self.gimbal_cmd_los_rate_pub = self.create_publisher(
            Vector3, 'gimbal_cmd_los_rate', qos_default,
        )
        self.zoom_rate_cmd_pub = self.create_publisher(
            Float32, 'zoom_rate_cmd', qos_default,
        )
        self.zoom_level_set_pub = self.create_publisher(
            Float64, 'zoom_level_set', qos_default,
        )

        # -- Heartbeat timer --
        self.create_timer(1.0 / heartbeat_rate, self._heartbeat_cb)

        # Publish initial state
        self._publish_state()
        self.get_logger().info(
            f'MissionNode started in {_STATE_NAMES.get(self.state, "UNKNOWN")} state'
        )

    # ── Operator command ────────────────────────────────────────────────

    def _mission_cmd_cb(self, msg: Int8) -> None:
        requested = msg.data
        if requested not in _STATE_NAMES:
            self.get_logger().warn(f'Unknown state requested: {requested}')
            return

        old_name = _STATE_NAMES[self.state]
        new_name = _STATE_NAMES[requested]

        if requested == self.state and requested != IDLE:
            self.get_logger().info(f'Already in {new_name}, ignoring')
            return

        self.state = requested
        self._publish_state()
        if old_name != new_name:
            self.get_logger().info(f'State transition: {old_name} → {new_name}')

        # Reset zoom on IDLE (always, even if already IDLE)
        if requested == IDLE:
            zoom_rate_msg = Float32()
            zoom_rate_msg.data = 0.0
            self.zoom_rate_cmd_pub.publish(zoom_rate_msg)
            zoom_level_msg = Float64()
            zoom_level_msg.data = 1.0
            self.zoom_level_set_pub.publish(zoom_level_msg)
            self.get_logger().info('Zoom reset to 1.0')

    # ── Policy command callbacks (active in MISSION) ────────────────────

    def _policy_cmd_vel_cb(self, msg: TwistStamped) -> None:
        if self.state == MISSION:
            self.cmd_vel_pub.publish(msg)

    def _policy_gimbal_rate_cb(self, msg: Vector3) -> None:
        if self.state == MISSION:
            self.gimbal_cmd_los_rate_pub.publish(msg)

    def _policy_zoom_cb(self, msg: Float32) -> None:
        if self.state == MISSION:
            self.zoom_rate_cmd_pub.publish(msg)

    # ── Tracking command callbacks (active in TRACKING) ─────────────────

    def _tracking_gimbal_pos_cb(self, msg: Vector3) -> None:
        if self.state == TRACKING:
            self.gimbal_cmd_los_world_deg_pub.publish(msg)

    def _tracking_gimbal_rate_cb(self, msg: Vector3) -> None:
        if self.state == TRACKING:
            self.gimbal_cmd_los_rate_pub.publish(msg)

    def _tracking_zoom_cb(self, msg: Float32) -> None:
        if self.state == TRACKING:
            self.zoom_rate_cmd_pub.publish(msg)

    # ── Heartbeat ───────────────────────────────────────────────────────

    def _heartbeat_cb(self) -> None:
        self._publish_state()

    def _publish_state(self) -> None:
        msg = Int8()
        msg.data = self.state
        self.mission_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
