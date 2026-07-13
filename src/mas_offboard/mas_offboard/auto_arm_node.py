"""auto_arm — one-shot helper that arms + engages OFFBOARD for a sim PX4 vehicle.

The offboard_control node is deliberately passive: it streams the
OffboardControlMode + TrajectorySetpoint heartbeat PX4 needs and then waits for
``mavros/state`` to report ``armed && mode == 'OFFBOARD'`` before taking off —
arming and the mode switch are left to "the operator's own tool" (QGC / RC /
external tool). In the Isaac-Sim / PX4-SITL stack there is no such operator, and
``mavros_replicator`` mirrors state only (it does NOT serve
``mavros/cmd/arming`` / ``mavros/set_mode`` — deferred, ticket 040). So this
helper *is* that tool: it drives arm + OFFBOARD by publishing
``px4_msgs/VehicleCommand`` to ``fmu/in/vehicle_command`` (whitelisted in PX4's
uXRCE-DDS ``dds_topics.yaml``), the same recipe offboard_py uses.

Because offboard_control already streams the setpoint heartbeat, this node only
has to send the two commands (DO_SET_MODE offboard, then ARM) and retry until
``mavros/state`` confirms, then exit — one-shot, meant to run from a tmux pane
once the stack is up.

    ros2 run mas_offboard auto_arm --ros-args -r __ns:=/px4_1 \\
        -p target_system:=2 -p use_sim_time:=true

``target_system`` MUST match the PX4 MAVLink system id for the namespace
(px4_1 -> 2, px4_2 -> 3; see config/vehicles_engagement.yaml).
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from mavros_msgs.msg import State
from px4_msgs.msg import VehicleCommand


class AutoArm(Node):
    """Publishes DO_SET_MODE(offboard)+ARM until mavros/state confirms, then quits."""

    def __init__(self) -> None:
        super().__init__('auto_arm')

        gp = self.declare_parameter
        # PX4 MAVLink system id for this vehicle (px4_1 -> 2, px4_2 -> 3).
        self.target_system = int(gp('target_system', 1).value)
        # How long (wall-clock) to keep retrying before giving up.
        self.timeout_s = float(gp('timeout_s', 90.0).value)
        # Resend period while waiting for confirmation.
        self.retry_period_s = float(gp('retry_period_s', 1.0).value)
        # Grace period after mavros reports `connected` before the first command,
        # so offboard_control is past INIT and streaming the setpoint heartbeat
        # PX4 requires before it will accept the OFFBOARD switch.
        self.stream_wait_s = float(gp('stream_wait_s', 3.0).value)

        # mavros/state is republished RELIABLE/VOLATILE by mavros_replicator.
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # PX4 uXRCE-DDS input QoS (mirrors the proven offboard_py publisher).
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._state: State | None = None
        self.create_subscription(State, 'mavros/state', self._on_state, qos_state)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos_cmd)

        # Run bookkeeping (wall-clock, independent of /clock so a sim-time jump
        # cannot trip the deadline — same rationale as the experiment conductor).
        self._t_start = time.monotonic()
        self._t_connected: float | None = None
        self._attempts = 0
        self.finished = False
        self.success = False

        self.get_logger().info(
            f'auto_arm: target_system={self.target_system}  '
            f'timeout={self.timeout_s:.0f}s  waiting for mavros/state…')
        self.create_timer(self.retry_period_s, self._tick)

    def _on_state(self, msg: State) -> None:
        self._state = msg
        if msg.connected and self._t_connected is None:
            self._t_connected = time.monotonic()

    def _publish_cmd(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(msg)

    def _tick(self) -> None:
        if self.finished:
            return

        now = time.monotonic()
        if now - self._t_start > self.timeout_s:
            armed = bool(self._state.armed) if self._state else False
            mode = self._state.mode if self._state else '<no state>'
            self.get_logger().error(
                f'auto_arm: TIMEOUT after {self.timeout_s:.0f}s '
                f'(armed={armed}, mode={mode}, attempts={self._attempts}) — giving up')
            self.finished = True
            return

        # Already there? (mas_offboard takes over from here → TAKEOFF.)
        if self._state and self._state.armed and self._state.mode == 'OFFBOARD':
            self.get_logger().info(
                f'auto_arm: armed + OFFBOARD confirmed after {self._attempts} '
                f'attempt(s) — offboard_control now owns the flight')
            self.success = True
            self.finished = True
            return

        if self._state is None or not self._state.connected:
            self.get_logger().info('auto_arm: waiting for PX4 (mavros/state)…',
                                   throttle_duration_sec=5.0)
            return

        # Give the setpoint heartbeat time to establish before the first attempt.
        if now - (self._t_connected or now) < self.stream_wait_s:
            return

        # Mode switch first, then arm — matches the proven offboard_py order.
        # VEHICLE_CMD_DO_SET_MODE param1=1 (custom mode), param2=6 (PX4 OFFBOARD).
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                          param1=1.0, param2=6.0)
        # VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1 (arm).
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                          param1=1.0)
        self._attempts += 1
        armed = bool(self._state.armed)
        self.get_logger().info(
            f'auto_arm: attempt {self._attempts} — sent OFFBOARD+ARM '
            f'(armed={armed}, mode={self._state.mode})',
            throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoArm()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
