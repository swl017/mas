#!/usr/bin/env python3
"""Minimal operator CLI for MAS mission control.

An rclpy node that provides a text menu for mission state transitions and
target auto-pick toggling. Subscribes to per-agent mission_state for feedback.

Usage:
    python3 scripts/operator.py
    # Or with custom vehicles:
    python3 scripts/operator.py --ros-args -p vehicles:="['px4_1','px4_2','px4_3']"
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Int8

# Mission state constants (from mas_mission)
IDLE = 0
TRACKING = 1
MISSION = 2
STATE_NAMES = {IDLE: 'IDLE', TRACKING: 'TRACKING', MISSION: 'MISSION'}


class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator')

        self.declare_parameter('vehicles', ['px4_1', 'px4_2'])
        self._vehicles = self.get_parameter('vehicles').get_parameter_value().string_array_value

        # QoS for mission_state: RELIABLE + transient_local (latched)
        mission_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher: global mission state command
        self._cmd_pub = self.create_publisher(Int8, '/mission_state_cmd', mission_qos)

        # Per-vehicle auto-pick publishers and mission state subscribers
        self._auto_pick_pubs = {}
        self._mission_states = {}
        for veh in self._vehicles:
            self._auto_pick_pubs[veh] = self.create_publisher(
                Int8, f'/{veh}/set_auto_pick_mode', 10)
            self._mission_states[veh] = '?'
            self.create_subscription(
                Int8, f'/{veh}/mission_state',
                lambda msg, v=veh: self._mission_state_cb(msg, v),
                mission_qos)

        self.get_logger().info(f'Operator node started. Vehicles: {self._vehicles}')

    def _mission_state_cb(self, msg: Int8, veh: str):
        self._mission_states[veh] = STATE_NAMES.get(msg.data, f'UNKNOWN({msg.data})')

    def pub_mission_cmd(self, state: int):
        msg = Int8()
        msg.data = state
        self._cmd_pub.publish(msg)
        self.get_logger().info(f'Published mission_state_cmd = {STATE_NAMES.get(state, state)}')

    def pub_auto_pick(self, veh: str, mode: int):
        if veh not in self._auto_pick_pubs:
            self.get_logger().warn(f'Unknown vehicle: {veh}')
            return
        msg = Int8()
        msg.data = mode
        self._auto_pick_pubs[veh].publish(msg)
        self.get_logger().info(f'Published set_auto_pick_mode={mode} to {veh}')

    def print_status(self):
        print('\n--- Agent Status ---')
        for veh in self._vehicles:
            print(f'  {veh}: {self._mission_states[veh]}')
        print()


def menu_loop(node: OperatorNode):
    """Run the interactive text menu in a separate thread."""
    while rclpy.ok():
        print('\n=== MAS Operator ===')
        print(f'  Vehicles: {node._vehicles}')
        print('  1) Start Tracking (IDLE → TRACKING)')
        print('  2) Approve Mission (TRACKING → MISSION)')
        print('  3) Abort (→ IDLE)')
        print('  4) Enable auto-pick on vehicle')
        print('  5) Disable auto-pick on vehicle')
        print('  6) Status')
        print('  q) Quit')

        try:
            choice = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == '1':
            node.pub_mission_cmd(TRACKING)
        elif choice == '2':
            node.pub_mission_cmd(MISSION)
        elif choice == '3':
            node.pub_mission_cmd(IDLE)
        elif choice == '4':
            veh = input(f'  Vehicle name {node._vehicles}: ').strip()
            node.pub_auto_pick(veh, 1)
        elif choice == '5':
            veh = input(f'  Vehicle name {node._vehicles}: ').strip()
            node.pub_auto_pick(veh, 0)
        elif choice == '6':
            node.print_status()
        elif choice in ('q', 'Q'):
            break
        else:
            print('  Invalid choice.')


def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()

    # Spin in background so subscriptions work while menu is interactive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        menu_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
