"""OperatorNode — unified monitoring and command interface for MAS.

Subscribes to per-vehicle state topics, computes derived metrics,
evaluates alert conditions, and publishes mission commands.
"""

import logging
import os
import threading
import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Vector3
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int8
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray

from mas_msgs.msg import TriangulatedPointArray

from mas_operator.alerts import AlertThresholds, evaluate_alerts
from mas_operator.fleet_state import FleetState
from mas_operator.markers import build_marker_array
from mas_operator.metrics import Metrics, compute_metrics

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


class OperatorNode(Node):
    def _now(self) -> float:
        """Current time in seconds, using ROS clock (respects use_sim_time)."""
        return self.get_clock().now().nanoseconds / 1e9

    def __init__(self) -> None:
        super().__init__('operator_node')

        # -- Parameters --
        self.declare_parameter('vehicles', ['px4_1', 'px4_2'])
        self.declare_parameter('aoi_warn_ms', 500.0)
        self.declare_parameter('aoi_critical_ms', 2000.0)
        self.declare_parameter('cov_warn_threshold', 5.0)
        self.declare_parameter('safety_distance_m', 9.5)
        self.declare_parameter('tri_timeout_s', 1.0)
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('num_object_classes', 1)

        self.vehicle_names: list[str] = (
            self.get_parameter('vehicles').value
        )
        self.num_object_classes: int = (
            self.get_parameter('num_object_classes').value
        )
        status_rate: float = self.get_parameter('status_rate_hz').value
        self.tri_timeout_s: float = self.get_parameter('tri_timeout_s').value

        # -- Alert thresholds --
        self.alert_thresholds = AlertThresholds(
            aoi_warn_ms=self.get_parameter('aoi_warn_ms').value,
            aoi_critical_ms=self.get_parameter('aoi_critical_ms').value,
            cov_warn_threshold=self.get_parameter('cov_warn_threshold').value,
            safety_distance_m=self.get_parameter('safety_distance_m').value,
            tri_timeout_s=self.tri_timeout_s,
        )

        # -- Shared state --
        self.fleet = FleetState(self.vehicle_names)

        # -- QoS profiles --
        self._qos_reliable_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Publishers --
        self.mission_cmd_pub = self.create_publisher(
            Int8, '/mission_state_cmd', self._qos_reliable_latched,
        )
        self.auto_pick_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self.set_target_pos_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for veh in self.vehicle_names:
            self.auto_pick_pubs[veh] = self.create_publisher(
                Int8, f'/{veh}/set_auto_pick_mode', self._qos_reliable,
            )
            self.set_target_pos_pubs[veh] = self.create_publisher(
                PointStamped, f'/{veh}/set_target_position', self._qos_reliable,
            )

        self.reset_hidden_clients: dict[str, rclpy.client.Client] = {}
        for veh in self.vehicle_names:
            self.reset_hidden_clients[veh] = self.create_client(
                Trigger, f'/{veh}/policy_node/reset_hidden_state',
            )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/operator/markers', 1,
        )

        # -- Per-vehicle subscriptions --
        for veh in self.vehicle_names:
            self._create_vehicle_subscriptions(veh)

        # -- Metrics timer --
        self.create_timer(1.0 / status_rate, self._metrics_timer_cb)

        self.get_logger().info(
            f'OperatorNode started — monitoring {self.vehicle_names}'
        )

    # ── Subscription setup ─────────────────────────────────────────────

    def _create_vehicle_subscriptions(self, veh: str) -> None:
        """Create all subscriptions for one vehicle."""

        self.create_subscription(
            Int8,
            f'/{veh}/mission_state',
            partial(self._mission_state_cb, veh),
            self._qos_reliable_latched,
        )
        self.create_subscription(
            Odometry,
            f'/{veh}/common_frame/odom',
            partial(self._odom_cb, veh),
            self._qos_best_effort,
        )
        self.create_subscription(
            Vector3,
            f'/{veh}/gimbal_state_rpy_deg',
            partial(self._gimbal_cb, veh),
            self._qos_best_effort,
        )
        self.create_subscription(
            MavrosState,
            f'/{veh}/mavros/state',
            partial(self._mavros_state_cb, veh),
            self._qos_best_effort,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{veh}/chosen_target_pose',
            partial(self._chosen_target_cb, veh),
            self._qos_reliable,
        )
        self.create_subscription(
            TriangulatedPointArray,
            f'/{veh}/triangulated_points',
            partial(self._triangulated_cb, veh),
            self._qos_reliable,
        )
        self.create_subscription(
            Float32,
            f'/{veh}/policy/value',
            partial(self._policy_value_cb, veh),
            self._qos_best_effort,
        )
        for ci in range(self.num_object_classes):
            self.create_subscription(
                Detection3DArray,
                f'/{veh}/tracked_objects/class_{ci}',
                partial(self._tracked_objects_cb, veh, ci),
                self._qos_reliable,
            )

    # ── Subscription callbacks ─────────────────────────────────────────

    def _mission_state_cb(self, veh: str, msg: Int8) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.mission_state = msg.data
            vs.last_heard['mission_state'] = now

    def _odom_cb(self, veh: str, msg: Odometry) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.odom = msg
            vs.last_heard['odom'] = now

    def _gimbal_cb(self, veh: str, msg: Vector3) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.gimbal_rpy = msg
            vs.last_heard['gimbal'] = now

    def _mavros_state_cb(self, veh: str, msg: MavrosState) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.mavros_state = msg
            vs.last_heard['mavros_state'] = now

    def _chosen_target_cb(
        self, veh: str, msg: PoseWithCovarianceStamped,
    ) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.chosen_target = msg
            vs.last_heard['chosen_target'] = now

            # Resolve track ID by matching position to tracked objects
            cp = msg.pose.pose.position
            best_id = None
            best_dist_sq = float('inf')
            for det_array in vs.tracked_objects.values():
                for det in det_array.detections:
                    if not det.results:
                        continue
                    tp = det.bbox.center.position
                    dx = cp.x - tp.x
                    dy = cp.y - tp.y
                    dz = cp.z - tp.z
                    d_sq = dx * dx + dy * dy + dz * dz
                    if d_sq < best_dist_sq:
                        best_dist_sq = d_sq
                        best_id = det.results[0].hypothesis.class_id
            if best_dist_sq < 25.0:  # within 5m
                vs.chosen_track_id = best_id
            # else: keep previous chosen_track_id

    def _tracked_objects_cb(
        self, veh: str, class_idx: int, msg: Detection3DArray,
    ) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.tracked_objects[class_idx] = msg
            vs.last_heard[f'tracked_objects_{class_idx}'] = now

    def _policy_value_cb(self, veh: str, msg: Float32) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.policy_value = msg.data
            vs.last_heard['policy_value'] = now

    def _triangulated_cb(
        self, veh: str, msg: TriangulatedPointArray,
    ) -> None:
        now = self._now()
        with self.fleet.lock:
            vs = self.fleet.vehicles[veh]
            vs.triangulated_points = msg
            vs.last_heard['triangulated_points'] = now

    # ── Metrics timer (placeholder — implemented in Slice 2) ───────────

    def _metrics_timer_cb(self) -> None:
        now = self._now()
        with self.fleet.lock:
            metrics = compute_metrics(
                self.fleet, now, self.tri_timeout_s,
            )
            self.fleet.metrics = metrics
            self.fleet.alerts = evaluate_alerts(
                metrics, self.fleet, self.alert_thresholds, now,
            )

            # Log summary
            self._log_metrics_summary(metrics)

            # Publish RViz markers
            marker_array = build_marker_array(
                self.fleet, metrics,
                self.alert_thresholds.aoi_warn_ms,
                self.alert_thresholds.aoi_critical_ms,
            )
        self.marker_pub.publish(marker_array)

    def _log_metrics_summary(self, metrics: Metrics) -> None:
        """Log a one-line summary of key metrics."""
        parts = []

        # Fleet consensus
        if metrics.fleet_consensus:
            states = set(
                vs.mission_state
                for vs in self.fleet.vehicles.values()
                if vs.mission_state is not None
            )
            state_name = _STATE_NAMES.get(
                next(iter(states), -1), '?',
            )
            parts.append(f'fleet={state_name}')
        else:
            parts.append('fleet=MISMATCH')

        # Covariance
        if metrics.cov_trace is not None:
            parts.append(f'cov={metrics.cov_trace:.2f}')

        # Triangulation
        parts.append(f'tri={"OK" if metrics.tri_valid else "LOST"}')

        # Inter-agent distances
        for (v_i, v_j), dist in metrics.inter_agent_distances.items():
            parts.append(f'd({v_i[-1]},{v_j[-1]})={dist:.1f}m')

        # Cross-agent AoI
        for (v_i, v_j), aoi in metrics.cross_agent_aoi.items():
            if aoi < float('inf'):
                parts.append(f'aoi({v_i[-1]},{v_j[-1]})={aoi:.0f}ms')

        # Baseline-to-range
        for (v_i, v_j), btr in metrics.baseline_to_range.items():
            if btr is not None:
                parts.append(f'b/r({v_i[-1]},{v_j[-1]})={btr:.2f}')

        # Alerts
        n_alerts = len(self.fleet.alerts)
        if n_alerts > 0:
            parts.append(f'ALERTS={n_alerts}')
            for a in self.fleet.alerts:
                parts.append(f'[{a.severity[0]}]{a.name}')

        self.get_logger().info(' | '.join(parts))

    # ── Command publishers ─────────────────────────────────────────────

    def publish_mission_cmd(self, state: int) -> None:
        msg = Int8()
        msg.data = state
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(
            f'Published mission_state_cmd: {_STATE_NAMES.get(state, state)}'
        )

    def publish_auto_pick(self, enable: bool) -> None:
        msg = Int8()
        msg.data = 1 if enable else 0
        for veh, pub in self.auto_pick_pubs.items():
            pub.publish(msg)
        self.get_logger().info(
            f'Published set_auto_pick_mode: {msg.data} to all vehicles'
        )

    def publish_set_target_position(self, track_id: str) -> None:
        """Look up track position by ID and publish to all vehicles."""
        # Find the track position from cached tracked_objects
        pos = None
        with self.fleet.lock:
            for vs in self.fleet.vehicles.values():
                for det_array in vs.tracked_objects.values():
                    for det in det_array.detections:
                        if det.results and det.results[0].hypothesis.class_id == track_id:
                            pos = det.bbox.center.position
                            break
                    if pos:
                        break
                if pos:
                    break

        if pos is None:
            self.get_logger().warn(f'Track ID {track_id} not found')
            return

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'common_frame'
        msg.point = pos
        for pub in self.set_target_pos_pubs.values():
            pub.publish(msg)
        self.get_logger().info(
            f'Published set_target_position: track {track_id} at '
            f'({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) to all vehicles'
        )


    def call_reset_hidden(self) -> None:
        """Call reset_hidden_state service on all vehicles (async, non-blocking)."""
        for veh, client in self.reset_hidden_clients.items():
            if client.service_is_ready():
                client.call_async(Trigger.Request())
            else:
                self.get_logger().warn(
                    f'reset_hidden_state service not ready for {veh}'
                )
        self.get_logger().info('Reset GRU hidden state on all vehicles')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorNode()

    use_display = os.environ.get('MAS_OPERATOR_NODISPLAY', '') == ''

    if use_display:
        # Suppress ROS2 logging to stdout (conflicts with curses)
        logging.getLogger('rcl').setLevel(logging.CRITICAL)
        rclpy.logging.set_logger_level(
            node.get_logger().name, rclpy.logging.LoggingSeverity.WARN,
        )

        from mas_operator.display import run_display
        display_thread = threading.Thread(
            target=run_display,
            args=(node, node.fleet),
            daemon=True,
        )
        display_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
