"""Point the interceptor gimbal at the target's ground-truth position.

Bring-up / experiment crutch for the PN harness (RA-L ticket 004). The
detection-based pointing path (``gimbal_los_tracker_node`` -> ``/target_region``)
cannot acquire the target until it is already in the field of view, and the
mission gate that normally routes tracking commands is not running in the
Slice-1 sessions. This node breaks that deadlock by commanding the gimbal
directly from ground truth so the camera holds the target while the ego-only
sensing + EKF + PN chain is validated.

It does NOT feed the estimator: the EKF still works from the rendered image /
YOLO bbox. This only aims the camera (a stand-in for a perfect operator/cue),
so the ego-only bearing pipeline downstream remains honest.

Computes the world-frame line of sight from the interceptor to the target and
publishes it as ``gimbal_cmd_los_world_deg`` (Vector3, z=azimuth, y=elevation,
deg, ENU, +el up) — the topic ``gimbal_stabilizer/los_rate_controller`` consumes
in ``control_mode=position``. Convention matches that controller's IK
``los = [cos(el)cos(az), cos(el)sin(az), sin(el)]`` in ENU.

Run under the interceptor namespace::

    ros2 run mas_pn_guidance gimbal_gt_tracker --ros-args \
        -r __ns:=/px4_1 -p target_namespace:=px4_2 -p use_sim_time:=true

`target_namespace` defaults to the `target` role in roles.yaml.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class GimbalGtTracker(Node):
    def __init__(self):
        super().__init__('gimbal_gt_tracker')

        self.declare_parameter('target_namespace', '')
        self.declare_parameter('publish_rate_hz', 30.0)
        # Range-based zoom: high zoom for detection at distance, jump to low zoom
        # when the drones close (wide FOV keeps the oscillating target in frame).
        self.declare_parameter('zoom_far', 10.0)
        self.declare_parameter('zoom_near', 2.0)
        self.declare_parameter('zoom_switch_range_m', 20.0)
        self.declare_parameter('zoom_hysteresis_m', 5.0)
        # If the LOS leaves the gimbal's reachable cone the controller clamps;
        # warn when the body-relative angle is implausibly large so a bad
        # interceptor heading is visible during bring-up.
        self.declare_parameter('warn_when_idle_s', 2.0)

        target_ns = self.get_parameter('target_namespace').value
        if not target_ns:
            try:
                from .roles import Roles
                target_ns = Roles.load().namespace('target')
            except Exception as exc:  # pragma: no cover - config fallback
                self.get_logger().error(
                    f"target_namespace empty and roles.yaml lookup failed: {exc}")
                raise
        self._target_ns = target_ns.strip('/')

        rate = float(self.get_parameter('publish_rate_hz').value)

        self._self_p = None    # interceptor position (ENU), from this namespace
        self._target_p = None  # target position (ENU), ground truth
        self._last_self_t = None
        self._last_target_t = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # Ego odom: relative topic -> resolves under this node's namespace.
        self.create_subscription(
            Odometry, 'common_frame/odom', self._on_self_odom, sensor_qos)
        # Target GT odom: absolute, in the target's namespace.
        self.create_subscription(
            Odometry, f'/{self._target_ns}/common_frame/odom',
            self._on_target_odom, sensor_qos)

        # Pointing command -> los_rate_controller (position mode).
        self._cmd_pub = self.create_publisher(
            Vector3, 'gimbal_cmd_los_world_deg', reliable_qos)
        # Absolute zoom command -> los_rate_controller / siyi.
        self._zoom_pub = self.create_publisher(Float64, 'zoom_level_set', reliable_qos)
        self._zoom_far = float(self.get_parameter('zoom_far').value)
        self._zoom_near = float(self.get_parameter('zoom_near').value)
        self._zoom_switch = float(self.get_parameter('zoom_switch_range_m').value)
        self._zoom_hyst = float(self.get_parameter('zoom_hysteresis_m').value)
        self._zoom_is_near = False

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._warned_no_data = False
        self.get_logger().info(
            f"gimbal_gt_tracker: aiming at GT of '{self._target_ns}' at {rate:.0f} Hz "
            f"-> gimbal_cmd_los_world_deg")

    def _on_self_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._self_p = (p.x, p.y, p.z)

    def _on_target_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._target_p = (p.x, p.y, p.z)

    def _tick(self):
        if self._self_p is None or self._target_p is None:
            if not self._warned_no_data:
                self.get_logger().warn(
                    "waiting for both odom streams "
                    f"(self={'ok' if self._self_p else 'none'}, "
                    f"target={'ok' if self._target_p else 'none'})")
                self._warned_no_data = True
            return
        self._warned_no_data = False

        dx = self._target_p[0] - self._self_p[0]
        dy = self._target_p[1] - self._self_p[1]
        dz = self._target_p[2] - self._self_p[2]
        horiz = math.hypot(dx, dy)
        if horiz < 1e-6 and abs(dz) < 1e-6:
            return  # coincident; nothing to point at

        az = math.atan2(dy, dx)          # ENU: East->North
        el = math.atan2(dz, horiz)       # +up

        cmd = Vector3()
        cmd.x = 0.0
        cmd.y = math.degrees(el)
        cmd.z = math.degrees(az)
        self._cmd_pub.publish(cmd)

        # Range-based zoom with hysteresis: near (wide FOV) once close, far again
        # only after backing well off the switch range.
        rng = math.hypot(horiz, dz)
        if self._zoom_is_near and rng > self._zoom_switch + self._zoom_hyst:
            self._zoom_is_near = False
        elif (not self._zoom_is_near) and rng < self._zoom_switch - self._zoom_hyst:
            self._zoom_is_near = True
        self._zoom_pub.publish(
            Float64(data=self._zoom_near if self._zoom_is_near else self._zoom_far))


def main(argv=None):
    rclpy.init(args=argv)
    node = GimbalGtTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
