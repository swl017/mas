"""Raw world-LOS publisher — the image feature for raw-IBVS guidance (ticket 012).

Composes the target line-of-sight **directly from the raw YOLO detection**, fully
decoupled from any EKF 3-D state, and publishes it as a stamped unit vector in
world ENU at the detection rate:

    n̂^e(t) = unit( R_b^e(t) · R_c^b(gimbal(t)) · unit([x̄, ȳ, 1]) )

using the same `camera_model.world_los_from_pixel` composition the EKF nodes use
for their bearing update. This is range-free by construction — it depends only on
the pixel direction through the gimbal ∘ attitude chain, never on target range —
which is exactly the property `raw_ibvs` guidance servos (ticket 011 review #5:
the position-derived `bearing_pn` bearing is perturbed by parallax as the observer
translates; the raw image feature is not).

The published `bearing_raw/los` preserves the **detection header timestamp** so the
guidance node can differentiate the LOS on its true sample time (stamped LOS-rate
discipline), not on the control tick. The node stays silent when there is no fresh
detection; the guidance node owns the dropout / target-lost policy.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray

from .camera_model import CameraIntrinsics, world_los_from_pixel
from .quaternion import quat_to_rot


def _be_qos(depth=10):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _quat_msg_to_wxyz(q):
    return np.array([q.w, q.x, q.y, q.z])


class RawLosNode(Node):
    def __init__(self):
        super().__init__("raw_los_node")

        self.declare_parameter("target_class_name", "drone")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("publish_frame", "common_frame")

        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.publish_frame = str(self.get_parameter("publish_frame").value)

        self.intrinsics: Optional[CameraIntrinsics] = None
        self.zoom = 1.0
        self.gimbal_rpy_rad = np.zeros(3)
        self.aircraft_quat: Optional[np.ndarray] = None
        self._published_once = False

        be = _be_qos()
        # Attitude only — the LOS is a direction from the camera; target range and
        # the aircraft position never enter the composition.
        self.create_subscription(Odometry, "common_frame/odom", self._on_odom, be)
        self.create_subscription(Detection2DArray, "yolo_result_vision",
                                 self._on_detections, be)
        self.create_subscription(CameraInfo, "camera/color/camera_info",
                                 self._on_camera_info, 10)
        self.create_subscription(Vector3, "gimbal_state_rpy_deg",
                                 self._on_gimbal, be)
        self.create_subscription(Float64, "camera/zoom_level", self._on_zoom, be)

        self.pub_los = self.create_publisher(Vector3Stamped, "bearing_raw/los", 10)

        self.get_logger().info(
            "raw_los_node ready: publishing bearing_raw/los (world-ENU unit LOS) "
            f"at the detection rate; class='{self.target_class_name}'")

    def _on_camera_info(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = CameraIntrinsics(
                fx=float(msg.k[0]), fy=float(msg.k[4]),
                cx=float(msg.k[2]), cy=float(msg.k[5]),
                width=int(msg.width), height=int(msg.height),
            )
            self.get_logger().info(
                f"CameraInfo: fx={self.intrinsics.fx:.1f} cx={self.intrinsics.cx:.1f} "
                f"cy={self.intrinsics.cy:.1f}")

    def _on_zoom(self, msg: Float64):
        z = float(msg.data)
        if z > 0:
            self.zoom = z

    def _on_gimbal(self, msg: Vector3):
        # gimbal_state_rpy_deg: msg.x=roll, msg.y=pitch, msg.z=yaw (deg), ZXY.
        self.gimbal_rpy_rad = np.array([
            math.radians(msg.x), math.radians(msg.y), math.radians(msg.z)])

    def _on_odom(self, msg: Odometry):
        self.aircraft_quat = _quat_msg_to_wxyz(msg.pose.pose.orientation)

    def _pick_best(self, msg: Detection2DArray):
        """Highest-confidence detection of the target class (mirrors the EKF nodes)."""
        best = None
        best_score = -1.0
        for det in msg.detections:
            for r in det.results:
                hyp = getattr(r, "hypothesis", None)
                if hyp is not None:
                    cls = str(getattr(hyp, "class_id", ""))
                    score = float(getattr(hyp, "score", 0.0))
                else:
                    cls = str(getattr(r, "id", ""))
                    score = float(getattr(r, "score", 0.0))
                if score < self.min_conf:
                    continue
                if self.target_class_name and cls != self.target_class_name:
                    continue
                if score > best_score:
                    best_score = score
                    best = det
        return best

    def _on_detections(self, msg: Detection2DArray):
        if self.intrinsics is None or self.aircraft_quat is None:
            return
        det = self._pick_best(msg)
        if det is None:
            return
        center = det.bbox.center
        if hasattr(center, "position"):
            u_pix, v_pix = float(center.position.x), float(center.position.y)
        else:
            u_pix, v_pix = float(center.x), float(center.y)

        R_b_e = quat_to_rot(self.aircraft_quat)
        n = world_los_from_pixel(u_pix, v_pix, self.zoom, self.intrinsics,
                                 self.gimbal_rpy_rad, R_b_e)
        if not np.all(np.isfinite(n)):
            return

        out = Vector3Stamped()
        out.header.stamp = msg.header.stamp          # detection time (stamped rate)
        out.header.frame_id = self.publish_frame
        out.vector.x, out.vector.y, out.vector.z = float(n[0]), float(n[1]), float(n[2])
        self.pub_los.publish(out)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(f"first bearing_raw/los published: n̂={n}")


def main(args=None):
    rclpy.init(args=args)
    node = RawLosNode()
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
