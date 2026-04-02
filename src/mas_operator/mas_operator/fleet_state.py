"""Fleet state data structures for operator monitoring."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

from mas_msgs.msg import TriangulatedPointArray


@dataclass
class VehicleState:
    """Cached state for a single vehicle."""

    mission_state: int | None = None
    odom: Odometry | None = None
    gimbal_rpy: Vector3 | None = None
    mavros_state: MavrosState | None = None
    chosen_target: PoseWithCovarianceStamped | None = None
    chosen_track_id: str | None = None  # resolved from chosen_target_pose
    tracked_objects: dict[int, Detection3DArray] = field(default_factory=dict)
    triangulated_points: TriangulatedPointArray | None = None
    policy_value: float | None = None  # V(s) from policy value network
    # topic_key → monotonic reception time
    last_heard: dict[str, float] = field(default_factory=dict)


@dataclass
class Alert:
    """An active alert condition."""

    name: str
    severity: str  # "WARNING" or "CRITICAL"
    message: str
    expires: float  # monotonic time when this alert should clear


class FleetState:
    """Thread-safe shared state between ROS callbacks and display."""

    def __init__(self, vehicle_names: list[str]) -> None:
        self.vehicles: dict[str, VehicleState] = {
            v: VehicleState() for v in vehicle_names
        }
        self.metrics = None  # set by metrics computation
        self.alerts: list[Alert] = []
        self.lock = threading.Lock()
