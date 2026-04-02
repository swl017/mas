## Structure Outline

### New files

- `src/mas_operator/package.xml`
  - ament_python format=3, depends: rclpy, std_msgs, geometry_msgs, nav_msgs, vision_msgs, visualization_msgs, mavros_msgs, mas_msgs

- `src/mas_operator/setup.py`
  - console_scripts: `operator_node = mas_operator.operator_node:main`
  - data_files: launch/, config/

- `src/mas_operator/setup.cfg`
  - standard ament_python setup.cfg

- `src/mas_operator/resource/mas_operator`
  - empty marker file (ament resource index)

- `src/mas_operator/mas_operator/__init__.py`

- `src/mas_operator/mas_operator/operator_node.py`
  - `class OperatorNode(Node)`
    - `__init__(self)` — declare parameters, load vehicle list, create subscriptions/publishers/timer
    - `_create_vehicle_subscriptions(self, veh: str)` — create all subs for one vehicle
    - `_mission_state_cb(self, veh: str, msg: Int8)`
    - `_odom_cb(self, veh: str, msg: Odometry)`
    - `_gimbal_cb(self, veh: str, msg: Vector3)`
    - `_mavros_state_cb(self, veh: str, msg: State)`
    - `_chosen_target_cb(self, veh: str, msg: PoseWithCovarianceStamped)`
    - `_tracked_objects_cb(self, veh: str, class_idx: int, msg: Detection3DArray)`
    - `_triangulated_cb(self, veh: str, msg: TriangulatedPointArray)`
    - `_metrics_timer_cb(self)` — compute metrics, evaluate alerts, publish markers
    - `_publish_mission_cmd(self, state: int)`
    - `_publish_auto_pick(self, enable: bool)`
  - `def main()` — init rclpy, create node, start display thread, spin

- `src/mas_operator/mas_operator/fleet_state.py`
  - `@dataclass class VehicleState`
    - `mission_state: int | None`
    - `odom: Odometry | None`
    - `gimbal_rpy: Vector3 | None`
    - `mavros_state: State | None`
    - `chosen_target: PoseWithCovarianceStamped | None`
    - `tracked_objects: dict[int, Detection3DArray]` — keyed by class index
    - `triangulated_points: TriangulatedPointArray | None`
    - `last_heard: dict[str, float]` — topic_key → reception time (monotonic)
  - `@dataclass class Alert`
    - `name: str`
    - `severity: str` — "WARNING" | "CRITICAL"
    - `message: str`
    - `expires: float` — monotonic time when alert should clear
  - `class FleetState`
    - `__init__(self, vehicles: list[str])`
    - `vehicles: dict[str, VehicleState]`
    - `metrics: Metrics | None`
    - `alerts: list[Alert]`
    - `lock: threading.Lock`

- `src/mas_operator/mas_operator/metrics.py`
  - `@dataclass class Metrics`
    - `aoi: dict[str, dict[str, float]]` — `veh → {topic_key → age_ms}`
    - `cross_agent_aoi: dict[tuple[str,str], float]` — `(veh_i, veh_j) → max_age_ms`
    - `cov_trace: float | None` — triangulation covariance trace
    - `tri_valid: bool` — triangulated points received within timeout
    - `inter_agent_distances: dict[tuple[str,str], float]` — pairwise distances (m)
    - `fleet_consensus: bool` — all agents in same mission_state
    - `baseline_to_range: dict[tuple[str,str], float | None]` — observation geometry per pair
  - `def compute_metrics(fleet: FleetState, now: float) -> Metrics`
  - `def compute_aoi(vehicle: VehicleState, now: float) -> dict[str, float]`
  - `def compute_cross_agent_aoi(fleet: FleetState, now: float) -> dict[tuple[str,str], float]`
  - `def compute_cov_trace(points: TriangulatedPointArray | None) -> float | None`
  - `def compute_inter_agent_distances(fleet: FleetState) -> dict[tuple[str,str], float]`
  - `def compute_baseline_to_range(fleet: FleetState, target_pos) -> dict[tuple[str,str], float | None]`

- `src/mas_operator/mas_operator/alerts.py`
  - `@dataclass class AlertThresholds`
    - `aoi_warn_ms: float`
    - `aoi_critical_ms: float`
    - `cov_warn_threshold: float`
    - `safety_distance_m: float`
    - `tri_timeout_s: float`
  - `def evaluate_alerts(metrics: Metrics, fleet: FleetState, thresholds: AlertThresholds, now: float, linger_s: float) -> list[Alert]`

- `src/mas_operator/mas_operator/markers.py`
  - `def build_marker_array(fleet: FleetState, metrics: Metrics, frame_id: str) -> MarkerArray`
  - `def _agent_markers(fleet: FleetState, frame_id: str) -> list[Marker]` — position spheres + text labels
  - `def _target_markers(fleet: FleetState, frame_id: str) -> list[Marker]` — tracked target spheres + text ID labels
  - `def _aoi_line_markers(fleet: FleetState, metrics: Metrics, frame_id: str) -> list[Marker]` — inter-agent lines color-coded by AoI + text labels

- `src/mas_operator/mas_operator/display.py`
  - `def run_display(node: OperatorNode, fleet: FleetState)` — curses wrapper, called from daemon thread
  - `def _draw_screen(stdscr, fleet: FleetState, node: OperatorNode)` — main render loop
  - `def _draw_fleet_table(stdscr, row: int, fleet: FleetState) -> int` — returns next row
  - `def _draw_targets_table(stdscr, row: int, fleet: FleetState) -> int` — tracked objects with IDs
  - `def _draw_alerts(stdscr, row: int, alerts: list[Alert]) -> int`
  - `def _draw_commands(stdscr, row: int) -> int` — key legend
  - `def _handle_key(key: int, node: OperatorNode, fleet: FleetState)` — dispatch keypresses to node publishers

- `src/mas_operator/launch/operator.launch.py`
  - Launch arguments: `config_file` (default: `config/operator.yaml`)
  - `OpaqueFunction` pattern matching mas_mission
  - Single node launch (not per-vehicle — one node monitors all)

- `src/mas_operator/config/operator.yaml`
  - `vehicles: ["px4_1", "px4_2"]`
  - `aoi_warn_ms`, `aoi_critical_ms`, `cov_warn_threshold`, `safety_distance_m`, `tri_timeout_s`, `status_rate_hz`, `num_object_classes`

### Modified files

- `src/ARCHITECTURE.md`
  - [modify] Topic table line 30 — change `triangulated_points` to show per-vehicle namespace
  - [add] `mas_operator` to Package Summary table
  - [add] operator subscription/publication topics to Topic/Service Interface table
