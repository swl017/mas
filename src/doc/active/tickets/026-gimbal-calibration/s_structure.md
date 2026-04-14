## Structure Outline

### New files

- `src/gimbal_controller/gimbal_controller/gimbal_calibration.py`
  - `@dataclass class CalibrationSample`
    - `phase: str`
    - `axis: str`
    - `sweep_direction: str`
    - `commanded_deg: float | None`
    - `encoder_yaw_deg: float | None`
    - `encoder_pitch_deg: float | None`
    - `encoder_roll_deg: float | None`
    - `camera_yaw_deg: float | None`
    - `camera_pitch_deg: float | None`
    - `camera_roll_deg: float | None`
    - `timestamp_sec: float`
    - `note: str`
  - `@dataclass class CalibrationSessionPaths`
    - `session_root: Path`
    - `bag_dir: Path`
    - `csv_path: Path`
    - `summary_json_path: Path`
    - `notes_path: Path`
  - `@dataclass class SweepConfig`
    - `yaw_min_deg: float`
    - `yaw_max_deg: float`
    - `pitch_min_deg: float`
    - `pitch_max_deg: float`
    - `step_deg: float`
    - `settle_time_sec: float`
    - `verification_step_deg: float`
    - `verification_hold_sec: float`
  - `class GimbalCalibrationNode(Node)`
    - `__init__(self)` — declare parameters, connect SDK, create subscriptions, prepare session paths and writers
    - `_connect_sdk(self) -> SIYISDK`
    - `_declare_parameters(self) -> None`
    - `_camera_info_cb(self, msg: CameraInfo) -> None`
    - `_image_cb(self, msg: Image) -> None`
    - `_gimbal_state_cb(self, msg: Vector3) -> None`
    - `_run_session(self) -> None` — top-level phase runner
    - `_run_encoder_verification(self) -> list[CalibrationSample]`
    - `_run_sweep(self, axis: str, targets_deg: list[float], direction: str) -> list[CalibrationSample]`
    - `_run_zero_offset_estimation(self) -> list[CalibrationSample]`
    - `_command_angle(self, yaw_deg: float | None, pitch_deg: float | None) -> None`
    - `_read_encoder_state(self) -> tuple[float | None, float | None, float | None]`
    - `_estimate_camera_angles(self) -> tuple[float | None, float | None, float | None]`
    - `_detect_checkerboard(self, image: np.ndarray, camera_info: CameraInfo) -> tuple[bool, np.ndarray | None, np.ndarray | None]`
    - `_record_sample(self, sample: CalibrationSample) -> None`
    - `_write_summary(self, samples: list[CalibrationSample]) -> None`
    - `_start_rosbag_record(self) -> subprocess.Popen[str] | None`
    - `_stop_rosbag_record(self) -> None`
    - `_shutdown_sdk(self) -> None`
  - `def build_session_paths(dataset_root: Path, session_name: str) -> CalibrationSessionPaths`
  - `def generate_sweep_targets(min_deg: float, max_deg: float, step_deg: float) -> list[float]`
  - `def compute_hysteresis(samples: list[CalibrationSample]) -> dict[str, float]`
  - `def compute_zero_offset_stats(samples: list[CalibrationSample]) -> dict[str, dict[str, float | None]]`
  - `def main() -> int`

- `src/gimbal_controller/scripts/init_gimbal_calibration_session.py`
  - `def parse_args() -> argparse.Namespace`
  - `def normalize_session_name(raw_name: str) -> str`
  - `def write_text_if_missing(path: Path, content: str) -> None`
  - `def main() -> int`

- `src/gimbal_controller/scripts/summarize_gimbal_calibration.py`
  - `def parse_args() -> argparse.Namespace`
  - `def load_samples(csv_path: Path) -> list[dict[str, str]]`
  - `def summarize_verification(rows: list[dict[str, str]]) -> dict[str, object]`
  - `def summarize_hysteresis(rows: list[dict[str, str]]) -> dict[str, object]`
  - `def summarize_zero_offsets(rows: list[dict[str, str]]) -> dict[str, object]`
  - `def write_summary_json(path: Path, summary: dict[str, object]) -> None`
  - `def main() -> int`

- `src/gimbal_controller/README_calibration.md`
  - bench workflow for ticket 026
  - dataset layout for `datasets/gimbal_calibration/<date>/`
  - usage examples for session init, calibration run, and summary generation

### Modified files

- `src/gimbal_controller/setup.py`
  - `[add]` console script `gimbal_calibration = gimbal_controller.gimbal_calibration:main`
  - `[add]` package data install entries for calibration helper scripts and README if needed

- `src/gimbal_controller/package.xml`
  - `[add]` runtime dependencies needed by the calibration executable: `sensor_msgs`, `std_msgs`, and any existing image / OpenCV bridge dependency required by checkerboard estimation

- `src/gimbal_controller/CONTEXT.md`
  - `[add]` `gimbal_calibration` as a bench executable
  - `[add]` its subscriptions, outputs, and operator-facing purpose
  - `[add]` package-local calibration helper scripts and bench dataset convention
