#!/usr/bin/env python3

import csv
import json
import math
import signal
import subprocess
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from statistics import mean, pstdev
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from gimbal_controller.siyi_sdk.siyi_sdk import SIYISDK

try:
    import cv2
except ImportError:  # pragma: no cover - optional bench dependency
    cv2 = None


@dataclass
class CalibrationSample:
    phase: str
    axis: str
    sweep_direction: str
    commanded_deg: Optional[float]
    encoder_yaw_deg: Optional[float]
    encoder_pitch_deg: Optional[float]
    encoder_roll_deg: Optional[float]
    camera_yaw_deg: Optional[float]
    camera_pitch_deg: Optional[float]
    camera_roll_deg: Optional[float]
    timestamp_sec: float
    note: str


@dataclass
class CalibrationSessionPaths:
    session_root: Path
    bag_dir: Path
    csv_path: Path
    summary_json_path: Path
    notes_path: Path


@dataclass
class SweepConfig:
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float
    step_deg: float
    settle_time_sec: float
    verification_step_deg: float
    verification_hold_sec: float


def build_session_paths(dataset_root: Path, session_name: str) -> CalibrationSessionPaths:
    session_root = dataset_root / session_name
    session_root.mkdir(parents=True, exist_ok=True)
    bag_dir = session_root / "bag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    return CalibrationSessionPaths(
        session_root=session_root,
        bag_dir=bag_dir,
        csv_path=session_root / "samples.csv",
        summary_json_path=session_root / "summary.json",
        notes_path=session_root / "notes.md",
    )


def generate_sweep_targets(min_deg: float, max_deg: float, step_deg: float) -> List[float]:
    if step_deg <= 0.0:
        raise ValueError("step_deg must be positive")
    targets: List[float] = []
    value = min_deg
    while value <= max_deg + 1e-6:
        targets.append(round(value, 6))
        value += step_deg
    return targets


def _mean_and_std(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"mean": None, "std": None, "count": 0}
    if len(values) == 1:
        return {"mean": values[0], "std": 0.0, "count": 1}
    return {"mean": mean(values), "std": pstdev(values), "count": len(values)}


def compute_hysteresis(samples: List[CalibrationSample]) -> Dict[str, Optional[float]]:
    results: Dict[str, Optional[float]] = {"yaw": None, "pitch": None}
    encoder_field = {
        "yaw": "encoder_yaw_deg",
        "pitch": "encoder_pitch_deg",
    }
    for axis in ("yaw", "pitch"):
        forward = {
            sample.commanded_deg: getattr(sample, encoder_field[axis])
            for sample in samples
            if sample.phase == "sweep"
            and sample.axis == axis
            and sample.sweep_direction == "forward"
            and sample.commanded_deg is not None
            and getattr(sample, encoder_field[axis]) is not None
        }
        reverse = {
            sample.commanded_deg: getattr(sample, encoder_field[axis])
            for sample in samples
            if sample.phase == "sweep"
            and sample.axis == axis
            and sample.sweep_direction == "reverse"
            and sample.commanded_deg is not None
            and getattr(sample, encoder_field[axis]) is not None
        }
        diffs = [
            abs(forward[commanded] - reverse[commanded])
            for commanded in forward.keys() & reverse.keys()
        ]
        if diffs:
            results[axis] = max(diffs)
    return results


def compute_zero_offset_stats(
    samples: List[CalibrationSample],
) -> Dict[str, Dict[str, Any]]:
    results: Dict[str, Dict[str, Any]] = {}
    for axis, encoder_key, camera_key in (
        ("yaw", "encoder_yaw_deg", "camera_yaw_deg"),
        ("pitch", "encoder_pitch_deg", "camera_pitch_deg"),
        ("roll", "encoder_roll_deg", "camera_roll_deg"),
    ):
        offsets = []
        for sample in samples:
            encoder_value = getattr(sample, encoder_key)
            camera_value = getattr(sample, camera_key)
            if sample.phase != "zero_offset" or encoder_value is None or camera_value is None:
                continue
            offsets.append(encoder_value - camera_value)
        results[axis] = _mean_and_std(offsets)
    return results


class GimbalCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_calibration")
        self._declare_parameters()

        self.bridge_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.gimbal_state_msg: Optional[Vector3] = None
        self._bag_process: Optional[subprocess.Popen] = None
        self._csv_handle = None
        self._csv_writer = None

        self.session_paths = build_session_paths(
            Path(self.get_parameter("dataset_root").value),
            self.get_parameter("session_name").value,
        )
        self._write_default_notes()
        self.cam = self._connect_sdk()
        self._setup_subscriptions()
        self._open_csv_writer()

    def _declare_parameters(self) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.declare_parameter("dataset_root", "datasets/gimbal_calibration")
        self.declare_parameter("session_name", timestamp)
        self.declare_parameter("server_ip", "192.168.144.26")
        self.declare_parameter("server_port", 37260)
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera/color/camera_info")
        self.declare_parameter("gimbal_state_topic", "gimbal_state_rpy_deg")
        self.declare_parameter("yaw_min_deg", -135.0)
        self.declare_parameter("yaw_max_deg", 135.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 25.0)
        self.declare_parameter("step_deg", 5.0)
        self.declare_parameter("settle_time_sec", 1.5)
        self.declare_parameter("verification_step_deg", 10.0)
        self.declare_parameter("verification_hold_sec", 1.0)
        self.declare_parameter("zero_offset_measurements", 5)
        self.declare_parameter("enable_rosbag", True)
        self.declare_parameter("enable_checkerboard", True)
        self.declare_parameter("checkerboard_rows", 9)
        self.declare_parameter("checkerboard_cols", 6)
        self.declare_parameter("checkerboard_square_size_m", 0.03)
        self.declare_parameter("record_topics", [])

    def _write_default_notes(self) -> None:
        if self.session_paths.notes_path.exists():
            return
        self.session_paths.notes_path.write_text(
            "\n".join(
                (
                    f"# {self.get_parameter('session_name').value}",
                    "",
                    "- Gimbal hardware:",
                    "- Camera mode / zoom:",
                    "- Checkerboard size:",
                    "- Bench fixture:",
                    "- Operator notes:",
                    "",
                )
            ),
            encoding="utf-8",
        )

    def _connect_sdk(self) -> SIYISDK:
        server_ip = self.get_parameter("server_ip").value
        server_port = int(self.get_parameter("server_port").value)
        cam = SIYISDK(server_ip=server_ip, port=server_port)
        self.get_logger().info(f"Connecting to SIYI SDK at {server_ip}:{server_port}")
        if not cam.connect():
            raise ConnectionError(f"Failed to connect to SIYI SDK at {server_ip}:{server_port}")

        try:
            cam.requestHardwareID()
        except Exception as exc:  # pragma: no cover - hardware path
            self.get_logger().warning(f"Hardware ID request failed: {exc}")
        return cam

    def _setup_subscriptions(self) -> None:
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            best_effort_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_cb,
            best_effort_qos,
        )
        self.create_subscription(
            Vector3,
            self.get_parameter("gimbal_state_topic").value,
            self._gimbal_state_cb,
            best_effort_qos,
        )

    def _open_csv_writer(self) -> None:
        fieldnames = list(CalibrationSample.__annotations__.keys())
        self._csv_handle = self.session_paths.csv_path.open("w", encoding="utf-8", newline="")
        self._csv_writer = csv.DictWriter(self._csv_handle, fieldnames=fieldnames)
        self._csv_writer.writeheader()

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _image_cb(self, msg: Image) -> None:
        self.bridge_image = self._image_to_numpy(msg)

    def _gimbal_state_cb(self, msg: Vector3) -> None:
        self.gimbal_state_msg = msg

    def _image_to_numpy(self, msg: Image) -> Optional[np.ndarray]:
        channels_by_encoding = {
            "mono8": 1,
            "8UC1": 1,
            "bgr8": 3,
            "rgb8": 3,
        }
        channels = channels_by_encoding.get(msg.encoding)
        if channels is None:
            self.get_logger().debug(f"Unsupported image encoding for checkerboard: {msg.encoding}")
            return None
        array = np.frombuffer(msg.data, dtype=np.uint8)
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        if channels == 1:
            image = array.reshape((height, step))[:, :width]
            return image.copy()
        image = array.reshape((height, step // channels, channels))[:, :width, :]
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        return image.copy()

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.time() + duration_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _spin_until(self, predicate: Callable[[], bool], timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return predicate()

    def run(self) -> List[CalibrationSample]:
        samples: List[CalibrationSample] = []
        self._start_rosbag_record()
        try:
            samples.extend(self._run_encoder_verification())
            yaw_targets = generate_sweep_targets(
                self.get_parameter("yaw_min_deg").value,
                self.get_parameter("yaw_max_deg").value,
                self.get_parameter("step_deg").value,
            )
            pitch_targets = generate_sweep_targets(
                self.get_parameter("pitch_min_deg").value,
                self.get_parameter("pitch_max_deg").value,
                self.get_parameter("step_deg").value,
            )
            samples.extend(self._run_sweep("yaw", yaw_targets, "forward"))
            samples.extend(self._run_sweep("yaw", list(reversed(yaw_targets)), "reverse"))
            samples.extend(self._run_sweep("pitch", pitch_targets, "forward"))
            samples.extend(self._run_sweep("pitch", list(reversed(pitch_targets)), "reverse"))
            samples.extend(self._run_zero_offset_estimation())
        finally:
            self._stop_rosbag_record()
            self._write_summary(samples)
            if self._csv_handle is not None:
                self._csv_handle.close()
        return samples

    def _run_encoder_verification(self) -> List[CalibrationSample]:
        self.get_logger().info("Phase 1/3: encoder verification")
        samples: List[CalibrationSample] = []
        step = float(self.get_parameter("verification_step_deg").value)
        hold_sec = float(self.get_parameter("verification_hold_sec").value)
        for axis in ("yaw", "pitch"):
            for direction in ("positive", "negative", "return"):
                command = 0.0
                if direction == "positive":
                    command = step
                elif direction == "negative":
                    command = -step
                if axis == "yaw":
                    self._command_angle(yaw_deg=command, pitch_deg=0.0)
                else:
                    self._command_angle(yaw_deg=0.0, pitch_deg=command)
                self._spin_for(hold_sec)
                sample = self._capture_sample(
                    phase="verification",
                    axis=axis,
                    sweep_direction=direction,
                    commanded_deg=command,
                    note=f"{axis} {direction} verification step",
                )
                samples.append(sample)
        self._command_angle(yaw_deg=0.0, pitch_deg=0.0)
        self._spin_for(hold_sec)
        return samples

    def _run_sweep(
        self,
        axis: str,
        targets_deg: List[float],
        direction: str,
    ) -> List[CalibrationSample]:
        self.get_logger().info(f"Phase 2/3: {axis} sweep ({direction})")
        samples: List[CalibrationSample] = []
        settle = float(self.get_parameter("settle_time_sec").value)
        for target_deg in targets_deg:
            if axis == "yaw":
                self._command_angle(yaw_deg=target_deg, pitch_deg=0.0)
            else:
                self._command_angle(yaw_deg=0.0, pitch_deg=target_deg)
            self._spin_for(settle)
            sample = self._capture_sample(
                phase="sweep",
                axis=axis,
                sweep_direction=direction,
                commanded_deg=target_deg,
                note=f"{axis} sweep sample",
            )
            samples.append(sample)
        return samples

    def _run_zero_offset_estimation(self) -> List[CalibrationSample]:
        self.get_logger().info("Phase 3/3: zero-offset estimation")
        samples: List[CalibrationSample] = []
        if not bool(self.get_parameter("enable_checkerboard").value):
            samples.append(
                self._capture_sample(
                    phase="zero_offset",
                    axis="all",
                    sweep_direction="skipped",
                    commanded_deg=None,
                    note="checkerboard estimation disabled",
                )
            )
            return samples
        measurements = int(self.get_parameter("zero_offset_measurements").value)
        self._command_angle(yaw_deg=0.0, pitch_deg=0.0)
        self._spin_for(float(self.get_parameter("settle_time_sec").value))
        for _ in range(measurements):
            sample = self._capture_sample(
                phase="zero_offset",
                axis="all",
                sweep_direction="hold",
                commanded_deg=0.0,
                note="checkerboard zero-offset sample",
            )
            samples.append(sample)
            self._spin_for(0.3)
        return samples

    def _command_angle(self, yaw_deg: Optional[float], pitch_deg: Optional[float]) -> None:
        if yaw_deg is None or pitch_deg is None:
            current_yaw, current_pitch, _ = self.cam.getAttitude()
            if yaw_deg is None:
                yaw_deg = current_yaw
            if pitch_deg is None:
                pitch_deg = current_pitch
        self.cam.requestSetAngles(float(yaw_deg), float(pitch_deg))

    def _read_encoder_state(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        try:
            self.cam.requestGimbalEncoderAngle()
        except Exception as exc:  # pragma: no cover - hardware path
            self.get_logger().debug(f"Encoder request failed: {exc}")
        self._spin_for(0.15)
        encoder_yaw, encoder_pitch, encoder_roll = self.cam.getGimbalEncoderAngles()
        if encoder_yaw == 0.0 and encoder_pitch == 0.0 and encoder_roll == 0.0:
            att_yaw, att_pitch, att_roll = self.cam.getAttitude()
            return att_yaw, att_pitch, att_roll
        return encoder_yaw, encoder_pitch, encoder_roll

    def _estimate_camera_angles(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        if cv2 is None:
            return None, None, None
        if self.bridge_image is None or self.camera_info is None:
            return None, None, None
        found, corners, object_points = self._detect_checkerboard(self.bridge_image, self.camera_info)
        if not found or corners is None or object_points is None:
            return None, None, None
        camera_matrix = np.array(self.camera_info.k, dtype=np.float64).reshape((3, 3))
        distortion = np.array(self.camera_info.d, dtype=np.float64)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None, None, None
        board_center = tvec.reshape(3)
        yaw_deg = math.degrees(math.atan2(board_center[0], board_center[2]))
        pitch_deg = math.degrees(
            math.atan2(-board_center[1], math.sqrt(board_center[0] ** 2 + board_center[2] ** 2))
        )
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll_deg = math.degrees(math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2]))
        else:
            roll_deg = 0.0
        return yaw_deg, pitch_deg, roll_deg

    def _detect_checkerboard(
        self,
        image: np.ndarray,
        camera_info: CameraInfo,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        del camera_info
        if cv2 is None:
            return False, None, None
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        pattern_size = (
            int(self.get_parameter("checkerboard_cols").value),
            int(self.get_parameter("checkerboard_rows").value),
        )
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            return False, None, None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        square_size = float(self.get_parameter("checkerboard_square_size_m").value)
        cols, rows = pattern_size
        object_points = np.zeros((rows * cols, 3), np.float32)
        grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        object_points[:, :2] = grid * square_size
        return True, refined, object_points

    def _capture_sample(
        self,
        phase: str,
        axis: str,
        sweep_direction: str,
        commanded_deg: Optional[float],
        note: str,
    ) -> CalibrationSample:
        encoder_yaw, encoder_pitch, encoder_roll = self._read_encoder_state()
        camera_yaw, camera_pitch, camera_roll = self._estimate_camera_angles()
        sample = CalibrationSample(
            phase=phase,
            axis=axis,
            sweep_direction=sweep_direction,
            commanded_deg=commanded_deg,
            encoder_yaw_deg=encoder_yaw,
            encoder_pitch_deg=encoder_pitch,
            encoder_roll_deg=encoder_roll,
            camera_yaw_deg=camera_yaw,
            camera_pitch_deg=camera_pitch,
            camera_roll_deg=camera_roll,
            timestamp_sec=time.time(),
            note=note,
        )
        self._record_sample(sample)
        return sample

    def _record_sample(self, sample: CalibrationSample) -> None:
        if self._csv_writer is None:
            raise RuntimeError("CSV writer not initialized")
        self._csv_writer.writerow(asdict(sample))
        self._csv_handle.flush()

    def _verification_summary(self, samples: List[CalibrationSample]) -> Dict[str, Any]:
        results: Dict[str, Any] = {}
        for axis, key in (("yaw", "encoder_yaw_deg"), ("pitch", "encoder_pitch_deg")):
            positive = next(
                (
                    sample
                    for sample in samples
                    if sample.phase == "verification"
                    and sample.axis == axis
                    and sample.sweep_direction == "positive"
                ),
                None,
            )
            negative = next(
                (
                    sample
                    for sample in samples
                    if sample.phase == "verification"
                    and sample.axis == axis
                    and sample.sweep_direction == "negative"
                ),
                None,
            )
            pos_value = getattr(positive, key) if positive else None
            neg_value = getattr(negative, key) if negative else None
            sign_ok = None
            if pos_value is not None and neg_value is not None:
                sign_ok = pos_value > neg_value
            results[axis] = {
                "positive_sample_deg": pos_value,
                "negative_sample_deg": neg_value,
                "positive_increases_encoder": sign_ok,
            }
        return results

    def _write_summary(self, samples: List[CalibrationSample]) -> None:
        summary = {
            "session_name": self.get_parameter("session_name").value,
            "sample_count": len(samples),
            "verification": self._verification_summary(samples),
            "hysteresis_max_deg": compute_hysteresis(samples),
            "zero_offset_deg": compute_zero_offset_stats(samples),
            "paths": {
                "csv": str(self.session_paths.csv_path),
                "bag_dir": str(self.session_paths.bag_dir),
                "notes": str(self.session_paths.notes_path),
            },
        }
        self.session_paths.summary_json_path.write_text(
            json.dumps(summary, indent=2) + "\n",
            encoding="utf-8",
        )

    def _start_rosbag_record(self) -> Optional[subprocess.Popen]:
        if not bool(self.get_parameter("enable_rosbag").value):
            return None
        topics = list(self.get_parameter("record_topics").value)
        if not topics:
            topics = [
                self.get_parameter("image_topic").value,
                self.get_parameter("camera_info_topic").value,
                self.get_parameter("gimbal_state_topic").value,
            ]
        command = ["ros2", "bag", "record", "-o", str(self.session_paths.bag_dir)] + topics
        try:
            self._bag_process = subprocess.Popen(command)
            self.get_logger().info(f"Started rosbag record: {' '.join(command)}")
        except FileNotFoundError:
            self.get_logger().warning("ros2 bag not available; continuing without rosbag capture")
            self._bag_process = None
        return self._bag_process

    def _stop_rosbag_record(self) -> None:
        if self._bag_process is None:
            return
        self._bag_process.send_signal(signal.SIGINT)
        try:
            self._bag_process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self._bag_process.kill()
        self._bag_process = None

    def _shutdown_sdk(self) -> None:
        try:
            self.cam.disconnect()
        except Exception as exc:  # pragma: no cover - hardware cleanup
            self.get_logger().warning(f"SDK disconnect failed: {exc}")

def main() -> int:
    rclpy.init()
    node = GimbalCalibrationNode()
    try:
        node.run()
    finally:
        node._shutdown_sdk()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
