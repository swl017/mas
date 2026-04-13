"""Tests for mas_policy.observation_assembler.

Uses sys.modules mocking to avoid requiring a ROS2 environment.
All mocks are scoped to this module and cleaned up after import.
"""

import sys
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

# Save original sys.modules state before mocking
_original_modules = {}
_mock_module_names = [
    "rclpy", "rclpy.node", "rclpy.qos",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "sensor_msgs", "sensor_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "vision_msgs", "vision_msgs.msg",
]

for _name in _mock_module_names:
    _original_modules[_name] = sys.modules.get(_name)

# Install mocks
_mock_rclpy = types.ModuleType("rclpy")
_mock_rclpy.node = types.ModuleType("rclpy.node")
_mock_rclpy.qos = types.ModuleType("rclpy.qos")
_mock_rclpy.node.Node = MagicMock
_mock_rclpy.qos.QoSProfile = MagicMock
_mock_rclpy.qos.QoSReliabilityPolicy = MagicMock()
_mock_rclpy.qos.QoSHistoryPolicy = MagicMock()

sys.modules["rclpy"] = _mock_rclpy
sys.modules["rclpy.node"] = _mock_rclpy.node
sys.modules["rclpy.qos"] = _mock_rclpy.qos

for _mod_name in [
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "sensor_msgs", "sensor_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "vision_msgs", "vision_msgs.msg",
]:
    sys.modules[_mod_name] = types.ModuleType(_mod_name)

sys.modules["geometry_msgs.msg"].PoseWithCovarianceStamped = MagicMock
sys.modules["geometry_msgs.msg"].Vector3 = MagicMock
sys.modules["geometry_msgs.msg"].Vector3Stamped = MagicMock
sys.modules["nav_msgs.msg"].Odometry = MagicMock
sys.modules["sensor_msgs.msg"].CameraInfo = MagicMock
sys.modules["sensor_msgs.msg"].Imu = MagicMock
sys.modules["std_msgs.msg"].Bool = MagicMock
sys.modules["std_msgs.msg"].Float32 = MagicMock
sys.modules["std_msgs.msg"].Float64 = MagicMock
sys.modules["vision_msgs.msg"].Detection2DArray = MagicMock

# Import the module under test while mocks are active
from mas_policy.observation_assembler import ObservationAssembler, VehicleState

# Restore original sys.modules so other test files see the real modules
for _name in _mock_module_names:
    if _original_modules[_name] is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _original_modules[_name]

# Also remove cached mas_policy submodules so they can be re-imported with real deps
for _key in list(sys.modules.keys()):
    if _key.startswith("mas_policy.") and _key != "mas_policy.utils":
        sys.modules.pop(_key, None)


def _make_mock_node():
    """Create a minimal mock node that records subscriptions."""
    node = MagicMock()
    node.create_subscription = MagicMock(return_value=MagicMock())
    clock = MagicMock()
    clock.now.return_value.nanoseconds = 1_000_000_000_000  # 1000.0 seconds
    node.get_clock.return_value = clock
    return node


class TestObsDim:
    def test_2_agents_tri(self):
        node = _make_mock_node()
        asm = ObservationAssembler(node, "px4_1", ["px4_2"], enable_triangulation=True)
        assert asm.obs_dim == 52  # 30 + 16*1 + 6

    def test_3_agents_tri(self):
        node = _make_mock_node()
        asm = ObservationAssembler(node, "px4_1", ["px4_2", "px4_3"], enable_triangulation=True)
        assert asm.obs_dim == 68  # 30 + 16*2 + 6


class TestAssemble:
    def _make_assembler(self, num_peers=1, enable_tri=True):
        node = _make_mock_node()
        peers = [f"px4_{i+2}" for i in range(num_peers)]
        asm = ObservationAssembler(node, "px4_1", peers, enable_triangulation=enable_tri)
        return asm

    def _populate_state(self, state: VehicleState):
        """Fill a VehicleState with non-zero values."""
        state.position_w = np.array([1.0, 2.0, 3.0])
        state.velocity_w = np.array([0.5, -0.5, 0.1])
        state.orientation_w = np.array([1.0, 0.0, 0.0, 0.0])
        state.angular_velocity_b = np.array([0.01, -0.01, 0.02])
        state.linear_acceleration_b = np.array([0.0, 0.0, -9.81])
        state.gimbal_yaw_body = 0.1
        state.gimbal_pitch_body = -0.2
        state.combined_ang_vel_w = np.array([0.05, -0.03, 0.01])
        state.chosen_target_ray_w = np.array([0.7, 0.7, 0.0])
        state.bbox_xywh = np.array([0.5, 0.5, 0.1, 0.1])
        state.bbox_empty = 0.0
        state.detection_timestamp = 999.0
        state.zoom_level = 2.0
        state.motion_timestamp = 999.5
        state.odom_received = True

    def test_output_shape_2_agents_tri(self):
        asm = self._make_assembler(num_peers=1, enable_tri=True)
        for name in asm.all_names:
            self._populate_state(asm.get_vehicle_state(name))
        obs = asm.assemble()
        assert len(obs) == asm.obs_dim == 52

    def test_output_shape_3_agents_tri(self):
        asm = self._make_assembler(num_peers=2, enable_tri=True)
        for name in asm.all_names:
            self._populate_state(asm.get_vehicle_state(name))
        obs = asm.assemble()
        assert len(obs) == asm.obs_dim == 68

    def test_gimbal_yaw_no_offset(self):
        """Verify obs[15] is the raw gimbal_yaw_body, no offset subtracted."""
        asm = self._make_assembler(num_peers=1, enable_tri=True)
        for name in asm.all_names:
            self._populate_state(asm.get_vehicle_state(name))
        asm.get_vehicle_state("px4_1").gimbal_yaw_body = 0.42
        obs = asm.assemble()
        np.testing.assert_allclose(obs[15], 0.42, atol=1e-10)

    def test_bbox_aoi_clipped(self):
        """bbox_aoi should be clipped to max_bbox_aoi."""
        asm = self._make_assembler(num_peers=1, enable_tri=True)
        for name in asm.all_names:
            self._populate_state(asm.get_vehicle_state(name))
        asm.get_vehicle_state("px4_1").detection_timestamp = 1.0  # 999s ago
        obs = asm.assemble()
        assert obs[23] <= 20.0
        np.testing.assert_allclose(obs[23], 20.0, atol=1e-10)

    def test_combined_ang_vel_from_state(self):
        """Verify obs[20:23] uses the cached combined_ang_vel_w, not computed."""
        asm = self._make_assembler(num_peers=1, enable_tri=True)
        for name in asm.all_names:
            self._populate_state(asm.get_vehicle_state(name))
        expected = np.array([0.11, -0.22, 0.33])
        asm.get_vehicle_state("px4_1").combined_ang_vel_w = expected
        obs = asm.assemble()
        np.testing.assert_allclose(obs[20:23], expected, atol=1e-10)
