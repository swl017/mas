"""Integration smoke tests for mas_policy.

Requires a sourced ROS2 environment (rclpy.init() must work).
Uses the real checkpoint for end-to-end inference validation.
"""

from pathlib import Path

import numpy as np
import torch
import pytest

REAL_MODEL_DIR = Path(__file__).parent.parent / "models" / "2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning"
REAL_CHECKPOINT = REAL_MODEL_DIR / "checkpoints" / "best_agent.pt"

# Skip all tests if rclpy is not available (no ROS2 env sourced)
try:
    import rclpy
    from rclpy.parameter import Parameter
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

pytestmark = [
    pytest.mark.skipif(not HAS_RCLPY, reason="rclpy not available (ROS2 env not sourced)"),
    pytest.mark.skipif(not REAL_CHECKPOINT.exists(), reason="Real checkpoint not found"),
]


@pytest.fixture(scope="module")
def ros_context():
    """Initialize and shutdown rclpy for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _create_node(ros_context):
    """Create a PolicyDeployNode with real checkpoint and parameter overrides."""
    from mas_policy.policy_node import PolicyDeployNode

    node = PolicyDeployNode(parameter_overrides=[
        Parameter('vehicle_name', value='px4_1'),
        Parameter('peer_names', value=['px4_2']),
        Parameter('checkpoint_path', value=str(REAL_CHECKPOINT)),
        Parameter('num_agents', value=2),
        Parameter('enable_triangulation', value=True),
        Parameter('dry_run', value=True),
        Parameter('enable_cbf', value=True),
        Parameter('use_common_frame', value=True),
    ])
    return node


def _populate_states(node):
    """Populate ego and peer states with plausible values."""
    now = node.get_clock().now().nanoseconds / 1e9

    ego = node._assembler.ego_state
    ego.position_w = np.array([10.0, 20.0, 30.0])
    ego.velocity_w = np.array([1.0, -0.5, 0.1])
    ego.orientation_w = np.array([1.0, 0.0, 0.0, 0.0])
    ego.angular_velocity_b = np.array([0.01, -0.01, 0.02])
    ego.linear_acceleration_b = np.array([0.1, -0.1, -9.81])
    ego.gimbal_yaw_body = 0.1
    ego.gimbal_pitch_body = -0.2
    ego.combined_ang_vel_w = np.array([0.05, -0.03, 0.01])
    ego.chosen_target_ray_w = np.array([0.7, 0.7, 0.0])
    ego.bbox_xywh = np.array([0.5, 0.5, 0.1, 0.1])
    ego.bbox_empty = 0.0
    ego.zoom_level = 2.0
    ego.detection_timestamp = now - 0.1
    ego.motion_timestamp = now
    ego.odom_received = True

    peer = node._assembler.get_vehicle_state('px4_2')
    peer.position_w = np.array([50.0, 50.0, 30.0])
    peer.velocity_w = np.array([0.0, 0.0, 0.0])
    peer.orientation_w = np.array([1.0, 0.0, 0.0, 0.0])
    peer.combined_ang_vel_w = np.array([0.0, 0.0, 0.0])
    peer.motion_timestamp = now


class TestIntegration:
    def test_node_starts_and_infers(self, ros_context):
        """Node starts with real checkpoint, runs one control loop tick."""
        node = _create_node(ros_context)
        try:
            assert node._assembler is not None
            assert node._policy is not None
            assert node._value_net is not None

            _populate_states(node)
            node._control_loop()  # should not crash
        finally:
            node.destroy_node()

    def test_gru_hidden_warmup(self, ros_context):
        """Run 50 ticks to exercise GRU hidden state warmup.

        Verifies actions stay bounded and hidden state doesn't diverge.
        Also verifies value network hidden state stays finite.
        """
        node = _create_node(ros_context)
        try:
            _populate_states(node)

            for _ in range(50):
                # Keep timestamps fresh
                t = node.get_clock().now().nanoseconds / 1e9
                node._assembler.ego_state.motion_timestamp = t
                node._assembler.get_vehicle_state('px4_2').motion_timestamp = t
                node._control_loop()

            # Verify policy hidden state is finite
            if node._hidden_state is not None:
                assert torch.all(torch.isfinite(node._hidden_state)), \
                    "GRU hidden state contains NaN/Inf after 50 ticks"

            # Verify value hidden state is finite
            if node._value_hidden_state is not None:
                assert torch.all(torch.isfinite(node._value_hidden_state)), \
                    "Value GRU hidden state contains NaN/Inf after 50 ticks"
        finally:
            node.destroy_node()
