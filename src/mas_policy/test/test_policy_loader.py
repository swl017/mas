"""Tests for mas_policy.policy_loader."""

import os
import tempfile
from pathlib import Path

import numpy as np
import torch
import pytest

from mas_policy.policy_loader import (
    load_checkpoint,
    PolicyNetRNN,
    ValueNetRNN,
    ScalerState,
)

REAL_MODEL_DIR = Path(__file__).parent.parent / "models" / "2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning"
REAL_CHECKPOINT = REAL_MODEL_DIR / "checkpoints" / "best_agent.pt"

# Real checkpoint config: 2 agents, obs_dim=52 (tri enabled), action_dim=7
REAL_OBS_DIM = 52
REAL_ACTION_DIM = 7
REAL_HIDDEN_SIZE = 64
REAL_GRU_HIDDEN_SIZE = 64


def _make_broken_checkpoint(tmp_path, obs_dim=10, action_dim=3, include_scaler=True, scaler_dim=None):
    """Create a minimal SKRL-format checkpoint for negative tests."""
    if scaler_dim is None:
        scaler_dim = obs_dim

    policy = PolicyNetRNN(obs_dim, action_dim, hidden_size=32, gru_hidden_size=32)
    ckpt = {
        "drone_0": {
            "policy": policy.state_dict(),
        }
    }
    if include_scaler:
        ckpt["drone_0"]["state_preprocessor"] = {
            "running_mean": torch.zeros(scaler_dim),
            "running_variance": torch.ones(scaler_dim),
            "current_count": torch.tensor(1000.0),
        }

    path = tmp_path / "test_ckpt.pt"
    torch.save(ckpt, path)
    return str(path)


REAL_NUM_AGENTS = 2


@pytest.mark.skipif(not REAL_CHECKPOINT.exists(), reason="Real checkpoint not found")
class TestLoadRealCheckpoint:
    def test_load_valid(self):
        policy, scaler, value_net, shared_scaler = load_checkpoint(
            str(REAL_CHECKPOINT),
            obs_dim=REAL_OBS_DIM,
            action_dim=REAL_ACTION_DIM,
            architecture="mappo_rnn",
            hidden_size=REAL_HIDDEN_SIZE,
            gru_hidden_size=REAL_GRU_HIDDEN_SIZE,
            num_agents=REAL_NUM_AGENTS,
        )
        # Verify forward pass shape
        obs = torch.randn(1, REAL_OBS_DIM)
        hidden = policy.init_hidden()
        action, new_hidden = policy(obs, hidden)
        assert action.shape == (1, REAL_ACTION_DIM)
        assert new_hidden.shape == hidden.shape

    def test_scaler(self):
        _, scaler, _, _ = load_checkpoint(
            str(REAL_CHECKPOINT),
            obs_dim=REAL_OBS_DIM,
            action_dim=REAL_ACTION_DIM,
            num_agents=REAL_NUM_AGENTS,
        )
        assert scaler.running_mean.shape[0] == REAL_OBS_DIM
        assert scaler.running_var.shape[0] == REAL_OBS_DIM
        assert torch.all(torch.isfinite(scaler.running_mean))
        assert torch.all(torch.isfinite(scaler.running_var))

    def test_normalize_output_finite(self):
        _, scaler, _, _ = load_checkpoint(
            str(REAL_CHECKPOINT),
            obs_dim=REAL_OBS_DIM,
            action_dim=REAL_ACTION_DIM,
            num_agents=REAL_NUM_AGENTS,
        )
        obs = torch.randn(1, REAL_OBS_DIM)
        normalized = scaler.normalize(obs)
        assert torch.all(torch.isfinite(normalized))

    def test_value_network_loads(self):
        """Value network loads from real checkpoint with correct architecture."""
        _, _, value_net, shared_scaler = load_checkpoint(
            str(REAL_CHECKPOINT),
            obs_dim=REAL_OBS_DIM,
            action_dim=REAL_ACTION_DIM,
            architecture="mappo_rnn",
            hidden_size=REAL_HIDDEN_SIZE,
            gru_hidden_size=REAL_GRU_HIDDEN_SIZE,
            num_agents=REAL_NUM_AGENTS,
        )
        assert value_net is not None
        assert shared_scaler is not None
        assert isinstance(value_net, ValueNetRNN)

        # Verify shared state scaler dim = obs_dim * num_agents
        shared_dim = REAL_OBS_DIM * REAL_NUM_AGENTS
        assert shared_scaler.running_mean.shape[0] == shared_dim

        # Verify forward pass: scalar output
        shared_obs = torch.randn(1, shared_dim)
        hidden = value_net.init_hidden()
        value, new_hidden = value_net(shared_obs, hidden)
        assert value.shape == (1, 1)
        assert new_hidden.shape == hidden.shape
        assert torch.all(torch.isfinite(value))

    def test_value_network_finite_over_50_ticks(self):
        """V(s) stays finite over 50 inference steps (GRU stability)."""
        _, scaler, value_net, shared_scaler = load_checkpoint(
            str(REAL_CHECKPOINT),
            obs_dim=REAL_OBS_DIM,
            action_dim=REAL_ACTION_DIM,
            architecture="mappo_rnn",
            hidden_size=REAL_HIDDEN_SIZE,
            gru_hidden_size=REAL_GRU_HIDDEN_SIZE,
            num_agents=REAL_NUM_AGENTS,
        )
        assert value_net is not None

        hidden = value_net.init_hidden()
        obs = torch.randn(1, REAL_OBS_DIM)
        obs_norm = scaler.normalize(obs)
        shared_obs = obs_norm.repeat(1, REAL_NUM_AGENTS)
        shared_obs_norm = shared_scaler.normalize(shared_obs)

        for _ in range(50):
            value, hidden = value_net(shared_obs_norm, hidden)
            assert torch.all(torch.isfinite(value)), "V(s) contains NaN/Inf"
            assert torch.all(torch.isfinite(hidden)), "Value GRU hidden contains NaN/Inf"


class TestLoadCheckpointValidation:
    def test_missing_keys_raises(self, tmp_path):
        """Checkpoint built for obs_dim=10 but loaded as obs_dim=20 → RuntimeError."""
        path = _make_broken_checkpoint(tmp_path, obs_dim=10, action_dim=3)
        with pytest.raises(RuntimeError):
            load_checkpoint(path, obs_dim=20, action_dim=3, hidden_size=32, gru_hidden_size=32)

    def test_scaler_dim_mismatch_raises(self, tmp_path):
        """Scaler dim=10 but obs_dim=10 with wrong scaler_dim=5."""
        path = _make_broken_checkpoint(tmp_path, obs_dim=10, action_dim=3, scaler_dim=5)
        with pytest.raises(RuntimeError, match="scaler dimension mismatch"):
            load_checkpoint(path, obs_dim=10, action_dim=3, hidden_size=32, gru_hidden_size=32)

    def test_no_scaler_fallback(self, tmp_path):
        """Checkpoint without state_preprocessor → identity scaler."""
        path = _make_broken_checkpoint(tmp_path, obs_dim=10, action_dim=3, include_scaler=False)
        policy, scaler, _, _ = load_checkpoint(path, obs_dim=10, action_dim=3, hidden_size=32, gru_hidden_size=32)
        # Identity scaler: mean=0, var=1
        np.testing.assert_allclose(scaler.running_mean.numpy(), 0.0)
        np.testing.assert_allclose(scaler.running_var.numpy(), 1.0)
