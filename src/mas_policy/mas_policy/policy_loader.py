"""Standalone policy network definitions and checkpoint loading.

Replicates the SKRL MAPPORNNPolicy and PPO MLP architectures as standalone
PyTorch nn.Modules for inference without SKRL dependencies.

Source reference: IsaacLab/scripts/reinforcement_learning/skrl/mappo_rnn.py:961-1083
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import torch
import torch.nn as nn

logger = logging.getLogger(__name__)


@dataclass
class ScalerState:
    """Running standard scaler state from training."""

    running_mean: torch.Tensor  # (obs_dim,)
    running_var: torch.Tensor   # (obs_dim,)
    count: float = 0.0

    def normalize(self, obs: torch.Tensor, eps: float = 1e-8) -> torch.Tensor:
        """Apply normalization: (obs - mean) / sqrt(var + eps)."""
        return (obs - self.running_mean) / torch.sqrt(self.running_var + eps)


class PolicyNetRNN(nn.Module):
    """Standalone MAPPO-RNN policy network for inference.

    Replicates MAPPORNNBaseModel + MAPPORNNPolicy from mappo_rnn.py.
    Architecture: Linear→ReLU→Linear→ReLU→GRU→Linear (mean action).
    """

    def __init__(
        self,
        obs_dim: int,
        action_dim: int,
        hidden_size: int = 64,
        gru_hidden_size: int = 64,
        gru_num_layers: int = 1,
    ):
        super().__init__()
        self.gru_hidden_size = gru_hidden_size
        self.gru_num_layers = gru_num_layers

        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
        )
        self.gru = nn.GRU(
            hidden_size, gru_hidden_size,
            num_layers=gru_num_layers, batch_first=True,
        )
        self.policy_layer = nn.Linear(gru_hidden_size, action_dim)

    def forward(
        self,
        obs: torch.Tensor,
        hidden: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Forward pass for single-step inference.

        Args:
            obs: Observation tensor (1, obs_dim).
            hidden: GRU hidden state (num_layers, 1, gru_hidden_size).

        Returns:
            mean_action: (1, action_dim) mean action (no sampling).
            new_hidden: Updated GRU hidden state.
        """
        x = self.net(obs)                    # (1, hidden_size)
        x = x.unsqueeze(1)                   # (1, 1, hidden_size)
        out, h = self.gru(x, hidden)         # (1, 1, gru_hidden_size), (layers, 1, gru_hidden_size)
        out = out.squeeze(1)                  # (1, gru_hidden_size)
        return self.policy_layer(out), h      # (1, action_dim), (layers, 1, gru_hidden_size)

    def init_hidden(self, device: torch.device = torch.device("cpu")) -> torch.Tensor:
        """Create zero-initialized hidden state."""
        return torch.zeros(self.gru_num_layers, 1, self.gru_hidden_size, device=device)


class PolicyNetMLP(nn.Module):
    """Standalone PPO MLP policy network for inference.

    Architecture: Linear→ELU→Linear→ELU→Linear (mean action).
    """

    def __init__(
        self,
        obs_dim: int,
        action_dim: int,
        hidden_size: int = 64,
    ):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_size),
            nn.ELU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ELU(),
            nn.Linear(hidden_size, action_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """Forward pass.

        Args:
            obs: Observation tensor (1, obs_dim).

        Returns:
            mean_action: (1, action_dim).
        """
        return self.net(obs)


def _strip_prefix(state_dict: dict, prefix: str) -> dict:
    """Remove a key prefix from state dict keys."""
    stripped = {}
    for k, v in state_dict.items():
        if k.startswith(prefix):
            stripped[k[len(prefix):]] = v
        else:
            stripped[k] = v
    return stripped


def load_checkpoint(
    checkpoint_path: str,
    obs_dim: int,
    action_dim: int,
    architecture: str = "mappo_rnn",
    hidden_size: int = 64,
    gru_hidden_size: int = 64,
    gru_num_layers: int = 1,
    device: torch.device = torch.device("cpu"),
) -> tuple[nn.Module, ScalerState]:
    """Load SKRL checkpoint and construct standalone policy network.

    SKRL checkpoints contain per-agent dicts with policy weights and
    preprocessor state. We load from the first agent (shared weights).

    Args:
        checkpoint_path: Path to .pt checkpoint file.
        obs_dim: Observation dimension.
        action_dim: Action dimension.
        architecture: "mappo_rnn" or "ppo_mlp".
        hidden_size: MLP hidden layer size.
        gru_hidden_size: GRU hidden size (RNN only).
        gru_num_layers: Number of GRU layers (RNN only).
        device: Torch device.

    Returns:
        Tuple of (policy_net, scaler_state).
    """
    path = Path(checkpoint_path)
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    logger.info(f"Loading checkpoint: {checkpoint_path}")
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)

    # SKRL saves checkpoints as dict. Find the policy weights.
    # Structure varies: could be top-level or nested under agent UIDs.
    policy_state_dict = None
    scaler_mean = None
    scaler_var = None

    # Try to find policy and scaler in the checkpoint
    if "policy" in ckpt:
        # Direct top-level format
        policy_state_dict = ckpt["policy"]
        if "state_preprocessor" in ckpt:
            sp = ckpt["state_preprocessor"]
            scaler_mean = sp.get("running_mean", sp.get("mean"))
            scaler_var = sp.get("running_var", sp.get("var"))
    else:
        # Try SKRL per-agent format: iterate over keys to find first agent
        for key, value in ckpt.items():
            if isinstance(value, dict):
                if "policy" in value:
                    policy_state_dict = value["policy"]
                    if "state_preprocessor" in value:
                        sp = value["state_preprocessor"]
                        scaler_mean = sp.get("running_mean", sp.get("mean"))
                        scaler_var = sp.get("running_var", sp.get("var"))
                    break
                # Nested checkpoint_modules format
                if "checkpoint_modules" in value:
                    modules = value["checkpoint_modules"]
                    if "policy" in modules:
                        policy_state_dict = modules["policy"]
                    if "state_preprocessor" in modules:
                        sp = modules["state_preprocessor"]
                        scaler_mean = sp.get("running_mean", sp.get("mean"))
                        scaler_var = sp.get("running_var", sp.get("var"))
                    break

    if policy_state_dict is None:
        # Last resort: try treating entire checkpoint as a flat state dict
        logger.warning("Could not find 'policy' key; attempting flat state dict load")
        policy_state_dict = ckpt

    # Construct network
    if architecture == "mappo_rnn":
        policy = PolicyNetRNN(obs_dim, action_dim, hidden_size, gru_hidden_size, gru_num_layers)
    elif architecture == "ppo_mlp":
        policy = PolicyNetMLP(obs_dim, action_dim, hidden_size)
    else:
        raise ValueError(f"Unknown architecture: {architecture}")

    # Load weights — try direct load, then try stripping common prefixes
    try:
        policy.load_state_dict(policy_state_dict, strict=False)
    except RuntimeError:
        # Try stripping "module." prefix (DataParallel wrapping)
        stripped = _strip_prefix(policy_state_dict, "module.")
        policy.load_state_dict(stripped, strict=False)

    policy.to(device)
    policy.eval()

    # Log loaded parameters
    loaded_keys = set(policy.state_dict().keys())
    ckpt_keys = set(policy_state_dict.keys())
    matched = loaded_keys & ckpt_keys
    missing = loaded_keys - ckpt_keys
    unexpected = ckpt_keys - loaded_keys
    logger.info(f"Loaded {len(matched)} parameters, {len(missing)} missing, {len(unexpected)} unexpected")
    if missing:
        logger.warning(f"Missing keys: {missing}")

    # Build scaler state
    if scaler_mean is not None and scaler_var is not None:
        scaler = ScalerState(
            running_mean=scaler_mean.to(device).float(),
            running_var=scaler_var.to(device).float(),
        )
        logger.info(f"Loaded observation scaler (dim={scaler.running_mean.shape[0]})")
    else:
        logger.warning("No observation scaler found in checkpoint — using identity normalization")
        scaler = ScalerState(
            running_mean=torch.zeros(obs_dim, device=device),
            running_var=torch.ones(obs_dim, device=device),
        )

    return policy, scaler
