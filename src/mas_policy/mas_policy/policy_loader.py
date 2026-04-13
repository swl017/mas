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


class ValueNetRNN(nn.Module):
    """Standalone MAPPO-RNN value network for inference.

    Same architecture as PolicyNetRNN but outputs a scalar V(s).
    Input is shared state (obs_dim * num_agents) in MAPPO.
    """

    def __init__(
        self,
        input_dim: int,
        hidden_size: int = 64,
        gru_hidden_size: int = 64,
        gru_num_layers: int = 1,
    ):
        super().__init__()
        self.gru_hidden_size = gru_hidden_size
        self.gru_num_layers = gru_num_layers

        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
        )
        self.gru = nn.GRU(
            hidden_size, gru_hidden_size,
            num_layers=gru_num_layers, batch_first=True,
        )
        self.value_layer = nn.Linear(gru_hidden_size, 1)

    def forward(
        self,
        obs: torch.Tensor,
        hidden: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Forward pass for single-step inference.

        Args:
            obs: Shared state tensor (1, input_dim).
            hidden: GRU hidden state (num_layers, 1, gru_hidden_size).

        Returns:
            value: (1, 1) scalar value estimate.
            new_hidden: Updated GRU hidden state.
        """
        x = self.net(obs)
        x = x.unsqueeze(1)
        out, h = self.gru(x, hidden)
        out = out.squeeze(1)
        return self.value_layer(out), h

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


def _load_net(net: nn.Module, state_dict: dict, name: str, device: torch.device) -> None:
    """Load weights into a network, stripping prefixes if needed. Validates all keys present."""
    try:
        net.load_state_dict(state_dict, strict=False)
    except RuntimeError:
        stripped = _strip_prefix(state_dict, "module.")
        net.load_state_dict(stripped, strict=False)

    net.to(device)
    net.eval()

    loaded_keys = set(net.state_dict().keys())
    ckpt_keys = set(state_dict.keys())
    matched = loaded_keys & ckpt_keys
    missing = loaded_keys - ckpt_keys
    unexpected = ckpt_keys - loaded_keys
    logger.info(f"{name}: loaded {len(matched)} params, {len(missing)} missing, {len(unexpected)} unexpected")
    if missing:
        raise RuntimeError(
            f"Checkpoint missing {len(missing)} {name} keys (architecture mismatch?): {missing}"
        )


def _build_scaler(
    mean: torch.Tensor | None,
    var: torch.Tensor | None,
    expected_dim: int,
    device: torch.device,
    name: str = "observation",
) -> ScalerState:
    """Build a ScalerState from checkpoint tensors, with validation."""
    if mean is not None and var is not None:
        scaler = ScalerState(
            running_mean=mean.to(device).float(),
            running_var=var.to(device).float(),
        )
        scaler_dim = scaler.running_mean.shape[0]
        if scaler_dim != expected_dim:
            raise RuntimeError(
                f"{name} scaler dimension mismatch: scaler has {scaler_dim}, expected {expected_dim}"
            )
        logger.info(f"Loaded {name} scaler (dim={scaler_dim})")
        return scaler

    logger.warning(f"No {name} scaler found in checkpoint — using identity normalization")
    return ScalerState(
        running_mean=torch.zeros(expected_dim, device=device),
        running_var=torch.ones(expected_dim, device=device),
    )


def _extract_scaler_tensors(sp: dict) -> tuple[torch.Tensor | None, torch.Tensor | None]:
    """Extract mean/var tensors from a preprocessor dict."""
    mean = sp.get("running_mean", sp.get("mean"))
    var = sp.get("running_var", sp.get("running_variance", sp.get("var")))
    return mean, var


def load_checkpoint(
    checkpoint_path: str,
    obs_dim: int,
    action_dim: int,
    architecture: str = "mappo_rnn",
    hidden_size: int = 64,
    gru_hidden_size: int = 64,
    gru_num_layers: int = 1,
    num_agents: int = 1,
    device: torch.device = torch.device("cpu"),
    agent_id: str = "",
) -> tuple[nn.Module, ScalerState, ValueNetRNN | None, ScalerState | None]:
    """Load SKRL checkpoint and construct standalone policy + value networks.

    SKRL checkpoints contain per-agent dicts with policy weights, value weights,
    and preprocessor states. Policy weights are shared; preprocessor states
    (running_mean/running_variance) differ per agent.

    Args:
        checkpoint_path: Path to .pt checkpoint file.
        obs_dim: Observation dimension (per agent).
        action_dim: Action dimension.
        architecture: "mappo_rnn" or "ppo_mlp".
        hidden_size: MLP hidden layer size.
        gru_hidden_size: GRU hidden size (RNN only).
        gru_num_layers: Number of GRU layers (RNN only).
        num_agents: Number of agents (for value network shared state dim).
        device: Torch device.
        agent_id: Agent key to load preprocessor states from (e.g., "drone_0").
            If empty, loads from the first agent found in the checkpoint.

    Returns:
        Tuple of (policy_net, scaler_state, value_net_or_None, shared_scaler_or_None).
    """
    path = Path(checkpoint_path)
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    logger.info(f"Loading checkpoint: {checkpoint_path}")
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)

    # SKRL saves checkpoints as dict. Find the policy weights.
    # Structure varies: could be top-level or nested under agent UIDs.
    policy_state_dict = None
    value_state_dict = None
    scaler_mean = None
    scaler_var = None
    shared_scaler_mean = None
    shared_scaler_var = None

    # Try to find policy, value, and scalers in the checkpoint.
    # If agent_id is specified, load from that agent's entry.
    if "policy" in ckpt:
        # Direct top-level format
        policy_state_dict = ckpt["policy"]
        value_state_dict = ckpt.get("value")
        if "state_preprocessor" in ckpt:
            scaler_mean, scaler_var = _extract_scaler_tensors(ckpt["state_preprocessor"])
        if "shared_state_preprocessor" in ckpt:
            shared_scaler_mean, shared_scaler_var = _extract_scaler_tensors(ckpt["shared_state_preprocessor"])
    else:
        # SKRL per-agent format. Select agent by agent_id or fall back to first.
        def _extract_from_agent_dict(agent_dict):
            nonlocal policy_state_dict, value_state_dict
            nonlocal scaler_mean, scaler_var, shared_scaler_mean, shared_scaler_var
            if "policy" in agent_dict:
                policy_state_dict = agent_dict["policy"]
                value_state_dict = agent_dict.get("value")
                if "state_preprocessor" in agent_dict:
                    scaler_mean, scaler_var = _extract_scaler_tensors(agent_dict["state_preprocessor"])
                if "shared_state_preprocessor" in agent_dict:
                    shared_scaler_mean, shared_scaler_var = _extract_scaler_tensors(agent_dict["shared_state_preprocessor"])
                return True
            if "checkpoint_modules" in agent_dict:
                modules = agent_dict["checkpoint_modules"]
                if "policy" in modules:
                    policy_state_dict = modules["policy"]
                value_state_dict = modules.get("value")
                if "state_preprocessor" in modules:
                    scaler_mean, scaler_var = _extract_scaler_tensors(modules["state_preprocessor"])
                if "shared_state_preprocessor" in modules:
                    shared_scaler_mean, shared_scaler_var = _extract_scaler_tensors(modules["shared_state_preprocessor"])
                return True
            return False

        if agent_id and agent_id in ckpt and isinstance(ckpt[agent_id], dict):
            _extract_from_agent_dict(ckpt[agent_id])
            logger.info(f"Loaded from agent_id='{agent_id}'")
        else:
            if agent_id:
                available = [k for k in ckpt if isinstance(ckpt[k], dict) and "policy" in ckpt[k]]
                logger.warning(
                    f"agent_id='{agent_id}' not found in checkpoint. "
                    f"Available: {available}. Falling back to first agent."
                )
            for key, value in ckpt.items():
                if isinstance(value, dict) and _extract_from_agent_dict(value):
                    logger.info(f"Loaded from agent key '{key}'")
                    break

    if policy_state_dict is None:
        # Last resort: try treating entire checkpoint as a flat state dict
        logger.warning("Could not find 'policy' key; attempting flat state dict load")
        policy_state_dict = ckpt

    # --- Construct and load policy network ---
    if architecture == "mappo_rnn":
        policy = PolicyNetRNN(obs_dim, action_dim, hidden_size, gru_hidden_size, gru_num_layers)
    elif architecture == "ppo_mlp":
        policy = PolicyNetMLP(obs_dim, action_dim, hidden_size)
    else:
        raise ValueError(f"Unknown architecture: {architecture}")

    _load_net(policy, policy_state_dict, "policy", device)

    # --- Build observation scaler ---
    scaler = _build_scaler(scaler_mean, scaler_var, obs_dim, device, "observation")

    # --- Construct and load value network (optional) ---
    value_net = None
    shared_scaler = None
    if value_state_dict is not None and architecture == "mappo_rnn":
        # Infer shared state dim from value network's first layer
        first_weight = value_state_dict.get("net.0.weight")
        if first_weight is not None:
            shared_state_dim = first_weight.shape[1]
        else:
            shared_state_dim = obs_dim * num_agents

        value_net = ValueNetRNN(shared_state_dim, hidden_size, gru_hidden_size, gru_num_layers)
        _load_net(value_net, value_state_dict, "value", device)

        shared_scaler = _build_scaler(
            shared_scaler_mean, shared_scaler_var, shared_state_dim, device, "shared_state",
        )
        logger.info(f"Value network loaded: input_dim={shared_state_dim}")
    elif value_state_dict is None:
        logger.info("No value network found in checkpoint — V(s) monitoring disabled")

    return policy, scaler, value_net, shared_scaler
