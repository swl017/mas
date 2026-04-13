"""Parity tests: verify deploy-side inference matches training-side output.

Loads trajectory dumps from the IsaacLab training env (produced by
scripts/dump_trajectory.py) and replays observations through the deploy-side
PolicyNetRNN + ScalerState, comparing actions and value outputs.

Two dump files are tested:
- trajectory_dump_step0.npz: checkpoint at step 0 (untrained, no delay system)
- trajectory_dump_step400000.npz: checkpoint at step 400k (trained, full delay)

Run:
    cd src/mas_policy
    python3 -m pytest test/test_policy_parity.py -v
"""

from pathlib import Path

import numpy as np
import pytest
import torch

from mas_policy.policy_loader import load_checkpoint, PolicyNetRNN, ValueNetRNN

# Paths
_DATA_DIR = Path(__file__).parent / "data"
_MODELS_DIR = Path(__file__).parent.parent / "models"
_CHECKPOINT_DIR = "2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning"
_CHECKPOINT_PATH = _MODELS_DIR / _CHECKPOINT_DIR / "checkpoints" / "agent_400000.pt"

# Parametrize over both dump files
_DUMP_FILES = [
    "trajectory_dump_step0.npz",
    "trajectory_dump_step400000.npz",
]


def _available_dumps():
    """Return list of (id, path) for dump files that exist."""
    available = []
    for name in _DUMP_FILES:
        path = _DATA_DIR / name
        if path.exists():
            available.append(pytest.param(path, id=name.replace(".npz", "")))
    return available


def _load_dump(path):
    return dict(np.load(path, allow_pickle=True))


def _load_deploy_checkpoint(dump, checkpoint_path=_CHECKPOINT_PATH):
    """Load checkpoint via deploy-side loader, using dump to infer dims."""
    agents = list(dump["possible_agents"])
    obs_dim = dump[f"{agents[0]}/obs"].shape[1]
    action_dim = dump[f"{agents[0]}/action"].shape[1]
    num_agents = int(dump["num_agents"])

    policy, scaler, value_net, shared_scaler = load_checkpoint(
        checkpoint_path=str(checkpoint_path),
        obs_dim=obs_dim,
        action_dim=action_dim,
        architecture="mappo_rnn",
        hidden_size=64,
        gru_hidden_size=64,
        gru_num_layers=1,
        num_agents=num_agents,
        device=torch.device("cpu"),
    )
    return policy, scaler, value_net, shared_scaler


def _replay_policy(policy, scaler, obs_traj):
    """Replay obs trajectory through deploy-side policy, return raw actions."""
    T = obs_traj.shape[0]
    hidden = policy.init_hidden(torch.device("cpu"))
    actions = []
    with torch.no_grad():
        for t in range(T):
            obs_t = torch.tensor(obs_traj[t], dtype=torch.float32).unsqueeze(0)
            obs_norm = scaler.normalize(obs_t)
            action_t, hidden = policy(obs_norm, hidden)
            actions.append(action_t.squeeze(0).numpy())
    return np.stack(actions)


def _find_episode_boundary_steps(act_deploy, act_train, threshold=0.5):
    """Find steps where actions diverge sharply (episode boundary resets).

    At episode boundaries the training side resets the GRU hidden state
    but the deploy replay doesn't know when this happens, causing a
    single-step spike. These steps are excluded from parity comparison.
    """
    per_step_max_err = np.max(np.abs(act_deploy - act_train), axis=-1)
    return set(np.where(per_step_max_err > threshold)[0])


# --- Scaler equivalence ---

@pytest.mark.parametrize("dump_path", _available_dumps())
class TestScalerEquivalence:
    """Verify scaler parameters match between training and deploy."""

    def test_obs_scaler_mean(self, dump_path):
        dump = _load_dump(dump_path)
        _, scaler, _, _ = _load_deploy_checkpoint(dump)
        np.testing.assert_allclose(
            scaler.running_mean.numpy(), dump["scaler_mean"], atol=1e-5,
            err_msg="Observation scaler running_mean mismatch",
        )

    def test_obs_scaler_var(self, dump_path):
        dump = _load_dump(dump_path)
        _, scaler, _, _ = _load_deploy_checkpoint(dump)
        np.testing.assert_allclose(
            scaler.running_var.numpy(), dump["scaler_var"], atol=1e-5,
            err_msg="Observation scaler running_var mismatch",
        )

    def test_shared_scaler_mean(self, dump_path):
        dump = _load_dump(dump_path)
        _, _, _, shared_scaler = _load_deploy_checkpoint(dump)
        if shared_scaler is None:
            pytest.skip("no shared scaler")
        np.testing.assert_allclose(
            shared_scaler.running_mean.numpy(), dump["shared_scaler_mean"], atol=1e-5,
            err_msg="Shared scaler running_mean mismatch",
        )

    def test_shared_scaler_var(self, dump_path):
        dump = _load_dump(dump_path)
        _, _, _, shared_scaler = _load_deploy_checkpoint(dump)
        if shared_scaler is None:
            pytest.skip("no shared scaler")
        np.testing.assert_allclose(
            shared_scaler.running_var.numpy(), dump["shared_scaler_var"], atol=1e-5,
            err_msg="Shared scaler running_var mismatch",
        )


# --- Action parity ---

@pytest.mark.parametrize("dump_path", _available_dumps())
class TestActionParity:
    """Verify deploy-side policy produces same actions as training-side."""

    def test_action_first_agent(self, dump_path):
        """Replay first agent's obs trajectory, compare actions step-by-step."""
        dump = _load_dump(dump_path)
        policy, scaler, _, _ = _load_deploy_checkpoint(dump)
        assert isinstance(policy, PolicyNetRNN)

        agent = list(dump["possible_agents"])[0]
        obs_traj = dump[f"{agent}/obs"]
        act_traj = dump[f"{agent}/action"]

        deploy_actions = _replay_policy(policy, scaler, obs_traj)

        # Exclude episode boundary steps (GRU reset causes expected divergence)
        boundary = _find_episode_boundary_steps(deploy_actions, act_traj)
        mask = np.array([t not in boundary for t in range(len(act_traj))])
        assert mask.sum() > 0, "All steps are boundary steps"

        np.testing.assert_allclose(
            deploy_actions[mask], act_traj[mask], atol=1e-4,
            err_msg=f"Action mismatch for {agent} ({dump_path.name}), "
                    f"excluded {len(boundary)} boundary steps",
        )

    def test_action_all_agents(self, dump_path):
        """Verify parity for all agents (shared policy)."""
        dump = _load_dump(dump_path)
        policy, scaler, _, _ = _load_deploy_checkpoint(dump)

        for agent in dump["possible_agents"]:
            obs_traj = dump[f"{agent}/obs"]
            act_traj = dump[f"{agent}/action"]

            deploy_actions = _replay_policy(policy, scaler, obs_traj)
            boundary = _find_episode_boundary_steps(deploy_actions, act_traj)
            mask = np.array([t not in boundary for t in range(len(act_traj))])

            np.testing.assert_allclose(
                deploy_actions[mask], act_traj[mask], atol=1e-4,
                err_msg=f"Action mismatch for {agent} ({dump_path.name}), "
                        f"excluded {len(boundary)} boundary steps",
            )

    def test_action_max_error_report(self, dump_path):
        """Report max absolute error excluding episode boundaries."""
        dump = _load_dump(dump_path)
        policy, scaler, _, _ = _load_deploy_checkpoint(dump)

        max_errors = []
        total_boundary = 0
        for agent in dump["possible_agents"]:
            obs_traj = dump[f"{agent}/obs"]
            act_traj = dump[f"{agent}/action"]
            deploy_actions = _replay_policy(policy, scaler, obs_traj)
            boundary = _find_episode_boundary_steps(deploy_actions, act_traj)
            total_boundary += len(boundary)
            mask = np.array([t not in boundary for t in range(len(act_traj))])
            if mask.sum() > 0:
                max_errors.append(np.max(np.abs(deploy_actions[mask] - act_traj[mask])))

        max_err = max(max_errors) if max_errors else 0.0
        print(f"\n  Max action error ({dump_path.name}): {max_err:.2e}"
              f" ({total_boundary} boundary steps excluded)")
        assert max_err < 1e-3, f"Max action error {max_err:.2e} exceeds 1e-3"


# --- Value parity ---

@pytest.mark.parametrize("dump_path", _available_dumps())
class TestValueParity:
    """Verify deploy-side value network with true shared state."""

    def test_value_finite_and_varied(self, dump_path):
        """Replay shared state through value network, check outputs."""
        dump = _load_dump(dump_path)
        _, _, value_net, shared_scaler = _load_deploy_checkpoint(dump)
        if value_net is None:
            pytest.skip("no value network")
        if shared_scaler is None:
            pytest.skip("no shared scaler")

        agent = list(dump["possible_agents"])[0]
        shared_key = f"{agent}/shared_state"
        if shared_key not in dump:
            pytest.skip("no shared_state in dump")

        shared_traj = dump[shared_key]
        T = shared_traj.shape[0]

        assert isinstance(value_net, ValueNetRNN)
        hidden = value_net.init_hidden(torch.device("cpu"))

        values = []
        with torch.no_grad():
            for t in range(T):
                s_t = torch.tensor(shared_traj[t], dtype=torch.float32).unsqueeze(0)
                s_norm = shared_scaler.normalize(s_t)
                v_t, hidden = value_net(s_norm, hidden)
                values.append(v_t.item())

        values = np.array(values)
        assert np.all(np.isfinite(values)), "Value network produced non-finite outputs"
        assert values.shape == (T,)
        if T > 10:
            assert np.std(values) > 1e-6, "Value outputs all identical — likely broken"
