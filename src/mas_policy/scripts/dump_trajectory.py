#!/usr/bin/env python3
"""Dump observation/action/value trajectories from IsaacLab training env.

Runs in conda env (env_isaaclab) — NO ROS2 imports.
Steps the training env for 2 episodes across N parallel envs, collecting
the full inference trajectory for deploy-side parity verification.

Usage:
    conda activate env_isaaclab
    python dump_trajectory.py \
        --checkpoint /path/to/agent_400000.pt \
        --task Isaac-Iris-MA6-Direct-v0 \
        --num_envs 16 \
        --output trajectory_dump.npz
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Dump trajectory for deploy parity test")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to SKRL .pt checkpoint")
parser.add_argument("--task", type=str, default="Isaac-Iris-MA6-Direct-Test-v0", help="Task name")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel environments")
parser.add_argument("--output", type=str, default="trajectory_dump.npz", help="Output .npz path")
parser.add_argument("--max_steps", type=int, default=250, help="Max steps per episode (2 episodes)")
parser.add_argument("--env_idx", type=int, default=0, help="Env index to extract for single-env dump")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- All non-Isaac imports AFTER AppLauncher ---

import os
import sys
import copy
import pickle
import numpy as np
import torch
import gymnasium as gym
import yaml
from typing import Optional, Sequence

# mappo_rnn.py lives alongside the SKRL training scripts
_skrl_scripts = os.path.expanduser(
    "~/IsaacPX4/IsaacLab/scripts/reinforcement_learning/skrl"
)
if _skrl_scripts not in sys.path:
    sys.path.insert(0, _skrl_scripts)
from mappo_rnn import MAPPO_RNN, MAPPO_RNN_DEFAULT_CONFIG, MAPPORNNPolicy, MAPPORNNValue
from skrl.memories.torch import RandomMemory
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.utils import set_seed

from isaaclab_rl.skrl import SkrlVecEnvWrapper
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.envs import DirectMARLEnv


def get_run_dir(checkpoint_path: str) -> str:
    """Derive run directory from checkpoint path."""
    checkpoint_path = os.path.abspath(checkpoint_path)
    parent = os.path.dirname(checkpoint_path)
    if os.path.basename(parent) == "checkpoints":
        return os.path.dirname(parent)
    return parent


def load_agent_config(run_dir: str) -> Optional[dict]:
    """Load agent config from run_dir/params/agent.pkl or agent.yaml."""
    pkl_path = os.path.join(run_dir, "params", "agent.pkl")
    if os.path.exists(pkl_path):
        with open(pkl_path, 'rb') as f:
            config = pickle.load(f)
            return config if isinstance(config, dict) else None
    yaml_path = os.path.join(run_dir, "params", "agent.yaml")
    if os.path.exists(yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception:
            return None
    return None


def main():
    set_seed(42)

    # --- Create env ---
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=True,
    )
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    assert isinstance(env.unwrapped, DirectMARLEnv)

    env = SkrlVecEnvWrapper(env, ml_framework="torch")
    device = env.device

    possible_agents = env.possible_agents
    num_agents = len(possible_agents)
    print(f"[INFO] Agents: {possible_agents}, device: {device}")

    # --- Shared observation space ---
    try:
        shared_observation_spaces = env.shared_observation_spaces
    except AttributeError:
        obs_shape = sum(space.shape[0] for space in env.observation_spaces.values())
        shared_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_shape,), dtype=np.float32)
        shared_observation_spaces = {aid: shared_space for aid in possible_agents}

    # --- Load config and models ---
    run_dir = get_run_dir(args_cli.checkpoint)
    agent_cfg = load_agent_config(run_dir)
    model_cfg = agent_cfg.get("models", {}) if agent_cfg else {}
    policy_cfg = model_cfg.get("policy", {})
    value_cfg = model_cfg.get("value", {})
    sequence_length = agent_cfg.get("agent", {}).get("sequence_length", 32) if agent_cfg else 32

    hidden_size = policy_cfg.get("hidden_size", 64)
    gru_num_layers = policy_cfg.get("gru_num_layers", 1)
    gru_hidden_size = policy_cfg.get("gru_hidden_size", 64)

    print(f"[INFO] Model: hidden={hidden_size}, gru_layers={gru_num_layers}, gru_hidden={gru_hidden_size}")

    # Create shared policy + value
    shared_policy = MAPPORNNPolicy(
        observation_space=env.observation_spaces[possible_agents[0]],
        action_space=env.action_spaces[possible_agents[0]],
        device=device,
        hidden_size=hidden_size,
        gru_num_layers=gru_num_layers,
        gru_hidden_size=gru_hidden_size,
        num_envs=env.num_envs,
        sequence_length=sequence_length,
        initial_log_std=policy_cfg.get("initial_log_std", -0.5),
        min_log_std=policy_cfg.get("min_log_std", -5.0),
        max_log_std=policy_cfg.get("max_log_std", 0.7),
    )

    shared_value = MAPPORNNValue(
        observation_space=shared_observation_spaces[possible_agents[0]],
        action_space=env.action_spaces[possible_agents[0]],
        device=device,
        hidden_size=value_cfg.get("hidden_size", hidden_size),
        gru_num_layers=value_cfg.get("gru_num_layers", gru_num_layers),
        gru_hidden_size=value_cfg.get("gru_hidden_size", gru_hidden_size),
        num_envs=env.num_envs,
        sequence_length=sequence_length,
    )

    # Load checkpoint weights
    checkpoint = torch.load(args_cli.checkpoint, map_location=device, weights_only=False)
    first_key = None
    for k, v in checkpoint.items():
        if isinstance(v, dict) and "policy" in v:
            first_key = k
            break
    if first_key:
        shared_policy.load_state_dict(checkpoint[first_key]["policy"])
        shared_value.load_state_dict(checkpoint[first_key]["value"])
    elif "policy" in checkpoint:
        shared_policy.load_state_dict(checkpoint["policy"])
        shared_value.load_state_dict(checkpoint["value"])
    else:
        raise RuntimeError("Cannot find policy weights in checkpoint")

    models = {aid: {"policy": shared_policy, "value": shared_value} for aid in possible_agents}
    memories = {aid: RandomMemory(memory_size=1, num_envs=env.num_envs, device=device) for aid in possible_agents}

    # --- Create MAPPO agent ---
    cfg = copy.deepcopy(MAPPO_RNN_DEFAULT_CONFIG)
    cfg["rollouts"] = 1
    cfg["random_timesteps"] = 0
    cfg["learning_starts"] = 0
    cfg["state_preprocessor"] = RunningStandardScaler
    cfg["state_preprocessor_kwargs"] = {"size": env.observation_spaces[possible_agents[0]], "device": device}
    cfg["shared_state_preprocessor"] = RunningStandardScaler
    cfg["shared_state_preprocessor_kwargs"] = {"size": shared_observation_spaces[possible_agents[0]], "device": device}
    cfg["value_preprocessor"] = RunningStandardScaler
    cfg["value_preprocessor_kwargs"] = {"size": 1, "device": device}
    cfg["experiment"] = {"write_interval": 0, "checkpoint_interval": 0, "wandb": False}

    agent = MAPPO_RNN(
        possible_agents=possible_agents,
        models=models,
        memories=memories,
        observation_spaces=env.observation_spaces,
        action_spaces=env.action_spaces,
        device=device,
        cfg=cfg,
        shared_observation_spaces=shared_observation_spaces,
    )
    agent.init()
    agent.set_mode("eval")

    # Load preprocessor states
    for aid in possible_agents:
        key = first_key or aid
        if key in checkpoint and isinstance(checkpoint[key], dict):
            ckpt_data = checkpoint[key]
        elif "state_preprocessor" in checkpoint:
            ckpt_data = checkpoint
        else:
            continue
        if ckpt_data.get("state_preprocessor") is not None:
            agent._state_preprocessor[aid].load_state_dict(ckpt_data["state_preprocessor"])
        if ckpt_data.get("shared_state_preprocessor") is not None:
            agent._shared_state_preprocessor[aid].load_state_dict(ckpt_data["shared_state_preprocessor"])
        if ckpt_data.get("value_preprocessor") is not None:
            agent._value_preprocessor[aid].load_state_dict(ckpt_data["value_preprocessor"])

    # Extract scaler params for deploy-side comparison
    first_agent = possible_agents[0]
    scaler_mean = agent._state_preprocessor[first_agent].running_mean.cpu().numpy()
    scaler_var = agent._state_preprocessor[first_agent].running_variance.cpu().numpy()
    shared_scaler_mean = agent._shared_state_preprocessor[first_agent].running_mean.cpu().numpy()
    shared_scaler_var = agent._shared_state_preprocessor[first_agent].running_variance.cpu().numpy()

    print(f"[INFO] Scaler loaded: obs_dim={len(scaler_mean)}, shared_dim={len(shared_scaler_mean)}")

    # --- Collection loop: 2 episodes ---
    ei = args_cli.env_idx  # which parallel env to extract
    collected = {aid: [] for aid in possible_agents}  # list of per-step dicts

    states, infos = env.reset()
    shared_states = env.unwrapped.state() if hasattr(env, 'unwrapped') else None

    episodes_done = 0
    step = 0

    print(f"[INFO] Collecting trajectories (2 episodes, env_idx={ei})...")

    while episodes_done < 2 and step < args_cli.max_steps * 2:
        with torch.inference_mode():
            actions, log_probs, outputs = agent.act(states, timestep=step, timesteps=10000)

            # Use mean_actions for deterministic evaluation (raw linear output, no tanh)
            for aid in possible_agents:
                actions[aid] = outputs[aid]["mean_actions"]

            # Record pre-step data
            for aid in possible_agents:
                obs_i = states[aid][ei].cpu().numpy()
                act_i = actions[aid][ei].cpu().numpy()

                # Extract RNN hidden state for this agent
                hidden_policy = None
                if agent._rnn_states[aid]["policy"]:
                    # shape: (layers, num_envs, hidden) -> extract env_idx
                    hidden_policy = agent._rnn_states[aid]["policy"][0][:, ei, :].cpu().numpy()

                record = {
                    "obs": obs_i,
                    "action": act_i,
                    "hidden_policy": hidden_policy,
                }

                # Shared state (for value network)
                if shared_states is not None:
                    record["shared_state"] = shared_states[ei].cpu().numpy()

                collected[aid].append(record)

            # Step environment
            next_states, rewards, terminated, truncated, infos = env.step(actions)
            shared_states = env.unwrapped.state() if hasattr(env, 'unwrapped') else None

            # Record transition for value computation
            infos_with_shared = dict(infos)
            if shared_states is not None:
                infos_with_shared["shared_states"] = shared_states
            agent.record_transition(
                states, actions, rewards, next_states, terminated, truncated, infos_with_shared, step, 10000
            )

            # Check episode end
            for aid in possible_agents:
                if (terminated[aid] | truncated[aid])[ei].item():
                    episodes_done += 1
                    print(f"[INFO] Episode boundary at step {step} (episodes_done={episodes_done})")
                    break

            # Reset RNN states for finished envs
            for aid in possible_agents:
                finished = (terminated[aid] | truncated[aid]).nonzero(as_tuple=False)
                if finished.numel():
                    for rnn_state in agent._rnn_states[aid]["policy"]:
                        rnn_state[:, finished[:, 0]] = 0
                    if agent.policies[aid] is not agent.values[aid]:
                        for rnn_state in agent._rnn_states[aid]["value"]:
                            rnn_state[:, finished[:, 0]] = 0

            states = next_states
            step += 1

    print(f"[INFO] Collected {step} steps across {episodes_done} episodes")

    # --- Assemble and save ---
    save_dict = {
        "scaler_mean": scaler_mean,
        "scaler_var": scaler_var,
        "shared_scaler_mean": shared_scaler_mean,
        "shared_scaler_var": shared_scaler_var,
        "num_agents": num_agents,
        "possible_agents": np.array(possible_agents),
        "total_steps": step,
    }

    for aid in possible_agents:
        records = collected[aid]
        if not records:
            continue
        save_dict[f"{aid}/obs"] = np.stack([r["obs"] for r in records])
        save_dict[f"{aid}/action"] = np.stack([r["action"] for r in records])
        if records[0]["hidden_policy"] is not None:
            save_dict[f"{aid}/hidden_policy"] = np.stack([r["hidden_policy"] for r in records])
        if "shared_state" in records[0]:
            save_dict[f"{aid}/shared_state"] = np.stack([r["shared_state"] for r in records])

    np.savez_compressed(args_cli.output, **save_dict)
    print(f"[INFO] Saved to {args_cli.output}")
    print(f"[INFO] Keys: {list(save_dict.keys())}")
    for aid in possible_agents:
        if f"{aid}/obs" in save_dict:
            print(f"[INFO]   {aid}/obs: {save_dict[f'{aid}/obs'].shape}")
            print(f"[INFO]   {aid}/action: {save_dict[f'{aid}/action'].shape}")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
