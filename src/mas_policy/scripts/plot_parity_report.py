#!/usr/bin/env python3
"""Generate parity report plot and statistics.

Replays trajectory dumps through the deploy-side policy and value networks,
compares against training-side outputs, and produces a visual report.

Usage:
    cd src/mas_policy
    source ~/ros2_humble/install/setup.bash && source ~/mas/install/setup.bash
    python3 scripts/plot_parity_report.py
"""

import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

from mas_policy.policy_loader import load_checkpoint, PolicyNetRNN, ValueNetRNN

# Paths
_BASE = Path(__file__).parent.parent
_DATA_DIR = _BASE / "test" / "data"
_CKPT = _BASE / "models" / "2026-04-01_13-19-18_mappo_rnn_torch_928b9585f2_min_lr_tuning" / "checkpoints" / "agent_400000.pt"

ACTION_LABELS = ['vx', 'vy', 'vz', 'yaw_rate', 'gimbal_az', 'gimbal_el', 'zoom']
DUMP_FILES = [
    ("trajectory_dump_step0.npz", "Checkpoint step 0 (untrained scaler)"),
    ("trajectory_dump_step400000.npz", "Checkpoint step 400k (trained scaler)"),
]


def replay(policy, scaler, value_net, shared_scaler, obs, shared):
    """Replay trajectory through deploy-side networks."""
    T = len(obs)
    hidden_p = policy.init_hidden(torch.device("cpu"))
    hidden_v = value_net.init_hidden(torch.device("cpu"))
    actions, values = [], []
    with torch.no_grad():
        for t in range(T):
            obs_t = torch.tensor(obs[t], dtype=torch.float32).unsqueeze(0)
            a_t, hidden_p = policy(scaler.normalize(obs_t), hidden_p)
            actions.append(a_t.squeeze(0).numpy())
            s_t = torch.tensor(shared[t], dtype=torch.float32).unsqueeze(0)
            v_t, hidden_v = value_net(shared_scaler.normalize(s_t), hidden_v)
            values.append(v_t.item())
    return np.stack(actions), np.array(values)


def print_stats(name, errors, boundary_mask):
    """Print per-action-dim error statistics."""
    clean = ~boundary_mask
    print(f"\n=== {name} ===")
    print(f"  Total steps: {len(errors)}, boundary steps: {boundary_mask.sum()}")
    print(f"  Max error (clean): {errors[clean].max():.2e}")
    print(f"  Mean error (clean): {errors[clean].mean():.2e}")
    print(f"  99th percentile:    {np.percentile(errors[clean], 99):.2e}")
    for d in range(7):
        print(f"  {ACTION_LABELS[d]:>12s}: max={errors[clean, d].max():.2e}  mean={errors[clean, d].mean():.2e}")


def main():
    policy, scaler, value_net, shared_scaler = load_checkpoint(
        str(_CKPT), obs_dim=52, action_dim=7, architecture="mappo_rnn",
        hidden_size=64, gru_hidden_size=64, gru_num_layers=1, num_agents=2,
        device=torch.device("cpu"),
    )

    # Filter to available dumps
    available = [(f, t) for f, t in DUMP_FILES if (_DATA_DIR / f).exists()]
    if not available:
        print("No trajectory dumps found in test/data/. Run dump_trajectory.py first.")
        return

    ncols = len(available)
    fig, axes = plt.subplots(4, ncols, figsize=(9 * ncols, 20))
    if ncols == 1:
        axes = axes[:, np.newaxis]
    fig.suptitle("Policy Deployment Parity Report", fontsize=16, fontweight="bold")

    for col, (dump_file, dump_title) in enumerate(available):
        dump = dict(np.load(_DATA_DIR / dump_file, allow_pickle=True))
        obs = dump["drone_0/obs"]
        act_train = dump["drone_0/action"]
        shared = dump["drone_0/shared_state"]
        T = len(obs)
        steps = np.arange(T) * 0.04

        act_deploy, val_deploy = replay(policy, scaler, value_net, shared_scaler, obs, shared)
        errors = np.abs(act_deploy - act_train)
        boundary = errors.max(axis=1) > 0.5

        print_stats(dump_file, errors, boundary)

        # Row 0: Per-action-dim error (log scale)
        ax = axes[0, col]
        for d in range(7):
            ax.plot(steps, errors[:, d], alpha=0.7, linewidth=0.8, label=ACTION_LABELS[d])
        ax.axhline(1e-4, color="red", linestyle="--", alpha=0.5, label="atol=1e-4")
        ax.set_ylabel("|deploy - train|")
        ax.set_title(f"{dump_title}\nPer-action-dim absolute error")
        ax.legend(fontsize=7, ncol=4, loc="upper left")
        ax.set_yscale("log")
        ax.set_ylim(1e-8, 10)
        ax.grid(True, alpha=0.3)

        # Row 1: Velocity action trajectories
        ax = axes[1, col]
        for d in range(3):
            ax.plot(steps, act_train[:, d], "-", alpha=0.7, linewidth=1.2, label=f"{ACTION_LABELS[d]} (train)")
            ax.plot(steps, act_deploy[:, d], "--", alpha=0.7, linewidth=1.2, label=f"{ACTION_LABELS[d]} (deploy)")
        ax.set_ylabel("Raw action value")
        ax.set_title("Action trajectories (vx, vy, vz)")
        ax.legend(fontsize=7, ncol=3, loc="upper left")
        ax.grid(True, alpha=0.3)

        # Row 2: Gimbal + zoom actions
        ax = axes[2, col]
        for d in [4, 5, 6]:
            ax.plot(steps, act_train[:, d], "-", alpha=0.7, linewidth=1.2, label=f"{ACTION_LABELS[d]} (train)")
            ax.plot(steps, act_deploy[:, d], "--", alpha=0.7, linewidth=1.2, label=f"{ACTION_LABELS[d]} (deploy)")
        ax.set_ylabel("Raw action value")
        ax.set_title("Action trajectories (gimbal_az, gimbal_el, zoom)")
        ax.legend(fontsize=7, ncol=3, loc="upper left")
        ax.grid(True, alpha=0.3)

        # Row 3: Value function
        ax = axes[3, col]
        ax.plot(steps, val_deploy, "b-", linewidth=1.2, label="V(s) deploy")
        ax.set_ylabel("Value V(s)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Value network output (true shared state)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = _DATA_DIR / "parity_report.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nSaved to {out_path}")


if __name__ == "__main__":
    main()
