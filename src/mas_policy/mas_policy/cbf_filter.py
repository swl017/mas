"""Deployment CBF safety filter.

Ported from iris_ma6/cbf_safety/deploy_filter.py.
Simple distance-based CBF with inflated margin for delay robustness.
Uses closed-form halfspace projection (no QP solver needed).

Ref: iris_ma6/cbf_safety/cbf_cfg.py, safety_spec.md Section 2.4, 2.5, 4.2
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class DeploymentFilterConfig:
    """Configuration for deployment-time CBF filter."""

    D_s: float = 2.0
    """Physical safety distance in meters."""

    v_max: float = 15.0
    """Maximum expected agent velocity in m/s."""

    tau_delay_max: float = 0.2
    """Maximum communication delay in seconds."""

    tau_px4: float = 0.3
    """PX4 velocity controller time constant in seconds."""

    gamma_deploy: float = 1.0
    """CBF decay rate for deployment."""

    num_iters: int = 2
    """Gauss-Seidel projection iterations."""

    epsilon: float = 1e-8
    """Numerical stability constant."""

    @property
    def D_deploy(self) -> float:
        """Inflated safety distance: D_s + v_max * (tau_delay_max + tau_px4)."""
        return self.D_s + self.v_max * (self.tau_delay_max + self.tau_px4)


class DeploymentCBFFilter:
    """Distance-based CBF with inflated margin for deployment.

    Designed for delayed, noisy observations at deployment. Uses the simplest
    possible barrier with a margin that covers worst-case position error from
    communication delay.

    Usage:
        filter = DeploymentCBFFilter(config, num_agents)
        v_safe, info = filter.filter(v_nom, positions, velocities)
    """

    def __init__(self, cfg: DeploymentFilterConfig, num_agents: int):
        self.cfg = cfg
        self.num_agents = num_agents
        self.D_deploy = cfg.D_deploy
        self.D_deploy_sq = self.D_deploy ** 2

        logger.info(
            f"CBF filter initialized: D_s={cfg.D_s}m, D_deploy={self.D_deploy:.1f}m, "
            f"gamma={cfg.gamma_deploy}, iters={cfg.num_iters}"
        )

    def _project_halfspace(
        self,
        v: np.ndarray,
        a: np.ndarray,
        b: float,
    ) -> tuple[np.ndarray, bool]:
        """Closed-form projection onto halfspace {v : a^T v >= b}.

        Args:
            v: Velocity to project (3,).
            a: Constraint normal (3,).
            b: Constraint bound (scalar).

        Returns:
            v_safe: Projected velocity (3,).
            violated: Whether constraint was violated.
        """
        a_dot_v = np.dot(a, v)
        violated = a_dot_v < b

        if violated:
            a_norm_sq = np.dot(a, a) + self.cfg.epsilon
            scale = (b - a_dot_v) / a_norm_sq
            v_safe = v + scale * a
        else:
            v_safe = v

        return v_safe, violated

    def filter(
        self,
        v_nom: np.ndarray,
        positions: np.ndarray,
        neighbor_velocities: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, float]]:
        """Hard CBF-QP via iterative Gauss-Seidel projection.

        For each agent i, the CBF constraint with agent j is:
        2 * dp_ij^T * (v_i - v_j) + gamma * h_ij >= 0

        Args:
            v_nom: Nominal velocities from policy (N, 3).
            positions: Latest received (delayed) positions (N, 3).
            neighbor_velocities: Latest received velocities (N, 3).
                If None, assumes zero (most conservative).

        Returns:
            v_safe: Filtered safe velocities (N, 3).
            info: Dictionary with diagnostics.
        """
        v_safe = v_nom.copy()

        if neighbor_velocities is None:
            neighbor_velocities = np.zeros_like(v_nom)

        filter_active = [False] * self.num_agents

        for _ in range(self.cfg.num_iters):
            for i in range(self.num_agents):
                for j in range(self.num_agents):
                    if i == j:
                        continue

                    # Relative position: dp = p_i - p_j
                    dp = positions[i] - positions[j]

                    # Distance-based barrier: h = ||dp||^2 - D_deploy^2
                    h = np.dot(dp, dp) - self.D_deploy_sq

                    # Constraint: 2*dp^T*(v_i - v_j) + gamma*h >= 0
                    # Rearranged: (2*dp)^T * v_i >= -2*dp^T*v_j - gamma*h
                    a = 2.0 * dp
                    v_j = neighbor_velocities[j]
                    b = -np.dot(a, v_j) - self.cfg.gamma_deploy * h

                    v_safe[i], violated = self._project_halfspace(v_safe[i], a, b)
                    if violated:
                        filter_active[i] = True

        info = {
            "deploy_cbf/filter_active_fraction": sum(filter_active) / max(self.num_agents, 1),
            "deploy_cbf/agents_filtered": sum(filter_active),
        }

        return v_safe, info
