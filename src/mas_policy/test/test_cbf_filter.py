"""Tests for mas_policy.cbf_filter."""

import numpy as np
import pytest

from mas_policy.cbf_filter import DeploymentCBFFilter, DeploymentFilterConfig


def _default_cfg(**overrides) -> DeploymentFilterConfig:
    kwargs = dict(D_s=2.0, v_max=15.0, tau_delay_max=0.2, tau_px4=0.3, gamma_deploy=1.0, num_iters=2)
    kwargs.update(overrides)
    return DeploymentFilterConfig(**kwargs)


class TestCBFFilter:
    def test_no_constraint_far_agents(self):
        cfg = _default_cfg()
        cbf = DeploymentCBFFilter(cfg, num_agents=2)
        # Agents 100m apart — well beyond D_deploy
        positions = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        velocities = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        v_nom = np.array([[5.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        v_safe, info = cbf.filter(v_nom, positions, velocities)
        np.testing.assert_allclose(v_safe, v_nom, atol=1e-10)
        assert info["deploy_cbf/agents_filtered"] == 0

    def test_corrects_close_agents(self):
        cfg = _default_cfg()
        cbf = DeploymentCBFFilter(cfg, num_agents=2)
        D_deploy = cfg.D_deploy
        # Place agents just inside D_deploy, ego moving toward peer
        positions = np.array([[0.0, 0.0, 0.0], [D_deploy * 0.8, 0.0, 0.0]])
        velocities = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        v_nom = np.array([[10.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # ego rushes toward peer

        v_safe, info = cbf.filter(v_nom, positions, velocities)
        # Ego velocity should be reduced (x component smaller)
        assert v_safe[0, 0] < v_nom[0, 0]
        assert info["deploy_cbf/agents_filtered"] > 0

    def test_single_agent_noop(self):
        cfg = _default_cfg()
        cbf = DeploymentCBFFilter(cfg, num_agents=1)
        positions = np.array([[5.0, 5.0, 5.0]])
        velocities = np.array([[1.0, 2.0, 3.0]])
        v_nom = np.array([[10.0, -5.0, 3.0]])

        v_safe, info = cbf.filter(v_nom, positions, velocities)
        np.testing.assert_allclose(v_safe, v_nom, atol=1e-10)

    def test_stale_peer_zero_velocity(self):
        """Zeroed peer velocity (stale) should still produce a valid filtered result."""
        cfg = _default_cfg()
        cbf = DeploymentCBFFilter(cfg, num_agents=2)
        D_deploy = cfg.D_deploy
        # Agents close, ego rushing toward peer, peer velocity zeroed (stale)
        positions = np.array([[0.0, 0.0, 0.0], [D_deploy * 0.8, 0.0, 0.0]])
        v_nom = np.array([[10.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        peer_stale = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        v_safe, info = cbf.filter(v_nom.copy(), positions, peer_stale)
        # Ego velocity should be reduced
        assert v_safe[0, 0] < v_nom[0, 0]
        assert info["deploy_cbf/agents_filtered"] > 0
        # Result should still be finite
        assert np.all(np.isfinite(v_safe))
