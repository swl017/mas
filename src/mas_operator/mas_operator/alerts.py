"""Alert condition evaluation for operator monitoring."""

from __future__ import annotations

from dataclasses import dataclass

from mas_operator.fleet_state import Alert, FleetState
from mas_operator.metrics import Metrics

# Mission state constants
IDLE = 0
TRACKING = 1
MISSION = 2
_STATE_NAMES = {IDLE: 'IDLE', TRACKING: 'TRACKING', MISSION: 'MISSION'}


@dataclass
class AlertThresholds:
    """Configurable thresholds for alert evaluation."""

    aoi_warn_ms: float = 500.0
    aoi_critical_ms: float = 2000.0
    cov_warn_threshold: float = 5.0
    safety_distance_m: float = 9.5
    tri_timeout_s: float = 1.0


def evaluate_alerts(
    metrics: Metrics,
    fleet: FleetState,
    thresholds: AlertThresholds,
    now: float,
    linger_s: float = 1.0,
) -> list[Alert]:
    """Evaluate all alert conditions, returning active alerts.

    Merges new alerts with existing ones, respecting linger time.
    Must be called with fleet.lock held.
    """
    new_alerts: list[Alert] = []
    expires = now + linger_s

    # 1. Fleet state mismatch
    if not metrics.fleet_consensus:
        states = {
            veh: _STATE_NAMES.get(vs.mission_state, '?')
            for veh, vs in fleet.vehicles.items()
            if vs.mission_state is not None
        }
        if len(states) > 0:
            state_str = ', '.join(f'{v}={s}' for v, s in states.items())
            new_alerts.append(Alert(
                name='fleet_mismatch',
                severity='WARNING',
                message=f'Fleet state mismatch: {state_str}',
                expires=expires,
            ))

    # 2. Communication stale / lost (per-agent odom AoI)
    for veh, topic_aoi in metrics.aoi.items():
        odom_aoi = topic_aoi.get('odom')
        if odom_aoi is not None:
            if odom_aoi > thresholds.aoi_critical_ms:
                new_alerts.append(Alert(
                    name=f'comm_lost_{veh}',
                    severity='CRITICAL',
                    message=f'{veh}: communication lost '
                            f'(AoI {odom_aoi:.0f}ms)',
                    expires=expires,
                ))
            elif odom_aoi > thresholds.aoi_warn_ms:
                new_alerts.append(Alert(
                    name=f'comm_stale_{veh}',
                    severity='WARNING',
                    message=f'{veh}: communication stale '
                            f'(AoI {odom_aoi:.0f}ms)',
                    expires=expires,
                ))

    # 3. Triangulation lost
    if not metrics.tri_valid:
        new_alerts.append(Alert(
            name='tri_lost',
            severity='CRITICAL',
            message='Triangulation lost: no valid points',
            expires=expires,
        ))

    # 4. Covariance spike
    if (
        metrics.cov_trace is not None
        and metrics.cov_trace > thresholds.cov_warn_threshold
    ):
        new_alerts.append(Alert(
            name='cov_spike',
            severity='WARNING',
            message=f'Covariance spike: trace={metrics.cov_trace:.2f} '
                    f'> {thresholds.cov_warn_threshold}',
            expires=expires,
        ))

    # 5. Collision proximity
    for (v_i, v_j), dist in metrics.inter_agent_distances.items():
        if dist < thresholds.safety_distance_m:
            new_alerts.append(Alert(
                name=f'collision_{v_i}_{v_j}',
                severity='CRITICAL',
                message=f'Collision proximity: {v_i}-{v_j} = {dist:.1f}m '
                        f'< {thresholds.safety_distance_m}m',
                expires=expires,
            ))

    # 6. Agent disarmed during TRACKING or MISSION
    for veh, vs in fleet.vehicles.items():
        if vs.mavros_state is not None and vs.mission_state is not None:
            if vs.mission_state in (TRACKING, MISSION) and not vs.mavros_state.armed:
                new_alerts.append(Alert(
                    name=f'disarmed_{veh}',
                    severity='CRITICAL',
                    message=f'{veh}: disarmed during '
                            f'{_STATE_NAMES.get(vs.mission_state, "?")}',
                    expires=expires,
                ))

    # Merge with lingering alerts: keep old alerts that haven't expired
    # and aren't superseded by a new alert with the same name
    new_names = {a.name for a in new_alerts}
    lingering = [
        a for a in fleet.alerts
        if a.expires > now and a.name not in new_names
    ]

    return new_alerts + lingering
