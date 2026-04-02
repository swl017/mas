"""Derived metrics for operator situational awareness (Comprehension layer)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations

from mas_msgs.msg import TriangulatedPointArray

from mas_operator.fleet_state import FleetState


@dataclass
class Metrics:
    """Computed metrics snapshot."""

    # Per-vehicle AoI: veh → {topic_key → age_ms}
    aoi: dict[str, dict[str, float]] = field(default_factory=dict)
    # Cross-agent AoI: (veh_i, veh_j) → age_ms (max of the pair's odom AoI)
    cross_agent_aoi: dict[tuple[str, str], float] = field(default_factory=dict)
    # Triangulation covariance trace (from first vehicle with data)
    cov_trace: float | None = None
    # Whether any vehicle received triangulated points within timeout
    tri_valid: bool = False
    # Pairwise inter-agent distances in meters
    inter_agent_distances: dict[tuple[str, str], float] = field(
        default_factory=dict,
    )
    # All agents report the same mission_state
    fleet_consensus: bool = False
    # Baseline-to-range ratio per agent pair
    baseline_to_range: dict[tuple[str, str], float | None] = field(
        default_factory=dict,
    )


def compute_metrics(
    fleet: FleetState, now: float, tri_timeout_s: float,
) -> Metrics:
    """Compute all derived metrics from current fleet state.

    Must be called with fleet.lock held.
    """
    m = Metrics()

    # Per-vehicle AoI
    for veh, vs in fleet.vehicles.items():
        m.aoi[veh] = _compute_aoi(vs.last_heard, now)

    # Cross-agent AoI (based on odom freshness)
    m.cross_agent_aoi = _compute_cross_agent_aoi(fleet, now)

    # Covariance trace — use first vehicle that has triangulated points
    for vs in fleet.vehicles.values():
        trace = _compute_cov_trace(vs.triangulated_points)
        if trace is not None:
            m.cov_trace = trace
            break

    # Triangulation validity
    m.tri_valid = _check_tri_valid(fleet, now, tri_timeout_s)

    # Inter-agent distances
    m.inter_agent_distances = _compute_inter_agent_distances(fleet)

    # Fleet consensus
    m.fleet_consensus = _check_fleet_consensus(fleet)

    # Baseline-to-range ratio
    target_pos = _get_target_position(fleet)
    m.baseline_to_range = _compute_baseline_to_range(fleet, target_pos)

    return m


def _compute_aoi(
    last_heard: dict[str, float], now: float,
) -> dict[str, float]:
    """Age-of-Information per topic in milliseconds."""
    return {
        topic: (now - t) * 1000.0
        for topic, t in last_heard.items()
    }


def _compute_cross_agent_aoi(
    fleet: FleetState, now: float,
) -> dict[tuple[str, str], float]:
    """Cross-agent AoI for each pair, based on odom freshness."""
    result: dict[tuple[str, str], float] = {}
    vehs = list(fleet.vehicles.keys())
    for v_i, v_j in combinations(vehs, 2):
        aoi_i = _get_odom_age_ms(fleet.vehicles[v_i], now)
        aoi_j = _get_odom_age_ms(fleet.vehicles[v_j], now)
        result[(v_i, v_j)] = max(aoi_i, aoi_j)
    return result


def _get_odom_age_ms(vs, now: float) -> float:
    """Get odom AoI in ms, or infinity if never heard."""
    t = vs.last_heard.get('odom')
    if t is None:
        return float('inf')
    return (now - t) * 1000.0


def _compute_cov_trace(
    points: TriangulatedPointArray | None,
) -> float | None:
    """Covariance trace from triangulated points (average across points)."""
    if points is None or len(points.points) == 0:
        return None

    traces = []
    for pt in points.points:
        cov = pt.covariance
        if len(cov) >= 9:
            # Diagonal: cov[0]=xx, cov[4]=yy, cov[8]=zz
            traces.append(cov[0] + cov[4] + cov[8])
    if not traces:
        return None
    return sum(traces) / len(traces)


def _check_tri_valid(
    fleet: FleetState, now: float, timeout_s: float,
) -> bool:
    """True if any vehicle received triangulated points within timeout."""
    for vs in fleet.vehicles.values():
        t = vs.last_heard.get('triangulated_points')
        if t is not None and (now - t) < timeout_s:
            if vs.triangulated_points and len(vs.triangulated_points.points) > 0:
                return True
    return False


def _compute_inter_agent_distances(
    fleet: FleetState,
) -> dict[tuple[str, str], float]:
    """Pairwise Euclidean distances between agents."""
    result: dict[tuple[str, str], float] = {}
    vehs = list(fleet.vehicles.keys())
    for v_i, v_j in combinations(vehs, 2):
        pos_i = _get_position(fleet.vehicles[v_i])
        pos_j = _get_position(fleet.vehicles[v_j])
        if pos_i is not None and pos_j is not None:
            dx = pos_i[0] - pos_j[0]
            dy = pos_i[1] - pos_j[1]
            dz = pos_i[2] - pos_j[2]
            result[(v_i, v_j)] = math.sqrt(dx * dx + dy * dy + dz * dz)
    return result


def _check_fleet_consensus(fleet: FleetState) -> bool:
    """True if all vehicles report the same mission_state."""
    states = [
        vs.mission_state
        for vs in fleet.vehicles.values()
        if vs.mission_state is not None
    ]
    if not states:
        return False
    return len(set(states)) == 1


def _get_position(vs) -> tuple[float, float, float] | None:
    """Extract (x, y, z) from vehicle odom."""
    if vs.odom is None:
        return None
    p = vs.odom.pose.pose.position
    return (p.x, p.y, p.z)


def _get_target_position(
    fleet: FleetState,
) -> tuple[float, float, float] | None:
    """Get target position from chosen_target_pose (first vehicle with data)."""
    for vs in fleet.vehicles.values():
        if vs.chosen_target is not None:
            p = vs.chosen_target.pose.pose.position
            return (p.x, p.y, p.z)
    return None


def _compute_baseline_to_range(
    fleet: FleetState,
    target_pos: tuple[float, float, float] | None,
) -> dict[tuple[str, str], float | None]:
    """Baseline-to-range ratio per agent pair.

    baseline-to-range = ||p_i - p_j|| / ||target - midpoint(p_i, p_j)||
    """
    result: dict[tuple[str, str], float | None] = {}
    vehs = list(fleet.vehicles.keys())
    for v_i, v_j in combinations(vehs, 2):
        if target_pos is None:
            result[(v_i, v_j)] = None
            continue

        pos_i = _get_position(fleet.vehicles[v_i])
        pos_j = _get_position(fleet.vehicles[v_j])
        if pos_i is None or pos_j is None:
            result[(v_i, v_j)] = None
            continue

        dx = pos_i[0] - pos_j[0]
        dy = pos_i[1] - pos_j[1]
        dz = pos_i[2] - pos_j[2]
        baseline = math.sqrt(dx * dx + dy * dy + dz * dz)

        mid = (
            (pos_i[0] + pos_j[0]) / 2.0,
            (pos_i[1] + pos_j[1]) / 2.0,
            (pos_i[2] + pos_j[2]) / 2.0,
        )
        rx = target_pos[0] - mid[0]
        ry = target_pos[1] - mid[1]
        rz = target_pos[2] - mid[2]
        range_dist = math.sqrt(rx * rx + ry * ry + rz * rz)

        if range_dist < 1e-6:
            result[(v_i, v_j)] = None
        else:
            result[(v_i, v_j)] = baseline / range_dist

    return result
