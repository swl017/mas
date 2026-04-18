"""RViz marker publishing for operator spatial awareness."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from mas_operator.fleet_state import FleetState
from mas_operator.metrics import Metrics


# Common frame used by all MAS nodes
FRAME_ID = 'common_frame'


def _get_chosen_position(fleet: FleetState):
    """Get the chosen target position from any vehicle's chosen_target_pose."""
    for vs in fleet.vehicles.values():
        if vs.chosen_target is not None:
            return vs.chosen_target.pose.pose.position
    return None


def _is_chosen(pos, chosen_pos, threshold_sq: float = 4.0) -> bool:
    """Check if a track position is near the chosen target (within ~2m)."""
    if chosen_pos is None:
        return False
    dx = pos.x - chosen_pos.x
    dy = pos.y - chosen_pos.y
    dz = pos.z - chosen_pos.z
    return (dx * dx + dy * dy + dz * dz) < threshold_sq


def _match_cov_trace(vs, pos) -> float | None:
    """Find the closest triangulated point's covariance trace for a tracked position."""
    tri = vs.triangulated_points
    if tri is None or len(tri.points) == 0:
        return None

    best_dist_sq = float('inf')
    best_trace = None
    for pt in tri.points:
        dx = pt.position.x - pos.x
        dy = pt.position.y - pos.y
        dz = pt.position.z - pos.z
        d_sq = dx * dx + dy * dy + dz * dz
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            cov = pt.covariance
            if len(cov) >= 9:
                best_trace = cov[0] + cov[4] + cov[8]

    if best_trace is not None and best_dist_sq < 25.0:  # within 5m
        return best_trace
    return None


def _find_chosen_track_id(fleet: FleetState) -> str | None:
    """Get the cached chosen track ID from any vehicle."""
    for vs in fleet.vehicles.values():
        if vs.chosen_track_id is not None:
            return vs.chosen_track_id
    return None


def build_marker_array(
    fleet: FleetState, metrics: Metrics | None,
    aoi_warn_ms: float, aoi_critical_ms: float,
) -> MarkerArray:
    """Build complete MarkerArray for RViz visualization.

    Must be called with fleet.lock held.
    """
    ma = MarkerArray()
    marker_id = 0

    # Agent position spheres + text labels
    for veh, vs in fleet.vehicles.items():
        if vs.odom is None:
            continue
        p = vs.odom.pose.pose.position

        # Sphere marker
        sphere = _make_marker(marker_id, Marker.SPHERE, FRAME_ID)
        sphere.pose.position = p
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.8
        sphere.scale.y = 0.8
        sphere.scale.z = 0.8
        sphere.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)
        ma.markers.append(sphere)
        marker_id += 1

        # Text label
        text = _make_marker(marker_id, Marker.TEXT_VIEW_FACING, FRAME_ID)
        text.pose.position = Point(x=p.x, y=p.y, z=p.z + 1.2)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.6
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = veh
        ma.markers.append(text)
        marker_id += 1

    # Get chosen target position for proximity-based highlighting
    chosen_pos = _get_chosen_position(fleet)

    # Tracked target markers + text ID labels
    seen_ids: set[str] = set()
    for veh, vs in fleet.vehicles.items():
        for ci, det_array in vs.tracked_objects.items():
            for det in det_array.detections:
                if det.results:
                    track_id = det.results[0].hypothesis.class_id
                else:
                    continue
                if not track_id or track_id in seen_ids:
                    continue
                seen_ids.add(track_id)

                p = det.bbox.center.position
                is_chosen = _is_chosen(p, chosen_pos)
                cov_trace = _match_cov_trace(vs, p)

                # Target sphere — chosen target is larger and green
                target = _make_marker(marker_id, Marker.SPHERE, FRAME_ID)
                target.pose.position = p
                target.pose.orientation.w = 1.0
                if is_chosen:
                    target.scale.x = 0.8
                    target.scale.y = 0.8
                    target.scale.z = 0.8
                    target.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.9)
                else:
                    target.scale.x = 0.5
                    target.scale.y = 0.5
                    target.scale.z = 0.5
                    target.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.8)
                ma.markers.append(target)
                marker_id += 1

                # Text label with track ID
                label = _make_marker(
                    marker_id, Marker.TEXT_VIEW_FACING, FRAME_ID,
                )
                label.pose.position = Point(x=p.x, y=p.y, z=p.z + 0.8)
                label.pose.orientation.w = 1.0
                label.scale.z = 0.5
                cov_str = f' cov={cov_trace:.3f}' if cov_trace is not None else ''
                if is_chosen:
                    label.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=1.0)
                    label.text = f'T{track_id} [SEL]{cov_str}'
                else:
                    label.color = ColorRGBA(r=1.0, g=1.0, b=0.3, a=1.0)
                    label.text = f'T{track_id}{cov_str}'
                ma.markers.append(label)
                marker_id += 1

    # Inter-agent AoI lines
    if metrics is not None:
        vehs = list(fleet.vehicles.keys())
        for (v_i, v_j), aoi_ms in metrics.cross_agent_aoi.items():
            vs_i = fleet.vehicles.get(v_i)
            vs_j = fleet.vehicles.get(v_j)
            if vs_i is None or vs_j is None:
                continue
            if vs_i.odom is None or vs_j.odom is None:
                continue

            p_i = vs_i.odom.pose.pose.position
            p_j = vs_j.odom.pose.pose.position

            # Line marker
            line = _make_marker(marker_id, Marker.LINE_STRIP, FRAME_ID)
            line.points = [
                Point(x=p_i.x, y=p_i.y, z=p_i.z),
                Point(x=p_j.x, y=p_j.y, z=p_j.z),
            ]
            line.scale.x = 0.08  # line width
            line.color = _aoi_color(aoi_ms, aoi_warn_ms, aoi_critical_ms)
            line.pose.orientation.w = 1.0
            ma.markers.append(line)
            marker_id += 1

            # AoI text at midpoint
            mid = Point(
                x=(p_i.x + p_j.x) / 2.0,
                y=(p_i.y + p_j.y) / 2.0,
                z=(p_i.z + p_j.z) / 2.0 + 0.5,
            )
            aoi_text = _make_marker(
                marker_id, Marker.TEXT_VIEW_FACING, FRAME_ID,
            )
            aoi_text.pose.position = mid
            aoi_text.pose.orientation.w = 1.0
            aoi_text.scale.z = 0.4
            aoi_text.color = _aoi_color(
                aoi_ms, aoi_warn_ms, aoi_critical_ms,
            )
            if aoi_ms < float('inf'):
                aoi_text.text = f'{aoi_ms:.0f}ms'
            else:
                aoi_text.text = 'N/A'
            ma.markers.append(aoi_text)
            marker_id += 1

    return ma


def _make_marker(
    marker_id: int, marker_type: int, frame_id: str,
) -> Marker:
    """Create a Marker with common defaults."""
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = 'mas_operator'
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.lifetime = Duration(sec=1, nanosec=0)
    return m


def _aoi_color(
    aoi_ms: float, warn_ms: float, critical_ms: float,
) -> ColorRGBA:
    """Color-code AoI: green → yellow → red."""
    if aoi_ms >= critical_ms:
        return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    elif aoi_ms >= warn_ms:
        # Interpolate yellow to red
        t = (aoi_ms - warn_ms) / max(critical_ms - warn_ms, 1.0)
        return ColorRGBA(r=1.0, g=1.0 - t, b=0.0, a=1.0)
    else:
        # Interpolate green to yellow
        t = aoi_ms / max(warn_ms, 1.0)
        return ColorRGBA(r=t, g=1.0, b=0.0, a=1.0)
