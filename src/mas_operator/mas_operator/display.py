"""Curses-based terminal UI for operator monitoring."""

from __future__ import annotations

import curses
import time
from typing import TYPE_CHECKING

from mas_operator.fleet_state import FleetState
from mas_operator.markers import _get_chosen_position, _is_chosen

if TYPE_CHECKING:
    from mas_operator.operator_node import OperatorNode

# Mission state constants
IDLE = 0
TRACKING = 1
MISSION = 2
_STATE_NAMES = {IDLE: 'IDLE', TRACKING: 'TRACKING', MISSION: 'MISSION'}

# Refresh rate for the display loop
_DISPLAY_HZ = 4


def run_display(node: OperatorNode, fleet: FleetState) -> None:
    """Entry point for the display daemon thread."""
    try:
        curses.wrapper(lambda stdscr: _draw_screen(stdscr, fleet, node))
    except Exception:
        # If curses fails (e.g., no terminal), fall back silently.
        # Metrics still log to console via the timer callback.
        pass


def _draw_screen(
    stdscr: curses.window, fleet: FleetState, node: OperatorNode,
) -> None:
    """Main render + input loop."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()

    # Color pairs
    curses.init_pair(1, curses.COLOR_GREEN, -1)    # healthy
    curses.init_pair(2, curses.COLOR_YELLOW, -1)   # warning
    curses.init_pair(3, curses.COLOR_RED, -1)       # critical
    curses.init_pair(4, curses.COLOR_CYAN, -1)      # header
    curses.init_pair(5, curses.COLOR_WHITE, -1)     # normal

    interval = 1.0 / _DISPLAY_HZ

    while True:
        try:
            key = stdscr.getch()
            if key != -1:
                _handle_key(key, node, fleet)
                if key == ord('q') or key == 27:  # q or ESC
                    break
        except curses.error:
            pass

        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        row = 0
        with fleet.lock:
            row = _draw_header(stdscr, row, max_x)
            row = _draw_fleet_table(stdscr, row, fleet, max_x)
            row += 1
            row = _draw_triangulation(stdscr, row, fleet, max_x)
            row += 1
            row = _draw_targets_table(stdscr, row, fleet, max_x)
            row += 1
            row = _draw_alerts(stdscr, row, fleet.alerts, max_x, max_y)
            row += 1
            row = _draw_commands(stdscr, row, max_x, max_y)

        stdscr.refresh()
        time.sleep(interval)


def _safe_addstr(
    stdscr: curses.window, row: int, col: int, text: str,
    attr: int = 0, max_x: int = 80,
) -> None:
    """Write string, truncating to fit and ignoring out-of-bounds."""
    try:
        max_y, _ = stdscr.getmaxyx()
        if row >= max_y - 1:
            return
        text = text[:max(0, max_x - col - 1)]
        if text:
            stdscr.addstr(row, col, text, attr)
    except curses.error:
        pass


def _draw_header(stdscr: curses.window, row: int, max_x: int) -> int:
    _safe_addstr(
        stdscr, row, 0, ' MAS OPERATOR ',
        curses.color_pair(4) | curses.A_BOLD | curses.A_REVERSE, max_x,
    )
    return row + 2


def _draw_fleet_table(
    stdscr: curses.window, row: int, fleet: FleetState, max_x: int,
) -> int:
    """Draw per-vehicle status table."""
    header = f'{"VEH":<8} {"STATE":<10} {"ARMED":<6} {"MODE":<10} {"AoI(ms)":<10} {"V(s)":<8} {"POS (x,y,z)":<28} {"GIMBAL (r,p,y)":<20}'
    _safe_addstr(stdscr, row, 0, header, curses.color_pair(4) | curses.A_BOLD, max_x)
    row += 1
    _safe_addstr(stdscr, row, 0, '─' * min(len(header), max_x - 1), curses.color_pair(5), max_x)
    row += 1

    now = time.monotonic()

    for veh, vs in fleet.vehicles.items():
        # State
        state_str = _STATE_NAMES.get(vs.mission_state, '---') if vs.mission_state is not None else '---'

        # Armed / mode
        if vs.mavros_state is not None:
            armed_str = 'Yes' if vs.mavros_state.armed else 'No'
            mode_str = vs.mavros_state.mode or '---'
        else:
            armed_str = '---'
            mode_str = '---'

        # AoI (odom freshness)
        odom_t = vs.last_heard.get('odom')
        if odom_t is not None:
            aoi_ms = (now - odom_t) * 1000.0
            aoi_str = f'{aoi_ms:.0f}'
        else:
            aoi_str = '---'

        # Position
        if vs.odom is not None:
            p = vs.odom.pose.pose.position
            pos_str = f'({p.x:6.1f},{p.y:6.1f},{p.z:5.1f})'
        else:
            pos_str = '---'

        # V(s) — policy value function
        if vs.policy_value is not None:
            val_str = f'{vs.policy_value:.2f}'
        else:
            val_str = '---'

        # Gimbal
        if vs.gimbal_rpy is not None:
            g = vs.gimbal_rpy
            gim_str = f'({g.x:5.1f},{g.y:5.1f},{g.z:5.1f})'
        else:
            gim_str = '---'

        # Color based on state
        if vs.mission_state == MISSION:
            attr = curses.color_pair(1)
        elif vs.mission_state == TRACKING:
            attr = curses.color_pair(2)
        else:
            attr = curses.color_pair(5)

        line = f'{veh:<8} {state_str:<10} {armed_str:<6} {mode_str:<10} {aoi_str:<10} {val_str:<8} {pos_str:<28} {gim_str:<20}'
        _safe_addstr(stdscr, row, 0, line, attr, max_x)
        row += 1

    return row


def _draw_triangulation(
    stdscr: curses.window, row: int, fleet: FleetState, max_x: int,
) -> int:
    """Draw triangulation status line."""
    metrics = fleet.metrics
    if metrics is None:
        _safe_addstr(stdscr, row, 0, 'Triangulation: waiting...', curses.color_pair(5), max_x)
        return row + 1

    tri_str = 'OK' if metrics.tri_valid else 'LOST'
    tri_attr = curses.color_pair(1) if metrics.tri_valid else curses.color_pair(3)

    parts = [f'Tri: ']
    _safe_addstr(stdscr, row, 0, parts[0], curses.color_pair(4) | curses.A_BOLD, max_x)
    _safe_addstr(stdscr, row, len(parts[0]), tri_str, tri_attr | curses.A_BOLD, max_x)

    col = len(parts[0]) + len(tri_str) + 2

    if metrics.cov_trace is not None:
        cov_str = f'cov={metrics.cov_trace:.3f}'
        _safe_addstr(stdscr, row, col, cov_str, curses.color_pair(5), max_x)
        col += len(cov_str) + 2

    for (v_i, v_j), dist in metrics.inter_agent_distances.items():
        d_str = f'd({v_i[-1]},{v_j[-1]})={dist:.1f}m'
        _safe_addstr(stdscr, row, col, d_str, curses.color_pair(5), max_x)
        col += len(d_str) + 2

    for (v_i, v_j), btr in metrics.baseline_to_range.items():
        if btr is not None:
            btr_str = f'b/r({v_i[-1]},{v_j[-1]})={btr:.2f}'
            _safe_addstr(stdscr, row, col, btr_str, curses.color_pair(5), max_x)
            col += len(btr_str) + 2

    return row + 1


def _match_tri_cov(vs, pos) -> str:
    """Find closest triangulated point to pos and return its covariance trace."""
    tri = vs.triangulated_points
    if tri is None or len(tri.points) == 0:
        return '---'

    best_dist = float('inf')
    best_trace = None
    for pt in tri.points:
        dx = pt.position.x - pos.x
        dy = pt.position.y - pos.y
        dz = pt.position.z - pos.z
        dist = dx * dx + dy * dy + dz * dz
        if dist < best_dist:
            best_dist = dist
            cov = pt.covariance
            if len(cov) >= 9:
                best_trace = cov[0] + cov[4] + cov[8]

    if best_trace is not None and best_dist < 25.0:  # within 5m
        return f'{best_trace:.3f}'
    return '---'


def _draw_targets_table(
    stdscr: curses.window, row: int, fleet: FleetState, max_x: int,
) -> int:
    """Draw tracked targets table."""
    _safe_addstr(
        stdscr, row, 0, 'TRACKED TARGETS',
        curses.color_pair(4) | curses.A_BOLD, max_x,
    )
    row += 1

    header = f'{"ID":<6} {"POS (x,y,z)":<28} {"COV":<10} {"SRC":<8}'
    _safe_addstr(stdscr, row, 0, header, curses.color_pair(4), max_x)
    row += 1
    _safe_addstr(stdscr, row, 0, '─' * min(len(header), max_x - 1), curses.color_pair(5), max_x)
    row += 1

    # Get chosen target position for proximity-based highlighting
    chosen_pos = _get_chosen_position(fleet)

    # Collect tracked objects from all vehicles
    seen_ids: set[str] = set()
    for veh, vs in fleet.vehicles.items():
        for ci, det_array in vs.tracked_objects.items():
            for det in det_array.detections:
                # Track ID is in results[0].hypothesis.class_id (set by sort3d)
                if det.results:
                    track_id = det.results[0].hypothesis.class_id
                else:
                    track_id = '?'
                if not track_id:
                    track_id = '?'

                if track_id in seen_ids:
                    continue
                seen_ids.add(track_id)

                p = det.bbox.center.position
                pos_str = f'({p.x:6.1f},{p.y:6.1f},{p.z:5.1f})'

                # Match covariance from triangulated_points by closest position
                cov_str = _match_tri_cov(vs, p)

                is_sel = _is_chosen(p, chosen_pos)
                sel_str = ' [SEL]' if is_sel else ''
                line = f'{track_id:<6} {pos_str:<28} {cov_str:<10} {veh:<8}{sel_str}'
                attr = (curses.color_pair(1) | curses.A_BOLD) if is_sel else curses.color_pair(5)
                _safe_addstr(stdscr, row, 0, line, attr, max_x)
                row += 1

    if not seen_ids:
        _safe_addstr(stdscr, row, 0, '  (no tracked targets)', curses.color_pair(5), max_x)
        row += 1

    return row


def _draw_alerts(
    stdscr: curses.window, row: int, alerts: list, max_x: int, max_y: int,
) -> int:
    """Draw active alerts."""
    _safe_addstr(
        stdscr, row, 0, 'ALERTS',
        curses.color_pair(4) | curses.A_BOLD, max_x,
    )
    row += 1

    if not alerts:
        _safe_addstr(
            stdscr, row, 0, '  (none)',
            curses.color_pair(1), max_x,
        )
        return row + 1

    for alert in alerts:
        if row >= max_y - 3:
            _safe_addstr(stdscr, row, 0, '  ...', curses.color_pair(5), max_x)
            row += 1
            break

        if alert.severity == 'CRITICAL':
            attr = curses.color_pair(3) | curses.A_BOLD
            prefix = '[CRIT] '
        else:
            attr = curses.color_pair(2)
            prefix = '[WARN] '

        _safe_addstr(stdscr, row, 0, f'  {prefix}{alert.message}', attr, max_x)
        row += 1

    return row


def _draw_commands(
    stdscr: curses.window, row: int, max_x: int, max_y: int,
) -> int:
    """Draw command key legend."""
    if row >= max_y - 2:
        return row

    _safe_addstr(stdscr, row, 0, '─' * min(60, max_x - 1), curses.color_pair(5), max_x)
    row += 1

    if _target_input_mode:
        prompt = f'Select target ID: {_target_input_buf}_ (Enter=confirm, Esc=cancel)'
        _safe_addstr(stdscr, row, 0, prompt, curses.color_pair(2) | curses.A_BOLD, max_x)
    else:
        legend = '[1]IDLE  [2]TRACK  [3]MISSION  [a]AutoPick ON  [d]AutoPick OFF  [t]Select Target  [q]Quit'
        _safe_addstr(stdscr, row, 0, legend, curses.color_pair(4), max_x)
    return row + 1


# Target selection input state
_target_input_mode = False
_target_input_buf = ''


def _handle_key(
    key: int, node: OperatorNode, fleet: FleetState,
) -> None:
    """Dispatch keypress to node command publishers."""
    global _target_input_mode, _target_input_buf

    if _target_input_mode:
        if ord('0') <= key <= ord('9'):
            _target_input_buf += chr(key)
        elif key in (curses.KEY_ENTER, 10, 13):
            # Submit target selection by track ID → resolves to position
            if _target_input_buf:
                node.publish_set_target_position(_target_input_buf)
            _target_input_mode = False
            _target_input_buf = ''
        elif key == 27:  # ESC — cancel
            _target_input_mode = False
            _target_input_buf = ''
        return

    if key == ord('1'):
        node.publish_mission_cmd(IDLE)
    elif key == ord('2'):
        node.publish_mission_cmd(TRACKING)
    elif key == ord('3'):
        node.publish_mission_cmd(MISSION)
    elif key == ord('a'):
        node.publish_auto_pick(True)
    elif key == ord('d'):
        node.publish_auto_pick(False)
    elif key == ord('t'):
        _target_input_mode = True
        _target_input_buf = ''
