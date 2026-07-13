#!/bin/bash
# install_profile.sh — apply an EKF or PN profile to the LIVE stack, safely.
#
# Codifies the config-swap procedure from ticket-007 REPRODUCE.md §2 and the
# ticket-010 slice-0 log. Both target files are INSTALLED BY PLAIN COPY (not
# symlink), and the bearing-loc launch source is untracked in git — so a profile
# must be copied to BOTH the source tree and the install space, and the node
# window restarted, or the running system silently keeps the old values.
#
# Usage:
#   ./install_profile.sh ekf <OLD|TUNED|INTER|/path/to/launch.py>
#   ./install_profile.sh pn  <N2|N3|/path/to/pn.yaml>
#   ./install_profile.sh verify        # print what the live install space holds
#
# Requires: the interceptor tmux session up (windows: bearing_ekf, pn_guidance).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAS=/home/usrg/mas
EKF_SRC=$MAS/src/mas_bearing_loc/launch/engagement_ekf.launch.py
EKF_INST=$MAS/install/mas_bearing_loc/share/mas_bearing_loc/launch/engagement_ekf.launch.py
PN_SRC=$MAS/src/mas_pn_guidance/config/pn_guidance.yaml
PN_INST=$MAS/install/mas_pn_guidance/share/mas_pn_guidance/config/pn_guidance.yaml

pn_window() {  # window index of the pn_guidance window (name may carry a bell dash)
  tmux list-windows -t interceptor -F '#{window_index} #{window_name}' \
    | awk '/pn_guidance/{print $1; exit}'
}

verify() {
  echo "== EKF (installed: $EKF_INST) =="
  grep -nE "sigma_pix|sigma_target_acc|init_range" "$EKF_INST" | sed 's/^/  /'
  echo "== PN (installed: $PN_INST) =="
  grep -nE "nav_constant|v_max|a_max" "$PN_INST" | sed 's/^/  /'
  echo "== PN node (live ready-line, if visible) =="
  tmux capture-pane -t "interceptor:$(pn_window)" -p -S -100 2>/dev/null \
    | grep -E "ready:.*N=" | tail -1 | sed 's/^/  /' || true
}

case "${1:-}" in
  ekf)
    P="${2:?ekf profile name or path required}"
    [ -f "$P" ] || P="$HERE/engagement_ekf_${P}.launch.py"
    [ -f "$P" ] || { echo "no such EKF profile: $P" >&2; exit 1; }
    python3 -c "import ast; ast.parse(open('$P').read())"   # refuse broken files
    cp "$P" "$EKF_SRC"
    cp "$P" "$EKF_INST"
    tmux send-keys -t interceptor:bearing_ekf C-c; sleep 2
    tmux send-keys -t interceptor:bearing_ekf \
      "ros2 launch mas_bearing_loc engagement_ekf.launch.py" Enter
    sleep 4; verify
    ;;
  pn)
    P="${2:?pn profile name or path required}"
    [ -f "$P" ] || P="$HERE/pn_${P}.yaml"
    [ -f "$P" ] || { echo "no such PN profile: $P" >&2; exit 1; }
    python3 -c "import yaml; yaml.safe_load(open('$P'))"    # refuse broken yaml
    cp "$P" "$PN_SRC"
    cp "$P" "$PN_INST"
    W=$(pn_window); [ -n "$W" ] || { echo "no pn_guidance window found" >&2; exit 1; }
    tmux send-keys -t "interceptor:$W" C-c; sleep 2
    # env vars expand in the pane's own shell (set by the tmuxp session)
    tmux send-keys -t "interceptor:$W" \
      'ros2 launch mas_pn_guidance pn_guidance.launch.py ns:=/${ROBOT_NAME} estimate_source:=oracle v_max:=9.0 a_max:=6.0 use_sim_time:=${USE_SIM_TIME}' Enter
    sleep 6; verify
    ;;
  verify)
    verify
    ;;
  *)
    grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -14; exit 1
    ;;
esac
