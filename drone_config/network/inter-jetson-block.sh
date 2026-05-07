#!/usr/bin/env bash
# Surgical iptables policy between the two Jetsons over WiFi.
#
# Background:
#   - src/doc/active/tickets/038-mavros-dual-vehicle-imu-degradation/ticket.md
#     (root cause — Jetson WiFi softirq + DDS chatter starves MAVROS read loop)
#   - src/doc/active/tickets/039-zenoh-direct-cross-vehicle-bridge/ticket.md
#     (cross-vehicle topics now flow over a single TCP/7447 Zenoh stream)
#
# Policy on the WiFi link to the peer Jetson:
#   - ACCEPT TCP/7447 in both directions  (zenoh-bridge-dds)
#   - DROP   everything else
#
# Other LAN traffic (third-PC monitoring, SSH from operator's laptop, etc.)
# is unaffected — only the peer Jetson's IP is filtered.
#
# Idempotent: safe to re-run. Rules are inserted in a fixed order so that
# ACCEPT sits above DROP in the chain.
set -euo pipefail

VEH1_WIFI=192.168.0.14   # jetsonnx-jp62-01
VEH2_WIFI=192.168.0.8    # jetsonnx-jp62-02
ZENOH_PORT=7447

WIFI_IFACE=wlP1p1s0
SELF_IP=$(ip -4 addr show "$WIFI_IFACE" 2>/dev/null \
            | awk '/inet /{print $2}' | cut -d/ -f1)

case "$SELF_IP" in
  "$VEH1_WIFI") PEER_IP="$VEH2_WIFI" ;;
  "$VEH2_WIFI") PEER_IP="$VEH1_WIFI" ;;
  "")
    echo "inter-jetson-block: $WIFI_IFACE has no IPv4 yet — skipping" >&2
    exit 0
    ;;
  *)
    echo "inter-jetson-block: unknown WiFi IP $SELF_IP — update peer table" >&2
    exit 0
    ;;
esac

# 1. Append the DROP rules first (they go to the end of the chain).
iptables -C INPUT  -s "$PEER_IP" -j DROP 2>/dev/null || \
  iptables -A INPUT  -s "$PEER_IP" -j DROP
iptables -C OUTPUT -d "$PEER_IP" -j DROP 2>/dev/null || \
  iptables -A OUTPUT -d "$PEER_IP" -j DROP

# 2. Then insert the ACCEPT rules at position 1 so they're checked before
#    the DROP rules. multiport matches when src OR dst port is 7447,
#    covering both client→server and server→client legs of the TCP session.
iptables -C INPUT  -s "$PEER_IP" -p tcp -m multiport --ports "$ZENOH_PORT" -j ACCEPT 2>/dev/null || \
  iptables -I INPUT  1 -s "$PEER_IP" -p tcp -m multiport --ports "$ZENOH_PORT" -j ACCEPT
iptables -C OUTPUT -d "$PEER_IP" -p tcp -m multiport --ports "$ZENOH_PORT" -j ACCEPT 2>/dev/null || \
  iptables -I OUTPUT 1 -d "$PEER_IP" -p tcp -m multiport --ports "$ZENOH_PORT" -j ACCEPT

echo "inter-jetson-block: peer=$PEER_IP, ACCEPT TCP/$ZENOH_PORT, DROP everything else"
