## Ticket: Cross-vehicle topic flow via uxrce-DDS + custom MAVROS replicator

**Status**: Open — exploration phase
**Created**: 2026-05-08

**What**: Replace the stalled Zenoh-bridge approach (see [ticket 039](/home/usrg/mas/src/doc/active/tickets/039-zenoh-direct-cross-vehicle-bridge/ticket.md)) with a small custom ROS2 node that subscribes to peer topics from MAVROS / mas_common_frame / etc. on the local vehicle and replicates them onto a controlled cross-vehicle channel. Transport for the cross-vehicle channel is uxrce-DDS to keep the volume low and the topology explicit.

**Why**: Ticket 039 took the Zenoh path to its limit on this stack (PX4 v1.15.4, Humble, jetson aarch64) and stalled at the bridge handshake — peers connect over TCP but topics never get a real publisher on the receiving side. The amount of yak-shaving across DDS-impl interop, version mismatches, and apt-package staleness was disproportionate to the size of the actual cross-vehicle data path (5 topics from `mas_policy/CONTEXT.md`). A custom replicator removes ALL of that.

**Authoritative topic list** — from [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md), the peer topics that must reach the other vehicle:

| Topic | Type | QoS notes |
|---|---|---|
| `/{peer}/common_frame/odom` | `nav_msgs/Odometry` | default reliable |
| `/{peer}/combined_ang_vel_w` | `geometry_msgs/Vector3Stamped` | default reliable |
| `/{peer}/yolo_result_active` | `std_msgs/Bool` | default reliable |
| `/{peer}/camera/zoom_level` | `std_msgs/Float64` | default reliable |
| `/{peer}/policy/observation` | `std_msgs/Float32MultiArray` | **BEST_EFFORT, KEEP_LAST depth=1** |

### What carries forward from ticket 039

These are already in tree and don't need to be redone:
- iptables surgical block at [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) — drops all peer-to-peer WiFi except TCP/7447. **Punch a different / additional port hole for the new transport** — update the script's `ZENOH_PORT` variable to whatever the replicator uses. (NOTE: the block has been removed for test purpose. Can be brought back once we need it for network traffic control)
- PX4 v1.15.4 + Ethernet MAVLink config (vehicle1 = `192.168.144.10:14550`, vehicle2 = `192.168.144.20:14551`).
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` set in [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env). The replicator should work with whatever RMW the rest of the system uses; CycloneDDS is fine.
- `ros-humble-rmw-cyclonedds-cpp` installed on both Jetsons.
- ROS GPG key refresh applied — apt installs on Humble work cleanly now.

### Out of scope

- Bringing back any of the Zenoh-bridge work from ticket 039. If the replicator doesn't work either, the next pivot is a full rewrite to use `rmw_zenoh_cpp` system-wide rather than a bridge — but only after the replicator is fully ruled out.

### Open questions for the replicator design

- **Transport**: uxrce-DDS via the eProsima micro-XRCE-DDS-Agent? Or a small TCP-based custom protocol? uxrce keeps it ROS-graph-coherent but adds a daemon to manage; raw TCP is simpler but needs more handwritten code.
- **Topology**: each vehicle runs one replicator subscribing to its own `/px4_X/...` topics and publishing-on-the-other-side via the controlled channel. Or central aggregator on a third PC. Distributed is more autonomous; central is fewer moving parts.
- **QoS preservation**: the replicator must propagate the original publisher's QoS exactly — particularly BEST_EFFORT depth=1 for `policy/observation`. Mismatched QoS will silently fail.
- **Domain ID**: each vehicle's local ROS2 graph stays on its own `ROS_DOMAIN_ID` (1 for veh1, 2 for veh2). Replicator must therefore be able to participate in both domains, OR the cross-vehicle channel must be domain-agnostic.

### Acceptance criteria (inherited from ticket 039)

- Both vehicles' MAVROS at ≥ 180 Hz IMU steady-state with both running.
- All 5 peer topics from the table above visible in each vehicle's `ros2 topic list` AND `ros2 topic echo` returns actual data.
- Vehicle2 powered off / WiFi-disconnected: vehicle1 keeps publishing local topics without errors (replicator gracefully handles peer absence).
- Adding/removing the iptables ACCEPT for the replicator's port is the only firewall change beyond ticket 038's existing rules.

### References

- [Ticket 038](/home/usrg/mas/src/doc/active/tickets/038-mavros-dual-vehicle-imu-degradation/ticket.md) — root-cause diagnosis of the IMU throttle and the iptables fix that this ticket builds on.
- [Ticket 039](/home/usrg/mas/src/doc/active/tickets/039-zenoh-direct-cross-vehicle-bridge/ticket.md) — paused; "what carries forward" and "loose ends to clean up" sections there are directly relevant.
- [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md) — authoritative topic + QoS contract for what must cross.
- PX4 uxrce-DDS docs: <https://docs.px4.io/main/en/middleware/uxrce_dds>

**Flow**: TBD — depends on whether uxrce-DDS or custom-relay path is taken. Estimate small (1–2 days) if a clean simple replicator works on first try.
