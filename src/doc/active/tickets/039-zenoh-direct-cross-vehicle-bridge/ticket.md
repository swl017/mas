## Ticket: Direct Jetson↔Jetson cross-vehicle topic bridge via Zenoh (remove third-PC hop)

**Status**: PAUSED — pivoting to uxrce-DDS + custom MAVROS replicator (see ticket 040)
**Created**: 2026-05-07
**Paused**: 2026-05-08

### Why paused (2026-05-08)

The Zenoh-bridge approach was taken to its current limit on this hardware/ROS-distro stack and stalled at the **last mile**: bridges run cleanly, peer-to-peer TCP/7447 link looks correct from CLI, iptables ACCEPT is in place — but cross-vehicle topics never get a real Publisher on the receiving side (`ros2 topic info` shows `Publisher count: 0` for `/px4_2/...` on veh1 and vice versa, with only local subscribers). Without an `--allow` filter the same symptom persists, so it's not a regex format issue.

What's left to triage if anyone returns to this approach:
- `ss -tn | grep -E '192.168.0.8|7447'` to confirm an actual ESTABLISHED TCP session between the two bridges (could be silently failing despite ACCEPT rules).
- `nc -zv 192.168.0.8 7447` from veh1 to confirm raw TCP reachability outside Zenoh's stack.
- `RUST_LOG=info` bridge log filtered for `transport|opened|connect|Failed|Error` — should show "New transport opened with <zid>" if the peer was accepted; otherwise it's a Zenoh-layer issue (handshake, version, congestion).
- If TCP is healthy but Zenoh handshake fails: zenoh-bridge-dds v0.5.x may be too old. The newer prebuilt zenoh-bridge-ros2dds v1.9.0 we tried earlier needed CycloneDDS interop that broke the other end. So the workable Zenoh path on Humble may require either building zenoh-plugin-ros2dds from source against the apt CycloneDDS version, or moving to ROS2 Iron+ where these versions converge.

### Pivot direction

User is moving to **uxrce-DDS + a custom MAVROS replicator node** (ticket 040). That bypasses both DDS-bridge and DDS-interop questions entirely — a small ROS2 node subscribes to local MAVROS topics and republishes them on a controlled cross-vehicle channel (DDS or otherwise), with explicit topic + QoS handling per `mas_policy/CONTEXT.md`. Estimated to be much less yak-shaving than fighting Zenoh-bridge versions.

### Sunk progress that DOES carry forward (don't redo)

These changes are useful regardless of which cross-vehicle bridge approach wins:
- [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) and [.service](/home/usrg/mas/drone_config/network/inter-jetson-block.service) — IMU-throttle fix from ticket 038, with TCP/7447 hole punched, boot-race-safe (waits up to 30s for WiFi IP), `network-online + NetworkManager-wait-online` ordering.
- Pixhawk firmware upgraded to PX4 v1.15.4 on both vehicles (gives configurable Ethernet IP).
- Pixhawk Ethernet MAVLink configured: vehicle1 = `192.168.144.10:14550`, vehicle2 = `192.168.144.20:14551`. SD-card `net.cfg` per-vehicle. `MAV_2_CONFIG=1000`, `MAV_2_MODE=2` (Onboard), `MAV_2_BROADCAST=0`, `MAV_1_CONFIG=0` (TELEM2 instance disabled).
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) — `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `unset` of stale FastDDS / Discovery-Server / CycloneDDS-URI vars, and `FCU_URL` matching bind/remote port for safety.
- ROS GPG key refresh (`/usr/share/keyrings/ros-archive-keyring.gpg`) — required for any apt install on Humble at this point in 2026.
- `ros-humble-rmw-cyclonedds-cpp` and `ros-humble-zenoh-bridge-dds` installed on each Jetson via apt.

### Sunk progress that does NOT carry forward

- The Zenoh JSON5 configs at [drone_config/zenoh/](/home/usrg/mas/drone_config/zenoh/) — schema mismatch with v0.5.x; left on disk for the record.
- The CycloneDDS XML profiles at [drone_config/dds/cyclonedds_local_only_*.xml](/home/usrg/mas/drone_config/dds/) — apt's 0.10.5 rejects the modern `<Interfaces>` schema; left on disk.
- The FastDDS XML profile at [drone_config/dds/fastdds_local_only.xml](/home/usrg/mas/drone_config/dds/fastdds_local_only.xml) — `useBuiltinTransports=false` broke local IPC; abandoned.
- The third-PC `domain_bridge` files at [src/tmux/cross_vehicle_bridge.{yaml,tmuxp.yaml}](/home/usrg/mas/src/tmux/) — the original "option A" path, kept as fallback but never installed/run.
- The prebuilt zenoh-bridge-ros2dds v1.9.0 binary at `/home/usrg/thirdparty/zenoh-plugin-ros2dds/` — bundled CycloneDDS version mismatched with apt's; superseded by the apt zenoh_bridge_dds.

### Loose ends to clean up if/when ticket 040 succeeds

- Remove the `zenoh-bridge` window from [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml).
- Decide whether to remove the unused JSON5/XML config files from `drone_config/`.
- Update [src/ARCHITECTURE.md](/home/usrg/mas/src/ARCHITECTURE.md) with whatever the final cross-vehicle topic flow ends up being.


### Implementation note (2026-05-07)

Two pivots from the original ticket plan:

1. **Initially planned** to use `ros-humble-zenoh-bridge-ros2dds` from apt — not packaged for Humble (404 from packages.ros.org for the bridge-ros2dds variant).
2. **Tried `ros-humble-zenoh-bridge-dds`** (older DDS-level bridge, was in apt cache) — also 404 from the apt mirror at fetch time.
3. **Pivoted to prebuilt aarch64 binary** from [eclipse-zenoh-plugin-ros2dds 1.9.0 release](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.9.0). Standalone binary, no Rust toolchain or build needed. Installed at `/home/usrg/thirdparty/zenoh-plugin-ros2dds/zenoh-bridge-ros2dds` with versioned dir + symlink for upgrades.

Important consequence: **`zenoh-bridge-ros2dds` embeds CycloneDDS internally**, while local ROS2 nodes use FastDDS. Both DDS implementations need to be bound off WiFi. So the design now ships **two** XML profiles:

- [drone_config/dds/fastdds_local_only.xml](/home/usrg/mas/drone_config/dds/fastdds_local_only.xml) — applied to local ROS2 nodes via `FASTRTPS_DEFAULT_PROFILES_FILE` (set in [robot.env](/home/usrg/mas/drone_config/robot.env))
- [drone_config/dds/cyclonedds_local_only.xml](/home/usrg/mas/drone_config/dds/cyclonedds_local_only.xml) — applied to the bridge via `CYCLONEDDS_URI` (set in the zenoh-bridge window's `shell_command_before` in [drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml))

Both XMLs whitelist `lo` + `192.168.144.0/24` interfaces only; WiFi is excluded for both DDS impls.

Files now committed:
- [drone_config/dds/fastdds_local_only.xml](/home/usrg/mas/drone_config/dds/fastdds_local_only.xml)
- [drone_config/dds/cyclonedds_local_only.xml](/home/usrg/mas/drone_config/dds/cyclonedds_local_only.xml)
- [drone_config/zenoh/zenoh_bridge_px4_1.json5](/home/usrg/mas/drone_config/zenoh/zenoh_bridge_px4_1.json5) — `plugins.ros2dds` schema, lists `allow.publishers`/`subscribers` explicitly with empty `service_*`/`action_*` (per v1.9.0 schema, omitted = denied)
- [drone_config/zenoh/zenoh_bridge_px4_2.json5](/home/usrg/mas/drone_config/zenoh/zenoh_bridge_px4_2.json5)
- [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh)
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env)
- [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) — zenoh-bridge window invokes the absolute binary path and exports `CYCLONEDDS_URI`

### CycloneDDS XML interface restriction abandoned (2026-05-08)

`apt`'s `ros-humble-cyclonedds` 0.10.5 rejects the modern `<Interfaces><NetworkInterface address="..."/></Interfaces>` schema as "unknown element" at parse time. The legacy single-string `<NetworkInterfaceAddress>` form is supported but only takes one address per file (no list), and is being deprecated.

After the FastDDS XML, the FastDDS Discovery Server, and now the CycloneDDS XML, this is the third DDS-config restriction approach we've tried that broke things. Pattern: each DDS impl's interface-restriction config is finicky in subtle ways, and they each have different schema gotchas.

Decision: **drop the DDS-level interface restriction entirely**. The iptables block in [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) is comprehensive — drops ALL Jetson-to-Jetson traffic on WiFi except TCP/7447. So:
- CycloneDDS sending multicast to `239.255.0.1` on the WiFi interface: outbound packet egresses, but the peer Jetson can't deliver inbound (drop at INPUT chain). Some CPU cost on send side, but zero throttle effect on the peer.
- CycloneDDS receiving inbound multicast on WiFi: kernel drops at iptables before delivery to userspace.

Net effect: the iptables block is doing the same job as the XML would, at the kernel level, with no DDS-config to maintain.

Files updated:
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env): `unset CYCLONEDDS_URI` with explanatory comment
- The XML files under `drone_config/dds/` stay on disk for the record.

### Allow regex format mismatch — filter dropped for now (2026-05-08)

After getting the bridge listening + connected, cross-vehicle topics appeared in `ros2 topic list` but `Publisher count: 0` (only local subscribers) — the bridge wasn't actually republishing peer data.

Root cause: `zenoh_bridge_dds` v0.5.x's `--allow` regex matches against the **DDS topic name** (which for ROS2 has the `rt/` prefix internally — e.g. `rt/px4_2/camera/zoom_level`), not the ROS path with leading `/`. Our regex `^/(px4_1|px4_2)/...$` matched nothing → bridge routed nothing.

Decision: **drop the allowlist for now**. iptables already blocks all peer-to-peer WiFi traffic except TCP/7447, so the bridge is the only cross-vehicle channel — bandwidth gating happens there. Re-adding a correctly-formatted allowlist (probably with `rt/` prefix) can be done later if WiFi bandwidth becomes a concern with all topics flowing.

### zenoh_bridge_dds v0.5.x uses CLI flags, not JSON5 config (2026-05-08)

The JSON5 config files we wrote (`listen.endpoints`, `connect.endpoints`, `scouting.multicast.enabled`, `plugins.dds.allow`) used the newer Zenoh schema. v0.5.x silently ignores those keys — bridge bound to a random port (35337 instead of 7447) and multicast scouting stayed enabled.

CLI flag names in v0.5.x (from `--help`):
- `-l, --listener` (singular, not `--listen`)
- `-e, --peer` (not `--connect`)
- `--no-multicast-scouting`
- `-d, --dds-domain` (alias `--domain`)
- `-a, --dds-allow` (alias `--allow`) — single regex, not lists

Working invocation now in [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) — uses a per-vehicle if-then to pick SELF/PEER/DOMAIN based on `$ROBOT_NAME`. The JSON5 files at `drone_config/zenoh/` stay on disk for the record but are no longer consumed.

### Switched bridge from prebuilt zenoh-bridge-ros2dds v1.9.0 → apt zenoh-bridge-dds v0.5.x (2026-05-08)

After switching the local RMW to CycloneDDS, the bridge *still* didn't discover any publishers. Hypothesis: the prebuilt v1.9.0 binary bundles a much newer CycloneDDS than the apt `rmw_cyclonedds_cpp` 0.10.x, and CycloneDDS isn't wire-compatible across that version gap.

Pivoted back to the apt-installable older bridge (`ros-humble-zenoh-bridge-dds`, v0.5.x) — same Cyclone version family as `rmw_cyclonedds_cpp`. This was originally rejected when the apt mirror returned 404 due to an expired ROS GPG key (since fixed; see below).

Schema change in [drone_config/zenoh/zenoh_bridge_px4_{1,2}.json5](/home/usrg/mas/drone_config/zenoh/):
- `plugins.ros2dds` → `plugins.dds` (older schema, different keys)
- `allow.publishers/subscribers` lists → single `allow` regex string
- Topic name format unchanged (matches against ROS topic path with leading `/`)

Tmuxp invocation in [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml):
- `/home/usrg/thirdparty/zenoh-plugin-ros2dds/zenoh-bridge-ros2dds -c ...` → `ros2 run zenoh_bridge_dds zenoh_bridge_dds -c ...`

The prebuilt v1.9.0 binary at `/home/usrg/thirdparty/zenoh-plugin-ros2dds/` stays on disk for the record but is no longer invoked.

### ROS GPG key refresh (2026-05-08)

`packages.ros.org` apt index couldn't be verified — the Open Robotics signing key `F42ED6FBAB17C654` had expired. apt fell back to the stale local index, which pointed at packages versions that no longer existed on the mirror — that's the root cause of the earlier `zenoh-bridge-dds` 404 and the `rmw-cyclonedds-cpp` 404. Fix:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update     # now succeeds with valid signature
```

Has to be run once per Jetson before the install commands.

### RMW switched to CycloneDDS (2026-05-08)

After unsetting `ROS_DISCOVERY_SERVER` (below) the bridge *still* didn't discover MAVROS publishers — even with both processes confirmed to have clean env via `/proc/<pid>/environ`. Strong indicator of a FastDDS↔CycloneDDS interop quirk at the SPDP level (not a configuration issue).

Architectural fix: **switch the local RMW from `rmw_fastrtps_cpp` to `rmw_cyclonedds_cpp`**. Both local ROS2 nodes and the embedded-in-the-bridge CycloneDDS now use the same DDS implementation and the same XML profile. No interop layer.

Trade-offs considered:
- This is a real change to the ROS2 stack — QoS/timing/behavior may shift slightly. Per [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md) the only QoS-sensitive topic is `policy/observation` (BEST_EFFORT, depth=1) — both RMWs respect this.
- CycloneDDS is the official ROS2 default for several distros and is well-tested with MAVROS.
- iptables block + CycloneDDS XML interface whitelist is now defense-in-depth that *both* actually work (FastDDS XML version was abandoned because `useBuiltinTransports=false` broke local IPC).

Files changed:
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) — `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI=file://.../cyclonedds_local_only_${ROBOT_NAME}.xml` now exported globally
- [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) — removed the per-window `CYCLONEDDS_URI` override on the zenoh-bridge window (redundant; robot.env handles it)

Install on each Jetson (one-time):
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### ROS_DISCOVERY_SERVER abandoned (2026-05-08)

Bridge was running cleanly but never logged "Discovered Publisher" for any MAVROS topic, even when ros2 node list / topic list saw all of MAVROS from another FastDDS process. Root cause: [robot.env](/home/usrg/mas/drone_config/robot.env) had `ROS_DISCOVERY_SERVER="192.168.144.101:11811"` set (left over from ticket 038's per-vehicle separation attempt). FastDDS clients in Discovery Server mode do not broadcast standard SPDP — they only talk to the configured server. zenoh-bridge-ros2dds embeds CycloneDDS, which only speaks standard DDS-RTPS SPDP. Result: FastDDS-to-FastDDS works inside each vehicle, but FastDDS-to-bridge doesn't, so no cross-vehicle topic flow.

Fix: unset `ROS_DISCOVERY_SERVER`. Cross-vehicle isolation is now done at the iptables layer (drops all peer-to-peer WiFi except TCP/7447), so the Discovery Server is no longer load-bearing — and it was actively breaking the new bridge.

[robot.env](/home/usrg/mas/drone_config/robot.env) updated: `unset ROS_DISCOVERY_SERVER` with explanatory comment.

### Pixhawk MAV_2_BROADCAST=1 abandoned (2026-05-07)

Earlier in the diagnostic flow the suggestion was to set `MAV_2_BROADCAST=1` on the Pixhawk for MAVROS auto-discovery. That worked on UART but is harmful with the static-IP Ethernet config: PX4 in broadcast mode sends replies to `<subnet_broadcast>:remote_port`, not to the partner's source IP:port. With FCU_URL bind_port (14555) ≠ remote_port (14550), MAVROS never received replies. Symptom: Pixhawk receives 65k+ MAVROS messages, but MAVROS gets nothing back, then VER request times out → "FCU don't support AUTOPILOT_VERSION".

Fix: `param set MAV_2_BROADCAST 0; param save; reboot` on each Pixhawk. PX4 then learns the partner's source IP:port from inbound packets and unicasts replies back to that address — works regardless of bind/remote port match.

User also normalized `FCU_URL` to `udp://:14550@192.168.144.10:14550` (bind == remote port) which makes the link configuration robust even if broadcast is later toggled by mistake.

### FastDDS XML restriction abandoned (2026-05-07)

Initial design called for a FastDDS XML profile that whitelisted only `lo` + gimbal-side interfaces. In practice this broke things:
- Field test: bridge started cleanly but discovered only `/_ros2cli_daemon_*`, never the MAVROS publishers. `ros2 topic list` showed topics; `ros2 topic echo` produced no data.
- Root cause: `useBuiltinTransports=false` plus a custom UDPv4 transport doesn't automatically configure the SPDP discovery locators that FastDDS needs. Adding those would require explicit `<builtin>` `metatrafficMulticastLocatorList` config — extra fragility for what's already covered by iptables.

Decision: **disable the FastDDS XML restriction**. The iptables drop is comprehensive (drops all peer-to-peer WiFi traffic except TCP/7447), so the XML was strict defense-in-depth that broke local IPC. The export line in [robot.env](/home/usrg/mas/drone_config/robot.env) is commented out with a note. The XML file [drone_config/dds/fastdds_local_only.xml](/home/usrg/mas/drone_config/dds/fastdds_local_only.xml) stays in tree for the record.

**The CycloneDDS XML for the bridge is still active** — without it, the bridge's CycloneDDS would advertise on WiFi, which iptables would drop but at non-zero softirq cost. Per-vehicle XML works because CycloneDDS is strict about `<NetworkInterface>` matching local interfaces.

### Vehicle2 binary copy (one-time):
```bash
# From vehicle1 Jetson:
rsync -av /home/usrg/thirdparty/zenoh-plugin-ros2dds-1.9.0/ \
  jetsonnx-jp62-02:/home/usrg/thirdparty/zenoh-plugin-ros2dds-1.9.0/
ssh jetsonnx-jp62-02 \
  'ln -sfn /home/usrg/thirdparty/zenoh-plugin-ros2dds-1.9.0 /home/usrg/thirdparty/zenoh-plugin-ros2dds'
```

(SSH between Jetsons is iptables-blocked since ticket 038 — copy via the third PC or via the user's laptop instead.)

Files now committed:
- [drone_config/dds/fastdds_local_only.xml](/home/usrg/mas/drone_config/dds/fastdds_local_only.xml) — FastDDS profile, single XML for both vehicles
- [drone_config/zenoh/zenoh_bridge_px4_1.json5](/home/usrg/mas/drone_config/zenoh/zenoh_bridge_px4_1.json5)
- [drone_config/zenoh/zenoh_bridge_px4_2.json5](/home/usrg/mas/drone_config/zenoh/zenoh_bridge_px4_2.json5)
- [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) — updated to ACCEPT TCP/7447 above the existing DROP rules
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) — sets `FASTRTPS_DEFAULT_PROFILES_FILE`
- [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) — new `zenoh-bridge` window invokes the per-vehicle config via `${ROBOT_NAME}` substitution

**What**: Replace the [third-PC `domain_bridge`](/home/usrg/mas/src/tmux/cross_vehicle_bridge.yaml) (option A from ticket 038's resolution) with a direct Jetson-to-Jetson `zenoh-bridge-dds` running on each vehicle. Local DDS gets bound off WiFi via FastDDS XML; the only WiFi traffic between Jetsons is a fixed-port Zenoh stream carrying the explicit allowlist of peer topics from [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md).

**Why**: MAS is communication-heavy and the policy control loop runs at 25 Hz; the third-PC hop adds ~1–3 ms one-way latency and creates a hard ground-station dependency for multi-drone operations. Direct cross-vehicle saves the hop, removes the third-PC SPOF, and keeps the IMU-throttle fix bulletproof (DDS never touches WiFi, so the failure mode that triggered ticket 038 cannot recur even if iptables were relaxed).

**Out of scope**: Replacing FastDDS with `rmw_zenoh_cpp` system-wide (more disruptive, separate decision). This ticket keeps FastDDS as the local RMW; Zenoh is only the inter-vehicle transport.

### Why not the simpler alternatives

- **(A) Stay on third-PC `domain_bridge`** — works today, but +latency and +dependency. See [ticket 038's resolution](/home/usrg/mas/src/doc/active/tickets/038-mavros-dual-vehicle-imu-degradation/ticket.md) for the design that's currently deployed.
- **(C) Direct DDS with FastDDS strict peers + multicast off + static EDP** — theoretically lower latency than B, but FastDDS XML is gnarly and static EDP requires per-topic config; high risk of reintroducing the IMU throttle if any config detail leaks. Rejected.

### Architecture

```
veh1 Jetson                                            veh2 Jetson
  ├─ ROS2 nodes (DOMAIN=1)                               ├─ ROS2 nodes (DOMAIN=2)
  ├─ FastDDS bound to localhost+gimbal-side only         ├─ FastDDS bound to localhost+gimbal-side only
  │     (interfaceWhiteList in fastdds_local_only.xml)   │
  └─ zenoh-bridge-dds  ──TCP/7447 over WiFi──────────┘
       (allowlists exactly the 5 peer topics per         (zenoh-bridge-dds peer)
        mas_policy/CONTEXT.md)
```

DDS never crosses WiFi. Zenoh-bridge translates between local DDS and a Zenoh peer-to-peer session over a single fixed TCP port that's allowlisted in iptables.

### Files to add / modify

**New**:
- `drone_config/dds/fastdds_local_only.xml` — FastDDS profile with `interfaceWhiteList` of `127.0.0.1` + the gimbal-side IP, multicast disabled. **Per-vehicle** (different gimbal IPs `.101` / `.102`) or use interface-name binding instead of IP.
- `drone_config/zenoh/zenoh_bridge_veh1.json5` — Zenoh bridge config: peer mode, listen on `tcp/192.168.0.14:7447`, connect to `tcp/192.168.0.8:7447`, multicast scouting disabled, ros2dds plugin with topic allowlist scoped to `/px4_1/*` (publishers) and `/px4_2/*` (subscribers).
- `drone_config/zenoh/zenoh_bridge_veh2.json5` — symmetric: listen on `.8:7447`, connect to `.14:7447`, allowlist scoped to `/px4_2/*` (pub) and `/px4_1/*` (sub).

**Modified**:
- [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) — add ACCEPT rules for TCP/7447 between peers BEFORE the existing DROP rules, idempotent. Pseudo:
  ```
  iptables -I INPUT  1 -s "$PEER" -p tcp --match multiport --ports 7447 -j ACCEPT
  iptables -I OUTPUT 1 -d "$PEER" -p tcp --match multiport --ports 7447 -j ACCEPT
  # existing DROP rules stay (appended after)
  ```
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) — set `FASTRTPS_DEFAULT_PROFILES_FILE` to point at the new XML.
- [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) — new `zenoh-bridge` window invoking `zenoh-bridge-dds -c <vehicle's zenoh config>`.

**Kept as fallback (not deleted)**:
- [src/tmux/cross_vehicle_bridge.yaml](/home/usrg/mas/src/tmux/cross_vehicle_bridge.yaml) and [src/tmux/cross_vehicle_bridge.tmuxp.yaml](/home/usrg/mas/src/tmux/cross_vehicle_bridge.tmuxp.yaml) — the option A path stays runnable on the third PC. Useful as a fallback during initial Zenoh debug, and for monitoring scenarios where the third PC is involved anyway.

### Workflow

1. **Install zenoh-bridge-dds on each Jetson** (one-time):
   ```bash
   sudo apt install ros-humble-zenoh-bridge-dds
   ```
   If not packaged for Humble in apt, fall back to a prebuilt binary release from [eclipse-zenoh/zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases) and place at `/usr/local/bin/zenoh-bridge-dds`.

2. **Author and validate `fastdds_local_only.xml`** on a single vehicle first. The risk: a bad XML can make local ROS2 stop working entirely. Verification: launch a single MAVROS, confirm `ros2 topic list` and `mavros/imu/data` rate are healthy.

3. **Author Zenoh configs** for both vehicles. Validate that `zenoh-bridge-dds` starts cleanly and lists the allowlisted topics in its log.

4. **Update iptables script** to allow TCP/7447 between peers. Verify with `sudo iptables -nL` that ACCEPT rules sit above the DROP rules.

5. **Update tmuxp launch** to bring up zenoh-bridge alongside MAVROS. Initial test: single-vehicle, just MAVROS + Zenoh bridge running, no peer traffic — confirm nothing breaks.

6. **Dual-vehicle bench test**:
   - Both vehicles up.
   - Confirm `/px4_2/...` topics visible in `ros2 topic list` on vehicle1's Jetson.
   - Confirm `ros2 topic hz /px4_2/common_frame/odom` matches the source rate on vehicle2.
   - Confirm `mavros/imu/data` stays at 200 Hz on both vehicles (the original ticket 038 acceptance criterion).
   - Compare end-to-end latency on a peer topic against the option A path — should be ~1–3 ms lower on a typical run.

7. **Field test** — full mission with both drones flying, watch for any regression in policy behavior or IMU rate.

### Affected files

- POSSIBLY EDIT: [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env)
- POSSIBLY EDIT: [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh)
- POSSIBLY EDIT: [src/tmux/drone.tmuxp.yaml](/home/usrg/mas/src/tmux/drone.tmuxp.yaml)
- POSSIBLY NEW: `drone_config/dds/fastdds_local_only.xml`
- POSSIBLY NEW: `drone_config/zenoh/zenoh_bridge_veh{1,2}.json5`
- NO EDIT: [src/tmux/cross_vehicle_bridge.yaml](/home/usrg/mas/src/tmux/cross_vehicle_bridge.yaml) — kept as fallback
- POSSIBLY EDIT: [src/ARCHITECTURE.md](/home/usrg/mas/src/ARCHITECTURE.md) — once verified, document the Zenoh path as the cross-vehicle topic flow.

### Acceptance criteria

- Dual-vehicle launch, both vehicles' MAVROS up, both `mavros/imu/data` at ≥ 180 Hz steady-state for ≥ 60 s. (Same as ticket 038's criterion — must hold under the new transport.)
- All five peer topics from [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md) visible in each vehicle's `ros2 topic list` and at expected rates.
- Third PC powered off / WiFi-disconnected: dual-vehicle topic flow continues uninterrupted.
- `sudo iptables -nL` shows the surgical TCP/7447 ACCEPT rules above the peer DROP rules; no other traffic between the Jetsons crosses.
- Measured one-way latency on `/px4_2/policy/observation` (from vehicle2 publish to vehicle1 receive) is lower than the option A baseline by ≥ 1 ms, ideally ≥ 2 ms, on a typical WiFi LAN.

### Risk

- **Medium for FastDDS XML rollout** — a bad XML can silently break all local ROS2 on the affected Jetson. Mitigate by validating on one vehicle first, keep a known-good XML alongside, and have a quick-revert path (unset `FASTRTPS_DEFAULT_PROFILES_FILE` and restart).
- **Low for Zenoh installation** — well-maintained package; if apt doesn't have it for Humble, prebuilt binary is straightforward.
- **Low for iptables changes** — already idempotent, easy to revert by removing the ACCEPT rules.
- **Low for the kept-as-fallback approach** — if Zenoh path misbehaves in the field, run the third-PC bridge instead. No file deletions until the new path is field-proven.

### References

- [ticket 038 resolution](/home/usrg/mas/src/doc/active/tickets/038-mavros-dual-vehicle-imu-degradation/ticket.md) — the WiFi-DDS root cause and current option A bridge.
- [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md) — authoritative list of peer topics and their QoS.
- Zenoh ROS2 plugin: <https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds>
- FastDDS XML profiles reference: <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>

### Coupling

- Independent of all currently-open mas/0xx tickets except ticket 038 (which it supersedes architecturally — option A there becomes the fallback path here).
- Downstream effect: once verified, [ARCHITECTURE.md](/home/usrg/mas/src/ARCHITECTURE.md) gains a "cross-vehicle bridge" component on the system diagram.

**Flow**: Medium (~1 day work — most of it is FastDDS XML iteration; Zenoh side is straightforward once the bridge package installs cleanly).
