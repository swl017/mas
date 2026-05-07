## Ticket: Direct Jetson↔Jetson cross-vehicle topic bridge via Zenoh (remove third-PC hop)

**Status**: Files written, install + test pending
**Created**: 2026-05-07

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

**Vehicle2 binary copy** (one-time):
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
