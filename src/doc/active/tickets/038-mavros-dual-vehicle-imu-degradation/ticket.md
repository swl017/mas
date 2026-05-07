## Ticket: MAVROS IMU rate collapses on vehicle1 when vehicle2 launches MAVROS

**Status**: Diagnosed — fix files written, install pending
**Created**: 2026-05-07
**Resolved (root cause identified)**: 2026-05-07

### Resolution summary

**Root cause**: Cross-vehicle WiFi traffic on `192.168.0.0/24` starves vehicle1's Jetson — MAVROS's UDP read loop loses bytes between the kernel and userspace when veh2's MAVROS launches. Confirmed by isolating with `iptables -I INPUT -s <peer_wifi> -j DROP` on the WiFi interface: IMU snaps to 200 Hz with the rule in place, drops back to <1 Hz when removed. Independent of MAVROS transport — symptom persists with MAVROS on Ethernet (PX4 v1.15.4, MAV_2_CONFIG=1000) just as severely as on TELEM2 UART.

The originally-leading hypothesis (PX4 TELEM2 RX framing breaks under FMU CPU contention, matching [PX4#26144](https://github.com/PX4/PX4-Autopilot/issues/26144)) was disproven by the Ethernet test — vehicle1's Pixhawk Ethernet instance shows full 22 KB/s tx with `(lost: 0)` even while MAVROS at the Jetson is in HEARTBEAT-timeout cycle. The bytes reach the Jetson; they're dropped *inside* it.

Different `ROS_DOMAIN_ID` per vehicle (already in place: `1` / `2`) is **not** sufficient isolation. ROS2 / FastDDS binds to all interfaces by default; the kernel + softirq still process every inter-Jetson packet regardless of domain, and FastDDS broadcast announces cross-traffic. That CPU/IRQ cost is what starves MAVROS.

**Fix**: persistent iptables drop of inter-Jetson WiFi traffic, scoped narrowly so SSH, third-PC monitoring, and other LAN traffic stay open. Files version-controlled at [drone_config/network/inter-jetson-block.sh](/home/usrg/mas/drone_config/network/inter-jetson-block.sh) and [.service](/home/usrg/mas/drone_config/network/inter-jetson-block.service).

**Install on each Jetson** (one-time, requires sudo):

```bash
sudo install -m 755 /home/usrg/mas/drone_config/network/inter-jetson-block.sh /usr/local/sbin/
sudo install -m 644 /home/usrg/mas/drone_config/network/inter-jetson-block.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now inter-jetson-block.service
sudo iptables -nL INPUT | grep DROP   # verify rule active
```

**Side cleanups completed during diagnosis** (don't directly fix the bug, but improve hygiene):

- Pixhawks upgraded from PX4 v1.14 (NuttX-11.0.0) to PX4 v1.15.4. Per-vehicle params backed up to [drone_config/px4/vehicle1.params](/home/usrg/mas/drone_config/px4/vehicle1.params) (pre-upgrade, v1.14) and [vehicle1-2026-05-07.params](/home/usrg/mas/drone_config/px4/vehicle1-2026-05-07.params) (v1.15.4 baseline).
- Ethernet MAVLink instance configured on each Pixhawk (`MAV_2_CONFIG=1000`, `MAV_2_MODE=2` Onboard, `MAV_2_BROADCAST=1`). Static IPs via SD-card `/fs/microsd/net.cfg`: vehicle1 = `192.168.144.10:14550`, vehicle2 = `192.168.144.20:14551`.
- TELEM2 UART instance disabled on both Pixhawks (`MAV_1_CONFIG=0`).
- [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) `FCU_URL` updated from `udp://:14555@192.168.144.101:14550` (which was pointing at the Jetson's own IP — bug) to the Pixhawk's Ethernet IP.

**Cross-vehicle topic bridge (handles the side-effect of the iptables block)**:

The iptables block intentionally severs all direct Jetson↔Jetson DDS traffic. `mas_policy/policy_node` per [mas_policy/CONTEXT.md](/home/usrg/mas/src/mas_policy/CONTEXT.md) needs five peer topics (`common_frame/odom`, `combined_ang_vel_w`, `yolo_result_active`, `camera/zoom_level`, `policy/observation`). To re-introduce only those, route them through a `domain_bridge` running on the third (ground-station) PC — it sees both vehicles' DDS over WiFi (third PC isn't iptables-blocked from either Jetson) and explicitly republishes the allow-listed topics across domains.

Files version-controlled at:
- [src/tmux/cross_vehicle_bridge.yaml](/home/usrg/mas/src/tmux/cross_vehicle_bridge.yaml) — topic allowlist with from/to domains and QoS (BEST_EFFORT depth=1 for `policy/observation`, default reliable for the others)
- [src/tmux/cross_vehicle_bridge.tmuxp.yaml](/home/usrg/mas/src/tmux/cross_vehicle_bridge.tmuxp.yaml) — tmuxp launcher

**Install + run on the third PC** (one-time install, then run per session):

```bash
sudo apt install ros-humble-domain-bridge
tmuxp load /home/usrg/mas/src/tmux/cross_vehicle_bridge.tmuxp.yaml
# Or directly:
# ros2 run domain_bridge domain_bridge /home/usrg/mas/src/tmux/cross_vehicle_bridge.yaml
```

Why this preserves the IMU-throttle fix: each Jetson still only exchanges DDS with the third PC, never directly with the peer Jetson. The bridge on the third PC has separate DDS participants in domain 1 and domain 2; cross-domain traffic never crosses the iptables block. Should multiagent operations later need additional cross-vehicle topics, edit the YAML allowlist — don't loosen iptables.

**Other followups**:
- The user reported intermittent issues seeing own-Jetson topics from a separate terminal. Probably a `ROS_DISCOVERY_SERVER` config edge case; investigate separately if it persists.
- Consider updating [ARCHITECTURE.md](/home/usrg/mas/src/ARCHITECTURE.md) once the bridge is installed and verified — the third PC + cross-vehicle bridge is now a load-bearing component of multi-drone deployment.

---

### Original ticket below (preserved for context)



**What**: With both drones (px4_1 / SYS_ID 11, px4_2 / SYS_ID 2) on the same 192.168.144.0/24 LAN and each running its own MAVROS against its own Pixhawk over TELEM2 (`/dev/pixhawk` → ttyTHS1, 921600 baud, no flow control), launching vehicle2's MAVROS instantly drops vehicle1's `mavros/imu/data` rate from ~200 Hz down to ~25–60 Hz. Killing vehicle2's MAVROS makes vehicle1's IMU snap right back to ~200 Hz. The effect is asymmetric — vehicle2 stays at ~194 Hz regardless of vehicle1's MAVROS state. Each MAVROS instance, alone on its drone, works fine.

**Why this matters**: MAVROS IMU is a hard input to several downstream nodes ([mas_common_frame](/home/usrg/mas/src/mas_common_frame/CONTEXT.md), [mas_offboard](/home/usrg/mas/src/mas_offboard/CONTEXT.md), [mas_policy](/home/usrg/mas/src/mas_policy/CONTEXT.md)). Below ~50 Hz the policy and frame transforms degrade noticeably; below ~25 Hz mission-state-machine timing margins start failing. Multi-drone deployments are blocked until this is resolved.

**Out of scope**: Anything past identifying and fixing the cross-vehicle bridge / load source. Refactoring MAVROS launch into a non-tmuxp form is intentionally deferred — the current launch in [src/tmux/drone.tmuxp.yaml:19](/home/usrg/mas/src/tmux/drone.tmuxp.yaml#L19) stays the integration point.

### Symptom summary

- Vehicle1 MAVROS alone: stable ~200 Hz IMU, no errors.
- Vehicle2 MAVROS alone: stable ~194 Hz IMU, no errors.
- Both up: vehicle1 IMU collapses to ~25–60 Hz; MAVROS log shows repeated `CON: Lost connection, HEARTBEAT timed out` followed by `Got HEARTBEAT, connected` cycles. Each reconnect triggers full mission/rally/geofence re-pulls.
- "Plugin loading" itself takes longer on vehicle1 when vehicle2's MAVROS is already running — i.e. the perturbation begins before MAVROS has even reached steady-state on vehicle1.
- Toggle is instant: killing vehicle2's MAVROS restores vehicle1 within ~1 s.

Observed environment when reproducing:
- Pixhawk firmware: PX4 NuttX-11.0.0 (`mavlink status` reports MAVLink 2 on TELEM2 when partner is detected, MAVLink 1 in default Normal mode).
- TELEM2 hw flow control: OFF on both ends (`MAV_1_FLOW_CTRL=0`, `stty -F /dev/pixhawk` reports `-crtscts`).
- USB CDC instance auto-started on `/dev/ttyACM0 @2000000`, `mode: Config`, `Forwarding: On`, partner sysid 255 cid 190 (QGC over HereLink). Doing ~22.5 KB/s tx baseline.
- Discovery: each drone now uses its own `ROS_DISCOVERY_SERVER` (was previously shared at 192.168.144.102; separated as part of the diagnostic — no behavior change).
- `MAV_FWDEXTSP=1` (forward external setpoints between local instances, intra-vehicle).
- TELEM1 SiK radios physically removed from both vehicles. `MAV_0_FORWARD=0`, `MAV_1_FORWARD=0` confirmed in [drone_config/px4/vehicle1.params](/home/usrg/mas/drone_config/px4/vehicle1.params) and [vehicle2.params](/home/usrg/mas/drone_config/px4/vehicle2.params).

### What we've tried (and the result of each)

1. **Disable PX4-side radio rate scaling on TELEM2** — set `MAV_1_RADIO_CTL=0` and `MAV_0_FORWARD=0` so RADIO_STATUS from the SiK can't reach TELEM2's `_rate_mult` calculation. Also physically removed the SiK on TELEM1.
   - **Result**: Improved baseline somewhat (no longer pinned at 25 Hz from rate-mult throttling), but the "drops when vehicle2 comes up" behavior persisted.

2. **Disable MAVROS `param` plugin** — added `-p plugin_denylist:='[param]'` to the [mavros_node line](/home/usrg/mas/src/tmux/drone.tmuxp.yaml#L19). Motivation: the MAVLink log was filled with `PR: got an unsolicited param value idx=…` (>500 in a 3000-line window) — MAVROS was caught in a `PARAM_REQUEST_LIST` retry loop on every reconnect, dumping all 1095 PX4 params and saturating TELEM2.
   - **Result**: Param storm fully eliminated (count went 507 → 0). Corrupt-frame remote addresses (`0.X` sysids) on `link[1000]` also disappeared. **But the IMU rate throttling persisted** — the param storm was a symptom, not the root cause. `set_message_interval` (in the `cmd` plugin, not `param`) still works fine.

3. **Trim more MAVROS plugins** — extended denylist to `[param, ftp, waypoint, rallypoint, geofence, wind_estimation, wheel_odometry, vibration, vision_pose, vision_speed, mocap_pose_estimate]`.
   - **Result**: No measurable improvement.

4. **Separate the FastDDS Discovery Server per drone** — each drone now points `ROS_DISCOVERY_SERVER` at its own Jetson. Motivation: rule out cross-vehicle DDS discovery traffic as the perturbation source.
   - **Result**: No improvement. DDS isolation does not change the symptom. Excludes ROS-layer discovery storms as the cause.

5. **Inspected MAVROS network listeners** — `ss -tunap` confirms vehicle1's `mavros_node` is not listening on any standard MAVLink UDP port (no 14550 / 14555 / 14580). Only DDS ports (7400/7410/7411) and ephemeral DDS sockets. Cross-vehicle MAVLink can't be reaching vehicle1's MAVROS directly over UDP.

6. **Hypothesized USB-CDC instance as the bridge** — vehicle1's PX4 has `instance #2` on `/dev/ttyACM0` in `mode: Config` with `Forwarding: On`, and one early measurement showed USB rx jumping 46.9 → 116.8 B/s when vehicle2's MAVROS came up. Theory: QGC / HereLink controller on the GCS is bridging vehicle2's MAVLink onto vehicle1's USB, USB then forwards into TELEM2 → MAVROS load increases.
   - **Status**: **Not confirmed.** A second round of `mavlink status` captures (see below) does not reproduce the rx jump on USB. The earlier delta may have been measurement noise. The USB-bridge theory is now a **plausible-but-unverified hypothesis**, not a settled diagnosis.

### Most informative current data — `mavlink status` matrix on vehicle1

All four cells captured back-to-back; only difference between cells is which MAVROS is running:

| Scenario | TELEM2 (#1) tx / rx / version / partner | USB (#2) rx | Notes |
|---|---|---|---|
| veh1 MAVROS off, veh2 MAVROS off | 1690 B/s / 0 / v1 / no partner | 90.8 B/s | Idle baseline. |
| veh1 MAVROS off, veh2 MAVROS on  | 1655 B/s / 0 / v1 / no partner | 46.9 B/s | TELEM2 idle (expected), USB rx is *lower*. |
| veh1 MAVROS on, veh2 MAVROS on   | 1750 B/s / 0 / v1 / no partner | 46.9 B/s | **Critical — see below.** |
| (earlier capture) veh1 MAVROS on, veh2 MAVROS on (working window) | 10 370 B/s / 62.9 / v2 / sysid 11 cid 191 | 46.9 B/s | When MAVROS handshake actually completes. |

The critical cell is row 3: with vehicle1's MAVROS process running, **PX4 still does not see it as a partner on TELEM2** — no `Received Messages` block, MAVLink stuck at v1, tx pinned at default-Normal-mode rate (~1.7 KB/s). Compare to row 4: when handshake succeeds, tx jumps to 10 KB/s, MAVLink upgrades to v2, partner sysid 11 cid 191 is registered. **The failure mode is that MAVROS's outbound HEARTBEAT bytes never get cleanly parsed by PX4's TELEM2** when vehicle2's MAVROS is also running. Without HEARTBEAT detection, PX4 never honors `set_message_interval`, and IMU stays at default rate.

This shifts the suspect from "MAVLink layer being flooded after handshake" to "TELEM2 RX framing is breaking before handshake."

### Open hypotheses (ordered by current likelihood)

1. **PX4 FMU CPU contention starves TELEM2 UART RX FIFO.** The USB CDC instance is doing ~22.5 KB/s tx in `mode: Config` with `Forwarding: On`, plus servicing ping/heartbeat from QGC. When vehicle2's MAVROS adds even a small amount of work (e.g. a few extra messages that QGC then echoes/forwards), the FMU's mavlink threads spend enough extra time in USB-side serialization that the TELEM2 UART ISR misses bytes. With `-crtscts` off and `MAV_1_FLOW_CTRL=0`, there's no backpressure — bytes silently drop and HEARTBEAT framing fails. Asymmetric outcome (vehicle2 fine) is just which FMU happens to win the race on its own board.

2. **GCS-side MAVLink router is bridging the two vehicles' streams.** HereLink controller (or a `mavlink-router` running on it at 192.168.144.160) connects to both Pixhawks via USB-over-air-link, and its default config forwards every message it sees to every connected endpoint. Cross-vehicle traffic enters vehicle1's `/dev/ttyACM0`, USB has `Forwarding: On`, and the message gets queued onto TELEM2's tx — competing with the IMU stream and chewing through the small budget. (Same hypothesis as the original USB-bridge theory, but at the QGC/router layer rather than PX4's own forwarding.)

3. **Multicast DDS leakage despite separate discovery servers.** FastDDS in DiscoveryServer mode is supposed to suppress SPDP multicast, but several reported bugs allow leakage. Both Jetsons are on the same LAN; if vehicle2's nodes still emit SPDP/EDP multicast, vehicle1's Jetson processes them at the kernel/DDS layer, briefly stealing CPU from `mavros_node`'s read loop. Less likely (separating servers should have at least dampened this), but not ruled out by experiment.

4. **Shared HereLink RF channel cross-talk via TELEM1.** TELEM1's SiK was unplugged on both vehicles, but `MAV_0_CONFIG=101` is still set, so PX4 still allocates a MAVLink instance and runs it (visible as `instance #0` doing tx 500–700 B/s into nothing). If anything is still electrically connected to TELEM1 (HereLink companion port, idle pins floating, etc.), noise could be parsed as inbound MAVLink. Lowest likelihood given current data, but a free thing to rule out by setting `MAV_0_CONFIG=0`.

### Recommended next steps

In order — each step is cheap and either confirms or rules out a hypothesis.

1. **Unplug USB cable from vehicle1's Pixhawk, power-cycle the FCU, launch both MAVROS, measure `/px4_1/mavros/imu/data` rate.** If it stays at 200 Hz with vehicle2 up, hypotheses 1 and 2 are both confirmed (load + bridge are both via USB) and the fix is straightforward (next step). If it still degrades, move to step 4.

2. **If unplug fixes it, demote the USB MAVLink instance from Config to Onboard** to keep QGC connectivity but kill the high-rate dump and forwarding. From the PX4 NSH:
   ```
   mavlink stop  -d /dev/ttyACM0
   mavlink start -d /dev/ttyACM0 -b 2000000 -m onboard
   ```
   Verify `mavlink status` shows `mode: Onboard`, `Forwarding: Off`. To survive reboot, write the two lines to `/fs/microsd/etc/extras.txt`.
   - QGC dropdown for `MAV_X_CONFIG` does not expose USB, but `param set MAV_2_CONFIG 201; param set MAV_2_FORWARD 0; param set MAV_2_MODE 2; param save; reboot` from the NSH may be accepted directly. Try this first; fall back to `extras.txt` if rejected.

3. **If unplug fixes it but the goal is to keep QGC over USB**, alternative fix: configure HereLink / `mavlink-router` on the GCS host (192.168.144.160) to *not* bridge sysid 11 ↔ sysid 2; bridge each drone only to QGC. Out-of-band — needs whoever owns the GCS box.

4. **If unplug does NOT fix it**, set `MAV_0_CONFIG=0` on both vehicles (kill the dead TELEM1 instance entirely) and re-test. This rules out hypothesis 4. Then run a `tcpdump -i any -n 'host 192.168.144.0/24'` capture on vehicle1's Jetson while toggling vehicle2's MAVROS to look for any unexpected ingress traffic — that targets hypothesis 3.

5. **Independently, fix the link-layer fragility** (this is amplifier, not cause, but reduces the blast radius of any future load spike):
   - Verify whether RTS/CTS are wired between Pixhawk TELEM2 and the Jetson `ttyTHS1`. If yes: set `MAV_1_FLOW_CTRL=2` on PX4 and `stty -F /dev/pixhawk crtscts` on the Jetson (make persistent via udev rule or a systemd unit). If RTS/CTS are not wired, *do not* enable `crtscts` — the link will deadlock waiting for CTS.
   - If RTS/CTS aren't wired, lower `SER_TEL2_BAUD` to `460800`. TELEM2 tx is ~10 KB/s in steady state; `460800 / 10 ≈ 46 KB/s` ample headroom; slower bytes give the kernel and FMU UART more margin against bursts.

### Affected files

- POSSIBLY EDIT: [drone_config/px4/vehicle1.params](/home/usrg/mas/drone_config/px4/vehicle1.params), [drone_config/px4/vehicle2.params](/home/usrg/mas/drone_config/px4/vehicle2.params) — `MAV_2_CONFIG`, `MAV_2_FORWARD`, `MAV_2_MODE`, possibly `MAV_0_CONFIG`, `MAV_1_FLOW_CTRL`, `SER_TEL2_BAUD`.
- POSSIBLY NEW: `/fs/microsd/etc/extras.txt` on each Pixhawk's SD card if `param set MAV_2_CONFIG 201` is rejected by the firmware build.
- POSSIBLY EDIT: [src/tmux/drone.tmuxp.yaml:19](/home/usrg/mas/src/tmux/drone.tmuxp.yaml#L19) — keep the `plugin_denylist` change that's already in place; nothing else MAVROS-side is expected to need touching.
- NO EDIT EXPECTED: [drone_config/robot.env](/home/usrg/mas/drone_config/robot.env) — the per-drone discovery server separation already done as part of the diagnostic stays.

### Acceptance criteria

- With both vehicles' MAVROS launched (any order), `ros2 topic hz /px4_1/mavros/imu/data` and `ros2 topic hz /px4_2/mavros/imu/data` both report ≥ 180 Hz steady-state for at least 60 s, with no `CON: Lost connection, HEARTBEAT timed out` lines in either MAVROS log over that window.
- `mavlink status` on each Pixhawk shows TELEM2 with `Received Messages: sysid: <fcu_sys_id>, compid: 191` and MAVLink version 2 (handshake stable).
- Killing and relaunching either vehicle's MAVROS does not perturb the other's IMU rate by more than a transient < 2 s blip.

### Risk

Low for steps 1–2 (purely diagnostic + a runtime PX4 command that's reverted by reboot). Medium for step 5 (changing baud or enabling flow control could deadlock the link if RTS/CTS aren't actually wired — verify cable before toggling).

### References

- MAVROS log evidence (param storm, heartbeat-timeout cycle, cmd-400 ack timeouts, USB-instance forwarding flag): captured live from the [drone tmux session](/home/usrg/mas/src/tmux/drone.tmuxp.yaml) `mavros` window.
- Diagnostic conversation thread (assistant session, 2026-05-07): documents the elimination of the radio-rate-mult, MAVROS param-storm, plugin-overhead, and shared-discovery-server hypotheses.
- PX4 `mavlink status` output sets (4 cells × 2 captures) — pasted into the same conversation; preserved for follow-up.

### Coupling

- Independent of all currently-open mas/0xx tickets. No downstream ticket is blocked specifically on this; multi-drone field tests are.

**Flow**: Medium (diagnostic-first; once the bridge is identified, the fix is one or two PX4 param changes + possibly an SD-card script).
