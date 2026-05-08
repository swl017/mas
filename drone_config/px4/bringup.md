# PX4 uxrce-DDS bringup — NSH commands

One-time setup typed into the QGC MAVLink Console for each autopilot.
Both vehicles use the same internal Pixhawk↔Jetson Ethernet (PX4 at
`192.168.144.10`, Jetson at `192.168.144.101`, agent UDP/8888). Adjust
if the actual networking on a given airframe differs.

The Jetson side is handled by `src/tmux/drone.tmuxp.yaml`'s `uxrce-agent`
window, which loads `dds_xrce_profile.xml` so PX4's `px4_participant`
profile reference resolves.

## Common params (both vehicles)

```
param set UXRCE_DDS_CFG 0
param set UXRCE_DDS_PTCFG 0
param set UXRCE_DDS_DOM_ID 1
param set UXRCE_DDS_SYNCC 0
param set UXRCE_DDS_SYNCT 1
```

`UXRCE_DDS_CFG=0` disables the param-driven autostart so it doesn't race
with `extras.txt`. `UXRCE_DDS_PTCFG=0` is required: any non-zero value
in this firmware build flips the participant to "Localhost only" which
silently prevents the local ROS2 graph (Fast-DDS) from discovering the
agent's topics — agent connects, traffic flows, `ros2 topic list` empty.

## Per-vehicle params

Vehicle 1:
```
param set MAV_SYS_ID 11
param set UXRCE_DDS_KEY 1
```

Vehicle 2:
```
param set MAV_SYS_ID 2
param set UXRCE_DDS_KEY 2
```

`UXRCE_DDS_KEY` must be unique per vehicle so the agent can distinguish
sessions if both ever connect to the same agent (not the case today
but cheap insurance).

## Save and write the autostart hook

```
param save
mkdir -p /fs/microsd/etc
```

Vehicle 1:
```
echo "uxrce_dds_client start -t udp -h 192.168.144.101 -p 8888 -n px4_1" > /fs/microsd/etc/extras.txt
```

Vehicle 2:
```
echo "uxrce_dds_client start -t udp -h 192.168.144.102 -p 8888 -n px4_2" > /fs/microsd/etc/extras.txt
```

Verify, then reboot:
```
cat /fs/microsd/etc/extras.txt
reboot
```

## Verify after reboot

NSH:
```
uxrce_dds_client status
```
Expect: `Running, connected`, `timesync converged: true`, non-zero
`Payload tx`, and `Localhost only: no`.

Jetson (with `drone_config/robot.env` sourced — sets
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `ROS_DOMAIN_ID`):
```
ros2 topic list | grep px4_
ros2 topic hz /px4_1/fmu/out/vehicle_status   # or /px4_2/...
```
