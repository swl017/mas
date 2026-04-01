## Ticket #006: Build MAVROS from source for humble

### What
Currently MAVROS runs in a separate galactic-sourced process. Build it from source in the humble workspace so the entire stack runs under one ROS2 distro.

### Why
Running mixed distros (humble nodes + galactic MAVROS) adds fragility and complicates deployment. QoS and message compatibility issues are harder to debug across distros.

### Scope boundary
Only build MAVROS and its deps. Do not change node code or topic wiring. Do not drop galactic fallback until verified.

### Affected modules
`ros2_humble/`, `tmux/`

### Acceptance criteria
`ros2 node list` shows MAVROS running under humble; `mavros/local_position/odom` flows to `mas_common_frame` without QoS workarounds

### Flow
Light

### Status
Deferred
