# mas_offboard — Node Interface Contract

## offboard_control node

Per-vehicle offboard controller for PX4 via MAVROS. Runs a state machine (INIT → RAMP_UP → WAIT_OFFBOARD → TAKEOFF → HOVER → POLICY) and publishes velocity/position setpoints at 100 Hz.

Arming and OFFBOARD mode change are **out of band** — the operator (or the `auto_arm` helper below) drives both through QGC / RC / their own tool. The node only streams setpoints (the precondition PX4 requires before accepting an OFFBOARD switch) and waits passively for `mavros/state` to report `armed && mode == 'OFFBOARD'`.

## auto_arm node

One-shot helper that *is* the "external tool" for the sim, where no operator is present and `mavros_replicator` serves no arming/set_mode services (deferred, ticket 040). It publishes `px4_msgs/VehicleCommand` (`DO_SET_MODE` → OFFBOARD, then `COMPONENT_ARM_DISARM` → arm) to `fmu/in/vehicle_command`, retrying at `retry_period_s` until `mavros/state` reports `armed && mode == 'OFFBOARD'`, then exits (0 on success, 1 on `timeout_s`). It does **not** stream setpoints — it relies on `offboard_control` (same namespace) already doing so. Run once per vehicle from a tmux pane:

```
ros2 run mas_offboard auto_arm --ros-args -r __ns:=/px4_1 -p target_system:=2 -p use_sim_time:=true
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_system` | int | `1` | PX4 MAVLink system id (px4_1 → 2, px4_2 → 3) |
| `timeout_s` | float | `90.0` | Wall-clock deadline before giving up (exit 1) |
| `retry_period_s` | float | `1.0` | Resend period while waiting for confirmation |
| `stream_wait_s` | float | `3.0` | Grace after `mavros/state.connected` before the first command (lets the setpoint heartbeat establish) |

**Subscriptions:** `mavros/state` (`mavros_msgs/State`, RELIABLE). **Publishers:** `fmu/in/vehicle_command` (`px4_msgs/VehicleCommand`, BEST_EFFORT/TRANSIENT_LOCAL).

### State Machine

```
INIT ──(mavros topics received)──→ RAMP_UP ──(11 ticks)──→ WAIT_OFFBOARD
  ──(operator arms + sets OFFBOARD)──→ TAKEOFF ──(alt ≥ waypoint.z)──→ HOVER
  ──(dist < 2m, yaw < 10°, mission_state == MISSION)──→ POLICY
```

### Mission State Reactions (HOVER/POLICY only)

When airborne (HOVER or POLICY flight state), the node reacts to mission state changes:

- **HOVER_CMD (3):** Captures current local-frame position, switches to HOVER holding that position. Does not auto-transition to POLICY — operator must explicitly resume.
- **WAYPOINT (4):** Switches to HOVER at the configured waypoint (from `vehicles.yaml` parameters). Auto-transitions to POLICY when waypoint is reached and mission_state returns to MISSION.
- **MISSION (2) from HOVER_CMD/WAYPOINT:** Clears hover hold pose, resumes normal HOVER→POLICY transition logic.

### Subscriptions

This node consumes drone state **only** through `mas_common_frame` — it deliberately does not subscribe to `mavros/local_position/*`. The MAVROS link surfaces only through `mavros/state` (armed + flight mode).

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `mavros/state` | `mavros_msgs/State` | RELIABLE | Armed state, flight mode |
| `common_frame/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT | Drone pose in common frame (canonical state source) |
| `common_frame/local_origin` | `geometry_msgs/PointStamped` | RELIABLE, transient local | Constant common→local offset; subtracted at publish time to convert common-frame setpoints into the local frame MAVROS expects |
| `cmd_vel` | `geometry_msgs/TwistStamped` | BEST_EFFORT | Gated velocity command from mas_mission (ENU) |
| `mission_state` | `std_msgs/Int8` | RELIABLE, transient local | Mission state from mas_mission (gates HOVER→POLICY, triggers HOVER_CMD/WAYPOINT hold) |

### Publishers

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | 100 Hz | Velocity setpoint (RAMP_UP, TAKEOFF, POLICY) |
| `mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 100 Hz | Position setpoint (HOVER) |
| `initial_waypoint` | `nav_msgs/Odometry` | 100 Hz | Configured waypoint for downstream |

### Service Clients

None. Arming and mode change are operator-driven (QGC / RC / external tool); the node never calls `mavros/cmd/arming` or `mavros/set_mode`.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `''` | Vehicle namespace prefix |
| `update_rate` | float | `100.0` | Timer callback frequency (Hz) |
| `target_system` | int | `1` | PX4 MAVLink system ID |
| `position.x` | float | `0.0` | Waypoint X in common frame (meters) |
| `position.y` | float | `0.0` | Waypoint Y in common frame (meters) |
| `position.z` | float | `0.0` | Waypoint Z in common frame (meters, positive up) |
| `position.yaw_deg` | float | `0.0` | Waypoint yaw in common frame (degrees) |
| `takeoff_speed` | float | `3.0` | Climb rate (m/s) |

### Dependencies

**Upstream:** MAVROS node (same namespace), `mas_common_frame` (provides `common_frame/pose`), `mas_mission` (provides `cmd_vel` and `mission_state`)
**Downstream:** `gimbal_stabilizer` subscribes to MAVROS directly
