# mas_offboard — Node Interface Contract

## offboard_control node

Per-vehicle offboard controller for PX4 via MAVROS. Runs a state machine (INIT → RAMP_UP → ARM → TAKEOFF → HOVER → POLICY) and publishes velocity/position setpoints at 100 Hz.

### State Machine

```
INIT ──(mavros topics received)──→ RAMP_UP ──(11 ticks)──→ ARM
  ──(armed + OFFBOARD)──→ TAKEOFF ──(alt ≥ waypoint.z)──→ HOVER
  ──(dist < 2m, yaw < 10°, mission_state == MISSION)──→ POLICY
```

### Mission State Reactions (HOVER/POLICY only)

When airborne (HOVER or POLICY flight state), the node reacts to mission state changes:

- **HOVER_CMD (3):** Captures current local-frame position, switches to HOVER holding that position. Does not auto-transition to POLICY — operator must explicitly resume.
- **WAYPOINT (4):** Switches to HOVER at the configured waypoint (from `vehicles.yaml` parameters). Auto-transitions to POLICY when waypoint is reached and mission_state returns to MISSION.
- **MISSION (2) from HOVER_CMD/WAYPOINT:** Clears hover hold pose, resumes normal HOVER→POLICY transition logic.

### Subscriptions

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `mavros/state` | `mavros_msgs/State` | RELIABLE | Armed state, flight mode |
| `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT | Position + attitude in local frame (ENU-FLU) |
| `mavros/local_position/odom` | `nav_msgs/Odometry` | BEST_EFFORT | Full odometry in local frame (ENU-FLU) |
| `common_frame/pose` | `geometry_msgs/PoseStamped` | BEST_EFFORT | Position + attitude in common frame (from mas_common_frame) |
| `cmd_vel` | `geometry_msgs/TwistStamped` | BEST_EFFORT | Gated velocity command from mas_mission (ENU) |
| `mission_state` | `std_msgs/Int8` | RELIABLE, transient local | Mission state from mas_mission (gates HOVER→POLICY, triggers HOVER_CMD/WAYPOINT hold) |

### Publishers

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | 100 Hz | Velocity setpoint (RAMP_UP, TAKEOFF, POLICY) |
| `mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 100 Hz | Position setpoint (HOVER) |
| `initial_waypoint` | `nav_msgs/Odometry` | 100 Hz | Configured waypoint for downstream |

### Service Clients

| Service | Type | Notes |
|---------|------|-------|
| `mavros/cmd/arming` | `mavros_msgs/CommandBool` | Arm/disarm (called in ARM state, ~1 Hz) |
| `mavros/set_mode` | `mavros_msgs/SetMode` | Set OFFBOARD mode (called in ARM state, ~1 Hz) |

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
