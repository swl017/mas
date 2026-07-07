# `mavros_replicator` Spec

**Status**: Draft for review
**Created**: 2026-05-08 (ticket 040)
**Owns**: PX4 (`px4_msgs`, NED/FRD) ↔ MAVROS-shaped (`geometry_msgs`/`sensor_msgs`/`nav_msgs`/`mavros_msgs`, ENU/FLU) translation, per vehicle.

---

## 1. Purpose

A single ROS 2 node that lets the existing MAS graph keep its MAVROS-shaped interface after MAVROS itself is removed. The node subscribes to `/{ROBOT_NAME}/fmu/out/...` from uXRCE-DDS and republishes under `/{ROBOT_NAME}/mavros/...`, with frame and covariance conversion. It also accepts the few MAVROS-shaped command topics that downstream code uses (currently velocity setpoint) and translates them into PX4 inputs on `/{ROBOT_NAME}/fmu/in/...`.

**Design property** — `px4_msgs` is a build-dep of this node only. No other MAS package picks up `px4_msgs`.

---

## 2. Scope

### 2.1 In scope (v1 — required for current downstream consumers)

Verified against actual subscribers in [mas_common_frame](../mas_common_frame/), [mas_policy](../mas_policy/), [mas_operator](../mas_operator/):

**Outbound (PX4 → ROS, MAVROS-shaped):**
- `mavros/state` (`mavros_msgs/State`)
- `mavros/local_position/pose` (`geometry_msgs/PoseStamped`)
- `mavros/local_position/pose_cov` (`geometry_msgs/PoseWithCovarianceStamped`)
- `mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`)
- `mavros/local_position/odom` (`nav_msgs/Odometry`)
- `mavros/imu/data` (`sensor_msgs/Imu`)
- `mavros/home_position/home` (`mavros_msgs/HomePosition`)

**Inbound (ROS → PX4, MAVROS-shaped):**
- `mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/TwistStamped`) → `fmu/in/trajectory_setpoint` + `fmu/in/offboard_control_mode`

### 2.2 Deferred (v2 — not currently blocking)

- `mavros/imu/mag` — needs `vehicle_magnetometer`, only used in live-monitor tmux echoes today.
- `mavros/rc/in`, `mavros/rc/out` — same: monitoring-only consumers.
- `mavros/global_position/global` — no current consumer in deploy stack.
- `mavros/set_message_interval` service — used only by sim drones; uXRCE rates are baked into PX4's `dds_topics.yaml` so a no-op stub is sufficient if needed.
- Arming + mode-set commanding (`mavros/cmd/arming`, `mavros/set_mode`) — not subscribed by current MAS code.

### 2.3 Out of scope

- Cross-vehicle topic flow (separate concern; was the original ticket-040 scope, now decoupled).
- `mavros/odometry/in` (companion → autopilot vision pose).
- Anything that needs MAVLink-only parameters (the `param` plugin is already disabled in [drone.tmuxp.yaml](../tmux/drone.tmuxp.yaml)).

---

## 3. Topic Contract

`{ns}` = `/${ROBOT_NAME}` (e.g., `/px4_1`).

### 3.1 Outbound

| MAVROS topic (output) | Type | PX4 source(s) | Conversion notes |
|---|---|---|---|
| `{ns}/mavros/state` | `mavros_msgs/State` | `vehicle_status`, `vehicle_control_mode` | `nav_state` → mode string (§7); `armed` ← `arming_state == ARMING_STATE_ARMED` (2); `connected` ← uXRCE link freshness (last `vehicle_status` < 1 s old). |
| `{ns}/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `vehicle_odometry` | Position NED→ENU (§5); orientation FRD-in-NED → FLU-in-ENU (§5). |
| `{ns}/mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | `vehicle_odometry` | Same conversion as `pose`; covariance per §6.1. |
| `{ns}/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `vehicle_odometry` | Linear velocity NED→ENU (world frame); angular velocity FRD→FLU (body frame). **MAVROS quirk preserved**: linear is world-frame, angular is body-frame. |
| `{ns}/mavros/local_position/odom` | `nav_msgs/Odometry` | `vehicle_odometry` | Pose: world ENU. Twist: linear world ENU, angular body FLU — REP-147 aerial convention (matches PX4 `VehicleOdometry` and `velocity_local`). Deviates from `nav_msgs/Odometry` doc string ("twist in child_frame") because pure-body twist requires rotating world-NED velocity through current attitude, injecting attitude error into the velocity. `header.frame_id = {robot}/map`; `child_frame_id = {robot}/base_link`. Covariance per §6. |
| `{ns}/mavros/imu/data` | `sensor_msgs/Imu` | `sensor_combined`, `vehicle_odometry` | gyro/accel from `sensor_combined` FRD→FLU; orientation from `vehicle_odometry.q` (FRD-NED → FLU-ENU). Both topics arrive at 100 Hz so time alignment is implicit. Publish on `sensor_combined` arrival, using the most recent `vehicle_odometry.q`. Covariance: zero matrices (PX4 doesn't expose per-sample IMU cov; matches what current MAVROS install does in practice). |
| `{ns}/mavros/home_position/home` | `mavros_msgs/HomePosition` | `vehicle_local_position.ref_lat/ref_lon/ref_alt` (gated on `xy_global && z_global`) | `geo.lat/lon/alt` from the EKF local-frame reference; `position` is zero (the reference *is* the local origin); `orientation` derived from `heading` (NED→ENU yaw). Latch (publish only when the EKF origin changes, KEEP_LAST 1, TRANSIENT_LOCAL). PX4's stock `dds_topics.yaml` does **not** export `/fmu/out/home_position`, so `vehicle_local_position.ref_*` is the available source for the EKF local origin. |

### 3.2 Inbound

| MAVROS topic (input) | Type | PX4 sink(s) | Behavior |
|---|---|---|---|
| `{ns}/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | `fmu/in/trajectory_setpoint`, `fmu/in/offboard_control_mode` | linear ENU→NED, yawspeed FLU→NED; emits `OffboardControlMode{velocity=true}` plus `TrajectorySetpoint{position=NaN, velocity=v_ned, yaw=NaN, yawspeed=ω_z_ned}`. Streamed at the rate setpoints arrive (input must be ≥ 2 Hz to keep PX4 in offboard, PX4 hard requirement). Watchdog: if no setpoint for `setpoint_timeout_ms` (default 250 ms), stop publishing. |
| `{ns}/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | `fmu/in/trajectory_setpoint`, `fmu/in/offboard_control_mode` | position ENU→NED; ENU yaw (extracted from the FLU→ENU quaternion) → NED yaw (CW from north); emits `OffboardControlMode{position=true}` plus `TrajectorySetpoint{position=p_ned, velocity=NaN, yaw=ψ_ned, yawspeed=NaN}`. Same ≥ 2 Hz requirement. The position and velocity inbound topics share `/fmu/in/trajectory_setpoint`; the latest message on either topic wins, so callers may alternate between them without dropping OFFBOARD. |

---

## 4. Naming and Namespace

- Node name: `mavros_replicator`.
- Default namespace: `/{ROBOT_NAME}/mavros` (sourced from `robot.env`). Topics under that namespace match the MAVROS naming exactly.
- The PX4 side reads `/{ROBOT_NAME}/fmu/...` directly (uXRCE prefix is set by `UXRCE_DDS_PTCFG` on the autopilot, separate from the ROS node namespace).
- Package name: **`mavros_replicator`** (matches user terminology). Lives at `src/mavros_replicator/`.

---

## 5. Frame Conversions

Defined once and used everywhere; consistent with [frame_conventions.md](frame_conventions.md).

### 5.1 World rotation: NED → ENU

```
R_w = [[0, 1, 0],
       [1, 0, 0],
       [0, 0, -1]]              # swap x↔y, negate z
q_w = (x=√½, y=√½, z=0, w=0)    # ROS xyzw — represents R_w
```

Apply: `p_enu = R_w · p_ned`, `v_enu = R_w · v_ned`.

### 5.2 Body rotation: FRD → FLU

```
R_b = [[1,  0,  0],
       [0, -1,  0],
       [0,  0, -1]]              # negate y, z
q_b = (x=1, y=0, z=0, w=0)       # 180° around X
```

Apply: `ω_flu = R_b · ω_frd`, `a_flu = R_b · a_frd`.

### 5.3 Orientation (world↔body)

PX4's `vehicle_attitude.q` is body→world in NED/FRD, packed `[w, x, y, z]`. The ROS-side body-FLU→world-ENU quaternion is

```
q_ENU_FLU = q_w · q_NED_FRD · q_b⁻¹
```

In ROS xyzw form, output as `(q.x, q.y, q.z, q.w)`.

---

## 6. Covariance Propagation

All covariance fields are sourced from `vehicle_odometry` directly — no derivation across topics.

### 6.1 Pose covariance (6×6, order [x, y, z, rx, ry, rz])

`vehicle_odometry.position_variance` (3) and `vehicle_odometry.orientation_variance` (3) are diagonal-only in PX4 v1.15. Build the 6×6 in NED/FRD with those on the diagonal, then rotate:

```
Σ_enu = J · Σ_ned · Jᵀ
J = blockdiag(R_w, R_w)          # both translation and small-angle rotation rotate by world R_w
```

Pack into the 36-element row-major array.

### 6.2 Velocity covariance

`Odometry.twist` and `velocity_local.twist` use the same frame convention: linear world ENU, angular body FLU. PX4's `vehicle_odometry.velocity_variance` is linear-NED diagonal; the world-ENU covariance is simply `R_w · diag(Σ_v_ned) · R_wᵀ`, which reduces to swapping the x and y diagonal entries (R_w is a signed permutation, so a diagonal input stays diagonal).

```
Σ_v_enu = diag(σ_y_ned², σ_x_ned², σ_z_ned²)     # x↔y swap, z variance unchanged under sign flip
```

Angular velocity covariance: not exposed by PX4; fill with zeros.

`TwistStamped` has no covariance field, so `velocity_local` carries no covariance on the wire; only `odom.twist.covariance` is populated.

### 6.3 IMU covariance

Leave the `*_covariance` arrays at zero (PX4 doesn't publish per-sample variance; matches what current MAVROS install does in practice — downstream code in `mas_policy` reads field values, not covariances).

---

## 7. Mode String Mapping (`vehicle_status.nav_state` → `State.mode`)

Only the modes the current stack cares about are mapped exactly; the rest pass through as `"NAV_STATE_<n>"` so logs are still readable.

| `nav_state` | `State.mode` |
|---|---|
| 0 (MANUAL) | `"MANUAL"` |
| 1 (ALTCTL) | `"ALTCTL"` |
| 2 (POSCTL) | `"POSCTL"` |
| 3 (AUTO_MISSION) | `"AUTO.MISSION"` |
| 4 (AUTO_LOITER) | `"AUTO.LOITER"` |
| 5 (AUTO_RTL) | `"AUTO.RTL"` |
| 14 (OFFBOARD) | `"OFFBOARD"` |
| 17 (AUTO_TAKEOFF) | `"AUTO.TAKEOFF"` |
| 18 (AUTO_LAND) | `"AUTO.LAND"` |
| other | `f"NAV_STATE_{nav_state}"` |

`mas_operator` reads `state.mode` as a string; `state.armed` is the boolean.

---

## 8. Time Stamps

`header.stamp = node.get_clock().now()` at the moment the PX4 message is received. **Not** PX4's `timestamp` field (microseconds since autopilot boot, not ROS epoch — translating it directly produces an epoch-1970 footgun). This matches MAVROS's behavior. If sim-time alignment matters later, the `use_sim_time` parameter handles it through the rclpy clock layer.

---

## 9. QoS

Subscribers (PX4 side) — match PX4's uXRCE bridge defaults:

```
reliability   = BEST_EFFORT
durability    = VOLATILE
history       = KEEP_LAST
depth         = 5
```

Publishers (MAVROS side) — match what current MAVROS install advertises and what existing subscribers expect:

| Topic | reliability | durability | depth |
|---|---|---|---|
| `local_position/pose`, `pose_cov`, `velocity_local`, `odom` | RELIABLE | VOLATILE | 10 |
| `imu/data` | RELIABLE | VOLATILE | 10 |
| `state` | RELIABLE | VOLATILE | 10 |
| `home_position/home` | RELIABLE | **TRANSIENT_LOCAL** | 1 |

`mas_common_frame` uses `qos_profile_sensor_data` (BEST_EFFORT, KEEP_LAST 5) on its subscribers — that is compatible with our RELIABLE publishers (RELIABLE writer + BE reader is allowed).

---

## 10. Parameters

| Param | Default | Description |
|---|---|---|
| `robot_name` | `"px4_1"` | Top-level namespace (read first from env `ROBOT_NAME`, then param). Drives both PX4 and MAVROS topic prefixes. |
| `frame_id_world` | `"{robot_name}/map"` | `header.frame_id` for world-frame topics. |
| `frame_id_body` | `"{robot_name}/base_link"` | `child_frame_id` for body-frame fields. |
| `publish_tf` | `false` | If true, broadcast `frame_id_world → frame_id_body` from `local_position/odom`. Off by default — `mas_common_frame` owns the canonical TF. |
| `setpoint_timeout_ms` | `250` | Stop streaming `OffboardControlMode` if no `cmd_vel` arrives within this window. |
| `state_publish_period_ms` | `100` | Throttle `mavros/state` to 10 Hz; PX4 publishes status at 2 Hz native, but mode changes need to propagate quickly. Actually publish on every input msg, capped at this period. |

---

## 11. Acceptance Tests

The replicator passes if, with PX4 booted and uXRCE link up:

1. **Topic shape**: every `mavros/...` topic in §3.1 appears in `ros2 topic list`, with the correct ROS type (`ros2 topic info -v` matches the type column above).
2. **mas_common_frame compatibility**: launching `mas_common_frame` against the replicator (no MAVROS) produces non-empty `/{ns}/common_frame/odom` at the same rate it did with MAVROS — within 5%.
3. **mas_policy compatibility**: `observation_audit.py` runs clean against the replicator's `mavros/imu/data`.
4. **Frame correctness**: with vehicle stationary on the bench facing east-ish, `mavros/local_position/pose.position.x > 0` (east) when moved 1 m east; `pose.orientation` quaternion has `w ≈ 1` when level and forward; gravity reads as `linear_acceleration.z ≈ +9.8` on `imu/data` (FLU body frame).
5. **Setpoint round-trip**: publishing a TwistStamped with `linear.x=1, linear.y=0` (1 m/s east) results in `vehicle_local_position.vy_ned > 0.5` after 1 s of armed-offboard flight (north in NED == east in ENU, so y_ned, not x_ned). In sim if not on hardware.
6. **Setpoint watchdog**: stop publishing `cmd_vel` → after `setpoint_timeout_ms`, `OffboardControlMode` publication stops; PX4 falls out of OFFBOARD on its own per its own 0.5 s timeout.
7. **Two-vehicle**: with both vehicles' replicators running, each only consumes its own `fmu/...` topics and only produces its own `mavros/...` namespace. No cross-talk. (Re-uses the test harness from the uXRCE interference run.)

---

## 12. Resolved Decisions (post-review)

Resolved 2026-05-08:

1. **`State.connected`**: ≤ 1 s freshness of `vehicle_status` (no first-message gate).
2. **`pose` / `pose_cov` / `velocity_local` / `odom` source** → unified on **`vehicle_odometry`**. Rationale: the topic arrives at 100 Hz (measured, see §13), satisfying the ≥ 100 Hz requirement, and a single source guarantees time-alignment across all 4 derived topics. No deriving across separate messages.
3. **No data derivation across topics**: every field on every output is read directly from a PX4 topic. No differentiating, integrating, or composing across messages.
4. **Language**: **rclpy**. Math factored into a pure-Python `frames.py` module.
5. **`home_position`**: don't publish until PX4 sends one — same as current MAVROS.

### 12.1 Note on bridged topic set

`vehicle_angular_velocity` is **not in the uXRCE `dds_topics.yaml` whitelist** on the v1.15.4 build, so it never reaches the ROS graph. Body angular velocity is read from `vehicle_odometry.angular_velocity` instead (also 100 Hz, also direct, also time-aligned with the rest of `vehicle_odometry`).

`home_position` is likewise **not in the whitelist**, so `mavros/home_position/home` is derived from `vehicle_local_position.ref_*` (the EKF local-frame origin). This is the same quantity `mas_common_frame` consumes from `home_position.geo.*`; the `position` field of the MAVROS message is exactly zero in this scheme because the EKF reference is, by definition, the local origin.

## 13. Measured topic rates (PX4 v1.15.4, uXRCE-DDS, 20 s window, veh1)

| PX4 topic | Avg Hz | Min Hz | Max Hz |
|---|---|---|---|
| `vehicle_odometry` | 99.996 | 83.3 | 125.0 |
| `vehicle_local_position` | 99.997 | 83.3 | 125.0 |
| `vehicle_attitude` | 99.996 | 83.3 | 125.0 |
| `sensor_combined` | 99.997 | 83.3 | 125.0 |
| `vehicle_angular_velocity` | (not bridged) | — | — |
| `vehicle_status` | 1.975 (60 s window) | 2.0 | 2.0 |
