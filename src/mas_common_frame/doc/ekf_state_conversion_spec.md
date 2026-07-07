# EKF State Conversion ROS2 Node

**Package:** `mas_common_frame`
**Status:** Draft v1
**Last Updated:** 2026-03-24

---

## 1. Summary

Convert each vehicle's EKF local pose, orientation, and velocities into a shared *mission frame* using a one-time GPS-derived offset. Each agent runs its own instance of this conversion node. The mission frame is an ENU frame whose origin is a user-configured LLH point shared by all agents.

## 2. Motivation

The existing `common_frame_node` converts position via a per-tick GPS round-trip: the vehicle's current GPS (`mavros/global_position/global`) is transformed through ECEF to the mission ENU frame every cycle. This has two drawbacks:

1. **GPS noise injection** — the global position topic fuses GPS measurements, so the converted position inherits GPS noise even when the EKF has a smooth, drift-free local estimate.
2. **Unnecessary compute** — every tick performs a full GPS→ECEF→ENU chain, when the relationship between the local ENU frame and the mission frame is fixed (it only depends on the two origins, which don't change in flight).

The EKF-direct path computes the local-to-mission offset **once** (when the home position is received), then applies a simple vector addition per tick. This preserves the EKF's smooth local estimate without injecting GPS noise.

## 3. Coordinate Frames

| Frame | Origin | Axes | Set by |
|-------|--------|------|--------|
| **Local ENU** | Vehicle's home position (EKF origin) | East, North, Up | PX4 EKF (reported via `mavros/home_position/home`) |
| **Mission frame** | User-configured LLH point | East, North, Up | `common_frame_origin` parameter |

Both frames are ENU. They differ by a **translation** (distance between their origins) and a small **rotation** (the ENU axes at two different points on the WGS84 ellipsoid are not exactly parallel — they differ by a yaw angle proportional to the longitude difference).

## 4. Conversion Algorithm

### 4.1 One-Time Initialization

Triggered when `mavros/home_position/home` is received (provides the EKF local frame origin in LLH):

```
home_gps = (lat, lon, alt)  from HomePosition.geo

# Compute the position of the local origin in the mission frame
offset = CommonFrame.gps_to_enu(home_gps, mission_origin_gps)
       = GPS → ECEF → ENU  (WGS84 ellipsoid, standard geodetic transform)

# Compute the rotation matrix from local ENU to mission ENU
R_local_to_mission = R_ecef_to_enu(mission_origin) @ R_enu_to_ecef(home_gps)
```

The offset is a 3D vector `(east, north, up)` in meters. The rotation `R_local_to_mission` accounts for the fact that "East" at the home position points in a slightly different direction than "East" at the mission origin (due to Earth curvature).

After initialization, these values are **fixed** for the remainder of the flight.

### 4.2 Per-Tick Position

On each `mavros/local_position/pose` update:

```
p_local = (x, y, z)  from PoseStamped.pose.position

p_mission = p_local + offset
```

Simple vector addition. The EKF's local position is preserved exactly — no GPS noise injected.

### 4.3 Per-Tick Orientation

On each `mavros/local_position/pose` update:

```
q_local = quaternion from PoseStamped.pose.orientation

# Convert to Euler, apply frame rotation, convert back
orientation_local = (roll, pitch, yaw) from q_local
orientation_mission = CommonFrame.transform_orientation(orientation_local, home_gps)
q_mission = quaternion from orientation_mission
```

The `transform_orientation` function:
1. Builds `R_enu_to_ecef` at the local origin (home_gps)
2. Builds `R_ecef_to_enu` at the mission origin
3. Composes: `R_local_to_mission = R_ecef_to_enu_mission @ R_enu_to_ecef_local`
4. Applies this rotation to the orientation quaternion: `q_mission = q_rotation * q_local`

### 4.4 Per-Tick Velocities

On each `mavros/local_position/velocity_local` update:

```
v_linear  = (vx, vy, vz)  from TwistStamped.twist.linear
v_angular = (wx, wy, wz)  from TwistStamped.twist.angular

# Pass through directly (no rotation applied)
v_mission_linear  = v_linear
v_mission_angular = v_angular
```

**Caveat:** Strictly, linear velocities expressed in local ENU should be rotated by `R_local_to_mission` to be in mission ENU. The rotation is a yaw offset proportional to the longitude difference between the home position and mission origin. For typical operating distances (< 10 km), this yaw offset is < 0.1° and the error is negligible. If the system is deployed over larger distances or requires high-precision velocity alignment, the rotation should be applied:

```
v_mission_linear = R_local_to_mission @ v_local_linear
```

### 4.5 Per-Tick Covariance

On each `mavros/local_position/pose_cov` update:

```
P_local = 6x6 pose covariance from PoseWithCovarianceStamped.pose.covariance
        = [xx, xy, xz, xR, xP, xY;
           ...                     ;
           Yx, Yy, Yz, YR, YP, YY]   (row-major, 36 elements)
```

The covariance is propagated into the `Odometry` message's `pose.covariance` field. Since the position transform is a pure translation (`p_mission = p_local + offset`), the position covariance block (upper-left 3x3) is **unchanged**. The orientation covariance block (lower-right 3x3) should be rotated by `R_local_to_mission`, but for the same reason as velocities (Section 4.4), this rotation is negligible at typical operating distances and is passed through directly.

The `twist.covariance` in the `Odometry` message is left as zero since MAVRos does not provide twist covariance on a separate topic. If needed, it can be approximated from the EKF's velocity uncertainty via `mavros/local_position/odom`.

**Note:** PX4 must be streaming `LOCAL_POSITION_NED_COV` for the covariance fields to be populated. If `pose_cov` publishes zero covariance, verify the MAVLink stream rate configuration.

## 5. Flow Diagram

```
                     ONE-TIME (on home_position received)
                     ====================================

mavros/home_position/home ──► home_gps (lat, lon, alt)
         │
         ├──► offset = gps_to_enu(home_gps, mission_origin)
         │
         └──► R_local_to_mission = R_ecef_to_enu(mission) @ R_enu_to_ecef(home)


                     PER-TICK (timer at publish_rate Hz)
                     ====================================

mavros/local_position/pose ──► p_local, q_local
         │
         ├──► p_mission = p_local + offset
         │
         └──► q_mission = R_local_to_mission * q_local

mavros/local_position/pose_cov ──► P_local (6x6 covariance)
         │
         └──► pass through to odom pose.covariance

mavros/local_position/velocity_local ──► v_linear, v_angular
         │
         └──► pass through to odom twist (no rotation)
                    │
                    ▼
         ┌──────────────────────────┐
         │  Publish:                │
         │  common_frame/pose       │  (PoseStamped)
         │  common_frame/odom       │  (Odometry: pose + cov + twist)
         │  TF2 broadcast           │  (common_frame → {veh}_base_link)
         └──────────────────────────┘
```

## 6. ROS2 Interface

### 6.1 Subscriptions (per vehicle `/{veh}/`)

| Topic | Message Type | Purpose | QoS |
|-------|-------------|---------|-----|
| `mavros/home_position/home` | `mavros_msgs/HomePosition` | EKF local frame origin (one-time init) | BEST_EFFORT |
| `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF local position + orientation | BEST_EFFORT |
| `mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | EKF local pose + 6x6 covariance | BEST_EFFORT |
| `mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | EKF local linear + angular velocity | BEST_EFFORT |

### 6.2 Publishers (per vehicle `/{veh}/`)

| Topic | Message Type | Content | QoS |
|-------|-------------|---------|-----|
| `common_frame/pose` | `geometry_msgs/PoseStamped` | Position + orientation in mission frame | BEST_EFFORT |
| `common_frame/odom` | `nav_msgs/Odometry` | Full odometry (pose + twist + covariance) in mission frame | BEST_EFFORT |

### 6.3 TF2 Broadcasts

| Parent | Child | Content |
|--------|-------|---------|
| `common_frame` | `{veh}_base_link` | Position + orientation in mission frame |

### 6.4 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name_prefix` | string | `"px4_"` | Namespace prefix for vehicle topics |
| `num_vehicles` | int | `2` | Number of vehicles to track |
| `common_frame_origin` | float[3] | `[37.7749, -122.4194, 0.0]` | Mission frame origin [lat, lon, alt] in WGS84 |

### 6.5 Frames

| Frame ID | Description |
|----------|-------------|
| `common_frame` | Mission frame (ENU at `common_frame_origin`) |
| `{veh}_base_link` | Vehicle body frame |

## 7. Existing Utilities to Reuse

From `mas_common_frame/common_frame.py` (`CommonFrame` class):

| Function | Used for |
|----------|----------|
| `gps_to_enu(gps, origin_gps)` | One-time offset computation (GPS→ECEF→ENU) |
| `transform_orientation(orientation, local_origin_gps)` | Per-tick orientation rotation |
| `euler_to_quaternion(r, p, y)` / `quaternion_to_euler(q)` | Euler ↔ quaternion conversion |

## 8. Known Limitations

1. **Velocity rotation not applied** — linear velocities are passed through without rotating from local ENU to mission ENU. Acceptable for operating distances < 10 km; see Section 4.4 caveat.
2. **Twist covariance not available** — only pose covariance is propagated (from `pose_cov`). Twist covariance in the `Odometry` message is zero since MAVRos does not publish a separate twist covariance topic. Requires PX4 to stream `LOCAL_POSITION_NED_COV` for pose covariance to be non-zero.
3. **Home position assumed fixed** — the offset is computed once on first `home_position` message. If PX4 resets the EKF origin mid-flight, the offset would become stale. This is not expected during normal operation.
4. **Altitude reference** — the mission frame altitude convention follows `home_position.geo.altitude`, which is **AMSL** (mean sea level) per the MAVLink HOME_POSITION definition and matches PX4 `VehicleLocalPosition.ref_alt`. The `gps_to_ecef` math used to compute the offset is technically defined for WGS84 ellipsoid altitudes, but the resulting **ENU offset** is correct as long as `common_frame_origin` and the vehicle's home altitude use the **same** convention (both AMSL here). If WGS84-ellipsoid altitude is needed for some downstream consumer, a geoid separation correction must be added.
