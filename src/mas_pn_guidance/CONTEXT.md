# mas_pn_guidance

Proportional-navigation interception guidance for the RA-L ego-only PN engagement
harness (Ticket 004). Drop-in command source in the same slot as `mas_policy`:
publishes a dedicated `pn/cmd_vel` that `mas_mission` (`engagement_source=pn`)
forwards as `cmd_vel` in MISSION → `mas_offboard` → MAVROS → PX4.

## Nodes / executables

### pn_guidance_node
**File:** `mas_pn_guidance/pn_guidance_node.py` · per-vehicle (launch under the
interceptor namespace). Decoupled pattern (subscribers cache, control timer is the
sole compute/publish point). Guidance law = `pn_law.proportional_navigation`,
which mirrors the Ticket 003 point-mass `pn_guidance.py` EXACTLY (true PN; the
constant-speed pursuer is seeded as a `v_max` pursuit at engagement, then
`v_cmd += a_pn·dt` clamped to `v_max`).

#### Subscriptions (relative to namespace unless absolute)
- `common_frame/odom` (`nav_msgs/Odometry`) — ego pose + velocity (ENU)
- `mission_state` (`std_msgs/Int8`) — gates engagement (compute only in MISSION=2)
- target estimate, by `estimate_source`:
  - `oracle` → `/{target_namespace}/common_frame/odom` (`Odometry`, GT ceiling)
  - `simple_ekf` → `simple_loc/target_pose` (`PoseWithCovarianceStamped`) + `simple_loc/target_twist` (`TwistStamped`)
  - `direct_projection` → `direct_loc/target_pose` + `direct_loc/target_twist`

#### Publishers
- `pn/cmd_vel` (`geometry_msgs/TwistStamped`, ENU) — engagement velocity setpoint, 50 Hz
- `pn/diagnostics` (`std_msgs/Float64MultiArray`) — `[closing_speed, los_rate, range, saturated, |v_cmd|, cov_trace]`

#### Parameters (defaults = point-mass parity, paper_nominal_low band)
`nav_constant` 3.0 · `v_max` 9.0 · `a_max` 6.0 · `control_rate_hz` 50.0 ·
`estimate_source` oracle · `target_namespace` '' (→ roles.yaml `target`) ·
`stale_timeout_s` 0.5 · `cov_trace_gate` 0.0 (off; >0 skips high-cov EKF estimates) ·
`engage_in_mission_only` true · `guidance_uses_measured_velocity` false
(false mirrors point-mass = integrated commanded velocity).

**Launch:** `pn_guidance.launch.py ns:=/px4_1 estimate_source:=oracle v_max:=9.0 a_max:=6.0`

### Slice-1 tooling (also in this package)
- `roles` — role→namespace resolver (`roles.yaml`: ego_1v1, cooperative)
- `set_px4_limits` — pymavlink PARAM_SET of MPC limits per band (`px4_bands.yaml`)
- `gimbal_gt_tracker` — sim bring-up crutch: aim the gimbal at the target GT

## Dependencies
**Upstream:** `mas_common_frame` (ego odom), a `mas_bearing_loc` EKF or the target's
odom (target estimate), `mas_mission` (`mission_state`).
**Downstream:** `mas_mission` (`engagement_source=pn` forwards `pn/cmd_vel`).

## Key files
- `mas_pn_guidance/pn_law.py` — pure PN math; **keep in sync** with
  `research/bearing_localization/interception_baseline/pn_guidance.py`
  (`tests/test_pn_law.py` pins it).
- `config/{pn_guidance,roles,px4_bands}.yaml`
