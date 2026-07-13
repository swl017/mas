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
- target estimate, by `estimate_source` (used by `guidance_mode` pn/bearing_pn):
  - `oracle` → `/{target_namespace}/common_frame/odom` (`Odometry`, GT ceiling)
  - `simple_ekf` → `simple_loc/target_pose` (`PoseWithCovarianceStamped`) + `simple_loc/target_twist` (`TwistStamped`)
  - `direct_projection` → `direct_loc/target_pose` + `direct_loc/target_twist`
- `bearing_raw/los` (`geometry_msgs/Vector3Stamped`) — RAW world-ENU unit LOS from
  `mas_bearing_loc raw_los_node`, at the detection rate with the detection stamp;
  **only** input to `guidance_mode:=raw_ibvs` steering (decoupled from the EKF)

#### Publishers
- `pn/cmd_vel` (`geometry_msgs/TwistStamped`, ENU) — engagement velocity setpoint, 50 Hz
- `pn/diagnostics` (`std_msgs/Float64MultiArray`) — `[closing_speed, los_rate, range, saturated, |v_cmd|, cov_trace]`

#### Parameters (defaults = point-mass parity, paper_nominal_low band)
`nav_constant` 3.0 · `v_max` 9.0 · `a_max` 6.0 · `control_rate_hz` 50.0 ·
`estimate_source` oracle · `target_namespace` '' (→ roles.yaml `target`) ·
`stale_timeout_s` 0.5 · `cov_trace_gate` 0.0 (off; >0 skips high-cov EKF estimates) ·
`engage_in_mission_only` true · `guidance_uses_measured_velocity` false
(false mirrors point-mass = integrated commanded velocity).

**`guidance_mode`** ∈ `{pn, bearing_pn, raw_ibvs}` (runtime-settable A/B axis):
- `pn` — range-sensitive baseline (LOS rate reconstructed as `cross(r,v)/|r|²` from
  the 3-D estimate).
- `bearing_pn` — range-tolerant; LOS rate from the STAMPED **position-derived**
  bearing (ticket 011). Still parallax-coupled to a persistent range bias.
- `raw_ibvs` — range-free; LOS rate from the **raw detection** bearing
  (`bearing_raw/los`), decoupled from the EKF; own-speed-along-LOS closing
  (ticket 012). Fails loud (holds zero) if `bearing_raw/los` is absent — no
  silent fallback to `pn`.
- Shared LOS-rate knobs: `los_rate_ema_alpha` 0.7 (EMA on the measured rate).
  raw_ibvs dropout (aged by detection stamp): `los_timeout_s` 0.3 (full-weight
  window) · `los_lost_s` 0.8 (coast-to-zero then target-lost). Both differentiators
  reset on any `estimate_source`/`guidance_mode` switch (no cross-source diff).

**Launch:** `pn_guidance.launch.py ns:=/px4_1 estimate_source:=oracle v_max:=9.0 a_max:=6.0 guidance_mode:=raw_ibvs`
(for raw_ibvs also run `mas_bearing_loc raw_los.launch.py vehicle:=px4_1`).

### Slice-1 tooling (also in this package)
- `roles` — role→namespace resolver (`roles.yaml`: ego_1v1, cooperative)
- `set_px4_limits` — pymavlink PARAM_SET of MPC limits per band (`px4_bands.yaml`)
- `gimbal_gt_tracker` — sim bring-up crutch: aim the gimbal at the target GT

### experiments/ — operator sweep suite (ticket 042; NOT colcon-installed)
Self-contained sweep tooling run from source: `profiles/` (frozen EKF configs
OLD/TUNED/INTER + PN `pn_N2/pn_N3.yaml` + `install_profile.sh` applying one to
src+install with window restart), `manifests/` (YAML sweep definitions),
`run_sweep.py` (batch runner: readiness + oracle health gate + settle_error
streak watchdog + archiving with config-sha provenance + QA), `analysis/`
(qa_target_tracking, plot_boundary_3arm, boot_table CLIs). Runbook:
`experiments/EXPERIMENT.md`. Codifies ticket-007 REPRODUCE.md + the
ticket-008/010 sweep procedures.

## Dependencies
**Upstream:** `mas_common_frame` (ego odom), a `mas_bearing_loc` EKF or the target's
odom (target estimate), `mas_mission` (`mission_state`).
**Downstream:** `mas_mission` (`engagement_source=pn` forwards `pn/cmd_vel`).

## Key files
- `mas_pn_guidance/pn_law.py` — pure PN math; **keep in sync** with
  `research/bearing_localization/interception_baseline/pn_guidance.py`
  (`tests/test_pn_law.py` pins it).
- `config/{pn_guidance,roles,px4_bands}.yaml`
