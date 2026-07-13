## Ticket 041 — PN acceleration setpoint via `mavros_msgs/PositionTarget` (setpoint_raw/local)

**Status**: Open
**Created**: 2026-07-13

**What**: Give `mas_pn_guidance` a second output channel that commands PX4 with an
**acceleration** setpoint, in addition to the existing velocity setpoint. PX4 exposes
this through `mavros_msgs/PositionTarget` on `mavros/setpoint_raw/local`
(`acceleration_or_force` field, gated by `type_mask`). The mode is selected by a new
runtime-switchable `command_mode` param so it becomes an A/B experiment axis alongside
`guidance_mode` and `estimate_source`.

**Why**: The PN law already *is* an acceleration law —
`pn_law.proportional_navigation` returns `a = N·Vc·(Ω × n̂)` (perpendicular to the
LOS). Today the node throws that acceleration through an integrator
(`v_cmd = clamp(v_cmd + a_pn·dt, v_max)`) and ships only the resulting velocity as
`pn/cmd_vel`. PX4's multicopter controller then numerically re-differentiates that
velocity setpoint to recover an acceleration feedforward — so we integrate, ship, and
PX4 differentiates, losing the crisp lateral-accel command the PN law computed directly.
Commanding the acceleration outright (as a feedforward on top of the velocity, or as a
pure accel setpoint) removes that round-trip and lets us study how faithfully PX4 tracks
the point-mass PN acceleration versus the constant-speed-pursuer velocity approximation
(the ~10 s PX4 accel ramp that `guidance_uses_measured_velocity` was added to expose,
ticket 004).

**Blocked on**: Nothing. `mavros_msgs/PositionTarget` is already built in the Humble
workspace (`/home/usrg/ros2_humble/install/mavros_msgs`). No new message package.

**Depends on**: The existing velocity path (this ticket) and ticket 040
(mavros_replicator). **Interop note**: under the uXRCE-DDS + `mavros_replicator`
transport (ticket 040), `mavros/setpoint_raw/local` is NOT yet an inbound topic the
replicator translates — its inbound support today is `setpoint_velocity/cmd_vel`. So the
accel path lands first against **real MAVROS** (sim: `sim_interceptor.tmuxp.yaml`); the
replicator's inbound `setpoint_raw/local → fmu/in/trajectory_setpoint` translation is also
in the scope.

---

### Current command path (velocity — unchanged by this ticket)

```
pn_guidance_node                    mas_mission (engagement_source=pn)   mas_offboard (POLICY)
  PN law -> a_pn (lateral accel)      _policy_cmd_vel_cb:                   _state_policy:
  v_cmd = clamp(v_cmd+a_pn·dt, vmax)    if state==MISSION:                    if cmd_vel is not None:
  pub pn/cmd_vel (TwistStamped, ENU) ─▶  pub cmd_vel (TwistStamped) ───────▶   pub mavros/setpoint_velocity/cmd_vel
     50 Hz, frame=common_frame                                                    (TwistStamped) 100 Hz ─▶ MAVROS ─▶ PX4
```

Frames: everything internal is **ENU / common_frame**. `common_frame → local` is a pure
translation (`common_frame/local_origin` offset), so **velocity and acceleration vectors
pass through unrotated** — only *position* setpoints subtract the origin offset. MAVROS's
`setpoint_raw` plugin then flips ENU→NED on the way to PX4, exactly as it already does for
`setpoint_velocity/cmd_vel`. `coordinate_frame = FRAME_LOCAL_NED (1)`.

---

### Decision: additive `command_mode` param, keep the velocity path byte-for-byte

Add `command_mode ∈ {velocity, accel_ff, accel}` (default `velocity`). This mirrors the
package's existing "runtime-switchable comparison axis" pattern and is the lowest-risk,
most reversible design: the working `pn/cmd_vel` TwistStamped path is **not touched** in
`velocity` mode. The accel modes publish a **parallel** `pn/setpoint_raw`
(`PositionTarget`) stream that is plumbed through the same two gated hops
(`mas_mission` MISSION gate → `mas_offboard` POLICY state), so mission/flight-state gating
is preserved. We deliberately do NOT bypass `mas_offboard` by publishing
`mavros/setpoint_raw/local` straight from the guidance node — the POLICY-only, 100 Hz,
zero-on-dropout streaming discipline lives in offboard and must stay there.

| `command_mode` | `type_mask` (ignore bits set) | Fields sent | Meaning |
|---|---|---|---|
| `velocity` (default) | — (uses TwistStamped path, no PositionTarget) | velocity = `v_cmd` | Current behavior. Untouched. |
| `accel_ff` (**recommended new default for accel study**) | `IGNORE_PX\|PY\|PZ (7)` + `IGNORE_YAW (1024)` = **1031** | velocity = `v_cmd`, acceleration = `a_pn`, yaw_rate = 0 | Velocity setpoint **plus** PN lateral-accel feedforward. Closing still carried by `v_cmd`; PX4 gets the exact PN accel instead of re-differentiating. |
| `accel` (stretch) | `IGNORE_PX\|PY\|PZ (7)` + `IGNORE_VX\|VY\|VZ (56)` + `IGNORE_YAW (1024)` = **1087** | acceleration = `a_cmd`, yaw_rate = 0 | Pure acceleration; PX4 integrates accel→vel→pos. **Requires a closing/speed-regulation accel term** (below). |

Masks are the intended values; **step 6 confirms them against PX4 behavior** — some PX4
builds reject an OFFBOARD accel setpoint that also ignores velocity, and the yaw handling
(ignore-yaw + `yaw_rate=0` to hold heading, matching the current `cmd_vel` path which sends
`twist.angular.z=0`) must be verified not to command a yaw spin.

#### The closing-term problem (the load-bearing design point)

`pn_law` returns **lateral accel only** — `a_pn ⟂ LOS`. In the point-mass model, all
*closing* (along-LOS) motion comes from the `v_max` pursuit **velocity** seed, never from
the accel. Therefore:

- `accel_ff` is correct as-is: `v_cmd` still carries closing, `a_pn` is a pure lateral
  feedforward. **This is why `accel_ff` is the recommended primary deliverable.**
- `accel` (pure) with only `a_pn` would make the drone *turn but never build forward
  speed* — no closing. Pure-accel mode MUST synthesize a total command:

  ```
  a_cmd = limit_norm( a_pn + k_v · (v_cmd − v_meas), a_max )
  ```

  i.e. reuse the existing integrated `v_cmd` pursuit as the velocity *reference* and add a
  proportional velocity-tracking accel (`k_v`, new param) so PX4's integrator is driven
  toward the pursuit speed. `v_meas` = ego odom velocity (already cached as `self.own_v`).
  This keeps `accel` mode behaviorally comparable to the velocity baseline while exercising
  PX4's acceleration-control path.

---

### Workflow

1. **`mas_pn_guidance/pn_guidance_node.py`** — add the param + publisher, branch the
   output:
   - `declare_parameter("command_mode", "velocity")`, validate against
     `{"velocity","accel_ff","accel"}` (warn+fallback like `guidance_mode`), make it
     runtime-settable in `_on_set_params` (no differentiator reset needed — same guidance,
     different actuation).
   - `declare_parameter("accel_track_gain", 4.0)` (`k_v` for `accel` mode only).
   - New publisher `self.pub_setpoint = self.create_publisher(PositionTarget,
     "pn/setpoint_raw", 10)` (import from `mavros_msgs.msg`; add `mavros_msgs` to
     `package.xml` `<exec_depend>`).
   - In `_publish_cmd_and_diag(cmd)`: keep publishing `pn/cmd_vel` (diagnostics/plotting
     stays uniform), and **additionally** publish `pn/setpoint_raw` when `command_mode !=
     "velocity"`. The `a_pn` to ship is `cmd.acceleration_mps2` (already clamped to
     `a_max`). Build the mask/fields per the table; for `accel` compute
     `a_cmd = limit_norm(cmd.acceleration_mps2 + k_v·(self.v_cmd − self.own_v), a_max)`
     (guard `own_v is None`). `header.frame_id = "common_frame"`,
     `coordinate_frame = FRAME_LOCAL_NED`.
   - In `_publish_zero()` (idle/dropout): when in an accel mode, publish a **zero-velocity**
     PositionTarget (velocity=0, ignore accel+position, `IGNORE_YAW`) — NOT a zero-accel
     one. Zero accel with velocity ignored tells PX4 "hold current velocity", which drifts;
     zero *velocity* is the correct hold. Document this in the CONTEXT.md.
2. **`config/pn_guidance.yaml`** — add `command_mode: velocity` and `accel_track_gain: 4.0`
   with the same explanatory-comment style as the other params.
3. **`mas_mission/mission_node.py`** — forward the parallel stream, gated identically to
   `cmd_vel`:
   - Subscribe `PositionTarget` on `{source}/setpoint_raw` (extend the
     `_engagement_cmd_topic` map: `pn → pn/setpoint_raw`; `policy`/`maneuver` have no accel
     source yet, so only subscribe when the resolved source provides one).
   - Add `self.setpoint_raw_pub = create_publisher(PositionTarget, "setpoint_raw", ...)`
     (best-effort, same QoS as `cmd_vel`) and `_engagement_setpoint_raw_cb` that republishes
     **only when `self.state == MISSION`** (mirror `_policy_cmd_vel_cb`).
   - This is additive; `cmd_vel` forwarding is unchanged. In `velocity` mode the guidance
     node simply never publishes `pn/setpoint_raw`, so nothing is forwarded.
4. **`mas_offboard/offboard_control.py`** — add the accel actuation to the POLICY state:
   - New subscriber `setpoint_raw` (`PositionTarget`, best-effort) → cache
     `self.cmd_setpoint_raw`.
   - New publisher `mavros/setpoint_raw/local` (`PositionTarget`, reliable, matching
     `vel_pub` QoS).
   - In `_state_policy`: if `self.cmd_setpoint_raw is not None`, republish it with a fresh
     stamp to `mavros/setpoint_raw/local`; **else** fall back to the existing zero-velocity
     stream. Prefer setpoint_raw over cmd_vel when both are present (an operator running
     `command_mode=accel_ff` will have both; the accel path wins). PX4 OFFBOARD requires a
     *continuous* stream — the 100 Hz timer must emit *some* setpoint every tick, so when
     `cmd_setpoint_raw` goes stale (> `stale_timeout`), revert to zero-velocity, don't stop.
   - RAMP_UP / TAKEOFF / HOVER states are **unchanged** (still TwistStamped / PoseStamped).
     Only POLICY gains the accel branch.
5. **Launch** — thread `command_mode` (and `accel_track_gain`) through
   `pn_guidance.launch.py` as a declared launch arg (default `velocity`) so
   `sim_interceptor.tmuxp.yaml` / `experiment_conductor` can select it. No launch-file
   change needed in `mas_offboard`/`mas_mission` (new topics are relative, auto-namespaced).
6. **PX4 / MAVROS validation** (bench-before-fly, sim): with a hand-published
   `PositionTarget` on `mavros/setpoint_raw/local` in armed OFFBOARD, confirm (a) PX4 accepts
   the `accel_ff` mask (1031) and the vehicle responds to the accel feedforward, (b) whether
   the pure-`accel` mask (1087, velocity ignored) is accepted or rejected on this PX4 build
   (v1.14 sim / v1.15 hw — see [[project_px4_msgs_branch_per_target]]), and (c) `yaw_rate=0`
   holds heading rather than commanding a spin. Record the working masks back into the table
   in this ticket and the CONTEXT.md.
7. **Docs** — update `mas_pn_guidance/CONTEXT.md` (new publisher `pn/setpoint_raw`, param
   `command_mode`/`accel_track_gain`, the closing-term note), `mas_mission/CONTEXT.md`
   (forwarded `setpoint_raw`), `mas_offboard/CONTEXT.md` (new sub/pub + POLICY accel branch),
   and `src/ARCHITECTURE.md` if the accel topic wiring is drawn there.

---

### Scope boundary

- **DO** keep `command_mode=velocity` byte-for-byte identical to today — the accel path is
  purely additive.
- **DO** route the accel setpoint through `mas_mission` (MISSION gate) and `mas_offboard`
  (POLICY state, 100 Hz, zero-on-dropout). Preserve the streaming discipline.
- **DO** carry closing via `v_cmd` in `accel_ff`; synthesize the `a_pn + k_v·(v_cmd−v_meas)`
  total-accel command in pure `accel` mode.
- **DO** validate the exact `type_mask` against PX4 before flight (step 6) and write the
  confirmed values back into this ticket + CONTEXT.md.
- **DO NOT** convert the whole `mas_offboard` setpoint contract to `PositionTarget`.
  `mas_policy` (engagement_source=policy) still feeds `cmd_vel`; RAMP_UP/TAKEOFF/HOVER stay
  TwistStamped/PoseStamped. Out of scope, higher blast radius.
- **DO NOT** publish `mavros/setpoint_raw/local` directly from `pn_guidance_node` (bypasses
  gating).
- **DO NOT** change `pn_law.py` — the acceleration it returns is exactly what we ship. The
  `tests/test_pn_law.py` point-mass parity pin must still pass unchanged.
- **DEFERRED**: `mavros_replicator` inbound `setpoint_raw/local → fmu/in/trajectory_setpoint`
  translation (ticket 040 currently translates `cmd_vel` only). Until then, accel modes run
  against real MAVROS. **DEFERRED**: making `command_mode` a conductor-randomized A/B axis
  (`experiment_conductor.py`). **DEFERRED**: accel path for `engagement_source=policy`.

---

### Affected files

- EDIT: [mas_pn_guidance/pn_guidance_node.py](/home/usrg/mas/src/mas_pn_guidance/mas_pn_guidance/pn_guidance_node.py) — `command_mode`/`accel_track_gain` params, `pn/setpoint_raw` publisher, accel branch in `_publish_cmd_and_diag`, zero-velocity accel hold in `_publish_zero`.
- EDIT: [config/pn_guidance.yaml](/home/usrg/mas/src/mas_pn_guidance/config/pn_guidance.yaml) — new params with comments.
- EDIT: [launch/pn_guidance.launch.py](/home/usrg/mas/src/mas_pn_guidance/launch/pn_guidance.launch.py) — declare + pass `command_mode`.
- EDIT: [package.xml](/home/usrg/mas/src/mas_pn_guidance/package.xml) — `<exec_depend>mavros_msgs</exec_depend>`.
- EDIT: [mas_mission/mission_node.py](/home/usrg/mas/src/mas_mission/mas_mission/mission_node.py) — `setpoint_raw` sub/pub + MISSION-gated forward.
- EDIT: [mas_offboard/offboard_control.py](/home/usrg/mas/src/mas_offboard/mas_offboard/offboard_control.py) — `setpoint_raw` sub, `mavros/setpoint_raw/local` pub, POLICY accel branch.
- EDIT: CONTEXT.md ×3 (`mas_pn_guidance`, `mas_mission`, `mas_offboard`) + `src/ARCHITECTURE.md`.
- NEW (test): [mas_pn_guidance/tests/test_command_modes.py](/home/usrg/mas/src/mas_pn_guidance/tests/test_command_modes.py) — mask + field construction per mode; `accel` total-accel clamp.

---

### Acceptance criteria

- [ ] `command_mode=velocity` (default) produces `pn/cmd_vel` and the downstream
      `mavros/setpoint_velocity/cmd_vel` **identical** to pre-ticket; no `pn/setpoint_raw` is
      published. Existing behavior and `test_pn_law.py` unchanged.
- [ ] `command_mode=accel_ff` publishes `pn/setpoint_raw` (`PositionTarget`) at 50 Hz with
      `type_mask` per the confirmed value, `velocity == v_cmd`,
      `acceleration_or_force == a_pn` (the clamped PN lateral accel), `frame_id=common_frame`,
      `coordinate_frame=FRAME_LOCAL_NED`.
- [ ] `command_mode=accel` publishes acceleration-only setpoints with
      `a_cmd = limit_norm(a_pn + k_v·(v_cmd − v_meas), a_max)` and velocity ignored (if PX4
      accepts the mask; otherwise step 6 documents the fallback).
- [ ] Forwarding gates hold: `pn/setpoint_raw` reaches `mavros/setpoint_raw/local` **only**
      when mission_state==MISSION **and** offboard flight_state==POLICY; outside that,
      `mavros/setpoint_raw/local` is not driven (or streams zero-velocity in POLICY dropout).
- [ ] Dropout/idle emits a **zero-velocity** PositionTarget in POLICY (no drift), not a
      zero-accel one; the 100 Hz stream is never interrupted while armed OFFBOARD.
- [ ] Sim engagement (`sim_interceptor.tmuxp.yaml`, oracle source): `accel_ff` intercepts
      the target with the same or tighter miss distance than `velocity` on the same seed
      (intercept_radius_m=0.5 parity), demonstrating the accel feedforward doesn't degrade
      tracking.
- [ ] Runtime switch `command_mode` via `ros2 param set` takes effect without a node
      restart and without a transient bad setpoint (verified in a param-switch test).
- [ ] Confirmed `type_mask` values + PX4 acceptance results recorded in this ticket and
      `mas_pn_guidance/CONTEXT.md`.

---

### Risk

**Medium**. New actuation path on the live control chain, but additive and mode-gated —
`velocity` mode is untouched, so the default engagement is unaffected. The real unknowns are
PX4-side: whether this multicopter build honors a pure-acceleration OFFBOARD setpoint
(velocity ignored) and how it blends the accel feedforward with the velocity setpoint in
`accel_ff`. Both are contained by the step-6 bench validation before any flight, and by the
zero-velocity dropout hold. Mixing setpoint types on the same OFFBOARD stream (POLICY may
alternate accel and zero-velocity) is legal in PX4 as long as the stream stays continuous,
which the 100 Hz timer guarantees.

### Coupling

- **Ticket 040 (mavros_replicator)**: `accel_ff`/`accel` require real MAVROS until the
  replicator gains an inbound `setpoint_raw/local` translation. `velocity` mode remains the
  replicator-compatible path. Land this ticket against MAVROS; fold the replicator inbound
  translation in as a follow-on. See [[project_px4_msgs_branch_per_target]] for the
  sim/hardware PX4 version split that step 6 must test against.
- **Ticket 004 (engagement_source)**: extends the same `engagement_source=pn` slot; the
  `policy` source accel path is deferred.

**Reference**: `mavros_msgs/PositionTarget`
([/home/usrg/ros2_humble/install/mavros_msgs/share/mavros_msgs/msg/PositionTarget.msg](/home/usrg/ros2_humble/install/mavros_msgs/share/mavros_msgs/msg/PositionTarget.msg)),
[pn_law.py](/home/usrg/mas/src/mas_pn_guidance/mas_pn_guidance/pn_law.py) (accel is already
the native output), ticket 004 (PN engagement slot), ticket 040 (transport).

**Flow**: Medium (additive multi-package actuation path; needs a PX4/MAVROS accel-setpoint
bench check before flight, but no guidance-law change).
