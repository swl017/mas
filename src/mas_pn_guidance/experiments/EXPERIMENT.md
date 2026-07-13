# PN Engagement Experiment Suite

Self-contained operator suite for ego-only PN interception sweeps against the
Isaac Sim + PX4 SITL stack (crossing / tail-chase capability grids). Supersedes
the procedure scattered across ticket-007 `REPRODUCE.md` and the ticket-008/010
sweep logs; those remain the historical record.

```text
experiments/
├── EXPERIMENT.md            # this runbook
├── profiles/                # frozen estimator + PN configurations
│   ├── engagement_ekf_{OLD,TUNED,INTER}.launch.py
│   ├── pn_{N2,N3}.yaml
│   └── install_profile.sh   # apply a profile to the LIVE stack (src+install+restart)
├── manifests/               # sweep definitions consumed by run_sweep.py
│   ├── template.yaml
│   └── ticket010_n2_old_sweep.yaml
├── run_sweep.py             # manifest-driven batch runner (gates, watchdog, archive)
└── analysis/
    ├── qa_target_tracking.py    # realized-dynamics QA per boot
    ├── plot_boundary_3arm.py    # capability-grid boundary figure
    └── boot_table.py            # quick CSV result table
```

Nothing here is installed by colcon; run everything from source in a shell with
`source /home/usrg/mas/install/setup.bash`.

## 1. Quickstart

```bash
# 0. stack up: isaac_sim + interceptor + target tmux sessions, both drones
#    armed and holding at ICs (px4_1 (0,-50,~30); px4_2 (0,0,30))
cd /home/usrg/mas/src/mas_pn_guidance/experiments

# 1. pick the experiment arm configs (copies to src+install, restarts windows)
profiles/install_profile.sh ekf OLD     # or TUNED / INTER
profiles/install_profile.sh pn  N2      # or N3
profiles/install_profile.sh verify      # sanity: sigma lines + "ready: N=" line

# 2. pick a manifest — the default straight-target baseline is ready-made:
#      manifests/baseline_straight.yaml   (pn N3 + OLD; straight crossing + tail;
#                                          oracle + simple_ekf; vf {1,3,5,6,7})
#    or build a custom one: cp manifests/template.yaml manifests/my_sweep.yaml
#    (set a FRESH output.results_dir / boot ids per session)

# 3. run it (health gate first, then boots, one conductor at a time)
python3 run_sweep.py manifests/baseline_straight.yaml   # --dry-run to preview

# 4. look at results
python3 analysis/boot_table.py --data-dir results/my_sweep
python3 analysis/plot_boundary_3arm.py --data-dir results/my_sweep --output fig.png \
    --title "..." --subtitle "..."
```

## 2. What one engagement trial is

The conductor (`experiment_conductor.py`, launched per boot) owns the full
lifecycle per trial: set `estimate_source` on the PN node → `/reset` both EKFs →
configure the target maneuver (capability-grid cell `vf<v>_alat<a>_f<freq>`) →
settle both vehicles at ICs → synchronized engage (mission → MISSION; target
maneuver + PN start together) → run to terminal condition (hit < 0.5 m CPA /
miss / timeout) → write the result row + rosbag → fly both vehicles home.

Ego-only means PN consumes `/px4_1/simple_loc/target_pose|twist` (World-Frame
CV bearing EKF) or `direct_loc/*`; GT enters scoring and gimbal pointing only
(oracle-assisted pointing — results are an optimistic bound for ego-only).

## 3. Profiles (why install_profile.sh exists)

Two config files decide the experiment arms and BOTH are installed by plain
copy — editing the source alone changes nothing that runs:

| what | source | actually runs from |
|---|---|---|
| EKF params (σ_pix, σ_acc, init) | `mas_bearing_loc/launch/engagement_ekf.launch.py` (**untracked in git**) | `install/.../launch/engagement_ekf.launch.py` |
| PN params (nav_constant, band) | `mas_pn_guidance/config/pn_guidance.yaml` | `install/.../config/pn_guidance.yaml` |

`install_profile.sh` copies a tracked profile to both locations, restarts the
`bearing_ekf` / `pn_guidance` tmux window, and prints the live values. The PN
node reads `nav_constant` once at init (runtime SetParameters only handles
`estimate_source`), so the window restart is mandatory.

Profile provenance: OLD/TUNED/INTER are byte-copies of the ticket-007 preserved
configs (`OLD 50/1.0/5.0 + DP 30/0.05`, `TUNED 50/0.15/2.0`, `INTER 50/0.5/3.0`);
`pn_N3` is point-mass parity, `pn_N2` is parallax-preserving PN (ticket 010).

**Are the two `install` lines necessary every session?** No — they are
idempotent config-swaps, not launches. `verify` is read-only and always worth
running; the two `install_profile.sh` lines only *do* anything when the live
state has drifted from the target profile:

- If `verify` already shows `nav_constant 3.0`, `sigma 1.0/5.0`, and a live
  `ready: N=3.0` line, you can **skip** `pn N3` / `ekf OLD` — the freshly-loaded
  `bearing_ekf` / `pn_guidance` windows already came up on those installed values.
- Run them whenever `verify` disagrees. State drifts across sessions/tickets
  (e.g. N=2 left installed by ticket 010), and `engagement_ekf.launch.py` is
  **untracked in git** — so nothing restores it automatically. When in doubt,
  running all three is cheap (~15 s + a window restart each) and guarantees the
  arm config; the default baseline sweep assumes N=3 + OLD.

*Staged improvement (not yet done):* make `ekf_profile:=` / `nav_constant:=`
first-class launch arguments so the copy-restart dance disappears. Needs a
tracked, parameterized `engagement_ekf.launch.py` + a rebuild — do it between
sweep sessions, never mid-session.

## 4. Manifests + runner

See `manifests/template.yaml` for the schema. The runner enforces:

- **Fresh boot ids** — refuses an id whose `boot_<id>_results.csv` exists.
- **Readiness** before every boot: previous conductor process gone (its post-run
  return-to-IC included), interceptor armed and within `ic_tol_m` of
  `interceptor_ic`.
- **Health gate** (optional `gate:` block): 1-trial oracle boot; CPA must beat
  `max_cpa_m` (healthy reference: oracle ≈ 0.03–0.2 m on a straight low-speed
  cell). Fail → sweep refused (session-level sensing degradation; restart sim).
- **settle_error streak watchdog**: ≥ K consecutive `settle_error` rows
  (default 3) → SIGINT to the conductor's process group, boot marked
  `aborted_settle_streak`, sweep stops (`policies.on_boot_abort`).
- **Archive + provenance**: boot CSV/JSONL copied to `output.results_dir` with
  `boot_<id>_provenance.json` (sha256 of the installed EKF launch, installed PN
  yaml, and the manifest — proves which configs produced which rows).
- **QA per boot**: `analysis/qa_target_tracking.py` output saved alongside
  (`FWD-SAT`/`ALAT-SAT` = unrealized cell).

### 4a. Changing initial conditions (range / geometry) — the two-place rule

The `session` block of the manifest owns the ICs; the boot list owns the sweep
axes. What each knob controls:

| where | key | controls |
|---|---|---|
| `session.interceptor_ic: [x, y]` | interceptor settle point (common frame) | **range + geometry** — the target holds at the common-frame origin `(0,0)`, so **range = ‖interceptor_ic‖**. `[0,-50]` = 50 m. `[0,-30]` = 30 m; `[0,-70]` = 70 m. A non-axis-aligned `[x,y]` sets an oblique start. |
| `session.ic_tol_m` | settle tolerance | how close to `interceptor_ic` counts as ready (default 1.5 m; widen for large-range starts) |
| `boots[].geometries` | `crossing` \| `tail_chase` | engagement geometry (one per boot) |
| `boots[].forward_speeds` / `lateral_accels` | m/s, m/s² | target maneuver cells (`a_lat=0` = straight) |
| `boots[].estimators` | `oracle` \| `simple_ekf` \| `direct_projection` | the arm(s) |

**⚠ The two-place rule for range.** Changing `interceptor_ic` changes the
engagement range, but the BO-EKF's **initial range prior is set separately** and
must be changed to match — a bearing-only EKF is sensitive to it (starting at
50 m while flying a 30 m engagement biases/slows convergence and confounds the
ego-only result). It is a launch arg in the **interceptor tmux session**, not the
manifest:

```yaml
# src/tmux/sim_interceptor.tmuxp.yaml — window: bearing_ekf
ros2 launch mas_bearing_loc engagement_ekf.launch.py vehicle:=${ROBOT_NAME} \
    init_range_guess:=50.0 use_sim_time:=${USE_SIM_TIME}   # <- set to the new IC range
```

Also `direct_init_range_guess` (default 30.0) for the `direct_projection` arm.
After editing, reload the interceptor session (or restart just the `bearing_ekf`
window) so the node re-reads it. Rule of thumb: **`init_range_guess` ≈
‖interceptor_ic‖ every time you move the range.**

Not in the manifest (leave unless you mean to change them): **altitude** (~30 m,
set by the `offboard` hover config, not the ICs) and the **health gate** cell —
at a very different range re-confirm the oracle gate still lands < `max_cpa_m`
(it should; oracle is range-agnostic given GT state).

## 5. Scoring discipline (unchanged, critical)

- CPA = `min_range_m` vs `/{target}/common_frame/odom` GT **only**; never score
  an estimate against the recorded estimate.
- `settle_error` rows are invalid regardless of CPA — excluded everywhere.
- A valid `miss` near 50 m means engaged-but-never-closed; `timeout` is valid.
- Sim runs ~0.18× realtime; ~6 Hz estimator publish in a bag is the sim-rate
  artifact, not dropout.

## 6. Known failure modes

| symptom | cause | handling |
|---|---|---|
| first trial of a boot `settle_error` | settle window opens before vehicles re-settle (seen with seed 42) | lose 1 row; re-run the cell in a small fill-in boot |
| consecutive `settle_error` at ~constant range | **stranded chase cascade**: a failed high-speed (esp. tail vf≥6) chase leaves the interceptor tens of m out; every later settle times out | watchdog kills the boot; re-run lost cells as per-arm fill-in boots |
| interceptor disarmed on ground far from IC | PX4 auto-disarm after the stranded descent | `wait_ready` reports it; re-arm usually REFUSED (below) |
| re-arm rejected, PX4 console: `Preflight Fail: High Accelerometer Bias` | EKF bias learned during the abnormal descent | sim reboot (operator); force-arm (param2=21196) exists but bypasses safety checks — operator decision only |
| oracle gate misses by tens of m | session-level sensing degradation (long sessions) | restart `isaac_sim`, re-verify profiles, re-gate |
| numeric list launch args throw `InvalidParameterTypeException` | ROS2 parses `4.0` as DOUBLE, node expects STRING | runner appends the trailing comma automatically; keep it when launching by hand |

## 7. Figure regeneration policy

`analysis/plot_boundary_3arm.py` never writes to a paper-figure path by
default. When updating a cornerstone figure: snapshot the current one first
(`*_preNNN.png` convention, tickets 008/010), then pass `--output` explicitly.
Add separate result sets; never replace existing boot CSVs (repo evidence
discipline).

## 8. History / provenance

- Ticket 004 — harness + conductor; ticket 007 — EKF retune + REPRODUCE.md;
  ticket 008 — frozen-INTER 50-cell boundary sweep (robust rule, 3-arm figure);
  ticket 010 — N=2 + OLD sweep (this suite's procedures were extracted from its
  operational log, including the stranding incident and recovery).
- mas ticket 042 — this packaging. Related: mas ticket 041 (acceleration
  setpoint output — a future `command_mode` A/B axis for these sweeps).
