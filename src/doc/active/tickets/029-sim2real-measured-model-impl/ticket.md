## Ticket: Implement measured camera/gimbal/latency models in iris_ma6

**Status**: Open
**Created**: 2026-04-27

**What**: Replace the placeholder DR ranges and dynamics constants in iris_ma6 with values fit to the recently collected real-system measurements (end-to-end detection latency, per-zoom camera intrinsics, zoom command→effective-focal curve, gimbal rate-loop step response). Each input artifact maps to one or more existing iris_ma6 config sites; this ticket is the implementation of the model, not collection of new data.

**Why**: D1 (mrcal intrinsic calibration), D2 (gimbal calibration), and parts of C2/C4 (latency) are now measured but the sim DR config is still using the original ungrounded placeholders (`focal_length_range=(800, 1200)`, `fov_scale_range=(0.5, 1.0)`, gimbal joint `stiffness=2e3 / damping=1e1`, `ego_detection_latency_mean=0.1, std=0.015`). Closing the loop — measurement → fitted model → DR config — is what mas/021's checklist anticipates and what makes the sim2real residual interpretable.

**Blocked on**: Nothing. All input artifacts already exist on disk.

**Depends on**: mas/028 (camera intrinsics — done), mas/026 (gimbal calibration — produced rate model), mas/021 (sim2real checklist — defines DR sites). No flight needed.

### Input artifacts (read-only inputs to this ticket)

1. **End-to-end image age** (camera sensor → detection output, not just inference) — three latency regimes selected by YOLO model size:
   - [phase7_low_latency.csv](/home/usrg/mas/src/scripts/phase7_low_latency.csv) — ~50 ms, 1062 samples
   - [phase7_mid_latency.csv](/home/usrg/mas/src/scripts/phase7_mid_latency.csv) — ~70 ms, 598 samples
   - [phase7_high_latency.csv](/home/usrg/mas/src/scripts/phase7_high_latency.csv) — ~140 ms, 584 samples
   - Schema: `sample_index, t_received, t_stamp, age_ms`. `age_ms` is the e2e quantity to fit.
   - Reference plot: [phase7_latency_dist.png](/home/usrg/mas/src/scripts/phase7_latency_dist.png)
2. **Per-zoom camera intrinsics with mrcal uncertainty**:
   - [datasets/camera_calibration/2026-04-17/{1x,2x,4x,5x,6x}/intrinsics_summary.json](/home/usrg/mas/datasets/camera_calibration/2026-04-17/)
   - Trust only up to **5x**. 6x excluded from any fit (calibration uncertainty blows up: fx_std/fx ≈ 10%).
3. **Zoom curve** (operator zoom command → effective focal multiplier):
   - [zoom_curve.json](/home/usrg/mas/src/scripts/camera_calibration/zoom_curve.json)
   - Closed-form: `z_eff = 1 + 0.32489·(exp(0.4767·(cmd-1)) - 1)`, valid for cmd ∈ [1, 5].
4. **Gimbal rate-loop step response**:
   - [rate_model.json](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_model.json) — fitted per-axis `k_deg_s_per_u`, `tau_rate_s`
   - [rate_step_summary.csv](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_step_summary.csv) — per-step rise time, settling, w_ss, latency
   - [rate_step_trace.csv](/home/usrg/mas/src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_step_trace.csv) — raw 100 Hz traces if a richer fit is needed

### Mapping: artifact → iris_ma6 config site

| Measurement | Sim site to update | Current placeholder | Notes |
|---|---|---|---|
| Per-zoom `fx` (1x..5x) and 1σ | [domain_randomization_cfg.py:241](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/domain_randomization_cfg.py#L241) `focal_length_range` | `(800, 1200)` | Replace with measured per-zoom mean ± k·σ; or make zoom-conditional |
| `fov_scale_range` semantics | [domain_randomization_cfg.py:231](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/domain_randomization_cfg.py#L231) | `(0.5, 1.0)` | Re-derive from per-zoom fx ratios so the range matches the operational zoom envelope, not an arbitrary 0.5 |
| Zoom curve `cmd → z_eff` | [iris_ma_env6_test.py:911](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test.py#L911), [iris_ma_env6_test.py:923](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test.py#L923) | `self.zoom_level` used directly as a linear multiplier on `focal_length` | Apply the exponential `z_eff(cmd)` so that the policy-emitted zoom command produces the same effective focal length as on the real camera |
| Detection e2e age distribution | [iris_ma_env6_test_cfg.py:330-331](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg.py#L330-L331) `ego_detection_latency_mean/std` | `0.1 / 0.015` | Refit (mean, std) from `age_ms`. Decide whether to keep one Gaussian or expose three regimes (low/mid/high) selectable by curriculum since the distribution is clearly multi-modal across YOLO models |
| Gimbal rate-loop dynamics | [iris_gimbal3.py:71-86](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py#L71-L86) joint `stiffness=1e3, damping=5e1` (active yaw/pitch/roll axes); [domain_randomization_cfg.py:294-297](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/domain_randomization_cfg.py#L294-L297) `stiffness_scale_range, damping_scale_range` | `(0.8, 1.2)` ±20% | The real measurement is a **rate-loop** model (`tau ≈ 0.10 s`, `k ≈ 73 deg/s per unit u`), not a joint stiffness/damping. Pick one of: (a) fit equivalent (k_p, k_d) on the joint that reproduces τ ≈ 0.1 s for small steps, then DR around that fit; or (b) introduce a first-order rate-loop wrapper between policy command and joint torque, matching the real controller architecture more directly. mas/026 ticket already notes τ scales with step magnitude — handle small-step fit only here, document large-step nonlinearity as out of scope. iris_ma6 robot cfg is `IRIS_GIMBAL3_CFG` from [iris_ma_env6_test_cfg.py:22](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg.py#L22). |

### Workflow

1. **Aggregate fits** under a new bench script `src/scripts/sim2real_model_fitting/` (or add to existing calibration scripts):
   - `fit_detection_latency.py` → reads the three phase7 CSVs, emits a single JSON with `{regime: {mean_ms, std_ms, n_samples, distribution_notes}}`. Decide and document whether to use one combined Gaussian or per-regime Gaussians.
   - `summarize_intrinsics_for_sim.py` → reads `intrinsics_summary.json` per zoom, emits a single JSON with `{zoom: {fx, fx_std, fy, fy_std}}` for 1x–5x; rejects 6x with a logged reason.
   - The zoom curve and gimbal rate model are already in fitted JSON form — copy or symlink, don't re-fit.
2. **Wire the fits into iris_ma6**:
   - DR config: load measured ranges at `domain_randomization_cfg.py` defaults (or via a side-loaded JSON path so retraining can swap measurement campaigns without code edits).
   - Zoom curve: implement `z_eff = 1 + a·(exp(b·(cmd-1)) - 1)` in the camera processor; replace the linear `zoom_level` multiplier with `z_eff(zoom_level)` at the two sites in `iris_ma_env6_test.py`. Clamp cmd ∈ [1, 5]; document why 6x is rejected.
   - Latency: update `ego_detection_latency_mean/std`. If we keep three regimes, expose a curriculum knob; otherwise use the union distribution and record the choice in `iris_ma_env6_v1_cfg.py` defaults too.
   - Gimbal: implement chosen approach (rate-loop wrapper preferred for fidelity; equivalent stiffness/damping if simpler is enough). Re-validate that `GimbalRandomizationCfg.stiffness_scale_range / damping_scale_range = (0.8, 1.2)` is still the right DR width given the measured τ-spread across steps.
3. **Sanity tests** under `iris_ma6/tests/` following the project test convention:
   - `test_zoom_curve.py`: `z_eff(1.0) == 1.0`; `z_eff(5.0) ≈ 2.86`; clamping above 5x.
   - `test_camera_intrinsics_dr.py`: sampled focal lengths fall within measured per-zoom 1σ × N envelope.
   - `test_detection_latency_dr.py`: sampled latencies match measured mean/std within tolerance over N samples.
   - `test_gimbal_step.py`: at small step amplitude (≤2°) the simulated joint reaches 63% of commanded rate within ≈ 0.10 s ± tolerance.

### Scope boundary

- DO: Replace placeholders with fits derived from the listed artifacts.
- DO: Keep raw measurement files outside the package tree (they already live under `mas/datasets/` and `mas/src/scripts/`); sim ingests fitted summaries only.
- DO: Document each replacement in `iris_ma6/domain_randomization/doc/README.md` with a one-line provenance pointer to the source artifact.
- DO NOT: Re-collect or re-process raw images / bag traces. If a fit looks bad, file a follow-up ticket against mas/028 or mas/026.
- DO NOT: Expand zoom support past 5x. Operationally we trust only up to 5x and 6x calibration is unreliable.
- DO NOT: Change the policy interface or action space. This ticket is sim-side parameter and dynamics fidelity only.
- DEFERRED: Large-step gimbal nonlinearity (τ growing with amplitude); IMU/GPS latency model from C1; datalink dropout from C5. Those land in their own follow-ups.

### Affected files

- EDIT: [domain_randomization_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/domain_randomization_cfg.py)
- EDIT: [iris_ma_env6_test_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg.py) and [iris_ma_env6_v1_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_v1_cfg.py)
- EDIT: [iris_ma_env6_test.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test.py) (zoom curve sites)
- EDIT: [domain_randomization/camera_processor.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/camera_processor.py) (apply `z_eff`)
- EDIT (or REPLACE): [iris_gimbal3.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py) joint stiffness/damping if approach (a) is chosen; otherwise leave joint as-is and add rate-loop wrapper module under iris_ma6.
- NEW: `src/scripts/sim2real_model_fitting/` with the two aggregator scripts above.
- NEW: tests under [iris_ma6/tests/](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/) per project test convention.
- DOC: provenance lines in [iris_ma6/domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md).

### Acceptance criteria

- `focal_length_range` (or zoom-conditional equivalent) traces back to per-zoom mrcal `fx ± σ` for 1x–5x; 6x explicitly excluded with documented reason.
- Zoom curve `z_eff(cmd)` applied wherever `self.zoom_level` currently multiplies focal length; cmd clamped to [1, 5]; unit test passes.
- `ego_detection_latency_mean/std` matches the fit from phase7 CSVs (single distribution or curriculum-selected regime; choice documented).
- Gimbal small-step time constant in sim is within 20% of measured `tau_rate_s ≈ 0.10 s` for both yaw and pitch axes.
- New sanity tests pass on both CPU and GPU per project test convention.
- DR README documents the source artifact for every changed default.

### Decisions to lock in early (flag in PR description)

1. Zoom-conditional intrinsics vs single union range — recommended: zoom-conditional, since fx jumps from 1053 (1x) to 3139 (5x) and a single uniform DR over that span loses signal.
2. Single latency Gaussian vs three-regime curriculum — recommended: three-regime, exposed as a curriculum parameter; the YOLO-model selection is a real deployment choice the policy may need to be robust to.
3. Gimbal model: equivalent joint PD vs rate-loop wrapper — recommended: rate-loop wrapper, because the real controller is rate-loop and DR-ing joint stiffness will not reproduce the saturation nonlinearity that mas/026 documented.

**Reference**: [checklist.md](../021-sim2real-measurement-checklist/checklist.md) §D1, §D2, §C2; mas/028 (intrinsics); mas/026 (gimbal calibration).

**Flow**: Medium (multi-site implementation, but every input is already fit and on disk).
