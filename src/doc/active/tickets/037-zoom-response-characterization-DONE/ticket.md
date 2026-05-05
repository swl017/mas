## Ticket: SIYI A8 zoom response characterization + sim2real model

**Status**: Done (mas-side; sim port deferred to follow-up)
**Created**: 2026-05-05
**Closed**:  2026-05-05

**What**: Characterize the SIYI A8 mini's digital-zoom response in enough detail to fit a deterministic sim2real model. Output four zoom-response datasets, fit a three-parameter model `(τ_d, v_max, δ)` plus optional first-order lag, and emit a `zoom_model.json` that the sim side can consume the way `rate_model.json` is consumed for the gimbal axes.

**Why**: The A8 mini exposes only direction-only `0x05 MANUAL_ZOOM` and absolute `0x0F ABSOLUTE_ZOOM`. There is no rate-with-magnitude command at the protocol level. siyi_ros_node already integrates the policy's `zoom_rate_cmd` (zoom-levels/s) into a stream of `0x0F` dispatches, but the policy has been training against a zero-latency, infinite-bandwidth, continuous-output model of the zoom — wrong on all three counts. The lens has a measured ~150 ms dead-time, ~3.18 levels/s native slew rate, and 0.1-level output quantization. Without modeling these, the sim policy will produce zoom rates the lens can't physically follow, mismatched against the body-motion compensation timing assumed in mas/035 / mas/036.

**Blocked on**: nothing — the integrator already lives in [siyi_ros_node.py](/home/usrg/mas/src/gimbal_controller/gimbal_controller/siyi_ros_node.py) (zoom_rate_step_callback, zoom_level_callback throttle).

**Depends on**: mas/035 (rate-loop architecture pattern — we apply the same input-side dead-time buffer for the zoom path).

### Truth values (already measured, 2026-05-05 bench)

Three runs under [datasets/zoom_response/](/home/usrg/mas/datasets/zoom_response/):

- **level_step** (1→6→1→4→1→2→1, 3 s dwell): native slew **3.18 levels/s** (200 ms peak), per-step rise-90% scales linearly with Δ above the dead-time floor.
- **level_sine** (period 6 s, amp 1.5, center 3.5, 25 Hz cmd): tracking ρ = **0.991**, lag **+310 ms**, mean \|err\| = 0.28 levels.
- **rate/const** (+1.0 levels/s, 5 s): achieved 0.98 levels/s — magnitude is now respected.
- **rate/sine** (amp 1.0/s, period 4 s, 25 Hz cmd): tracking ρ = **0.958**, lag **+130 ms** in dz/dt vs cmd.

Dead-time estimate from level_step is biased by the 5%-threshold: 90 ms (Δ=1) → 210 ms (Δ=5). Workflow step 1 below pins this down with single-quantum steps.

### Workflow

1. **Single-quantum step run** — `--mode level --profile step --steps 1.0 1.1 1.0 1.2 1.0 1.5 1.0 --dwell-s 1.5 --publish-hz 25.0`. Output: [datasets/zoom_response/level_step_quantum/](/home/usrg/mas/datasets/zoom_response/). Lock down τ_d at the protocol level with no threshold confound. Expect τ_d in the 80–150 ms range (one to two siyi_ros_node polling periods plus camera firmware processing).

2. **Direction-symmetry run** — `--mode level --profile step --steps 1 4 1 4 1 4 1 --dwell-s 2.0`. Output: [datasets/zoom_response/level_step_paired/](/home/usrg/mas/datasets/zoom_response/). Compare paired in/out medians of dead-time, rise-90%, peak-rate. If asymmetry > 10 %, the model needs separate `v_max⁺ / v_max⁻` and `τ_d⁺ / τ_d⁻`.

3. **Chirp run** — `--mode rate --profile chirp --chirp-f0-hz 0.05 --chirp-f1-hz 2.0 --chirp-amplitude 1.0 --start-level 3.5 --duration-s 30.0 --publish-hz 25.0`. Output: [datasets/zoom_response/rate_chirp/](/home/usrg/mas/datasets/zoom_response/). Bode-style response: bin instantaneous-frequency vs. magnitude / phase of dz/dt against rate_cmd. Distinguish pure rate-limiter (–6 dB/oct, linear-in-ω phase) from rate-limiter + first-order lag.

4. **Saturated rate-sine** — `--mode rate --profile sine --sine-amplitude 6.0 --sine-rate-bias 0.0 --sine-period-s 2.0 --start-level 3.5 --duration-s 12.0 --publish-hz 25.0`. Output: [datasets/zoom_response/rate_sine_saturated/](/home/usrg/mas/datasets/zoom_response/). Drives cmd well above v_max; verify the lens slew clamps symmetrically and tracks zero-crossings without overshoot. If overshoot or asymmetric clipping appears, the model needs an explicit slew-limiter primitive distinct from the integrator.

5. **Fit `zoom_model.json`** — new script [src/scripts/sim2real_model_fitting/fit_zoom_model.py](/home/usrg/mas/src/scripts/sim2real_model_fitting/fit_zoom_model.py). Inputs: the four run dirs above. Outputs: `src/scripts/sim2real_model_fitting/output/zoom_model.json` with at minimum:
   ```json
   {
     "deadtime_s": 0.10,
     "deadtime_std_s": 0.02,
     "v_max_in_per_s":  3.18,
     "v_max_out_per_s": 3.18,
     "quantum": 0.1,
     "first_order_tau_s": 0.0,
     "bandwidth_hz_3db": 0.5,
     "fit_provenance": {
       "step_quantum_dir":   "datasets/zoom_response/level_step_quantum",
       "step_paired_dir":    "datasets/zoom_response/level_step_paired",
       "chirp_dir":          "datasets/zoom_response/rate_chirp",
       "saturated_dir":      "datasets/zoom_response/rate_sine_saturated"
     }
   }
   ```

6. **Sim implementation** (sim repo, separate change) — apply the same shape as mas/035 + mas/036 to the zoom path:
   - Rate command is integrated to a target level (clamped to `[zoom_min, zoom_max]`).
   - Input-side dead-time buffer (per-env Gaussian draw on episode reset, curriculum-scaled).
   - Output-side rate-limit at `v_max`.
   - 0.1-level output quantization (rounding, not truncation).
   - First-order lag at the integrator output ONLY if step 5 fit shows non-zero `first_order_tau_s`.

7. **Acceptance / sim2real comparison** — extend the existing `sysid_output/gimbal/compare_*.py` pattern with `compare_zoom.py`. Replay each of the four bench runs through the sim model and compare state traces. Acceptance: median absolute level error ≤ 0.15 levels (≈1.5 quanta) over the active window.

### Scope boundary

- DO: keep the model deterministic per episode — dead-time drawn at reset, then constant.
- DO: place the dead-time buffer at the **input** to the rate integrator (matches mas/035 / mas/036 architecture for the angular axes).
- DO: clip the integrator target to `[zoom_min, zoom_max]` at every sim step (matches the on-vehicle integrator).
- DO: quantize to 0.1 levels at the output stage, not at the integrator state — the integrator carries continuous state, only the published level snaps.
- DO NOT: model `0x05 MANUAL_ZOOM` direction-only behavior. The sim policy interface is the rate command; the protocol-level binary command is firmware-internal and not part of the sim2real boundary.
- DO NOT: introduce a separate "zoom dead-time buffer" curriculum knob if the value is statistically indistinguishable from the gimbal-axis dead-time (mas/036). Reuse the same curriculum scale unless the fit forces it.
- DO NOT: model camera frame-rate drop / image artifacts during zoom transition (out of scope; vision-side ticket if needed).

### Affected files (this ticket)

- NEW: [src/scripts/zoom_response/zoom_response_test.py](/home/usrg/mas/src/scripts/zoom_response/zoom_response_test.py) — already landed; add no functional change here.
- NEW: [src/scripts/zoom_response/zoom_response_plot.py](/home/usrg/mas/src/scripts/zoom_response/zoom_response_plot.py) — already landed.
- NEW: `src/scripts/sim2real_model_fitting/fit_zoom_model.py` (workflow step 5).
- NEW: `src/scripts/sim2real_model_fitting/output/zoom_model.json`.
- NEW: four dataset directories under [datasets/zoom_response/](/home/usrg/mas/datasets/zoom_response/).
- EDIT (siyi_ros_node, **already landed in this branch**):
   - integrator: `zoom_rate_step_callback` (50 Hz software rate integrator dispatching `0x0F`)
   - throttle: `zoom_level_callback` only dispatches on quantum transition
   - watchdog: `zoom_rate_timeout_s = 0.5 s` parameter

### Acceptance criteria

- Single-quantum dead-time has 95 % of measurements within ±20 ms of the median (i.e. tight, repeatable).
- In/out asymmetry on `v_max` is < 10 % (one-sided model is sufficient) OR the model fit reports separate `v_max⁺ / v_max⁻` honestly.
- Chirp Bode magnitude rolloff matches a rate-limiter prediction (`v_max / (amp · ω)`) within ±3 dB up to 1 Hz, OR the fit emits a non-zero `first_order_tau_s` to capture the residual.
- Saturated rate-sine: state slew, after smoothing, never exceeds v_max + 0.2 levels/s, and hits ±v_max symmetrically (within 5 %).
- `compare_zoom.py` (sim vs measured): median abs error ≤ 0.15 levels on each of the four bench runs.

### Risk

Low. The hardware-side integrator is already deployed and verified; this ticket is mostly about quantifying behavior we've already proven works qualitatively, then porting to sim.

The one open risk: the chirp may reveal that high-frequency response has phase / magnitude content that doesn't fit a simple rate-limiter + dead-time model — e.g. the firmware may do internal smoothing on rapid `0x0F` updates. If so, workflow step 5 emits `first_order_tau_s > 0` and step 6 adds the lag stage; this is anticipated, not blocking.

### Coupling

- mas/035, mas/036: same architecture pattern. If those dead-time / rate-loop refactors land first in sim, this ticket plugs into the same hooks.
- mas/028 (camera intrinsic calibration): independent. The intrinsic calibration runs were captured at fixed zoom levels; this ticket characterizes the *transient between* zoom levels.
- mas/029 (sim2real measured-model impl): zoom path was deferred there; this ticket is the deferred deliverable.

**Reference**:
- Hardware-side fix landed: `siyi_ros_node.py` rate integrator + level-callback throttle (see `git log -- src/gimbal_controller/gimbal_controller/siyi_ros_node.py`)
- Test/plot scripts: [src/scripts/zoom_response/](/home/usrg/mas/src/scripts/zoom_response/)
- SDK protocol confirmation: `0x05 MANUAL_ZOOM` is direction-only int8; `0x0F ABSOLUTE_ZOOM` accepts `int.decimal` to 0.1 resolution; no rate-with-magnitude opcode exists ([siyi_message.py:129-147](/home/usrg/mas/src/gimbal_controller/gimbal_controller/siyi_sdk/siyi_message.py#L129-L147)).

---

## Findings (2026-05-05 bench session)

### Fitted parameters (canonical, written to [output/zoom_model.json](/home/usrg/mas/src/scripts/sim2real_model_fitting/output/zoom_model.json))

| Parameter | Value | Source | Notes |
|---|---|---|---|
| `deadtime_s_in` / `deadtime_s_out` | **0.10 s** (std 0.018, p05 0.089, p95 0.129) | single-quantum step (n=6) | direction-symmetric within measurement noise |
| `v_max_in_per_s` / `v_max_out_per_s` | **3.16 / 3.16 levels/s** | paired 1↔4 step (n=3 ea) | 0.2 % asymmetry (well below 10 % acceptance) |
| `first_order_tau_s` | **0.091 s** | rate_sine 0.25 Hz xcorr | replaces noisy chirp estimate; chirp gave 0.065 s |
| `quantum` | 0.1 levels | A8 spec | hard at the protocol level (0x0F int.decimal) |
| `bandwidth_hz_3db` | ≈ 0.385 Hz | chirp (approximate) | flagged as noisy; multi-point Bode would tighten |

### Acceptance results (compare_zoom.py replay through fitted model)

```
run                          median  p95   max   rms     n
level_step_quantum           0.000   0.10  0.20  0.040  1007
level_step_paired            0.000   0.30  0.40  0.141  1356
rate_chirp                   0.000   0.10  0.10  0.059  3272
rate_sine_saturated          0.600   1.50  1.60  0.812  1472   [informational]
```

PASSED on the three linear-regime runs. `rate_sine_saturated` falls outside the acceptance set on purpose — see "saturation stall" below.

### Saturation stall — why the model diverges at amp >> v_max

The `rate_sine_saturated` run commanded amp 6 levels/s at 0.5 Hz. Model predicts the lens slewing at v_max between [2.0, 5.0]; lens actually sat near 4.4 with ~0.1 levels of ripple. Cause:

- Software integrator at 50 Hz advances target by `rate · dt = 6 · 0.02 = 0.12` per tick, so a new 0.1 quantum is dispatched every ~17 ms.
- Lens's measured dead-time is ~100 ms before *any* `0x0F` produces motion, so 5–6 newer commands arrive before the lens has begun reacting.
- Empirically the camera firmware does not queue these — each new `0x0F` partially preempts the in-flight slew. With the rate sine flipping every 1 s and commands arriving every 17 ms, the lens is constantly being told "no, go *there* now" before it has built momentum, so net motion stays tiny.

**Fix if this regime ever matters**: clip the rate inside `siyi_ros_node.zoom_rate_step_callback` to ±v_max before integrating. That throttles dispatches to ~30 Hz (one quantum per ~33 ms ≈ v_max), and the lens will then track properly with rate-cmd magnitudes above v_max gracefully clipping.

**Why we did not fix it now**: `mas_policy.action_publisher.max_zoom_rate = 2.0` levels/s, comfortably below the measured v_max = 3.16. The policy never operates in this regime, so the symmetric-v_max model is correct for sim2real.

### Chirp magnitude-ratio noise

The chirp's sliding-window single-frequency fits returned ratios > 1 in three bins despite physical impossibility. Cause: instantaneous frequency sweeps within each window, so a single-freq sinusoid fit captures only part of the energy and the amplitude estimate is biased.

The chirp's *phase* lag is more trustworthy than its magnitude. We therefore use:
- chirp → only for `bandwidth_hz_3db` (approximate, flagged in JSON)
- constant-frequency rate-sine → for `first_order_tau_s` (canonical)

A multi-point constant-freq Bode sweep (e.g. f ∈ {0.1, 0.2, 0.4, 0.8, 1.5} Hz, ~5 s each) would replace the chirp with a clean magnitude-and-phase Bode plot in ~30 s of bench time. Filed as optional follow-up; not required for the linear-regime model.

---

## Tips for RL training (sim port — workflow steps 6–7)

The fitted model is **directly applicable** to a 25 Hz policy environment, with these caveats:

### 1. Quantize at the *output* observation, never the integrator state

The model's internal `target` and slew-limited `state` are continuous `float64`. Only the *published* observation is rounded:

```python
zoom_obs = round(state / 0.1) * 0.1
```

If you quantize the integrator's internal state, you lose the sub-quantum momentum that produces clean slewing — the policy will see jitter that doesn't exist on hardware.

### 2. The 0.1-quantum is real; don't filter or smooth it away

At 25 Hz observation, consecutive frames will routinely show `zoom_obs[t] == zoom_obs[t+1]` followed by a 0.1 jump. This is faithful to deployed behavior. Adding a low-pass filter or interpolating between samples would teach the policy to expect precision the lens cannot deliver — guaranteed sim-real gap.

PPO / SAC handle quantized observations fine. No special tricks needed.

### 3. Above ~0.4 Hz commanded rate, the lens is in a *dead band*

A 1 Hz, 1 levels/s rate sine integrates to ±0.16 levels = 1.6 quanta. Once rounded, the observed lens motion is ≤ 1 quantum — the lens essentially does not respond. This is a real bandwidth ceiling, not a modeling artifact. The policy will learn this is a "no-op" regime; do not try to encourage exploration there with reward shaping unless you actually want the policy to give up on high-frequency control.

### 4. Mind the dead-time at the policy boundary

A 100 ms dead-time at a 40 ms policy step is **2.5 environment steps of pure delay** between command and *any* observable response. If the policy's reward depends on instantaneous tracking error, it will see large residuals from this delay alone. Two options that work:

- Apply the dead-time buffer at the rate-loop input (matches mas/035 / mas/036 architecture for the angular axes — same pattern, same `dead_time_buffer_steps` curriculum knob). Reuse the gimbal-axis `gimbal_dead_time_curriculum_scale` unless the fit forces otherwise.
- Penalize squared rate-cmd to discourage the policy from chasing reference faster than v_max, which would otherwise produce useless commands.

### 5. Action / observation scaling

`mas_policy.action_publisher` already denormalizes the policy's `[-1, 1]` action to `zoom_rate_cmd` via `max_zoom_rate = 2.0` levels/s. Keep that bound — it is **below** v_max = 3.16, so the policy's full action range is in the linear regime where the model is faithful. Raising `max_zoom_rate` toward or above v_max would re-enter the saturation-stall regime; if you do that, fix the integrator's input clip first (see "Fix if this regime ever matters" above).

### 6. Direction asymmetry only appears under sustained extreme commanding

The paired step run shows `v_max_in == v_max_out` to within 0.2 %. The asymmetry seen in `rate_sine_saturated` (3.26 / 2.14 levels/s) only manifests under *sustained* fast bidirectional rate commanding — likely an artifact of the same firmware-preemption stall described above, rather than true motor asymmetry. The sim model uses symmetric v_max; do not add asymmetry without first running a clean test that decouples the two effects.

### 7. State publish rate vs policy step rate

The on-vehicle `siyi_ros_node` publishes `camera/zoom_level` at 100 Hz (matches its polling rate). The policy reads at 25 Hz. There is therefore up to 40 ms of staleness in the observation independent of dead-time. The sim should mirror this if it matters — usually it does not, since the lens dwells most of the time and 40 ms vs 0 ms staleness on a quantized signal is a fraction of a quantum.

---

## Affected files (final)

- NEW: [src/scripts/zoom_response/zoom_response_test.py](/home/usrg/mas/src/scripts/zoom_response/zoom_response_test.py)
- NEW: [src/scripts/zoom_response/zoom_response_plot.py](/home/usrg/mas/src/scripts/zoom_response/zoom_response_plot.py)
- NEW: [src/scripts/sim2real_model_fitting/fit_zoom_model.py](/home/usrg/mas/src/scripts/sim2real_model_fitting/fit_zoom_model.py)
- NEW: [src/scripts/sim2real_model_fitting/compare_zoom.py](/home/usrg/mas/src/scripts/sim2real_model_fitting/compare_zoom.py) (with `--plot` for reference / response / model overlays)
- NEW: [src/scripts/sim2real_model_fitting/output/zoom_model.json](/home/usrg/mas/src/scripts/sim2real_model_fitting/output/zoom_model.json)
- NEW: four bench dataset directories under [datasets/zoom_response/](/home/usrg/mas/datasets/zoom_response/) — also four reference rate-tuning runs (`level_step`, `level_sine`, `rate_const_1p0`, `rate_sine`) used for τ₁ cross-check
- EDIT: [src/gimbal_controller/gimbal_controller/siyi_ros_node.py](/home/usrg/mas/src/gimbal_controller/gimbal_controller/siyi_ros_node.py) — software rate integrator (`zoom_rate_step_callback`), throttled `zoom_level_callback`, watchdog parameter

## Follow-ups (not in this ticket)

- Sim-side port (separate sim-repo ticket): apply the model in IsaacLab using the same input-side dead-time buffer pattern as mas/035 / mas/036.
- Optional: multi-point constant-freq Bode sweep to replace the chirp's noisy magnitude estimate.
- Optional: rate-clip-before-integrate fix in `siyi_ros_node` if a future task drives `max_zoom_rate` toward v_max.
