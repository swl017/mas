## Ticket: Latency-specialized policy variants (low / mid / high YOLO)

**Status**: Open
**Created**: 2026-04-28 (rewritten from "three-regime curriculum" scope; original Stage R showed the per-env-distribution refactor was disproportionate to the deployment reality, which is single-model)

**What**: Ship three iris_ma6 env-cfg variants — `_low`, `_mid` (= today's mas/029 baseline), `_high` — that differ from each other only in `delay_system_params.ego_detection_latency_mean / std`. Each variant trains a separate policy specialized to one YOLO model class. Deployment selects the policy matching the loaded YOLO model.

**Why**: At deployment time the YOLO model is fixed (one set of weights is loaded; latency does not vary per-frame across regimes). Training a single policy that averages over a multi-modal regime mixture pays a robustness tax the operating point never asks it to pay. Specializing each policy to its operating regime is both simpler to implement (config-only, no sampler refactor) and matches the actual operating constraint. Stage R confirmed the refactor cost: `delay_system_v3` has no per-env distribution-parameter support anywhere in the sampler stack, and `set_mode` resamples every step regardless of declared per-episode frequency.

**Blocked on**: Nothing.

**Depends on**: mas/029 (provides the per-regime fits, the +245 ms transit constant, and the existing `iris_ma_env6_test_cfg.py` that becomes the `_mid` baseline).

### Truth values (already fitted)

Glass-to-topic = upstream A8/RTSP transit (mas/031, ~245 ms) + YOLO inference (phase7). Stds add in quadrature (~17 ms transit + ~13 ms inference ≈ ~21 ms total).

| Variant | YOLO size | mean (s) | std (s) | Source |
|---|---|---:|---:|---|
| low | small (e.g. yolo11n) | 0.296 | 0.022 | 245 + 51 ms / sqrt(17² + 14²) |
| mid | medium (e.g. yolo11m) | 0.310 | 0.021 | 245 + 65 ms / sqrt(17² + 13²) |
| high | large (e.g. yolo11l) | 0.386 | 0.021 | 245 + 141 ms / sqrt(17² + 13²) |

Aggregator: [src/scripts/sim2real_model_fitting/output/detection_latency_fit.json](/home/usrg/mas/src/scripts/sim2real_model_fitting/output/detection_latency_fit.json).

### Workflow

1. Add three env-cfg variants under [iris_ma6/](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/):
   - `iris_ma_env6_test_cfg_low.py` — subclass of `IrisMA6TestEnvCfg` overriding `ego_detection_latency_mean=0.296`, `ego_detection_latency_std=0.022`.
   - Update existing `iris_ma_env6_test_cfg.py` mid baseline from `(0.31, 0.03)` → `(0.310, 0.021)` to align with the quadrature-sum truth value (the `0.03` shipped in mas/029 was a slightly conservative round-up; per Q4 of mas/034 the bit-exact regression was waived).
   - `iris_ma_env6_test_cfg_high.py` — subclass overriding to `(0.386, 0.021)`.
2. Register three new gym IDs in [iris_ma6/__init__.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/__init__.py):
   - `Isaac-Iris-MA6-Direct-Test-Low-v0`
   - `Isaac-Iris-MA6-Direct-Test-Mid-v0` (alias to the existing `Isaac-Iris-MA6-Direct-Test-v0` pointing at the same cfg, OR a new ID that re-uses the unchanged mid cfg — pick one in Stage I).
   - `Isaac-Iris-MA6-Direct-Test-High-v0`
3. Update the existing `test_measured_model_defaults.py` latency assertion to match the new mid value `(0.310, 0.021)`.
4. Add a new [tests/test_latency_variants.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/test_latency_variants.py) that instantiates each cfg, asserts `(mean, std)` matches the table above, and confirms all other delay-system params are unchanged from the mid baseline.
5. Document the three policies and their deployment mapping in [domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md) under the existing measured-model provenance section.

### Scope boundary

- DO: Three thin cfg subclasses differing only in two fields each.
- DO: Update the mid baseline to the quadrature-sum value `(0.310, 0.021)`. mas/029 ticket explicitly waived bit-exact regression.
- DO: Register one gym ID per variant.
- DO NOT: Add a regime sampler. Per-env latency variation is not implemented.
- DO NOT: Refactor `DistributionSampler`, `LatencySampler`, or `DelayPipelineV3`.
- DO NOT: Touch the curriculum manager. `set_delay_mode` continues to flow as today.
- DO NOT: Add the +245 ms transit as a separate field. It's baked into the regime mean per Q1.

### Affected files

- NEW: [iris_ma_env6_test_cfg_low.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg_low.py)
- NEW: [iris_ma_env6_test_cfg_high.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg_high.py)
- EDIT: [iris_ma_env6_test_cfg.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/iris_ma_env6_test_cfg.py) — `(0.31, 0.03)` → `(0.310, 0.021)`
- EDIT: [iris_ma6/__init__.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/__init__.py) — three new `gym.register` blocks
- EDIT: [tests/test_measured_model_defaults.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/test_measured_model_defaults.py) — assertion swap
- NEW: [tests/test_latency_variants.py](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/tests/test_latency_variants.py)
- DOC: [domain_randomization/doc/README.md](/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/domain_randomization/doc/README.md)

### Acceptance criteria

- All three cfgs instantiate without error and report the table values exactly.
- Each gym ID is discoverable via `gym.spec(...)`.
- `test_measured_model_defaults.py` and `test_latency_variants.py` both pass on CPU and GPU.
- `iris_ma_env6_v1.py` and any other env file outside the test variant remain untouched.
- Deployment-side documentation lists which gym ID corresponds to which YOLO model class.

**Reference**: mas/029 ticket.md "Deferred to follow-ups" entry, mas/031 (transit constant), mas/021 §C2/§C4 (regime measurements).

**Flow**: Light (config + register + tests; no architectural change).
