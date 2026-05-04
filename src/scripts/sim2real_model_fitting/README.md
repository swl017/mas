# sim2real_model_fitting

Aggregator scripts that turn measurement artifacts into compact JSONs that the
iris_ma6 sim consumes as DR defaults. Implementation backing for ticket
`mas/029-sim2real-measured-model-impl`.

These scripts do not collect new data. They only read existing measurement
artifacts and emit canonicalized fits under `output/`.

## Inputs (read-only)

| Source | Producer ticket |
|---|---|
| `phase7_low_latency.csv`, `phase7_mid_latency.csv`, `phase7_high_latency.csv` (under `src/scripts/`) | mas/021 §C2 / §C4 |
| `datasets/camera_calibration/2026-04-17/{1x,2x,4x,5x,6x}/intrinsics_summary.json` | mas/028 |
| `src/scripts/camera_calibration/zoom_curve.json` | mas/028 (derived) |
| `src/gimbal_controller/scripts/gimbal_rate_step_followspeed_tune/rate_model.json` | mas/026 |

## Outputs (consumed by sim)

| Script | Output JSON | iris_ma6 site |
|---|---|---|
| `fit_detection_latency.py` | `output/detection_latency_fit.json` | `iris_ma_env6_*_cfg.py` `ego_detection_latency_mean/std` |
| `summarize_intrinsics_for_sim.py` | `output/intrinsics_for_sim.json` | `domain_randomization_cfg.py` `CameraRandomizationCfg.focal_length_range` |

The zoom curve and gimbal rate model are already in the right form; sim
ingests them directly without re-fitting.

## Run

```bash
cd /home/usrg/mas
python3 src/scripts/sim2real_model_fitting/fit_detection_latency.py
python3 src/scripts/sim2real_model_fitting/summarize_intrinsics_for_sim.py
```

Both scripts are pure Python stdlib (no torch / no Isaac Sim) so they can
run on any host that can read the input artifacts.
