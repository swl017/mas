## Ticket 042 — PN engagement experiment suite packaging (`mas_pn_guidance/experiments/`)

**Status**: Delivered (additive-only; staged items open)
**Created**: 2026-07-13

**What**: Package the ego-only PN engagement sweep workflow (crossing /
tail-chase capability grids, tickets 004/007/008/010) into a self-contained
operator suite under `mas_pn_guidance/experiments/`:

```text
experiments/{EXPERIMENT.md, profiles/, manifests/, run_sweep.py, analysis/}
```

- `profiles/` — tracked copies of the frozen EKF configs (OLD/TUNED/INTER, from
  ticket-007 `results/launch_configs/`) and PN profiles (`pn_N2/pn_N3.yaml`),
  plus `install_profile.sh` which applies one to the live stack (copy to
  source AND install space + tmux window restart + verify) — codifying the
  REPRODUCE §2 procedure that previously lived as operator lore.
- `manifests/` — YAML sweep definitions (`template.yaml` schema;
  `ticket010_n2_old_sweep.yaml` reproduction manifest).
- `run_sweep.py` — manifest-driven batch runner: fresh-boot-id enforcement,
  readiness wait (conductor gone + interceptor armed at IC), 1-trial oracle
  health gate, settle_error-streak watchdog (kills the stranded-chase cascade,
  ticket-010 N2CT1 incident), grounded/disarmed detection with the
  High-Accelerometer-Bias lockout explained, CSV/JSONL archiving with sha256
  provenance of the installed configs, per-boot realized-dynamics QA.
- `analysis/` — canonical CLIs, generalized from the per-ticket copies:
  `qa_target_tracking.py` (was ticket-004 `analyze_target_tracking.py`),
  `plot_boundary_3arm.py` (was ticket-008/010 copies; run-specific text now
  CLI args; no longer defaults to the paper-figure path), `boot_table.py`.

**Why**: Every operational incident in the ticket-010 sweep session traced to
tribal knowledge: the install-by-copy config trap (untracked
`engagement_ekf.launch.py`, `pn_guidance.yaml` read once at node init), manual
IC/readiness eyeballing, and the unhandled stranded-chase cascade that burned
11 trials and ended in a preflight-locked disarmed vehicle. Items 1–2 of the
gap analysis (profiles as tracked files + a batch runner with gates and a
watchdog) would have prevented all of them. The suite also fixes figure-path
safety: the canonical plotter refuses to silently overwrite cornerstone
figures.

**Deliberately additive**: nothing modifies the running stack until
`install_profile.sh` is invoked; no package sources were edited (ticket-010's
live session must stay valid). Not colcon-installed — operator scripts run
from source.

**Staged follow-ups (open)**:
1. Parameterize `engagement_ekf.launch.py` (tracked, `ekf_profile:=` arg) and
   expose `nav_constant:=` in `pn_guidance.launch.py` so the copy-restart dance
   disappears. Needs a rebuild + node restarts — only between sweep sessions.
2. Conductor-internal stranding abort (detect settle-timeout with grounded
   vehicle, end boot cleanly) — candidate to fold the runner watchdog into the
   conductor itself.
3. Bring-up as code (programmatic armed-and-holding wait for the three tmux
   sessions).
4. Ticket 041's `command_mode` (acceleration setpoint) as a manifest axis once
   it lands.

**Verification**: `bash -n` on the shell script; `py_compile` on the three
Python tools; `run_sweep.py --dry-run` prints the six ticket-010 launch
commands with correct trailing-comma list args; `boot_table.py` and
`plot_boundary_3arm.py` run against the archived ticket-010 data
(42-cell classification reproduced); `install_profile.sh verify` reads the
live install space without modifying it.
