# Degradation Layer — Change Summary

## What was added and why

Two new test variables are added to the AEB harness:

1. **Perception latency** (`delay_frames`) — previously existed in the config but was hardcoded to 0 everywhere, and the old `det_buffer` deque delayed only the `detected` boolean. The new `PerceptionDegrader` delays the full `Perception` object (distance, rel_speed, lead_speed, ttc, detected) by `delay_frames` ticks — a genuine end-to-end perception latency.

2. **Sensor noise** (`noise_sigma_m`, `noise_sigma_vr`, `dropout_p`) — not present at all before. Gaussian noise is added to distance and velocity fields; dropout simulates sensor outage frames. These variables test how robust each controller is to realistic sensor degradation without writing any new controller logic.

**Research motivation:** Moving beyond a single operating point (μ, speed) expands the test domain and lets the paper characterise controller robustness as a function of sensor quality — a common reviewer question for simulation-based AEB evaluations.

---

## New files

| File | Purpose |
|---|---|
| `perception/degrade.py` | `PerceptionDegrader` class — all degradation logic in one place |
| `tests/test_degrade.py` | Unit tests: no-op, delay correctness, determinism, dropout modes (no CARLA) |
| `CHANGES_degradation.md` | This file |

---

## Modified files (minimal footprint)

| File | Change summary | ~Lines changed |
|---|---|---|
| `core/metrics.py` | 6 new fields appended to `RunRecord` (noise_sigma_m, noise_sigma_vr, dropout_p, dropout_mode, test_mode, seed) | +6 |
| `core/runner_lead_brake.py` | 6 new fields on `LeadBrakeRecord`; import PerceptionDegrader; create + reset degrader after controller.reset(); det_buffer maxlen=1; call `degrader.apply` before controller.decide | +23 |
| `core/runner.py` | Same changes as runner_lead_brake.py | +22 |
| `config/scenario_cutin.py` | `import os`; `CONTROLLERS` list; 4 sweep-value constants; `build_matrix_runs(mode)` function; `MATRIX_RUNS` now set dynamically from env var | +40 |
| `config/scenario_lead_brake.py` | Same as scenario_cutin.py | +39 |
| `run_matrix.py` | Read `TEST_MODE` env var; call `cfg.build_matrix_runs(test_mode)`; pass `run_spec`, `case_idx`, `test_mode` to `run_case` | +6 |
| `run_matrix_lead.py` | Same as run_matrix.py | +6 |
| `README.md` | Added "Test Modes" section | +28 |
| `TECHNICAL_DOC.md` | Updated Sections 4 (pipeline), 7 (CSV schema), 9 (latency mechanism), 12 (revision history) | +35 |

**Controller files not touched:** `baseline_static_ttc.py`, `proposed_dynamic_ttc.py`, `proposed_enhanced.py`.

---

## Run commands (for server)

```bash
# Original — paper results, no degradation
python run_matrix.py
LEAD_DECEL=6.0 python run_matrix_lead.py

# Latency sweep: delay_frames ∈ {0, 4, 8, 16} × 3 controllers = 12 run-specs
TEST_MODE=latency python run_matrix.py
TEST_MODE=latency LEAD_DECEL=6.0 python run_matrix_lead.py

# Noise sweep: noise_sigma_m ∈ {0.0, 0.5, 1.0, 2.0} m × 3 controllers = 12 run-specs
TEST_MODE=noise python run_matrix.py
TEST_MODE=noise LEAD_DECEL=6.0 python run_matrix_lead.py
```

Sweep values are constants at the top of each config file (`LATENCY_DELAY_FRAMES`, `NOISE_SIGMA_M_SWEEP`, etc.) — change them freely without touching the function.

---

## How to verify TEST_MODE=original gives unchanged results

`TEST_MODE=original` (or unset) calls `build_matrix_runs("original")` which returns:
```python
[
  dict(label="baseline",          controller="baseline",          delay_frames=0,
       noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze"),
  dict(label="proposed",          controller="proposed",          delay_frames=0, ...),
  dict(label="proposed_enhanced", controller="proposed_enhanced", delay_frames=0, ...),
]
```

In `run_case`, `PerceptionDegrader(delay_frames=0, noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0)` is created. Inside `apply()`:
- Latency buffer has `maxlen=1`; `buffer[0]` is always the frame just appended → no delay.
- Noise condition `noise_sigma_m > 0.0` is False → no mutation.
- Dropout condition `dropout_p > 0.0` is False → no dropout.
- Returns `copy.copy(perc)` with identical field values.

Behaviour is therefore bitwise-equivalent to the original code, modulo a shallow copy of `Perception` which has no observable effect. The seed recorded in CSV is derived deterministically from the run label; it does not affect any output when all params are 0.

---

## What was NOT touched

- Decision logic of `baseline_static_ttc`, `proposed_dynamic_ttc`, `proposed_enhanced`
- MFDD formula, kinematic brake cap, collision detection, scenario logic
- Existing CSV columns (new columns appended only)
- Files in `results/` (read-only)
- `FRAME_SYNC`, `FIXED_DT`, `DETECTION_SOURCE`, `BRAKE_MODEL`, `GRAVITY`
- `run_single.py`, `run_single_lead.py` (single-run scripts unchanged; `delay_frames` defaults to 0)
