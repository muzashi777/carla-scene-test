# AEB Test Harness — 3DGS + UE5.5 → CARLA

A simulation test harness for Automatic Emergency Braking (AEB) controllers, built for a 3D Gaussian Splatting scene imported into CARLA via UE5.5. Supports two independent test scenarios with three swappable controllers. Results are reported in the ICARCV 2026 paper *"A Real-to-Simulation Workflow for AEB Controller Testing Using 3D Gaussian Splatting in CARLA"*.

**Paper results:** cut-in `Rc_conflict` 44.0% → 72.0%; lead-brake 34.0% → 68.0% (baseline → proposed_enhanced).

---

## Scenarios

| Scenario | Description | Entry point |
|---|---|---|
| **Cut-in / Dart-out** | A dart vehicle launches laterally from the roadside and stops blocking the ego lane. | `run_single.py` / `run_matrix.py` |
| **Lead-brake (CCRb)** | A lead vehicle ahead in the same lane (matching ego speed) brakes suddenly to a stop. Euro-NCAP CCRb. | `run_single_lead.py` / `run_matrix_lead.py` |

Both scenarios share the same session, actors, controllers, YOLO, metrics, and viz infrastructure.

---

## Directory Structure

```
config/
  scenario_cutin.py        ★ All parameters for cut-in (scene, YOLO, μ, matrix, controllers)
  scenario_lead_brake.py   ★ All parameters for lead-brake (same structure, different values)
core/
  carla_session.py         Open/close sync mode, restore original settings
  actors.py                Spawn vehicles, set tire friction μ, attach sensors
  types.py                 Perception / EgoState dataclasses (controller input)
  metrics.py               RunRecord, 5 CPEIM indices, CSV writer, summarize()
  conflict.py              ★ Kinematic is_conflict (pre-sim, no CARLA needed)
  report.py                ★ Read CSV → print CPEIM table (standalone, no CARLA)
  viz.py                   OpenCV overlay (run_single* only)
  scenario_cutin.py        Cut-in scene logic (ego cruise, dart trigger, blocking stop)
  runner.py                Single-case runner for cut-in (returns RunRecord)
  scenario_lead_brake.py   Lead-brake scene logic
  runner_lead_brake.py     Single-case runner for lead-brake (returns LeadBrakeRecord)
control/
  base_controller.py       ★ Plugin interface: throttle/brake/steer (BaseController)
  baseline_static_ttc.py   @register("baseline")          — Static TTC
  proposed_dynamic_ttc.py  @register("proposed")          — Adaptive TTC (speed + μ)
  proposed_enhanced.py     @register("proposed_enhanced") — Required-deceleration
perception/
  yolo_detector.py         YOLOv8n wrapper: is there a vehicle in the ego lane?
  scene_logger.py          Perception log runner (background + with-actor passes)
  percep_viz.py            Visualisation helper for perception logs
tools/
  check_conflict.py        ★ Verify is_conflict for every matrix case (no CARLA needed)
run_single.py              Cut-in: run 1 case with OpenCV display
run_matrix.py              Cut-in: sweep full matrix → results/matrix_*.csv
run_single_lead.py         Lead-brake: run 1 case with display
run_matrix_lead.py         Lead-brake: sweep full matrix → results/lead_matrix_*.csv
run_perception_log.py      Run YOLO perception logging (background + actor passes)
```

---

## Prerequisites

- CARLA 0.9.x with your 3DGS scene loaded (collision mesh from UE5.5, no `.xodr`/waypoints needed)
- Python 3.8+, packages: `carla`, `ultralytics`, `opencv-python`, `numpy`
- `yolov8n.pt` placed in the project root

---

## How to Run

**Step 0 — Verify conflict cases before any matrix run (no CARLA needed):**
```bash
python tools/check_conflict.py
LEAD_DECEL=6.0 python tools/check_conflict.py   # verify with paper deceleration
```

**Scenario 1 — Cut-in / Dart-out:**
```bash
python run_single.py     # debug single case with display; edit SINGLE_* in config/scenario_cutin.py
python run_matrix.py     # sweep 50 cases × 3 controllers → results/matrix_*.csv
```

**Scenario 2 — Lead-brake (CCRb):**
```bash
python run_single_lead.py                    # debug single case with display
LEAD_DECEL=6.0 python run_matrix_lead.py    # Euro-NCAP CCRb §3.4 — value used in paper (Table III/IV)
python run_matrix_lead.py                    # code default LEAD_DECEL=4.0 (moderate braking)
```

**Summarise results from existing CSVs (no CARLA needed):**
```bash
python -m core.report results/matrix_*.csv
python -m core.report results/lead_matrix_*.csv
```

> Lead-brake results are prefixed `lead_matrix_` (set by `RESULTS_PREFIX`), clearly separated from cut-in results (`matrix_`).

---

## Test Modes

A `TEST_MODE` environment variable selects the degradation sweep. The degradation layer (`perception/degrade.py`) sits between the perception pipeline and every controller — all three controllers receive the same degraded input per run, preserving fairness.

| `TEST_MODE` | What it does | Runs (cut-in) |
|---|---|---|
| `original` (default) | All degradation params = 0; identical to original behaviour | 3 |
| `latency` | Sweep `delay_frames` ∈ {0, 4, 8, 16} (0–0.8 s) × 3 controllers | 12 |
| `noise` | Sweep `noise_sigma_m` ∈ {0.0, 0.5, 1.0, 2.0} m × 3 controllers | 12 |
| `latency_comp` | Sweep `delay_frames` × `COMP_CONTROLLERS` with `comp_source=oracle` (L known exactly). Measures how much of the latency-induced Rc loss each compensator recovers — an **upper bound**. | 16 |
| `latency_mismatch` | `enhanced_predictive` at a fixed injected delay (`COMP_MISMATCH_DELAY=8`) while sweeping the *compensated* `comp_L_frames` ∈ {4, 8, 12, 16} (under-/exact-/over-compensate). Measures **fragility** to mis-estimating L. | 4 |

**Run on server:**
```bash
# Original (paper results — no degradation)
python run_matrix.py
LEAD_DECEL=6.0 python run_matrix_lead.py

# Latency sweep
TEST_MODE=latency python run_matrix.py
TEST_MODE=latency LEAD_DECEL=6.0 python run_matrix_lead.py

# Noise sweep
TEST_MODE=noise python run_matrix.py
TEST_MODE=noise LEAD_DECEL=6.0 python run_matrix_lead.py

# Latency compensation — upper bound (L known exactly)
TEST_MODE=latency_comp python run_matrix.py
TEST_MODE=latency_comp LEAD_DECEL=6.0 python run_matrix_lead.py

# Latency compensation — fragility to mis-estimated L
TEST_MODE=latency_mismatch python run_matrix.py
TEST_MODE=latency_mismatch LEAD_DECEL=6.0 python run_matrix_lead.py
```

Sweep constants (`LATENCY_DELAY_FRAMES`, `NOISE_SIGMA_M_SWEEP`, `COMP_CONTROLLERS`, `COMP_MISMATCH_DELAY`, `COMP_MISMATCH_L_FRAMES`, etc.) are defined at the top of each config file and can be freely adjusted.

> `TEST_MODE=original` (or unset) produces exactly the same results as the original code — the degrader is a strict no-op when all parameters are 0.

---

## Test Matrix

| Scenario | Variable | Values |
|---|---|---|
| Cut-in | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| Cut-in | `trigger_d` (Δd) | 20, 25, 30, 35, 40 m |
| Cut-in | `mu` | 0.85 (dry), 0.40 (wet) |
| Cut-in | `dart_speed_kmh` | 20 km/h (fixed) |
| Lead-brake | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| Lead-brake | `HEADWAY_THW` | 1.0, 1.5, 2.0, 2.5, 3.0 s |
| Lead-brake | `mu` | 0.85 (dry), 0.40 (wet) |

**Case count:** 50 cases/controller × 3 controllers = **150 runs per scenario**.

---

## Controllers

| Code name | Paper name | File |
|---|---|---|
| `baseline` | Static TTC | `control/baseline_static_ttc.py` |
| `proposed` | Adaptive TTC | `control/proposed_dynamic_ttc.py` |
| `proposed_enhanced` | Required-Deceleration | `control/proposed_enhanced.py` |
| `enhanced_predictive` | Required-Decel + Latency Predictor | `control/enhanced_predictive.py` |
| `enhanced_inflation` | Required-Decel + Threshold Inflation | `control/enhanced_inflation.py` |

All controllers receive the same `Perception`/`EgoState` inputs; differences arise purely from decision logic.

**Latency-compensated controllers** (`enhanced_predictive`, `enhanced_inflation`) extend
`proposed_enhanced` to counteract perception latency `L`. `enhanced_predictive` extrapolates
the target state forward by `L` (feedforward predictor) and feeds the predicted inputs into
the *same* `required_decel()`; `enhanced_inflation` adds the distance travelled during `L`
(`v_close·L`) to the required stopping distance (worst-case robust). At `L=0` both reduce to
`proposed_enhanced` / no compensation. They read `L` from the run spec via `comp_source`
(`oracle` = exact injected delay, `mismatched` = a separately swept `comp_L_frames`); no
runner/metrics changes are required. See `CHANGES_latency_compensation.md` and the
`latency_comp` / `latency_mismatch` test modes below.

---

## Key Parameters

All parameters live in `config/scenario_cutin.py` or `config/scenario_lead_brake.py`. The most commonly tuned:

| Parameter | Default | Meaning |
|---|---|---|
| `LEAD_DECEL` | `4.0` m/s² (env var override) | Lead vehicle braking deceleration; paper uses **6.0** (`LEAD_DECEL=6.0 python ...`) |
| `TTC_WARN_FULL` | `1.6` s | Partial-brake TTC threshold (baseline & proposed) |
| `TTC_BRAKE_FULL` | `0.6` s | Full-brake TTC threshold (baseline & proposed) |
| `PARTIAL_BRAKE` | `0.4` | Partial brake fraction (all controllers) |
| `DYN_K_SPEED` | `1.2` | Speed sensitivity for adaptive TTC |
| `DYN_K_MU` | `1.5` | Friction sensitivity for adaptive TTC |
| `REQ_FULL_FRAC` | `0.9` | Required-decel full-brake urgency threshold |
| `REQ_WARN_FRAC` | `0.6` | Required-decel partial-brake urgency threshold |
| `DETECTION_SOURCE` | `"groundtruth"` | `"groundtruth"` or `"yolo"` — controls brake gating |
| `FIXED_DT` | `0.05` s | Sync-mode timestep (20 FPS) |
| `MAX_TICKS` | `400` | Max ticks per run (= 20 s) |

---

## Adding a New Controller

1. Create a file in `control/` that inherits `BaseController` and uses `@register("name")`.
2. Implement `reset()` and `decide(perc, ego) -> carla.VehicleControl`.
3. Add `dict(label="name", controller="name", delay_frames=0)` to `MATRIX_RUNS` in the relevant config file.

No changes to scenario logic or metrics are needed. The controller works with both scenarios automatically.

---

## Output

Results are written to `results/`:
- `matrix_*.csv` — cut-in runs
- `lead_matrix_*.csv` — lead-brake runs (prefixed by `RESULTS_PREFIX`)

Key CSV columns: `label`, `controller`, `ego_speed_kmh`, `mu`, `avoided`, `s_clearance` (surface gap, m), `a_b_mfdd` (MFDD, m/s²), `t_c_warn` (TTC at brake onset, s), `dv_speed_var` (Δv, km/h), `is_conflict`, `peak_decel`, `a_max`. Latency-compensation runs add `comp_source` (`""`/`oracle`/`mismatched`) and `comp_L_frames` (the L actually used to compensate, in frames — may differ from `delay_frames` under `mismatched`). These columns default empty/0, so older CSVs still load in `report.py`.

Run `python core/report.py results/*.csv` for a per-controller CPEIM summary table.
