# AEB Test Harness — 3DGS + UE5.5 → CARLA

A simulation test harness for Automatic Emergency Braking (AEB) controllers, built for a 3D Gaussian Splatting scene imported into CARLA via UE5.5. Supports four independent test scenarios with three swappable controllers. Results are reported in the ICARCV 2026 paper *"A Real-to-Simulation Workflow for AEB Controller Testing Using 3D Gaussian Splatting in CARLA"*.

**Paper results:** cut-in `Rc_conflict` 44.0% → 72.0%; lead-brake 34.0% → 68.0% (baseline → proposed_enhanced).

---

## Scenarios

| Scenario | Scene | Description | Entry point |
|---|---|---|---|
| **Cut-in / Dart-out** | scene03_2 | A dart vehicle launches laterally from the roadside and stops blocking the ego lane. | `run_single.py` / `run_matrix.py` |
| **Lead-brake (CCRb)** | scene03_2 | A lead vehicle ahead in the same lane (matching ego speed) brakes suddenly to a stop. Euro-NCAP CCRb. | `run_single_lead.py` / `run_matrix_lead.py` |
| **CCRs** | train000 | Ego drives toward a stationary target vehicle. Euro-NCAP CCRs style. | `run_single_ccrs.py` / `run_matrix_ccrs.py` |
| **Cut-out** | train000 | A lead vehicle occludes a stationary target, then cuts out to the right revealing it. | `run_single_cutout.py` / `run_matrix_cutout.py` |

All scenarios share the same session, actors, controllers, YOLO, metrics, and viz infrastructure.

**Before running scene03_2 scenarios:** load the `scene03_2` map in CARLA.  
**Before running train000 scenarios:** load the `train000` map in CARLA.  
The scene-name check will raise a `RuntimeError` with a clear message if the wrong map is loaded.

---

## Directory Structure

```
config/
  scenario_cutin.py        ★ All parameters for cut-in (scene03_2, YOLO, μ, matrix, controllers)
  scenario_lead_brake.py   ★ All parameters for lead-brake (scene03_2; same structure)
  scenario_ccrs.py         ★ All parameters for CCRs (train000; 5×5×2=50 cases/controller)
  scenario_cutout.py       ★ All parameters for cut-out (train000; 5×5×2=50 cases/controller)
core/
  carla_session.py         Open/close sync mode, restore original settings
  actors.py                Spawn vehicles, set tire friction μ, attach sensors;
                           + set_spectator() / check_scene() helpers
  types.py                 Perception / EgoState dataclasses (controller input)
  metrics.py               RunRecord, 5 CPEIM indices, CSV writer, summarize()
  conflict.py              ★ Kinematic is_conflict for all 4 scenarios (no CARLA needed)
  report.py                ★ Read CSV → print CPEIM table (standalone, no CARLA)
  viz.py                   OpenCV overlay (run_single* only)
  scenario_cutin.py        Cut-in scene logic (ego cruise, dart trigger, blocking stop)
  runner.py                Single-case runner for cut-in (returns RunRecord)
  scenario_lead_brake.py   Lead-brake scene logic
  runner_lead_brake.py     Single-case runner for lead-brake (returns LeadBrakeRecord)
  scenario_ccrs.py         CCRs scene logic (target always stationary; ego cruises)
  runner_ccrs.py           Single-case runner for CCRs (returns CCRsRecord)
  scenario_cutout.py       Cut-out scene logic (lead cuts right after CUTOUT_TRIGGER_D)
  runner_cutout.py         Single-case runner for cut-out (returns CutOutRecord)
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
  check_conflict.py        ★ Verify is_conflict for all 4 scenario matrices (no CARLA needed)
run_single.py              Cut-in: run 1 case with OpenCV display
run_matrix.py              Cut-in: sweep full matrix → results/matrix_*.csv
run_single_lead.py         Lead-brake: run 1 case with display
run_matrix_lead.py         Lead-brake: sweep full matrix → results/lead_matrix_*.csv
run_single_ccrs.py         CCRs: run 1 case with display  [train000]
run_matrix_ccrs.py         CCRs: sweep full matrix → results/ccrs_matrix_*.csv  [train000]
run_single_cutout.py       Cut-out: run 1 case with display  [train000]
run_matrix_cutout.py       Cut-out: sweep full matrix → results/cutout_matrix_*.csv  [train000]
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
# Checks all 4 scenarios: cut-in (50), lead-brake (50), CCRs (50), cut-out (50) = 200 total
```

**Scenario 1 — Cut-in / Dart-out (scene03_2):**
```bash
python run_single.py     # debug single case with display; edit SINGLE_* in config/scenario_cutin.py
python run_matrix.py     # sweep 50 cases × 3 controllers → results/matrix_*.csv
MATRIX_VIZ=1 python run_matrix.py   # enable display during matrix run (opt-in)
```

**Scenario 2 — Lead-brake (CCRb, scene03_2):**
```bash
python run_single_lead.py                    # debug single case with display
LEAD_DECEL=6.0 python run_matrix_lead.py    # Euro-NCAP CCRb §3.4 — value used in paper (Table III/IV)
python run_matrix_lead.py                    # code default LEAD_DECEL=4.0 (moderate braking)
MATRIX_VIZ=1 python run_matrix_lead.py      # enable display during matrix run
```

**Scenario 3 — CCRs (stationary target, train000):**
```bash
# Load train000 in CARLA first
python run_single_ccrs.py                # debug single case with display
python run_matrix_ccrs.py                # sweep 50 cases × 3 controllers → results/ccrs_matrix_*.csv
TEST_MODE=latency python run_matrix_ccrs.py
MATRIX_VIZ=1 python run_matrix_ccrs.py  # enable display
```

**Scenario 4 — Cut-out (occluded stationary target, train000):**
```bash
# Load train000 in CARLA first
python run_single_cutout.py               # debug single case with display
python run_matrix_cutout.py               # sweep 50 cases × 3 controllers → results/cutout_matrix_*.csv
TEST_MODE=latency python run_matrix_cutout.py
MATRIX_VIZ=1 python run_matrix_cutout.py # enable display
```

**Summarise results from existing CSVs (no CARLA needed):**
```bash
python -m core.report results/matrix_*.csv
python -m core.report results/lead_matrix_*.csv
python -m core.report results/ccrs_matrix_*.csv
python -m core.report results/cutout_matrix_*.csv
```

> Each scenario's results are independently prefixed: `matrix_` (cut-in), `lead_matrix_` (lead-brake), `ccrs_matrix_` (CCRs), `cutout_matrix_` (cut-out).

---

## Spectator Camera

Each scenario has a pre-tuned spectator camera pose in its config (`SPECTATOR_TF`). Every run script calls `actors.set_spectator()` immediately after session open — the camera is cosmetic only and has no effect on recorded results. To disable, set `SPECTATOR_TF = None` in the config.

| Config | Camera position |
|---|---|
| `scenario_cutin.py` / `scenario_lead_brake.py` | x=2.07, y=−0.69, z=1.87, yaw=−91.22° |
| `scenario_ccrs.py` / `scenario_cutout.py` | x=5.27, y=−0.18, z=0.67, yaw=−143.44° |

---

## Test Modes

A `TEST_MODE` environment variable selects the degradation sweep. The degradation layer (`perception/degrade.py`) sits between the perception pipeline and every controller — all three controllers receive the same degraded input per run, preserving fairness. Its mirror image, the **prediction layer** (`perception/predict.py`), sits in the same slot for `latency_comp_all`: *degrade* makes the perception input stale (delays it by `L`), *predict* extrapolates it forward by `L` to make it fresh again — so any base controller can be run "with predictive-oracle compensation" without changing its decision logic.

| `TEST_MODE` | What it does | Runs (cut-in) |
|---|---|---|
| `original` (default) | All degradation params = 0; identical to original behaviour | 3 |
| `latency` | Sweep `delay_frames` ∈ {0, 4, 8, 16} (0–0.8 s) × 3 controllers | 12 |
| `noise` | Sweep `noise_sigma_m` ∈ {0.0, 0.5, 1.0, 2.0} m × 3 controllers | 12 |
| `latency_comp` | Sweep `delay_frames` × `COMP_CONTROLLERS` with `comp_source=oracle` (L known exactly). Measures how much of the latency-induced Rc loss each compensator recovers — an **upper bound**. | 16 |
| `latency_mismatch` | `enhanced_predictive` at a fixed injected delay (`COMP_MISMATCH_DELAY=8`) while sweeping the *compensated* `comp_L_frames` ∈ {4, 8, 12, 16} (under-/exact-/over-compensate). Measures **fragility** to mis-estimating L. | 4 |
| `latency_comp_all` | Apply the predictive-oracle **layer** (`perception/predict.py`, `comp_source=oracle`, L = `delay_frames` exactly) in front of **every base controller** (`CONTROLLERS`), sweeping `delay_frames` ∈ {0, 4, 8, 16}. All base controllers get the *same* predicted Perception at the same latency, so you can compare across controllers: how much of each controller's Rc does the predictor recover, and does the best compensated controller beat the best uncompensated one? | 12 |

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

# Predictive-oracle compensation across ALL base controllers
TEST_MODE=latency_comp_all python run_matrix.py
TEST_MODE=latency_comp_all LEAD_DECEL=6.0 python run_matrix_lead.py
```

> **Output filenames embed `TEST_MODE`.** Every mode now writes to
> `results/matrix_<TEST_MODE>_<YYYYMMDD_HHMMSS>.csv` (cut-in) and
> `results/lead_matrix_<TEST_MODE>_<YYYYMMDD_HHMMSS>.csv` (lead-brake), e.g.
> `matrix_latency_comp_all_20260711_101500.csv`, so each result file states what it tested.
> The timestamp keeps every run unique; existing result files are never overwritten.

Sweep constants (`LATENCY_DELAY_FRAMES`, `NOISE_SIGMA_M_SWEEP`, `COMP_CONTROLLERS`, `COMP_MISMATCH_DELAY`, `COMP_MISMATCH_L_FRAMES`, etc.) are defined at the top of each config file and can be freely adjusted.

> `TEST_MODE=original` (or unset) produces exactly the same results as the original code — the degrader is a strict no-op when all parameters are 0.

---

## Test Matrix

| Scenario | Scene | Variable | Values | Cases/ctrl |
|---|---|---|---|---|
| Cut-in | scene03_2 | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h | 50 |
| Cut-in | scene03_2 | `trigger_d` (Δd) | 20, 25, 30, 35, 40 m | |
| Cut-in | scene03_2 | `mu` | 0.85 (dry), 0.40 (wet) | |
| Lead-brake | scene03_2 | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h | 50 |
| Lead-brake | scene03_2 | `HEADWAY_THW` | 1.0, 1.5, 2.0, 2.5, 3.0 s | |
| Lead-brake | scene03_2 | `mu` | 0.85 (dry), 0.40 (wet) | |
| **CCRs** | **train000** | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h | **50** |
| CCRs | train000 | `approach_d` | 30, 40, 50, 60, 70 m (centre-to-centre from ego spawn) | |
| CCRs | train000 | `mu` | 0.85 (dry), 0.40 (wet) | |
| **Cut-out** | **train000** | `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h | **50** |
| Cut-out | train000 | `reveal_ttc` | 1.0, 1.5, 2.0, 2.5, 3.0 s (TTC at cut-out trigger) | |
| Cut-out | train000 | `mu` | 0.85 (dry), 0.40 (wet) | |

**Cut-out fixed headway:** `FIXED_HEADWAY_THW = 0.8 s` (not swept). `headway_d` and `cutout_trigger_d` are derived per case from `reveal_ttc` in `run_matrix_cutout.py`. Diagnostic columns `range_at_reveal`, `ttc_at_reveal`, `time_reveal_to_brake` are written to the CSV and summarised by `core/report.py`. Hardest case (`reveal_ttc = 1.0`, 20 km/h): `cutout_trigger_d ≈ 5.6 m`, lead-to-target surface gap ≈ 1.1 m — verify CARLA physics before running; if the lead cannot complete the lane change, increase `GAP_OFFSET` slightly (e.g. 5.0 m), not the matrix values.

**CCRs approach distance:** Target spawned per case at `EGO_SPAWN + approach_d × forward_vector`, covering short/medium/long TTC-at-spawn scenarios (TTC@30m+60km/h ≈ 1.5 s; TTC@70m+20km/h ≈ 11.8 s). Target coordinates `target_x`/`target_y` are passed through the case dict.

**Case count:** 50 / 50 / 50 / 50 cases/controller × 3 controllers = 150 runs per scenario (= 600 total across all 4 scenarios for `TEST_MODE=original`).

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

## Visualisation (run_single* only)

`core/viz.py` draws two layers on the front-camera feed (OpenCV window):

| Layer | Colour | Condition |
|---|---|---|
| YOLO boxes (thin) | Green | Every object the YOLOv8n model detects, regardless of braking |
| Hazard box (thick) | **Green** | Target is in the ego corridor (`in_path=True`) but the controller has not yet engaged braking (`brake_cmd = 0`) |
| Hazard box (thick) | **Red** | Target is in the ego corridor **and** the controller is actively braking (`brake_cmd > 0`) |

The hazard box colour is purely cosmetic — it does not affect `Perception`, the braking decision, or any CSV value.

---

## Cut-out Occlusion Gate

In the cut-out scenario the stationary target is occluded by the lead vehicle until the lead cuts right. With `DETECTION_SOURCE="groundtruth"` the target would otherwise register as "detected" from the moment it enters `INPATH_MAX_RANGE`, even while fully blocked.

The **occlusion gate** (enabled by `OCCLUSION_GATE = True` in `config/scenario_cutout.py`) suppresses detection while the lead is between ego and target. Detection is restored once the lead has moved `OCCLUSION_LAT_CLEAR` metres laterally from the target's lane position (i.e. cut out far enough). Ground-truth range/TTC are still used once the target becomes visible, keeping full run-to-run repeatability.

| Config key | Default | Meaning |
|---|---|---|
| `OCCLUSION_GATE` | `True` | Enable the sight-line gate |
| `OCCLUSION_LAT_CLEAR` | `1.5` m | Lateral clearance (lead − target, in ego frame) before target is visible `[TO BE TUNED]` |
| `OCCLUSION_LON_MARGIN` | `2.0` m | Lead is no longer "in front of" target once it is within this distance behind `target_lon` `[TO BE TUNED]` |

The gate is cut-out only — no other scenario is affected. The geometry helper `core/occlusion.sight_line_occluded()` has no CARLA dependency and is covered by `tests/test_occlusion_gate.py`.

---

## Output

Results are written to `results/`:
- `matrix_*.csv` — cut-in runs
- `lead_matrix_*.csv` — lead-brake runs
- `ccrs_matrix_*.csv` — CCRs runs (train000)
- `cutout_matrix_*.csv` — cut-out runs (train000)

Key CSV columns: `label`, `controller`, `ego_speed_kmh`, `mu`, `avoided`, `s_clearance` (surface gap, m), `a_b_mfdd` (MFDD, m/s²), `t_c_warn` (TTC at brake onset, s), `dv_speed_var` (Δv, km/h), `is_conflict`, `peak_decel`, `a_max`. Latency-compensation runs add `comp_source` (`""`/`oracle`/`mismatched`) and `comp_L_frames` (the L actually used to compensate, in frames — may differ from `delay_frames` under `mismatched`). These columns default empty/0, so older CSVs still load in `report.py`.

Run `python core/report.py results/*.csv` for a per-controller CPEIM summary table.
