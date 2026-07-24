# Technical Documentation — AEB Test Harness

Consolidated technical reference for the AEB CARLA simulation harness.  
Target publication: **ICARCV 2026** — *"A Real-to-Simulation Workflow for AEB Controller Testing Using 3D Gaussian Splatting in CARLA"*

---

## Table of Contents

1. [Scenario Details](#1-scenario-details)
2. [Test Matrix Definition](#2-test-matrix-definition)
3. [Conflict Case Definition (`is_conflict`)](#3-conflict-case-definition-is_conflict)
4. [Per-Case Algorithm Pipeline](#4-per-case-algorithm-pipeline)
5. [Controller Logic](#5-controller-logic)
6. [CPEIM Metrics and Aggregation Rules](#6-cpeim-metrics-and-aggregation-rules)
7. [CSV Schema](#7-csv-schema)
8. [Pre-Run Checklist](#8-pre-run-checklist)
9. [Brake Timing Parameters — Equation (1)](#9-brake-timing-parameters--equation-1)
10. [YOLO Detection Table Verification (TABLE V)](#10-yolo-detection-table-verification-table-v)
11. [Pre-Submission Code Review Notes (ICARCV 2026)](#11-pre-submission-code-review-notes-icarcv-2026)
12. [Code Revision History](#12-code-revision-history)

---

## 1. Scenario Details

### Scenario 1 — Cut-in / Dart-out

**Files:** `core/scenario_cutin.py` + `core/runner.py`

Ego drives straight in the ego lane. A dart vehicle is parked to the side. When the longitudinal distance between ego and dart reaches `trigger_d`, the dart launches laterally across the lane at `dart_speed_kmh` and stops, blocking the path.

- **Surface gap formula:** `gap_offset = ego.extent.x + dart.extent.y` (front-of-ego to side-of-dart)
- **Predictive corridor:** `INPATH_PREDICT = True`; dart is detected as soon as its predicted lateral position intersects the ego corridor within `INPATH_LOOKAHEAD = 1.5 s`
- **Dart stop x:** `DART_STOP_X = 3.5` (world x, lane centre)

### Scenario 2 — Lead-brake (CCRb)

**Files:** `core/scenario_lead_brake.py` + `core/runner_lead_brake.py`

A lead vehicle spawns ahead of ego in the same lane, heading the same direction, initially matching ego's speed (`LEAD_SAME_AS_EGO = True`). After travelling `LEAD_BRAKE_AFTER_M = 8.0 m`, the lead vehicle brakes at constant deceleration `LEAD_DECEL` until stopped.

- **Surface gap formula:** `gap_offset = ego.extent.x + lead.extent.x` (rear-end collision geometry)
- **Predictive corridor:** `INPATH_PREDICT = False`; lead is detected immediately (already in lane), TTC ≈ ∞ while both vehicles travel at equal speed
- **LEAD_DECEL override:** `LEAD_DECEL=6.0 python run_matrix_lead.py` (Euro-NCAP CCRb §3.4 standard)

---

## 2. Test Matrix Definition

### Cut-in Matrix (50 cases/controller)

| Variable | Values |
|---|---|
| `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| `trigger_d` (Δd) | 20, 25, 30, 35, 40 m |
| `mu` | 0.85 (dry), 0.40 (wet) |
| `dart_speed_kmh` | 20 km/h (fixed) |

Formula: 5 × 5 × 2 × 1 = **50 cases**. Total with 3 controllers: **150 runs**.

**Domain justification:**
- **20 km/h** — low-speed urban / parking lot scenarios where AEB is still required
- **40 m trigger distance** — longer reaction horizon; tests improved performance with early hazard detection (longitudinal distance at trigger ≈ 38.5 m < `INPATH_MAX_RANGE = 40 m`, ground-truth detection works correctly)

### Lead-brake Matrix (50 cases/controller)

| Variable | Values |
|---|---|
| `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| `HEADWAY_THW` | 1.0, 1.5, 2.0, 2.5, 3.0 s |
| `mu` | 0.85 (dry), 0.40 (wet) |

Formula: 5 × 5 × 2 = **50 cases**. Total with 3 controllers: **150 runs**.

**Domain justification:**
- **THW = 3.0 s** — cautious following distance per highway code / ISO 15622; extends domain toward easy side

**THW → Actual Headway Distance (m):**

| Speed | 1.0 s | 1.5 s | 2.0 s | 2.5 s | 3.0 s |
|---|---|---|---|---|---|
| **20 km/h** | 5.6 m | 8.3 m | 11.1 m | 13.9 m | 16.7 m |
| 30 km/h | 8.3 m | 12.5 m | 16.7 m | 20.8 m | 25.0 m |
| 40 km/h | 11.1 m | 16.7 m | 22.2 m | 27.8 m | 33.3 m |
| 50 km/h | 13.9 m | 20.8 m | 27.8 m | 34.7 m | 41.7 m |
| 60 km/h | 16.7 m | 25.0 m | 33.3 m | 41.7 m | 50.0 m |

### CCRs Matrix (50 cases/controller)

| Variable | Values |
|---|---|
| `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| `approach_d` | 30, 40, 50, 60, 70 m (centre-to-centre from ego spawn) |
| `mu` | 0.85 (dry), 0.40 (wet) |

Formula: 5 × 5 × 2 = **50 cases**. Total with 3 controllers: **150 runs**.

**Domain justification:**
- TTC at spawn spans ≈ 1.5 s (30 m + 60 km/h, hardest) to ≈ 11.8 s (70 m + 20 km/h, easiest).
- Minimum ≥ 30 m ensures the ego has room to establish cruise speed before braking.
- All 50 cases are conflict cases (worst: 70 m @ 20 km/h → t = 11.8 s < 20 s window).

### Cut-out Matrix (50 cases/controller)

| Variable | Values |
|---|---|
| `ego_speed_kmh` | 20, 30, 40, 50, 60 km/h |
| `reveal_ttc` | 1.0, 1.5, 2.0, 2.5, 3.0 s (TTC to stationary target at cut-out trigger) |
| `mu` | 0.85 (dry), 0.40 (wet) |
| `FIXED_HEADWAY_THW` | **0.8 s** (fixed, not swept) |

Formula: 5 × 5 × 2 = **50 cases**. Total with 3 controllers: **150 runs**.

**Derived geometry (per case):**
```
headway_d        = FIXED_HEADWAY_THW × ego_ms   (= 0.8 × ego_ms)
cutout_trigger_d = (reveal_ttc − 0.8) × ego_ms + GAP_OFFSET
```

All 50 cases have `cutout_trigger_d > 0` (min: 5.61 m at 20 km/h + reveal_ttc=1.0). Conflict is defined against the stationary target and is independent of `reveal_ttc` — all 50 cases are conflict cases (worst: 20 km/h → t = 7.9 s < 20 s window).

**Spawn geometry:** At 20 km/h, `headway_d = 0.8 × 5.556 = 4.444 m`. This is below the combined ego + lead half-lengths (≈ 4.5 m from `GAP_OFFSET`), causing bounding-box overlap and a CARLA spawn rejection. `runner_cutout.py` enforces a minimum headway (`SPAWN_CLEARANCE_M = 0.5 m`) and clamps to ~5.0 m. At `reveal_ttc = 1.0` the post-clamp trigger surface gap is only 0.556 m (0.10 s at 20 km/h) — the physics-based lane change cannot complete; these cells are marked `result_txt = "SCENARIO_INFEASIBLE"` and retained in the CSV (filtered in analysis). See `tools/check_cutout_spawn.py` for the full per-cell feasibility table.

---

## 3. Conflict Case Definition (`is_conflict`)

**Definition (both scenarios):** A case is a *conflict case* if an ego that never brakes (constant speed throughout) would collide with the obstacle within `MAX_TICKS × FIXED_DT` seconds (= 20 s).

**Properties:**
- Computed **before simulation** from kinematics only — no extra CARLA runs
- **Identical for all three controllers** — not controller-dependent, not post-hoc
- Stored as the `is_conflict` column in every CSV row

**Code:** `core/conflict.py` (derivation proof in file)

**API:**
```python
cutin_is_conflict(case, cfg) -> bool       # case has 'trigger_d', 'ego_speed_kmh', 'dart_speed_kmh'
lead_brake_is_conflict(case, cfg) -> bool  # case has 'headway_d' (m), 'ego_speed_kmh'
```

### Lead-brake Kinematic Formula

```
Phase 1: gap unchanged (both at v_e)         t1 = LEAD_BRAKE_AFTER_M / v_e
Phase 2: lead brakes to stop                 t2 = v_e / LEAD_DECEL
                                             gap_at_stop = headway_d − v_e²/(2·LEAD_DECEL)
Phase 3: lead stationary, ego closes in      t3 = max(0, gap_at_stop) / v_e
is_conflict = (t1 + t2 + t3 ≤ MAX_TICKS · FIXED_DT)
```

### Conflict Check Results (current matrix, LEAD_DECEL=6.0)

| Scenario | Total cases | `is_conflict` | Boundary case |
|---|---|---|---|
| Cut-in | 50 | **all 50** | 20 km/h + trigger_d=40 m → t_conflict ≈ 7.1 s ≤ 20 s ✓ |
| Lead-brake (LEAD_DECEL=6.0) | 50 | **all 50** | 20 km/h + THW=3.0 s → t_conflict ≈ 4.9 s ≤ 20 s ✓ |

With the current parameters, `Rc_conflict = Rc_all` (all cases are conflict cases).

> **Note on wider matrices:** If `LEAD_DECEL` is reduced significantly or `THW` increased substantially, no-conflict cases may appear. Run `python tools/check_conflict.py` before any matrix run. No-conflict cases are **not excluded** from the CSV but are not counted in `Rc_conflict`.

---

## 4. Per-Case Algorithm Pipeline

Each case runs in sync-mode loop at `FIXED_DT = 0.05 s` (20 FPS). Every tick computes the following from **CARLA ground truth** and feeds it to the controller.

All three controllers receive the same inputs; performance differences come purely from decision logic.

### Step 1 — Initial Headway (THW → metres)

```python
if USE_THW:
    headway_d = v_e_ms * headway_thw      # v_e in m/s
# Lead vehicle spawns at: y = EGO_SPAWN.y - headway_d
```

### Step 2 — Surface Gap

```python
gap = max(0, dist_center - gap_offset)
# Cut-in:   gap_offset = ego.extent.x + dart.extent.y  (front-of-ego vs side-of-dart)
# Lead-brake: gap_offset = ego.extent.x + lead.extent.x (rear-end geometry)
```

`perc.distance` and the recorded `s_clearance` are both **surface gap** (not center-to-center).

### Step 3 — TTC

```python
rel_speed = max(0, (gap_prev - gap) / FIXED_DT)
TTC       = gap / rel_speed     # = ∞ when rel_speed ≈ 0
```

### Step 4 — Lead Vehicle Deceleration (Ground-truth + EMA)

```python
raw_decel      = max(0, (v_lead_prev - v_lead) / FIXED_DT)
lead_decel_ema = 0.3 * raw_decel + 0.7 * lead_decel_ema
```

`LEAD_DECEL` is never read from config directly by the controller — it is estimated via finite differences and EMA for realism.

### Step 4b — Perception Degradation (optional, `TEST_MODE` ≠ `original`)

`PerceptionDegrader.apply(perc, ego)` is called after Step 4 and before the controller. It applies three degradations in order:

1. **Latency** — FIFO buffer of length `delay_frames + 1`; controller receives the Perception from `delay_frames` ticks ago. Covers all fields (`distance`, `rel_speed`, `lead_speed`, `ttc`, `detected`). The old `det_buffer` (delayed only `detected`) is replaced by this; latency is counted once.
2. **Gaussian noise** — adds `N(0, noise_sigma_m)` to `distance` (clamped ≥ 0) and `N(0, noise_sigma_vr)` to `rel_speed` and `lead_speed`; `ttc` is recomputed from the noisy values.
3. **Dropout** — per-tick Bernoulli(`dropout_p`); on dropout, either freeze (return last-valid Perception) or miss (force `detected=False`).

`TEST_MODE=original` (default): all parameters are 0 → degrader is a strict no-op → original behaviour preserved exactly.

Seeding: each run's RNG is seeded from `(label_chars_sum × 10000 + case_idx) % 2^32` for reproducibility.

All three controllers receive the same degraded input per run (fairness).

### Step 5 — Ego Brake Model (kinematic, friction-limited)

```python
a_max     = mu * g                         # friction ceiling
a_cmd     = brake_cmd * a_max              # controller output in [0,1]
a_applied = min(a_cmd, a_max)              # hard clamp
v_new     = max(0.0, v_model - a_applied * FIXED_DT)
```

- Updated from `v_model` (captured at brake onset), not read back from CARLA
- `peak_decel` logged is always ≤ `a_max`
- Response time `t_d` and build-up time `t_s` are effectively 0 within one tick (0.05 s) — see Section 9

---

## 5. Controller Logic

### Baseline — Static TTC (`control/baseline_static_ttc.py`)

```
TTC ≤ TTC_BRAKE_FULL (0.6 s) → full brake (1.0)
TTC ≤ TTC_WARN_FULL  (1.6 s) → partial brake (PARTIAL_BRAKE = 0.4)
```

**Weakness:** Fixed thresholds, unaware of speed or road friction → fails to stop in time on wet roads at higher speeds.

### Proposed — Adaptive TTC (`control/proposed_dynamic_ttc.py`)

```
bump     = K_SPEED * max(0, (v_kmh - V0) / 100) + K_MU * max(0, (MU0 - mu))
thr_full = TTC_BRAKE_FULL + bump
thr_warn = max(TTC_WARN_FULL, thr_full + 0.5)
```

**Improvement:** Higher speed / lower friction → higher thresholds → earlier braking.

### Proposed Enhanced — Required-Deceleration (`control/proposed_enhanced.py`)

```
a_max   = mu * g
d_lead  = v_lead² / (2 * a_lead)          # distance lead still travels before stopping
a_req   = v_ego² / (2 * (gap + d_lead))   # deceleration ego needs to avoid collision
urgency = a_req / a_max
  urgency ≥ REQ_FULL_FRAC (0.9) → full brake
  urgency ≥ REQ_WARN_FRAC (0.6) → partial brake
```

**Mechanism:** When lead brakes: `a_lead ↑ → d_lead ↓ → a_req ↑ → urgency ↑` → braking triggered at the right moment.  
Cut-in scene: `v_lead = 0 → d_lead = 0 → a_req = v_e² / (2 × gap)` (stationary obstacle case).

**Controller parameter summary:**

| Parameter | Value | Applies to |
|---|---|---|
| `TTC_WARN_FULL` | 1.6 s | baseline, proposed |
| `TTC_BRAKE_FULL` | 0.6 s | baseline, proposed |
| `PARTIAL_BRAKE` | 0.4 | all three |
| `DYN_V0`, `DYN_MU0` | 40 km/h, 0.85 | proposed |
| `DYN_K_SPEED`, `DYN_K_MU` | 1.2, 1.5 | proposed |
| `REQ_FULL_FRAC`, `REQ_WARN_FRAC` | 0.9, 0.6 | proposed_enhanced, enhanced_predictive, enhanced_inflation |
| `COMP_R_SAFE` | 0.0 m | enhanced_inflation |

---

### Latency-Compensated Controllers

Both extend `proposed_enhanced` to counteract a **known** perception latency `L` (seconds).
`L = comp_L_frames × FIXED_DT`, sourced from the run spec via
`compensation_latency(run_spec, cfg)` in `control/base_controller.py`. Decision gate, brake
latch, urgency thresholds, and the μ·g ceiling are all inherited unchanged — only the inputs
to the required-decel computation differ.

**Conceptual framing (feedforward predictor).** Compensation predicts the target state
forward by `L` and decides on the *predicted* state instead of the stale (latency-delayed)
one — a discrete analog of a Smith predictor. The principle is borrowed from delay
compensation in cooperative ACC / time-delay control (Xing, Ploeg & Nijmeijer 2019; Richard
2003), **not** from AEB-native work.

**`enhanced_predictive` — feedforward predictor (`control/enhanced_predictive.py`), main method.**
Constant-acceleration extrapolation of the *inputs* to the existing `required_decel()`.
Before the ego brakes, `a_ego ≈ 0`, so the closing acceleration is `a_close = +lead_decel`
(the lead braking makes the gap close faster):

```
v_l_pred = max(0, lead_speed - lead_decel · L)            # lead keeps decelerating
gap_pred = max(0, distance - v_close · L - 0.5 · lead_decel · L²)   # v_close = rel_speed (closing)
a_req    = required_decel(ego.speed_ms, v_l_pred, lead_decel, gap_pred)   # SAME formula
urgency  = a_req / (μ·g)                                  # SAME full/partial thresholds
```

Because it extrapolates the *inputs* (not a new closed form), at `L=0` it calls
`required_decel(speed_ms, lead_speed, lead_decel, distance)` — **identical to
`proposed_enhanced` on every tick** (verified in `tests/test_latency_comp.py`).

**`enhanced_inflation` — threshold inflation (`control/enhanced_inflation.py`), worst-case robust.**
Does not predict the target; it adds the distance the ego travels during `L` to the required
stopping distance:

```
r_required = v_close² / (2·μ·g) + v_close · L            # physics stop dist + latency creep
urgency    = r_required / max(ε, gap - COMP_R_SAFE)      # ≥1 ⇒ not enough room
  urgency ≥ REQ_FULL_FRAC → full brake ; ≥ REQ_WARN_FRAC → partial
```

Advantage: needs only an **upper bound** `L_max`, not exact `L`. The `v_close·L` term is the
standard delay term in Mazda/Berkeley-PATH safety-distance formulas (Rajamani 2012). At `L=0`
the inflation term vanishes → non-compensated behaviour.

**Oracle vs mismatched (the experiment).** The predictor must know `L`. In this simulator the
harness *injects* `L`, so it can be known exactly — this is an **oracle / idealized upper
bound** on Rc recovery. Two `comp_source` values drive the two research questions:

| `comp_source` | Compensated L | Research question |
|---|---|---|
| `oracle` | `L = delay_frames · dt` (exact injected delay) | Upper bound: if `L` is known exactly, how much Rc is recovered? |
| `mismatched` | `L = comp_L_frames · dt`, set independently of `delay_frames` | Fragility: how does Rc degrade under under-/over-compensation? |

> **Why online L-measurement (timestamp) is future work.** Measuring `L` from sensor
> timestamps has no uncertainty *in simulation* — the timestamp equals the value we injected,
> so there is nothing to test. We therefore emulate "not knowing `L`" with the `mismatched`
> sweep (grounded in Richard 2003, which shows predictor-based control is sensitive to
> delay-mismatch). True online `L` estimation belongs to perception-in-the-loop testing; MPC
> and timestamp-jitter variants are likewise deferred.

### Predictive compensation as a *layer* — `TEST_MODE=latency_comp_all`

`enhanced_predictive` bakes the predictor *inside* a required-decel controller. The
`latency_comp_all` experiment instead makes prediction a **preprocessing layer**
(`perception/predict.py`, `PerceptionPredictor`) that can sit in front of *any* base
controller — the exact mirror of the degradation layer:

```
raw Perception → degrade (delay by L → stale) → predict (extrapolate +L → fresh) → controller
```

`PerceptionPredictor.apply(perc, ego)` rewrites the Perception fields using the same
constant-acceleration model as `enhanced_predictive` (closing acceleration `+lead_decel`):

```
lead_speed' = max(0, lead_speed − lead_decel·L)
distance'   = max(0, distance − v_close·L − 0.5·lead_decel·L²)   # v_close = rel_speed
rel_speed'  = v_close + lead_decel·L ;  ttc' = distance'/rel_speed'
lead_decel' = lead_decel (unchanged) ;  detected / box_h passed through
```

- **L = 0 identity.** `apply()` returns a copy with every field unchanged, so
  `compensated_X(L=0)` equals base controller `X` per tick for **every** controller.
- **enhanced_predictive consistency.** `proposed_enhanced` reads only
  `distance` / `lead_speed` / `lead_decel` / `detected`, and `distance'`/`lead_speed'`
  are exactly the inputs `enhanced_predictive` feeds to `required_decel()`. Hence
  *`PerceptionPredictor` + `proposed_enhanced` reproduces `enhanced_predictive` per tick*
  (proven for L ∈ {0,4,8,16}, both scenarios, in `tests/test_latency_comp_all.py`). The two
  predict at different points (Perception fields vs required-decel inputs) but are
  mathematically identical for the required-decel controller; `rel_speed'`/`ttc'` extend
  the same model to the TTC controllers (`baseline` / `proposed`).
- **Fairness.** All base controllers receive the *same* predicted Perception at the same
  latency (controller-swap protocol), so Rc differences are attributable to the controller.
- **Predictive only.** Inflation is distance-based and does not fit the time-based
  TTC controllers, so `latency_comp_all` sweeps the predictor only (oracle L).

The layer is engaged solely when a run spec carries `predict=True` (set only by
`build_matrix_runs("latency_comp_all")`); every other mode leaves `predictor=None`, so
`original` / `latency` / `noise` / `latency_comp` / `latency_mismatch` are unchanged. Rows
are recorded with `comp_source="oracle"`, `comp_L_frames == delay_frames`, and
`test_mode="latency_comp_all"` (no new CSV column — the base `controller` + `test_mode`
distinguish these rows from `latency_comp`, which uses the comp controllers).

---

## 6. CPEIM Metrics and Aggregation Rules

Five CPEIM indices are logged per run in every CSV:

| Index | CSV column | Formula / source |
|---|---|---|
| s (clearance) | `s_clearance` | Surface gap (m) when ego stops; **not** center-to-center |
| a_b (MFDD) | `a_b_mfdd` | `(v_b² - v_f²) / (25.92 × (S_f - S_b))`, v ∈ [0.1, 0.8]·v₀ |
| T_c (warning lead time) | `t_c_warn` | TTC at the moment braking is initiated (s) |
| Δv_spd (speed variation) | `dv_speed_var` | `v_ego_start - v_collision` (km/h); = `v_ego_start` for avoided cases |
| R_c | derived from `avoided` | `n_avoided / n_total` |

**Aggregation rules (used by `summarize()` and `core/report.py`):**

| Index | Aggregation rule | Reason |
|---|---|---|
| `Rc_all` | `n_avoided / n_total` | Avoidance rate over all cases |
| `Rc_conflict` | `n_avoided_conflict / n_conflict` (is_conflict=True) | Avoidance rate over safety-critical cases only |
| `mean_ab` (MFDD) | Average over `a_b_mfdd > 0` only | 0 means ego did not decelerate to 10% v₀; not a valid MFDD sample |
| `mean_sc` (s_clearance) | Average over `avoided=True` only | For collisions, s=0 is not a clearance measurement |
| `mean_tc` (T_c_warn) | Average over `t_c_warn > 0` only | 0 means controller never braked; not a T_c sample |
| `mean_dv` (Δv_spd) | Average over all rows | Speed reduction is informative in all cases |

> **Do not use the composite CPEIM score:** Weights `W_S/W_AB/W_TC/W_DV/W_RC` in `core/metrics.py` come from the liu2025 V-VRU (pedestrian) scenario, not from a car-to-car scenario. Report the 5 raw indices + Rc instead.

---

## 7. CSV Schema

### Cut-in results (`results/matrix_*.csv`)

| Column | Meaning |
|---|---|
| `label`, `controller`, `delay_frames` | Controller identifier + perception latency |
| `ego_speed_kmh`, `mu`, `trigger_d`, `dart_speed_kmh` | Case parameters |
| `avoided`, `collision_with`, `collision_speed_kmh` | Outcome: avoided? / collided with what / speed at impact |
| `s_clearance` | **Surface gap** (m) when ego stops; 0 = collision or did not stop |
| `a_b_mfdd` | MFDD (m/s²); 0 = ego speed did not fall to 10% of initial speed |
| `t_c_warn` | TTC at brake onset (s); 0 = no braking, or TTC=∞ at brake time |
| `dv_speed_var` | Speed reduction (km/h) = `v_start - v_collision` (or `v_start` if stopped) |
| `peak_decel` | Max applied deceleration (m/s²) after clamp; always ≤ `a_max` |
| `brake_distance` | Distance to stop (m) |
| `a_req_at_brake` | Required deceleration (m/s²) at brake onset |
| `a_max` | Friction ceiling = μ·g (m/s²) |
| `min_dist`, `result_txt` | Minimum centre-to-centre distance / text result summary |
| `is_conflict` | ★ `True` = ego at constant speed would collide within 20 s (kinematic, pre-sim) |
| `noise_sigma_m` | Distance noise sigma used (m); 0.0 = no noise |
| `noise_sigma_vr` | Velocity noise sigma used (m/s); 0.0 = no noise |
| `dropout_p` | Per-tick dropout probability; 0.0 = no dropout |
| `dropout_mode` | `"freeze"` or `"miss"` — dropout behaviour (see `perception/degrade.py`) |
| `test_mode` | `TEST_MODE` env value at run time (`"original"`, `"latency"`, `"noise"`, `"latency_comp"`, `"latency_mismatch"`) |
| `seed` | RNG seed used for this run (deterministic reproduction) |
| `comp_source` | Latency-compensation source: `""` (none), `"oracle"` (L = injected delay), `"mismatched"` (L set independently) |
| `comp_L_frames` | L actually used to compensate (frames); may differ from `delay_frames` under `mismatched`. `L_seconds = comp_L_frames × FIXED_DT` |

> `comp_source`/`comp_L_frames` default to `""`/`0`, so CSVs written before this feature still
> load unchanged in `core/report.py` (tolerant `.get`). Both scenarios share these two columns.

### Lead-brake results (`results/lead_matrix_*.csv`)

Same as above, with three additional columns:

| Column | Meaning |
|---|---|
| `lead_speed_kmh` | Lead vehicle speed before braking (km/h) |
| `headway_thw` | Time headway (s); 0 if using fixed-distance mode |
| `headway_d` | Actual headway distance (m) = THW × v_e (or fixed value) |

---

## 8. Pre-Run Checklist

Verify all of the following before pressing `run_matrix.py` / `run_matrix_lead.py`:

- [ ] **LEAD_DECEL** = 6.0 to match paper (Euro-NCAP CCRb §3.4) — must pass via env var: `LEAD_DECEL=6.0 python run_matrix_lead.py` (code default = 4.0)
- [ ] **MATRIX** (speed, THW, μ, trigger_d) matches paper Tables I–II (50 cases/controller):
      cut-in: speed={20,30,40,50,60}, Δd={20,25,30,35,40} m, μ={0.85,0.40}
      lead-brake: speed={20,30,40,50,60}, THW={1.0,1.5,2.0,2.5,3.0} s, μ={0.85,0.40}
- [ ] **Conflict check passes:** `python tools/check_conflict.py` (no CARLA needed)
      Confirm conflict/no-conflict counts match what is reported in the paper
- [ ] **MATRIX_RUNS** has 3 labels: `baseline`, `proposed`, `proposed_enhanced`, each with `delay_frames=0`
- [ ] **Controller params** match paper claims: `TTC_WARN_FULL=1.6`, `TTC_BRAKE_FULL=0.6`, `DYN_K_SPEED=1.2`, `DYN_K_MU=1.5`, `REQ_FULL_FRAC=0.9`, `REQ_WARN_FRAC=0.6`
- [ ] **DETECTION_SOURCE** = `"groundtruth"` (both scenarios, matches paper Section IV-B)
- [ ] **BRAKE_MODEL** = `"kinematic"` (both scenarios)
- [ ] Verify `peak_decel ≤ a_max` in all CSV rows: `python core/report.py results/*.csv`
- [ ] Verify `is_conflict` is consistent with paper definition: `python core/report.py results/*.csv`
- [ ] Record CSV filename + timestamp in paper Section IV-C for reproducibility

---

## 9. Brake Timing Parameters — Equation (1)

Equation (1) in the paper:

```
s = v_e * (t_d + t_s/2) + v_e² / (2 * μ * g)
```

This equation is **expository only** — it is not executed at runtime. It motivates the kinematic deceleration cap. The three controllers use TTC thresholds or required-deceleration urgency, not this formula.

### t_d — Actuation / Processing Delay

| Item | Value |
|---|---|
| Config variable | `delay_frames` (frame count); `SINGLE_DELAY_FRAMES` in configs |
| File | `config/scenario_cutin.py:100`, `config/scenario_lead_brake.py:114` (config); `core/runner.py:97–198`, `core/runner_lead_brake.py:141–235` (runtime) |
| Value in all matrix runs | **0 s** (= 0 frames × 0.05 s/frame) |

**Mechanism (updated):** Latency is now handled by `PerceptionDegrader` (see Section 4 Step 4b). The old `det_buffer` in `runner.py` / `runner_lead_brake.py` is a no-op (`maxlen=1`). `PerceptionDegrader` delays the **full Perception object** (all fields including `distance`, `rel_speed`, `lead_speed`, `detected`), not just the detection boolean. This makes `delay_frames` a genuine end-to-end perception latency variable.

`delay_frames=0` (default for `TEST_MODE=original`): degrader buffer has `maxlen=1`; controller always receives the current-frame Perception — identical to original behaviour.

To sweep latency: `TEST_MODE=latency python run_matrix.py` (see Section 4 Step 4b and README Test Modes).

### t_s — Brake Build-up (Ramp) Time

| Item | Value |
|---|---|
| Variable | *(none — no ramp is implemented)* |
| File | `core/actors.py:74–93` (`apply_kinematic_brake`) |
| Value in all runs | **0 s** — braking force reaches commanded level instantaneously |

**Mechanism:** `apply_kinematic_brake` converts `brake_cmd ∈ [0, 1]` directly to `a_cmd = brake_cmd × μg` and clamps at `a_max = μg` in a single tick. `PARTIAL_BRAKE = 0.4` is the fraction of full brake during the warning phase — it is not a ramp duration.

```python
def apply_kinematic_brake(ego, v_model, brake_cmd, mu, dt, g=9.81):
    a_max    = max(0.0, mu) * g
    a_cmd    = max(0.0, brake_cmd) * a_max
    a_applied = min(a_cmd, a_max)      # hard clamp
    v_new    = max(0.0, v_model - a_applied * dt)
```

### Summary

| Timing parameter | Effective value | How |
|---|---|---|
| `t_d` | 0 s | `delay_frames = 0` in all `MATRIX_RUNS` |
| `t_s` | 0 s | No ramp in `apply_kinematic_brake` |
| One-tick mechanical lag | 0.05 s | Controller at tick k; velocity change visible from tick k+1 (inherent to discrete-time sim) |

---

## 10. YOLO Detection Table Verification (TABLE V)

**Review date:** 2026-06-25  
**Scope:** Verify TABLE V in `paper-latex3/root.tex` against code and actual results.

### TABLE V in Paper

```
TABLE V: Descriptive YOLOv8n detection counts and confidences
         on the reconstructed 3DGS scene
┌──────────────────┬──────┬──────┬──────┬──────┐
│ Pass             │ Det. │ Cars │ Mean │ Max  │
├──────────────────┼──────┼──────┼──────┼──────┤
│ Background only  │  99  │  96  │ 0.64 │ 0.88 │
│ With actor       │ 112  │ 109  │ 0.63 │ 0.92 │
└──────────────────┴──────┴──────┴──────┴──────┘
```

### Source Log Files

Two runs were made on 2026-06-13. Paper uses **Log 2** (175303):

| Value | Log 1 (174714) | Log 2 (175303) | Paper |
|---|---|---|---|
| Background: cars | 96 | 96 | 96 ✓ |
| Background: mean conf | 0.6336 → 0.63 | 0.6363 → **0.64** | 0.64 ✓ Log 2 |
| Background: max conf | 0.878 → 0.88 | 0.8793 → 0.88 | 0.88 ✓ both |
| With actor: cars | **108** | **109** | 109 ✓ Log 2 |
| With actor: mean conf | 0.6347 → 0.63 | 0.6344 → 0.63 | 0.63 ✓ both |
| With actor: max conf | 0.9044 → **0.90** | 0.9238 → **0.92** | 0.92 ✓ Log 2 |
| Traffic light conf (bg) | 0.7166 → 0.72 | 0.8174 → **0.82** | 0.82 ✓ Log 2 |

**Conclusion: All TABLE V values match Log 2 (perception_log_20260613_175303) when rounded to 2 d.p.**

### Code vs. Paper Consistency

| Paper claim | Code value | File | Match |
|---|---|---|---|
| Model: YOLOv8n | `YOLO_MODEL = "yolov8n.pt"` | `config/scenario_cutin.py` | ✅ |
| conf threshold: 0.45 | `CONF_THRESH = 0.45` | `config/scenario_cutin.py` | ✅ |
| Resolution: 1280×720 | `CAM_W=1280, CAM_H=720` | `config/scenario_cutin.py` | ✅ |
| Frames sampled: 38 | `n_frames_sampled: 38` in `.meta.json` | both log files | ✅ |
| Background: no vehicles spawned | `if pass_type == "with_actor": dart = spawn_vehicle(...)` | `perception/scene_logger.py:107` | ✅ |
| With actor: dart vehicle parked | `actors.hold(dart)` (stationary throughout pass) | `perception/scene_logger.py:113` | ✅ |
| Counts are cumulative, not unique vehicles | `detect_all(frame)` called per frame, not deduplicated | `perception/scene_logger.py:159` | ✅ |

### Why 1 Spawned Vehicle → +13 Detections

Detection counts are **cumulative across all 38 frames**, not unique vehicle counts. The dart vehicle is visible in 10 consecutive frames (frames 14–24). In some frames YOLO generates 2 bounding boxes for the same vehicle (changing camera angle). Background count of 96 uses the same cumulative method. The paper correctly states "99 detections **over** 38 sampled frames," indicating this cumulative counting.

### Observations

1. Log 1 (174714) differs from Log 2 in cars (108 vs 109) and max confidence (0.90 vs 0.92) due to minor YOLO CPU stochasticity between runs. Only Log 2 is used in the paper.
2. **Reproducibility gap:** The specific log filename/timestamp (`perception_log_20260613_175303`) is not stated in the paper. Adding this to Section IV-D would improve reproducibility.

---

## 11. Pre-Submission Code Review Notes (ICARCV 2026)

**Review date:** 2026-06-18  
**Reviewer basis:** `paper-latex/root.tex`, all source files, results CSVs, README

### Severity Legend

| Tag | Meaning |
|---|---|
| **Critical** | Factual error or internal inconsistency; likely reviewer rejection |
| **Major** | Materially weakens claims or reproducibility |
| **Minor** | Polish / clarification |

---

### Critical Issues

**[C1] Lead-brake conflict-only Rc and collision counts do not match current results data**

The paper reports (Table II):
- baseline: 25.0% conflict-only Rc, Coll.=18
- adaptive-TTC: 33.3%, Coll.=16
- required-decel: **92.0%**, Coll.=2

The current CSV (`results/lead_matrix_20260613_112347.csv`) yields different conflict-only figures. These numbers originate from a superseded run with `LEAD_DECEL = 9.0` (too strong, marked "เดิม 9.0 แรงเกินจริง"). Current code uses `LEAD_DECEL = 4.0`.

**Fix:** Re-run `run_matrix_lead.py` with current code. Decide on one consistent `LEAD_DECEL` value (4.0 for moderate, 6.0 for Euro-NCAP standard). State it explicitly in the paper. Update Table II, abstract, and conclusion.

---

**[C2] Symbol `g` used for both gap and gravitational acceleration in the same equation block**

- Eq. (1): `g` = 9.81 m/s² (gravitational)
- Eq. (2): `TTC = g / Δv_r` — here `g` = inter-vehicle gap
- Section V.C gather block: `a_max = μg` (gravitational) and `a_req = v_e² / (2*(g + d_lead))` (gap) — in the **same block**

**Fix:** Replace gap symbol with `d` (or `d_g`) throughout Eq. (2) and the required-decel derivation. Reserve `g` solely for the gravitational constant.

---

**[C3] Abstract compares cut-in overall Rc with lead-brake conflict-only Rc**

Abstract line 31: "raising the conflict-case avoidance rate to 62.5% in cut-in and 92.0% in lead-braking."
- 62.5% cut-in = overall Rc (all 32 cases are conflict cases, so these happen to coincide)
- 92.0% lead-brake = conflict-only Rc (over 25 cases, not all 32)

This comparison is misleading. The lead-brake all-32 Rc is only 71.9%.

**Fix:** Report either all-32 Rc for both scenarios ("62.5% cut-in, 71.9% lead-braking"), or conflict-only for both with a clear explanation that all cut-in cases are conflict cases.

---

### Major Issues

**[M1] LEAD_DECEL not stated in the paper**

The paper never states the lead vehicle braking deceleration. Euro-NCAP CCRb typically uses 6–9 m/s²; the code uses 4.0 m/s² (a deliberate departure that must be justified).

**Fix:** Add to Table I or Section IV.B: "Lead deceleration: X m/s²." Justify the choice.

---

**[M2] Partial brake fraction (0.4) not stated**

The paper describes two brake levels for all controllers but never states the partial brake fraction. `PARTIAL_BRAKE = 0.4` is a tuned parameter that affects outcomes.

**Fix:** Add one sentence per controller: "Partial braking applies 40% of full brake command."

---

**[M3] Only Rc presented despite CPEIM defining 5 indices**

All five CPEIM indices are logged in every CSV run. Only Rc is shown in the results tables. A reviewer from the CPEIM community will flag this as selective reporting.

**Fix:** Add a compact table with all five indices (see aggregated table in Section 6 of this document). No new simulation runs are required — this is a reporting change only.

**Full CPEIM table from existing data:**

*Cut-in / Dart-out (n = 32 cases/controller)*

| Controller | R_c | s_c (m) | a_b (m/s²) | T_c (s) | Δv_av (km/h) | Δv_col (km/h) |
|---|---|---|---|---|---|---|
| baseline | 31.2% | 1.80 [0.26–3.93] | 4.91 [3.50–7.87] | 1.33 [0.87–1.59] | 38.0 | 32.1 |
| adaptive-TTC | 46.9% | 1.95 [0.26–3.93] | 4.49 [3.50–6.81] | 1.38 [0.87–1.87] | 36.0 | 39.8 |
| required-decel | 62.5% | **0.75** [0.18–2.09] | **6.33** [3.51–9.06] | 1.21 [0.15–2.09] | 41.0 | 37.1 |

*Lead-brake / CCRb (n = 32 cases/controller)*

| Controller | R_c | s_c (m) | a_b (m/s²)† | T_c (s) | Δv_av (km/h) | Δv_col (km/h) |
|---|---|---|---|---|---|---|
| baseline | 18.8% | 1.06 [0.67–1.55] | 6.30 [3.50–8.36] | 1.53 [1.47–1.59] | 33.3 | 38.0 |
| adaptive-TTC | 25.0% | 1.04 [0.26–1.71] | 5.80 [3.50–8.36] | 1.67 [1.47–1.98] | 32.5 | 41.2 |
| required-decel | 71.9% | 0.97 [0.23–4.23] | 5.80 [3.41–8.36] | **7.20** [0.80–24.68]‡ | 41.3 | 53.9 |

> †MFDD averaged over rows where `a_b_mfdd > 0` only.  
> ‡Required-decel T_c is structurally large: braking triggers when urgency ≥ 0.6 while TTC is still far above any TTC threshold. Large T_c reflects early intervention, not late detection.

*Source files: `results/matrix_20260613_140524.csv` and `results/lead_matrix_20260613_112347.csv`.*

---

**[M4] README stated s_clearance is center-to-center distance (now fixed)**

Original README incorrectly stated "s เป็นระยะ center-to-center." Actual code (`core/runner.py`, `core/runner_lead_brake.py`) computes `s_clearance = gap = dist2d - gap_offset` where `gap_offset` accounts for vehicle dimensions — this is a **surface gap**. Now corrected.

---

**[M5] T_c for required-decel controller is structurally different and requires explanation**

For TTC-based controllers, T_c ≈ threshold value (1.5–1.7 s). For required-decel, braking triggers when `a_req/a_max ≥ 0.6`, which can occur while TTC is still very large. Actual data: required-decel T_c mean = **7.20 s**, range [0.80, 24.68] s.

**Fix:** Add a sentence in Section IV.C: "T_c is the TTC at the moment braking is initiated; for the required-deceleration controller this value is expected to be large (early intervention), whereas it is structurally bounded by the fixed thresholds for TTC-based controllers."

---

**[M6] MFDD = 0 for collision cases and partial-braking cases — undocumented**

`MfddTracker.mfdd()` returns 0.0 when ego speed does not fall to 0.1 × v₀ (i.e., all collision runs and some partial-braking runs). Averaging this column without filtering substantially understates actual braking deceleration.

**Fix:** Aggregate MFDD only over rows where `a_b_mfdd > 0`. State this filter explicitly in any table reporting `a_b`. (Already implemented in `core/metrics.py` `summarize()` and `core/report.py`.)

---

**[M7] CPEIM composite score weights come from V-VRU scenario, not car-to-car**

`W_S, W_AB, W_TC, W_DV, W_RC = 0.1447, 0.0901, 0.2962, 0.0603, 0.4087` in `core/metrics.py` are from the liu2025 pedestrian scenario. These weights are **not** appropriate for car-to-car scenarios.

**Fix:** Remove the composite score from output, or obtain the correct car-to-car weights from liu2025. A WARNING comment is already in the code. Only raw indices are reported in the paper.

---

### Notation Issues (Section 3 of review)

| # | Issue |
|---|---|
| N2 | Eq. (1) uses `t_d` and `t_s`, but the code uses `delay_frames` and has no ramp. Add: "Eq.(1) serves as physical motivation for the deceleration cap; simulation uses kinematic integration per time step." |
| N3 | "dry-surface behaviour reduces to baseline" — only partially true (speed term `k_v` also applies on dry roads). Recommend: "the μ-adaptive term engages only on wet surfaces, while a small speed-dependent term applies at all speeds." |
| N5 | `a_max = μg` — with `g` meaning gravitational constant — should be `a_max = μ × 9.81` to distinguish from `g` (gap) in the same block. |
| N7 | `Δv` for avoided cases = initial_speed (since collision_speed=0), making it non-discriminating across controllers on the same case. Note this in the paper. For collision cases, Δv = speed reduction before impact, which is informative. |

---

### Reproducibility Gaps

| # | Missing from paper | Severity |
|---|---|---|
| Rep1 | `LEAD_DECEL` value | Major |
| Rep2 | `PARTIAL_BRAKE = 0.4` | Major |
| Rep3 | Perception log filename; YOLO CPU stochasticity (≤1 car difference between runs) | Minor |
| Rep4 | UE5.5 build version | Minor |
| Rep5 | YOLOv8n checkpoint version/hash | Minor |
| Rep7 | `INPATH_MAX_RANGE`: 40 m (cut-in) vs 80 m (lead-brake) | Minor |
| Rep8 | `LEAD_BRAKE_AFTER_M = 8.0 m` | Minor |

---

### Reviewer Red Flags

| # | Flag | Recommendation |
|---|---|---|
| RF1 | 92% conflict-case avoidance headline is not reproducible from current code | Re-run with current code; update all claims |
| RF2 | Baseline achieves 0% wet avoidance in both scenarios — may look deliberately weak | Add: "At μ=0.40, stopping distance exceeds available gap by construction at TTC≤0.6 s" |
| RF3 | 5 CPEIM indices logged, only Rc reported | Add compact CPEIM table (see M3 above) |
| RF4 | Range / closing speed are CARLA ground-truth, not sensor | Add one-clause caveat in abstract: "…using CARLA ground-truth range inputs…" |
| RF6 | 3DGS contribution only via YOLO; YOLO is gated by ground-truth for braking | Tighten claim: "feasibility that a 3DGS-reconstructed scene can serve as substrate for repeatable, discriminative AEB evaluation" |

### Top 3 Things to Fix Before Submission

1. **[C1 + C3]** Re-run lead-brake with current code; update Table II, abstract, and conclusion with consistent Rc basis (all-cases or conflict-only — apply the same basis to both scenarios).
2. **[C2]** Change gap symbol from `g` to `d` throughout (10-minute LaTeX edit).
3. **[M3]** Add aggregated CPEIM index means. The T_c column (7.2 s for required-decel vs 1.5 s for TTC controllers) provides a mechanistic explanation of *why* required-decel outperforms — early intervention, not tuning.

---

## 12. Code Revision History

### New Files Added

#### `core/conflict.py`

**Purpose:** Deterministic "conflict case" definition from kinematics — no CARLA run needed.

A case is a conflict case if ego at constant speed would collide within `MAX_TICKS × FIXED_DT` seconds. Definition is:
- Computed before simulation, before spawning actors
- Identical for all three controllers
- Not post-hoc

API:
```python
lead_brake_is_conflict(case, cfg) -> bool
cutin_is_conflict(case, cfg)      -> bool
```

---

#### `core/report.py`

**Purpose:** Read existing CSV → print CPEIM table per controller. Does not run simulation, does not modify CSV.

```bash
python core/report.py results/matrix_*.csv
python core/report.py results/lead_matrix_*.csv
```

Required columns: `label`, `avoided`, `is_conflict`, `a_b_mfdd`, `s_clearance`, `t_c_warn`, `dv_speed_var`

---

#### `tools/check_conflict.py`

**Purpose:** Compute `is_conflict` for every case in both matrices (pure kinematics, no CARLA).

```bash
python tools/check_conflict.py
LEAD_DECEL=6.0 python tools/check_conflict.py
```

---

### Modified Files

#### `core/metrics.py`

1. Added `is_conflict: bool = True` field to `RunRecord` (default `True` for cut-in; written to CSV via `asdict()`)
2. Removed `mean_score` (composite score with wrong V-VRU weights)
3. Fixed aggregation in `summarize()`:
   - `rc_all` = avoidance rate over all cases
   - `rc_conflict` = avoidance rate over `is_conflict=True` cases only
   - `mean_ab` = average over `a_b_mfdd > 0` only
   - `mean_sc` = average over `avoided=True` only
   - `mean_tc` = average over `t_c_warn > 0` only
   - `mean_dv` = average over all rows

Weight constants `W_S/W_AB/W_TC/W_DV/W_RC` retained with a WARNING comment (for reference only).

---

#### `core/runner.py` (cut-in)

Added `is_conflict` computation before the `try:` block so it is recorded even if actor spawn fails:
```python
from core.conflict import cutin_is_conflict
is_conflict = cutin_is_conflict(case, cfg)
rec = RunRecord(..., is_conflict=is_conflict)
```

---

#### `core/runner_lead_brake.py`

Added `is_conflict: bool = True` to `LeadBrakeRecord`. `headway_d` must be computed from THW first, then passed to `lead_brake_is_conflict`:
```python
from core.conflict import lead_brake_is_conflict
_case_for_conflict = {**case, "headway_d": headway_d}
is_conflict = lead_brake_is_conflict(_case_for_conflict, cfg)
rec = LeadBrakeRecord(..., is_conflict=is_conflict)
```

---

#### `config/scenario_lead_brake.py`

Added `import os`. Changed `LEAD_DECEL` to be overridable via environment variable:
```python
# Before:
LEAD_DECEL = 4.0

# After:
LEAD_DECEL = float(os.environ.get("LEAD_DECEL", "4.0"))
```

Code default remains 4.0. Paper runs use `LEAD_DECEL=6.0 python run_matrix_lead.py`.

---

#### `run_matrix.py` and `run_matrix_lead.py`

- Removed `mean_score` from summary printout
- Added `Rc_conflict` and 4 raw CPEIM indices with row counts used for averaging

---

### Matrix Widening — June 2026

**Purpose:** Expand test matrix domain for research credibility. This is not data selection — all results are reported as-is, including hard cases.

**Academic integrity:** All added values are domain-grounded; both low-speed and longer-reaction ends are expanded; all original hard cases are retained.

| Scenario | Variable | Old values | New values |
|---|---|---|---|
| Cut-in | `ego_speed_kmh` | [30, 40, 50, 60] | [**20**, 30, 40, 50, 60] |
| Cut-in | `trigger_d` (m) | [20, 25, 30, 35] | [20, 25, 30, 35, **40**] |
| Lead-brake | `ego_speed_kmh` | [30, 40, 50, 60] | [**20**, 30, 40, 50, 60] |
| Lead-brake | `HEADWAY_THW` (s) | [1.0, 1.5, 2.0, 2.5] | [1.0, 1.5, 2.0, 2.5, **3.0**] |

| Scenario | Old count | New count |
|---|---|---|
| Cut-in (speed × μ × trigger_d) | 4×2×4 = **32** | 5×2×5 = **50** |
| Lead-brake (speed × THW × μ) | 4×4×2 = **32** | 5×5×2 = **50** |
| Total runs (× 3 controllers) | 96/scenario | **150/scenario** |

**Conflict check results (matrix widened, LEAD_DECEL=4.0):**

| Scenario | Cases | Conflict | No-conflict | Boundary case |
|---|---|---|---|---|
| Cut-in | 50 | **50** | 0 | 20 km/h + 40 m → t ≈ 7.1 s ≤ 20 s |
| Lead-brake | 50 | **50** | 0 | 20 km/h + 3.0 s THW → t ≈ 5.1 s ≤ 20 s |

---

### Unchanged Elements

The following were **not modified** in any revision:
- Controller decision logic (`baseline_static_ttc`, `proposed_dynamic_ttc`, `proposed_enhanced`)
- MFDD formula in `MfddTracker`
- Kinematic brake cap (`apply_kinematic_brake`, `peak_decel ≤ a_max`)
- Existing CSV column names (only new columns appended; no old columns changed)
- `FRAME_SYNC = True` and `FIXED_DT = 0.05` (simulation determinism)
- Cut-in / lead-brake scene logic
- Existing result files in `results/` (read-only)

---

### Degradation Layer — July 2026

**Purpose:** Add latency and sensor noise as independent test variables without modifying any controller logic. All three controllers receive the same degraded input per run (fairness preserved).

#### New files

| File | Purpose |
|---|---|
| `perception/degrade.py` | `PerceptionDegrader` — latency buffer + Gaussian noise + dropout |
| `tests/test_degrade.py` | Unit tests (no CARLA): no-op, delay, determinism, dropout modes |
| `CHANGES_degradation.md` | Change summary for this feature |

#### Modified files

| File | Change | Lines added/changed |
|---|---|---|
| `core/metrics.py` | Add 6 fields to `RunRecord` (degradation params + seed) | +6 |
| `core/runner_lead_brake.py` | Add 6 fields to `LeadBrakeRecord`; import + wire degrader; det_buffer no-op | +23 |
| `core/runner.py` | Import + wire degrader; det_buffer no-op; pass degraded perc to controller | +22 |
| `config/scenario_cutin.py` | `import os`; `CONTROLLERS`; sweep constants; `build_matrix_runs(mode)`; dynamic `MATRIX_RUNS` | +40 |
| `config/scenario_lead_brake.py` | Same additions as cutin config | +39 |
| `run_matrix.py` | Use `cfg.build_matrix_runs(test_mode)`; pass `run_spec/case_idx/test_mode` | +6 |
| `run_matrix_lead.py` | Same as run_matrix.py | +6 |
| `README.md` | Add Test Modes section | +28 |
| `TECHNICAL_DOC.md` | Update Sections 4, 7, 9, 12 | +30 |

**Controller files not touched:** `baseline_static_ttc.py`, `proposed_dynamic_ttc.py`, `proposed_enhanced.py` — verified with `git diff HEAD`.

**Original mode unchanged:** `TEST_MODE=original` (default) → `PerceptionDegrader` with all params = 0 is a strict no-op; output equals original code on all existing test cases.

---

### Latency Compensation — July 2026

**Purpose:** Add two controllers that compensate a known perception latency `L` to recover
the Rc lost under latency, plus test modes measuring the upper bound (L known exactly) and
fragility (L mis-estimated). No controller decision logic changed; no CARLA/matrix run
performed (user runs on server).

#### New files

| File | Purpose |
|---|---|
| `control/enhanced_predictive.py` | `@register("enhanced_predictive")` — feedforward predictor; extrapolates inputs, reuses `required_decel()` |
| `control/enhanced_inflation.py` | `@register("enhanced_inflation")` — threshold inflation (`v_close·L`); worst-case robust |
| `tests/test_latency_comp.py` | Unit tests (mock `carla`): L=0 reduction, L-monotonicity, mismatched, matrix-mode counts |
| `CHANGES_latency_compensation.md` | Change summary for this feature |

#### Modified files

| File | Change | Lines added/changed |
|---|---|---|
| `control/base_controller.py` | `make_controller(name, cfg, run_spec=None)` attaches `ctrl.run_spec`; add `compensation_latency()` helper | +~25 |
| `core/runner.py` | Import 2 controllers; pass `run_spec=spec` to `make_controller`; record `comp_source`/`comp_L_frames` | +~7 |
| `core/runner_lead_brake.py` | Same as runner.py + 2 fields on `LeadBrakeRecord` | +~9 |
| `core/metrics.py` | Add `comp_source`, `comp_L_frames` to `RunRecord` (defaults `""`/`0`) | +2 |
| `config/scenario_cutin.py` | `COMP_*` constants + `latency_comp`/`latency_mismatch` in `build_matrix_runs` | +~30 |
| `config/scenario_lead_brake.py` | Same additions as cutin config | +~30 |
| `README.md` | Controllers table + 2 Test Modes + CSV columns | +~25 |
| `TECHNICAL_DOC.md` | §5 latency-comp subsection, §7 CSV columns, §12 this entry | +~70 |

**Design notes.** `enhanced_predictive` extrapolates the *inputs* to `required_decel()`
(so `L=0` reduces to `proposed_enhanced` exactly, verified per-tick). Closing acceleration is
`+lead_decel` (lead braking → gap closes faster). `L` is delivered through the run spec only —
controllers never read `PerceptionDegrader` state (loose coupling).

**Controller files not touched:** `baseline_static_ttc.py`, `proposed_dynamic_ttc.py`,
`proposed_enhanced.py` — verified with `git diff` (empty). Existing CSV columns unchanged;
`TEST_MODE=original/latency/noise` produce identical results (their run specs carry no
`comp_*` keys).

#### How to verify the L=0 reduction (no CARLA)

```bash
python3 tests/test_latency_comp.py   # 7 tests, incl. predictive@L=0 == proposed_enhanced per tick
```

---

### Predictive-Oracle Compensation Across All Base Controllers — July 2026

**Purpose:** Add `TEST_MODE=latency_comp_all` — apply predictive-oracle compensation
(exact latency `L` known) as a preprocessing *layer* in front of **every** base controller,
then sweep latency, so controllers can be compared at the same latency. No controller
decision logic changed; no CARLA/matrix run performed (user runs on server). See §5.

#### New files

| File | Purpose |
|---|---|
| `perception/predict.py` | `PerceptionPredictor` — feedforward predictor layer; extrapolates the Perception fields forward by `L` (mirror of `PerceptionDegrader`). L=0 → strict identity. |
| `tests/test_latency_comp_all.py` | Unit tests (mock `carla`): L=0 identity per controller; predictor-layer + `proposed_enhanced` == `enhanced_predictive` per tick (L ∈ {0,4,8,16}, both scenarios); gap/ttc monotonicity in L; run-count check; old modes unchanged. |
| `CHANGES_latency_comp_all.md` | Change summary for this feature. |

#### Modified files

| File | Change | Reason |
|---|---|---|
| `config/scenario_cutin.py` | `+15` — `latency_comp_all` branch in `build_matrix_runs` (+ docstring) | Build the run set `CONTROLLERS × LATENCY_DELAY_FRAMES`, all `comp_source="oracle"`, `comp_L_frames=delay_frames`, `predict=True`. |
| `config/scenario_lead_brake.py` | `+15` — same as cutin config | Same branch for the lead-brake matrix. |
| `core/runner.py` | `+14/−1` — import `PerceptionPredictor` + `compensation_latency`; build `predictor` when `spec["predict"]`; apply it after the degrader | Insert the prediction layer between degrader and controller for `latency_comp_all` only (no key → no-op elsewhere). |
| `core/runner_lead_brake.py` | `+14/−1` — same as runner.py | Same wiring for the lead-brake runner. |
| `run_matrix.py` | `+4/−1` — embed `test_mode` in the CSV filename | `matrix_<TEST_MODE>_<stamp>.csv`; consistent, self-describing filenames. |
| `run_matrix_lead.py` | `+4/−1` — same as run_matrix.py | `lead_matrix_<TEST_MODE>_<stamp>.csv`. |
| `README.md` | Test Modes table + run commands + filename note | Document the new mode and the filename convention. |
| `TECHNICAL_DOC.md` | §5 predictive-layer subsection + §12 this entry | Describe the layer, the L=0 / consistency guarantees, and the revision. |

**Design notes.** Prediction is now a **layer** symmetric with degradation (`degrade` makes
input stale, `predict` makes it fresh) rather than controller-internal logic. It uses the
same constant-acceleration model as `enhanced_predictive`; because `proposed_enhanced` reads
only the fields the predictor rewrites, `layer + proposed_enhanced` reproduces
`enhanced_predictive` per tick (verified — no math difference to escalate). The layer is
gated on `spec["predict"]`, set only by `latency_comp_all`, so all prior modes are byte-for-byte
unchanged. No CSV columns renamed/removed (uses existing `comp_source`/`comp_L_frames`/`test_mode`).

**Controller files not touched:** `baseline_static_ttc.py`, `proposed_dynamic_ttc.py`,
`proposed_enhanced.py`, `enhanced_predictive.py`, `enhanced_inflation.py` — verified with
`git diff` (empty). MFDD, kinematic brake cap, `FIXED_DT`, and scene logic unchanged.

#### How to verify (no CARLA)

```bash
python3 tests/test_latency_comp_all.py   # L=0 identity, enhanced_predictive consistency, run-count
```

---

### Rev 2026-07-22 — CCRs + Cut-out scenarios on train000

Added two new test scenarios running on the `train000` 3DGS scene. All existing `scene03_2` scenarios (`cut-in`, `lead-brake`) are byte-for-byte unchanged.

#### New files

| File | Purpose |
|---|---|
| `config/scenario_ccrs.py` | CCRs config: EGO/TARGET spawns, EXPECTED_SCENE, SPECTATOR_TF, 5×3×2=30 matrix |
| `config/scenario_cutout.py` | Cut-out config: EGO/TARGET/LEAD spawns, cut-out constants, 5×3×2=30 matrix |
| `core/scenario_ccrs.py` | `CCRsScenario`: target always stationary (`hold()`) |
| `core/scenario_cutout.py` | `CutOutScenario`: lead cruises → cuts right when ≤ `CUTOUT_TRIGGER_D` from target |
| `core/runner_ccrs.py` | `CCRsRecord` + `run_case()` for CCRs |
| `core/runner_cutout.py` | `CutOutRecord` + `run_case()` for Cut-out |
| `run_single_ccrs.py` | Entry point: 1 CCRs case with display |
| `run_matrix_ccrs.py` | Entry point: CCRs matrix → `results/ccrs_matrix_*.csv` |
| `run_single_cutout.py` | Entry point: 1 cut-out case with display |
| `run_matrix_cutout.py` | Entry point: cut-out matrix → `results/cutout_matrix_*.csv` |

#### Modified files (additive only — no existing code paths changed)

| File | Change |
|---|---|
| `core/actors.py` | Added `set_spectator()` and `check_scene()` helpers at end of file |
| `core/conflict.py` | Added `_kinematic_conflict_stationary_target()`, `ccrs_is_conflict()`, `cutout_is_conflict()` |
| `config/scenario_cutin.py` | Added `EXPECTED_SCENE = "scene03_2"`, `SPECTATOR_TF = dict(...)` |
| `config/scenario_lead_brake.py` | Same additions |
| `run_single.py` | Added `actors.check_scene()` + `actors.set_spectator()` after session open |
| `run_matrix.py` | Same + `MATRIX_VIZ=1` opt-in viz (default=None, existing behaviour unchanged) |
| `run_single_lead.py` | Added `check_scene` + `set_spectator` |
| `run_matrix_lead.py` | Same + `MATRIX_VIZ` opt-in viz |
| `tools/check_conflict.py` | Added `check_ccrs()` + `check_cutout()` sections |
| `README.md` | Added new scenarios, run commands, matrix tables, spectator, scene check, viz docs |
| `TECHNICAL_DOC.md` | This entry |

#### Conflict formula (train000)

Both CCRs and Cut-out use the stationary-target formula (conflict against unbraked ego):

```
dist         = hypot(EGO_SPAWN.x − TARGET_SPAWN.x, EGO_SPAWN.y − TARGET_SPAWN.y) ≈ 49.2 m
surface_gap  = dist − GAP_OFFSET                                                  ≈ 44.7 m
t_conflict   = surface_gap / v_ego
is_conflict  = t_conflict ≤ MAX_TICKS × FIXED_DT (20 s)
```

All 30 CCRs and all 30 cut-out matrix cases are conflict cases (worst: CCRs 60m@20km/h → t ≈ 10.0 s; cut-out 20km/h → t ≈ 7.9 s; both < 20 s).

#### Geometry generalisation (forward-vector lead spawn)

The existing axis-aligned formula `lead_y = EGO_SPAWN.y − headway_d` is the `yaw=−90°` special case of the general formula used in `runner_cutout.py`:

```python
yaw_rad = math.radians(cfg.EGO_SPAWN["yaw"])
lead_x  = cfg.EGO_SPAWN["x"] + headway_d * math.cos(yaw_rad)
lead_y  = cfg.EGO_SPAWN["y"] + headway_d * math.sin(yaw_rad)
```

Verification: yaw=−90° → cos=0, sin=−1 → lead_x=EGO.x, lead_y=EGO.y−headway_d ✓ (matches existing code).

#### End-of-run condition (direction-agnostic)

Replaces the axis-aligned `ego_y < END_Y` guard used in scene03_2:

```python
dist_from_start = math.hypot(ego.x − ego_x0, ego.y − ego_y0)
if dist_from_start > initial_ego_target_dist + 15.0:
    result_txt = "NO BRAKE / passed"; break
```

Computed from actual spawn coordinates — no manual tuning when coordinates change.

#### Configurable viz in matrix runners

```python
viz_enabled = os.environ.get("MATRIX_VIZ", "0") == "1"
viz = Viz(cfg) if viz_enabled else None
```

Default = `"0"` → `viz=None` (identical to all prior matrix runs). Enable: `MATRIX_VIZ=1 python run_matrix*.py`.

#### Existing behaviour guarantee

- `core/conflict.py` existing functions: `cutin_is_conflict()`, `lead_brake_is_conflict()` — unchanged.
- All `scene03_2` runner/config/scenario files: byte-for-byte identical to previous revision.
- `EXPECTED_SCENE = ""` would skip the check (no-op); the actual values set are `"scene03_2"` / `"train000"`, so the guard fires only if the wrong map is loaded.

#### How to verify (no CARLA)

```bash
python3 tools/check_conflict.py        # 160/160 conflict — all 4 scenarios
python3 -m py_compile core/runner_ccrs.py core/runner_cutout.py  # syntax check
```

---

---

### Rev 2026-07-22b — Cut-out lead: physics-based steering (replaces kinematic velocity injection)

#### Problem

The previous cut-out implementation drove the lead vehicle using `set_target_velocity()` every tick — both during the cruise phase and the lateral cut-out. This bypasses CARLA's rigid-body dynamics (no wheel steering, no tire slip) and produces a "floating/sliding" appearance where the car moves sideways without any wheel turn.

#### Root cause

`set_target_velocity()` directly overwrites the physics-engine velocity state each tick. CARLA's VehicleControl path (throttle / steer / brake → engine torque → wheel forces → rigid body) is never exercised, so the car has no visible steering arc.

#### Fix

The cut-out lead is now driven **entirely via `apply_control(VehicleControl(...))`** from spawn to end-of-run. The full control lifecycle:

| Phase | Trigger | Lead control |
|---|---|---|
| `CRUISE` | always until `dist(lead,target) ≤ CUTOUT_TRIGGER_D` | `VehicleControl(throttle, steer=0, brake)` — P-speed controller |
| `STEER` | trigger fires | `VehicleControl(throttle, steer, brake=0)` — P-heading controller targeting `trigger_yaw + CUTOUT_HEADING_DEG` |
| `STRAIGHTEN` | lateral offset ≥ `CUTOUT_LANE_WIDTH` | Same heading P-controller, target = `trigger_yaw` (straighten back) |
| `SETTLED` | `|heading error| < CUTOUT_SETTLE_DEG` | Cruise in right lane (`CUTOUT_AFTER_STOP=False`) or brake to stop |

**Velocity boot at start():** one `set_target_velocity()` call is made in `start()` to kick the physics engine to the correct forward speed before the main loop begins. After that, no `set_target_velocity` / `set_transform` is applied to the lead.

**Physics enabled:** `lead.set_simulate_physics(True)` called explicitly in `start()` (defensive; CARLA default is True for spawned vehicles).

#### Files changed

| File | Change |
|---|---|
| `core/scenario_cutout.py` | Full rewrite — 4-phase state machine, closed-loop heading and speed P-controllers, `VehicleControl` only in main loop |
| `config/scenario_cutout.py` | Replaced 3 kinematic params (`CUTOUT_LATERAL_SPEED`, `CUTOUT_FORWARD_SPEED`, `CUTOUT_TRAVEL_M`) with 9 physics-tuning constants (see below) |

**Files NOT changed:** `core/runner_cutout.py`, `core/actors.py`, all other scenario files, all entry points, all metrics.

#### New config constants (`config/scenario_cutout.py`)

| Constant | Default | Meaning |
|---|---|---|
| `CUTOUT_TRIGGER_D` | `10.0` m | Lead-to-target distance at which cut-out begins (unchanged) |
| `CUTOUT_LANE_WIDTH` | `3.5` m | Lateral displacement (in lead's right-frame) to consider the right lane reached |
| `CUTOUT_HEADING_DEG` | `30.0` ° | Target yaw offset during STEER phase. Positive = steer right in CARLA convention. **Negate to −30 if the lead turns left.** |
| `CUTOUT_STEER_K` | `0.05` | P-gain: steer per degree of heading error |
| `CUTOUT_STEER_MAX` | `0.4` | Hard clamp on `|steer|` (0–1) |
| `CUTOUT_SETTLE_DEG` | `5.0` ° | Heading error threshold to exit STRAIGHTEN → SETTLED |
| `CUTOUT_AFTER_STOP` | `False` | `True` = lead brakes to a stop in right lane; `False` = keeps cruising |
| `LEAD_SPEED_K` | `0.5` | P-gain: throttle/brake per m/s speed error |
| `LEAD_SPEED_MAX_THROTTLE` | `0.6` | Max throttle command (0–1) |

#### Reveal-timing note

A real steered arc takes longer to move the lead laterally than a velocity injection at the same trigger distance. The stationary target will be revealed to the ego **later** (smaller gap) than before. To restore the reveal distance: decrease `CUTOUT_TRIGGER_D`, or increase `CUTOUT_HEADING_DEG` / `CUTOUT_STEER_MAX` for a faster/tighter arc.

#### Tuning guide

| Symptom | Likely cause | What to change |
|---|---|---|
| Lead turns **left** instead of right | CARLA yaw convention opposite to assumed | Set `CUTOUT_HEADING_DEG = -30.0` |
| Lead **barely moves** into right lane | Steer gain too low or heading target too small | Increase `CUTOUT_STEER_K` (e.g. 0.08) or `CUTOUT_HEADING_DEG` (e.g. 45) |
| Lead **oscillates** (twitches) during steer | Steer gain too high | Decrease `CUTOUT_STEER_K` (e.g. 0.03) |
| Lead **overshoots** the right lane | Heading offset or lane width wrong | Decrease `CUTOUT_HEADING_DEG` or increase `CUTOUT_LANE_WIDTH` |
| Lead **drifts off target speed** | Speed gain too low | Increase `LEAD_SPEED_K` (e.g. 0.8) |
| Lead **oscillates** throttle/brake | Speed gain too high | Decrease `LEAD_SPEED_K` (e.g. 0.3) |
| Cut-out starts **too early/late** | Trigger distance wrong | Adjust `CUTOUT_TRIGGER_D` |
| Reveal happens **too close** to ego | Manoeuvre too slow | Decrease `CUTOUT_TRIGGER_D` or increase `CUTOUT_HEADING_DEG` |

#### Existing behaviour guarantee

- `core/scenario_cutin.py`, `core/scenario_lead_brake.py`, `core/scenario_ccrs.py` — **byte-for-byte unchanged**.
- `core/actors.py` (`cruise`, `hold`, `speed_ms`, etc.) — **unchanged**.
- `core/runner_cutout.py` — **unchanged** (all lead control is inside `CutOutScenario.update()`).
- Conflict formula, CSV schema, metrics, CPEIM indices — **unchanged**.
- `CUTOUT_TRIGGER_D` key is preserved, so existing `SINGLE_CASE` dicts are valid.

#### Verification (no CARLA)

```bash
python3 -m py_compile core/scenario_cutout.py config/scenario_cutout.py
# Confirm no set_transform / positional teleport on lead in main loop:
grep "set_transform\|set_target_velocity" core/scenario_cutout.py
# → only one set_target_velocity in start() (the velocity boot); none in update()
```

*Last updated: 2026-07-22.*

---

### Rev 2026-07-22c — Detection-box colour semantics + cut-out occlusion gate

#### Part A — Detection box colour (all scenarios, cosmetic only)

**Problem:** the hazard box in `core/viz.py` was drawn orange when the target entered the ego corridor but braking had not started, making the display look like a hazard state even during normal cruise.

**Fix:** `core/viz.py:73` — changed the non-braking hazard colour from orange `(0,165,255)` to green `(0,200,0)`. The braking colour (red `(0,0,255)`) is unchanged.

| Hazard box colour | Condition |
|---|---|
| **Green** | Target in ego corridor (`in_path=True`); `brake_cmd = 0` |
| **Red** | Target in ego corridor **and** controller actively braking (`brake_cmd > 0`) |

This is display-only — `Perception`, the braking decision, `is_conflict`, and all CSV values are unchanged. The `hazard["engaged"]` field (= `brake_engaged` latch, set on first `ctrl.brake > 0`) already encodes the correct condition; only the colour constant changed.

Affects all four runners identically since they all call the same `Viz._draw_hazard()`.

#### Part B — Cut-out occlusion gate (cut-out only)

**Problem:** with `DETECTION_SOURCE="groundtruth"`, `detected_now = actors.inpath_hazard(ego, target)`. This function checks only longitudinal range and lateral offset — it has no knowledge of the lead vehicle. The stationary target is directly ahead from tick 0, so `detected_now=True` before the lead has moved aside. The AEB arms while the target is still fully occluded.

**Root cause:** `core/actors.inpath_hazard()` is a two-actor function (ego, hazard). It cannot check whether a third actor (lead) blocks the sight line. With `DETECTION_SOURCE="groundtruth"` this means occlusion is invisible to the detection gate in the cut-out scenario.

**Fix — Option 2 (ground-truth + line-of-sight gate):**

New pure-geometry module `core/occlusion.py` (no CARLA dependency):

```python
def sight_line_occluded(lead_lon, lead_lat, target_lon, target_lat, lat_clear, lon_margin):
    between = (0.0 < lead_lon < target_lon - lon_margin)
    return between and (abs(lead_lat - target_lat) < lat_clear)
```

In `core/runner_cutout.py`, after the `DETECTION_SOURCE` block, the gate overrides `detected_now` to `False` while the lead blocks the sight line:

```python
if getattr(cfg, "OCCLUSION_GATE", False) and detected_now:
    _, lead_lon, lead_lat = actors.inpath_hazard(
        ego, lead, cfg.INPATH_MAX_RANGE + 50.0, 999.0, 0.0)
    if actors.sight_line_occluded(lead_lon, lead_lat, lon, lat,
                                   cfg.OCCLUSION_LAT_CLEAR, cfg.OCCLUSION_LON_MARGIN):
        detected_now = False
```

The gate is applied **before** `PerceptionDegrader`, so degradation layers (latency, noise, dropout) operate on the already-gated value. This composes correctly: a `dropout_mode="miss"` on a gated `False` stays `False`; a latency delay on the gate-open event delays the first `True` perception, as intended.

**New config keys** in `config/scenario_cutout.py`:

| Key | Default | Meaning |
|---|---|---|
| `OCCLUSION_GATE` | `True` | Enable the gate; set `False` for old always-detected behaviour |
| `OCCLUSION_LAT_CLEAR` | `1.5` m | Lateral clearance (lead − target in ego frame) before target is visible `[TO BE TUNED]` |
| `OCCLUSION_LON_MARGIN` | `2.0` m | Lead is no longer "in front of" target once `lead_lon >= target_lon − margin` `[TO BE TUNED]` |

**Scope protection:**
- Cut-in, lead-brake, CCRs runners: byte-for-byte unchanged.
- CCRs has no lead occluder; `inpath_hazard` result is correct as-is.
- `core/occlusion.sight_line_occluded` re-exported via `actors.sight_line_occluded` for call-site compatibility.

#### Verification (no CARLA)

```bash
# Unit-test the geometry (9 cases, no CARLA needed)
python3 tests/test_occlusion_gate.py

# Syntax check all changed files
python3 -m py_compile core/viz.py core/actors.py core/occlusion.py \
    core/runner_cutout.py config/scenario_cutout.py
```

*Last updated: 2026-07-22.*

---

### Rev 2026-07-23 — Cut-out matrix redesign + CCRs approach-distance axis

#### Cut-out: replace headway sweep with reveal_ttc

**Motivation:** The old `headway_thw` sweep axis controlled how close ego was to the lead, but did not directly control the difficulty metric that matters: TTC to the stationary target at the moment it is revealed. `reveal_ttc` controls this directly.

**Changes:**

| What | Old | New |
|---|---|---|
| Matrix primary axis | `headway_thw` 5 values | `reveal_ttc` 3 values (TTC at cut-out trigger) |
| Lead headway | swept (1.0–3.0 s) | fixed `FIXED_HEADWAY_THW = 1.5 s` |
| `cutout_trigger_d` | single config constant | derived per case (see formula below) |
| Cases/controller | 50 (5×5×2) | 30 (5×3×2) |
| New CSV columns | — | `reveal_ttc`, `headway_d`, `range_at_reveal`, `ttc_at_reveal`, `time_reveal_to_brake` |

**Closed-form trigger distance:** Since ego and lead cruise at the same speed until the trigger fires, the ego→target distance at the trigger tick equals `headway_d + cutout_trigger_d`. Setting this equal to `reveal_ttc × ego_ms` (desired TTC) plus `GAP_OFFSET` gives:

```
headway_d        = FIXED_HEADWAY_THW × ego_ms
cutout_trigger_d = (reveal_ttc − FIXED_HEADWAY_THW) × ego_ms + GAP_OFFSET
```

Verified positive for all 30 cases (min: 20 km/h, reveal_ttc=1.5 s → trigger=4.5 m). *Superseded by Rev 2026-07-23b — see below.*

**Diagnostic columns recorded in the CSV:**

| Column | Description |
|---|---|
| `range_at_reveal` | Surface gap (m) at the first tick the target becomes detectable (occlusion gate opens) |
| `ttc_at_reveal` | TTC (s) at that tick; −1 if never revealed |
| `time_reveal_to_brake` | Elapsed time (s) from first reveal tick to brake onset; includes all perception latency. −1 if no brake onset after reveal. |

These are populated in `runner_cutout.py` and summarised per controller by `core/report.py`.

#### CCRs: add approach-distance axis

**Motivation:** The previous 5×2=10 matrix had a fixed target position and therefore fixed TTC-at-spawn per speed. Adding `approach_d` sweeps TTC-at-spawn systematically, mirroring the Euro-NCAP spirit of varying the approach distance.

**Changes:**

| What | Old | New |
|---|---|---|
| Target position | Fixed `TARGET_SPAWN` | Computed per case: `EGO_SPAWN + approach_d × forward_vector` |
| `approach_d` values (m) | — | 30, 45, 60 (TTC@spawn: 1.5–10 s) |
| Cases/controller | 10 (5×2) | 30 (5×3×2) |
| New CSV column | — | `approach_d` |

**Target spawn formula:**
```python
yaw_rad  = math.radians(cfg.EGO_SPAWN["yaw"])        # −146.54° → forward ≈ (−0.835, −0.551)
target_x = EGO_SPAWN["x"] + approach_d × cos(yaw_rad)
target_y = EGO_SPAWN["y"] + approach_d × sin(yaw_rad)
```
Passed as `target_x`/`target_y` in the case dict; `runner_ccrs.py` uses them when present (falls back to `cfg.TARGET_SPAWN` for single-case / legacy runs).

**Conflict check:** All 30 CCRs cases are conflict cases (worst: 60 m @ 20 km/h → t=10.0 s < 20 s window). *Superseded by Rev 2026-07-23b — see below.*

#### Files changed

| File | Change |
|---|---|
| `config/scenario_cutout.py` | Removed `HEADWAY_THW`, added `FIXED_HEADWAY_THW`, `REVEAL_TTC`; updated `SINGLE_CASE`, `MATRIX` |
| `config/scenario_ccrs.py` | Added `APPROACH_DISTANCES`; updated `MATRIX` |
| `core/runner_cutout.py` | `CutOutRecord`: removed `headway_thw`, added `reveal_ttc`, `headway_d`, 3 diagnostic fields. `run_case()`: derive `headway_d`/`cutout_trigger_d` from `reveal_ttc`; capture reveal tick + gap + TTC + brake lag |
| `core/runner_ccrs.py` | `CCRsRecord`: added `approach_d`. `run_case()`: spawn target from per-case `target_x`/`target_y` |
| `run_matrix_cutout.py` | `build_cases()`: iterate `ego_speed × reveal_ttc × mu`; derive `headway_d` + `cutout_trigger_d` |
| `run_matrix_ccrs.py` | `build_cases()`: iterate `ego_speed × approach_d × mu`; compute target spawn coords |
| `tools/check_conflict.py` | `check_ccrs()`: add `approach_d` axis, compute target position per case. `check_cutout()`: replace `HEADWAY_THW` iteration with `reveal_ttc` |
| `core/conflict.py` | `ccrs_is_conflict()`: reads `target_x`/`target_y` from case (backward-compatible fallback) |
| `core/report.py` | `print_summary()`: print reveal-diagnostic table when cut-out diagnostic columns present |

#### Scope protection
- Cut-in and lead-brake: no changes; byte-for-byte identical results.
- `cutout_is_conflict()` in `conflict.py`: unchanged (uses fixed `cfg.TARGET_SPAWN`; reveal_ttc does not affect conflict).
- All new CSV columns default to `0.0` / `−1.0`; existing analysis scripts reading old CCRs CSVs are unaffected.

#### Verification (no CARLA)

```bash
python3 tools/check_conflict.py
# Expected at that revision: cut-in 50/50, lead-brake 50/50, CCRs 30/30, cut-out 30/30 = 160 conflict

python3 tests/test_occlusion_gate.py
# Expected: 9/9 passed

python3 -m py_compile core/runner_cutout.py core/runner_ccrs.py \
    run_matrix_cutout.py run_matrix_ccrs.py tools/check_conflict.py core/report.py
```

*Last updated: 2026-07-23.*

---

### Rev 2026-07-23b — Cut-out 5×5×2 = 50 cases; CCRs 5×5×2 = 50 cases

**Motivation:** Expand both train000 matrices to 50 cases/controller (matching cut-in and lead-brake) for consistency and finer resolution. For cut-out, finer resolution comes from adding harder `reveal_ttc` values (`reveal_ttc = 1.0 s`), not from re-introducing a headway sweep. For CCRs, two additional `approach_d` values extend TTC-at-spawn coverage toward longer reaction times.

Cut-in and lead-brake results are **unchanged** (byte-for-byte identical).

#### Cut-out: 5 reveal_ttc values, lower FIXED_HEADWAY_THW

| What | Old (Rev 2026-07-23) | New |
|---|---|---|
| `REVEAL_TTC` | `[1.5, 2.0, 2.5]` s | `[1.0, 1.5, 2.0, 2.5, 3.0]` s |
| `FIXED_HEADWAY_THW` | `1.5` s | **`0.8` s** |
| Cases/controller | 30 (5×3×2) | **50 (5×5×2)** |

**Why lower `FIXED_HEADWAY_THW` to 0.8 s?**  
The constraint `cutout_trigger_d ≥ GAP_OFFSET > 0` requires `reveal_ttc ≥ FIXED_HEADWAY_THW`. Adding `reveal_ttc = 1.0` with the old `FIXED_HEADWAY_THW = 1.5` would give a negative trigger distance. Setting `FIXED_HEADWAY_THW = 0.8` maintains a positive lead-to-target surface gap at the hardest case: 0.2 × ego_ms ≥ 1.1 m (at 20 km/h), giving the lead clearance to begin the lane change.

**Geometry verification (all 50 cases):**

| Case (hardest) | `headway_d` | `cutout_trigger_d` | lead→target surf. gap |
|---|---|---|---|
| 20 km/h, reveal_ttc=1.0 | 4.44 m | **5.61 m** ← min | **1.11 m** |
| 60 km/h, reveal_ttc=3.0 | 13.33 m | 41.17 m ← max | 36.67 m |

All 50 `cutout_trigger_d > 0` ✓ (verified with `tools/check_conflict.py` inline geometry check).

**Tuning note for CARLA:** At the hardest case (20 km/h, `reveal_ttc = 1.0`), the lead starts the cut-out with only ~1.1 m surface clearance from the target. If the lead cannot complete the lane change without clipping the target in CARLA, increase `GAP_OFFSET` in `config/scenario_cutout.py` (e.g. from 4.5 to 5.0 m). This shifts all 50 `cutout_trigger_d` values up by 0.5 m uniformly — the matrix axes (`reveal_ttc`, `ego_speed`, `mu`) are unchanged.

**Conflict:** All 50 cut-out cases remain conflict cases (worst: 20 km/h → t = 7.9 s). `reveal_ttc` does not affect conflict status (formula depends only on ego speed vs ego→target distance).

#### CCRs: 5 approach_d values

| What | Old (Rev 2026-07-23) | New |
|---|---|---|
| `APPROACH_DISTANCES` | `[30, 45, 60]` m | `[30, 40, 50, 60, 70]` m |
| Cases/controller | 30 (5×3×2) | **50 (5×5×2)** |
| TTC-at-spawn range | 1.5–10.0 s | 1.5–11.8 s |

**Domain justification:** Even 10 m steps; minimum ≥ 30 m (ego needs room to reach cruise speed); 70 m @ 20 km/h → t = 11.8 s < 20 s window (still conflict). Old values 30 and 60 are preserved.

**Conflict:** All 50 CCRs cases are conflict cases (verified by `tools/check_conflict.py`).

#### Files changed

| File | Change |
|---|---|
| `config/scenario_cutout.py` | `FIXED_HEADWAY_THW` 1.5 → **0.8**; `REVEAL_TTC` 3 → **5 values**; MATRIX comment; tuning note |
| `config/scenario_ccrs.py` | `APPROACH_DISTANCES` 3 → **5 values**; MATRIX comment |
| `run_matrix_cutout.py` | `build_cases()` docstring: 30 → 50 |
| `run_matrix_ccrs.py` | `build_cases()` docstring: 30 → 50 |
| `README.md` | Matrix table; How to Run counts; check_conflict count (160 → 200) |
| `TECHNICAL_DOC.md` | §2 CCRs + Cut-out matrix tables added; this revision entry |

**No changes to:** `core/runner_cutout.py`, `core/runner_ccrs.py`, `core/conflict.py`, `tools/check_conflict.py`, `core/report.py`, any cut-in / lead-brake file. CSV schema unchanged; older CSVs still load in `report.py`.

#### Verification (no CARLA)

```bash
python3 tools/check_conflict.py
# Expected: cut-in 50/50, lead-brake 50/50, CCRs 50/50, cut-out 50/50 = 200/200 conflict

python3 -m py_compile config/scenario_cutout.py config/scenario_ccrs.py \
    run_matrix_cutout.py run_matrix_ccrs.py
```

*Last updated: 2026-07-23.*

---

### Rev 2026-07-23c — Realistic front-camera mount + aligned occlusion-gate eye-point

#### Problem

`CAM_FRONT_TF = dict(x=3.5, y=0.2, z=1.60, pitch=8)` placed the ego's front camera 3.5 m ahead of the actor origin (≈ 1.4 m ahead of the front bumper, floating in air) and at z = 1.60 m (≈ 1.0 m above the estimated roofline at +0.63 m). The camera looked over the top of an occluding lead vehicle, undermining the cut-out occlusion premise.

Additionally, the occlusion gate in `core/runner_cutout.py` used the ego **actor origin** (x = 0) as the sight-line eye-point, while the camera sat 3.5 m forward — a 3.5 m mismatch between "what the gate calls occluded" and "what the camera can see."

#### Fix

**Shared camera mount constant** (all four configs, changed identically):

```python
# Old
CAM_FRONT_TF = dict(x=3.5, y=0.2, z=1.60, pitch=8)

# New
CAM_FRONT_TF = dict(x=1.0, y=0.0, z=0.5, pitch=0)
```

| Param | Old | New | Rationale |
|---|---|---|---|
| x | 3.5 m | **2.8 m** | Bonnet/windshield-base area; confirmed live in CARLA |
| y | 0.2 m | **0.0 m** | Centred |
| z | 1.60 m | **0.8 m** | Above bonnet surface; confirmed clears vehicle mesh → live image in CARLA |
| pitch | 8° down | **0°** | Level — realistic ADAS camera attitude |

> **Bounding-box note:** the exact `extent` values are not queryable from static code. Estimates use
> CARLA 0.9.x Audi TT-class geometry (extent.x ≈ 2.1 m, extent.z ≈ 0.63–0.70 m, actor origin at
> geometric centre). If the camera renders a **black image**, the camera is still inside the solid
> mesh — increase `z` by 0.1 m steps (e.g. 0.8 → 0.9) until the image is live.

**Occlusion gate eye-point alignment** (`core/runner_cutout.py`):

The gate call now subtracts `cfg.CAM_FRONT_TF["x"]` from both `lead_lon` and `lon` before passing
to `sight_line_occluded`, shifting the eye-point from the ego actor origin to the camera mount:

```python
_cam_x = cfg.CAM_FRONT_TF.get("x", 0.0)
if actors.sight_line_occluded(
        lead_lon - _cam_x, lead_lat, lon - _cam_x, lat,
        ...):
    detected_now = False
```

`sight_line_occluded` is 2D (forward/right only); z and y offsets of the camera have no effect on
the gate. After the fix the residual eye-point error is zero (old error was 3.5 m).

#### Scope protection

- `DETECTION_SOURCE = "groundtruth"` in all four scenarios → camera feeds YOLO/visualisation only.
  **Moving the mount changes zero recorded metrics.** Cut-in, lead-brake, CCRs, and cut-out paper
  results are byte-for-byte identical.
- `core/occlusion.sight_line_occluded` is unchanged; all 9 unit tests pass.
- Cut-in, lead-brake, CCRs runners: no change (occlusion gate is cut-out only).

#### Files changed

| File | Change |
|---|---|
| `config/scenario_cutin.py` | `CAM_FRONT_TF`: x 3.5→1.0, y 0.2→0.0, z 1.60→0.5, pitch 8→0 |
| `config/scenario_lead_brake.py` | same |
| `config/scenario_ccrs.py` | same |
| `config/scenario_cutout.py` | same |
| `core/runner_cutout.py` | Occlusion gate: subtract `_cam_x` from `lead_lon` and `lon` |
| `README.md` | Added "Front Camera Mount" section with visual check checklist |
| `TECHNICAL_DOC.md` | This revision entry |

#### Verification (no CARLA)

```bash
python3 tests/test_occlusion_gate.py
# Expected: 9/9 passed

python3 -m py_compile config/scenario_cutin.py config/scenario_lead_brake.py \
    config/scenario_ccrs.py config/scenario_cutout.py core/runner_cutout.py
```

#### Visual check in CARLA

1. **Camera position**: the front-camera window should show the lead/target vehicle roof appearing from the **bottom of the frame** as ego closes in — the vehicle should NOT be a small object near the top of the frame that can be "looked over."
2. **Roof clearance**: at ~5–10 m gap to the lead, the lead's roofline should be roughly at mid-frame height or lower (not near the top).
3. **No clipping**: the camera should not be inside the vehicle mesh (no fisheye / blacked-out corners). If it is, reduce `z` to 0.4 m.
4. **Occlusion gate (cut-out)**: run `run_single_cutout.py` with `SHOW_WINDOW=True`. The `det=False` state in the console should persist for a realistic duration while the lead is squarely in front of the target. `det=True` should switch on shortly after the lead visibly clears.

---

### Spawn Fixes — July 2026 (Round 1)

**Problem:** `TEST_MODE=original` on train000 produced 30/150 "LEAD spawn failed" rows at every 20 km/h cut-out cell, and 30/150 "TARGET spawn failed" rows at every CCRs `approach_d=60 m` cell (both affecting all three controllers identically — confirmed geometry, not controller logic).

**Root causes:**

| Scenario | Cell | Root cause |
|---|---|---|
| Cut-out | 20 km/h (all 5 reveal_ttc) | `headway_d = 0.8 × 5.556 = 4.444 m` < combined half-lengths ≈ 4.5 m → bounding-box overlap → CARLA spawn rejection |
| CCRs | approach_d = 60 m (all 5 speeds, both μ) | Fixed `z = 0.25` falls inside the baked static vehicle at `(−43.64, −32.74)` on train000 mesh |

**Reveal-TTC invariant preserved (cut-out):** `headway_d` cancels in the ego→target surface gap formula; clamping it does not change the TTC at which the target is revealed. ✓

**Feasibility table (25 cut-out speed×ttc cells after headway clamp to 5.0 m):**

| speed | reveal_ttc | trig_surf_gap | verdict |
|---|---|---|---|
| 20 km/h | 1.0 s | 0.556 m (0.10 s) | SCENARIO_INFEASIBLE — marked, kept in CSV |
| 20 km/h | 1.5–3.0 s | 3.3–11.7 m | Fixed by spawn clamp |
| 30–60 km/h | all | ≥ 1.7 m | No change needed |

**Modified files (Round 1):**

| File | Change |
|---|---|
| `core/runner_cutout.py` | After ego spawn: compute `min_headway = 2×ego_half + SPAWN_CLEARANCE_M`; clamp `headway_d` and recompute `cutout_trigger_d`; if post-clamp trigger surf gap < `MIN_TRIGGER_SURF_GAP_M`, set `result_txt="SCENARIO_INFEASIBLE"` and return early |
| `core/runner_ccrs.py` | Target spawn: z-sweep `SPAWN_Z_SWEEP` (now superseded — see Round 2) |
| `config/scenario_cutout.py` | Add `SPAWN_CLEARANCE_M = 0.5` and `MIN_TRIGGER_SURF_GAP_M = 1.0` |
| `config/scenario_ccrs.py` | Add `SPAWN_Z_SWEEP` (now removed — see Round 2) |
| `tools/check_cutout_spawn.py` | New — offline per-cell feasibility checker (no CARLA needed) |

---

### Spawn Fixes — July 2026 (Round 2: baked obstacle diagnosis + fail-loud)

**Problem diagnosed:** The z-sweep added in Round 1 silently placed the CCRS target vehicle on top of the baked static car at approach_d=60 m (spawn accepted at z=0.5 m by sitting on the baked car's collision mesh). This produces invalid geometry — the CCRS target is not on the road. Additionally:

- All train000 spawn z-values are fixed constants that do not vary with (x,y), potentially placing actors underground on uneven 3DGS terrain.
- The cut-out lead's post-cutout path enters the baked obstacle zone for 2 cells: 50 km/h / reveal_ttc=1.0 (forward extent ≈60.6 m) and 60 km/h / reveal_ttc=1.0 (forward extent ≈66.3 m). The baked car is at ≈60 m forward; the lead is in the right lane (1.5 m lateral) at that point — calculated center-to-center lateral separation ~1.5 m vs combined half-widths ~1.86 m.

**Baked obstacle position:** world coords `(−43.636, −32.741)` = exactly the 60 m point along ego heading from `EGO_SPAWN`. All CCRS target spawns at approach_d=60 m land directly on this obstacle.

**Root cause of z-sweep masking the problem:** `world.try_spawn_actor` uses bounding-box overlap. At z=0.25 the CCRS target overlaps the baked car's main body → rejected. At z=0.5 the target's bounding box barely clears the baked car's collision geometry → accepted. The vehicle is physically sitting on the roof of the baked car, not on road.

**Ground projection (`actors.ground_projection_z`):** uses `world.cast_ray()` downward at (x,y) to find the actual scene surface z. For road points this returns the road surface. For baked-obstacle points it returns the obstacle's roof — which is also above `SPAWN_SURFACE_Z_MAX=2.0 m`, triggering a warning. If CARLA then rejects the spawn (because target+baked-obstacle bounding boxes still overlap even at roof-level z), the code fails loudly with the coordinate and a reference to `probe_spawn_points.py`.

**Post-cutout lead path cap:** `CUTOUT_STOP_MAX_M = 18.0 m` in `config/scenario_cutout.py` hard-stops the lead once it travels 18 m from the trigger point (2-D, any phase). This prevents it from reaching the 60 m baked obstacle zone. Lower speeds (20–40 km/h) naturally stop within 14 m of trigger — unaffected.

**CCRS approach_d=60 m status:** still blocked in config (approach_d=60.0 retained in `APPROACH_DISTANCES` as a placeholder). Must be replaced with a probe-verified clear value (58 m or 62 m) before running the matrix. See `tools/probe_spawn_points.py`.

**Modified files (Round 2):**

| File | Change |
|---|---|
| `core/actors.py` | Add `ground_projection_z(world, x, y, probe_z=20.0)` — `world.cast_ray()` helper |
| `core/runner_ccrs.py` | Replace z-sweep with ground projection + fail-loud (single spawn attempt; no silent relocation) |
| `core/runner_cutout.py` | Apply ground projection to lead spawn (x,y vary per case) with fail-loud |
| `core/scenario_cutout.py` | Add `CUTOUT_STOP_MAX_M` hard-stop cap in `update()` (all post-trigger phases) |
| `config/scenario_ccrs.py` | Remove `SPAWN_Z_SWEEP`; add `SPAWN_SURFACE_Z_MAX=2.0`, `SPAWN_Z_OFFSET=0.5`; add obstacle note to approach_d=60 entry |
| `config/scenario_cutout.py` | Add `SPAWN_SURFACE_Z_MAX=2.0`, `SPAWN_Z_OFFSET=0.5`, `CUTOUT_STOP_MAX_M=18.0` |
| `tools/probe_spawn_points.py` | New — CARLA-live coordinate survey; tests CCRS and cut-out spawn points |

**Untouched:** `runner.py`, `runner_lead_brake.py`, `scenario_ccrs.py`, `scenario_cutin.py`, `scenario_lead_brake.py`, `conflict.py`, all config files for cut-in / lead-brake, CSV schema, `is_conflict` logic, matrix 5×5×2=50 dimensions.

#### Verify in CARLA (required before running either matrix)

1. **Run probe utility first:**
   ```
   python tools/probe_spawn_points.py
   ```
   Confirm approach_d=58 m or 62 m shows CLEAR + reasonable surface_z.  
   Update `APPROACH_DISTANCES` in `config/scenario_ccrs.py` with that value.  
   Confirm `CUT_LEAD_PATH_60m_*` rows (right lane at 60 m) show CLEAR or BLOCKED — adjust `CUTOUT_STOP_MAX_M` accordingly.

2. **Cut-out matrix:** run `TEST_MODE=original python run_matrix_cutout.py`. Confirm:
   - 20 km/h / reveal_ttc≥1.5: AVOIDED or COLLISION (not "LEAD spawn failed")
   - 20 km/h / reveal_ttc=1.0: `result_txt = SCENARIO_INFEASIBLE` in CSV
   - Console shows `[LEAD] headway_d=... z=X.XX` (projected z, not fixed constant)
   - No lead-baked-car collision at 50–60 km/h / reveal_ttc=1.0 (CUTOUT_STOP_MAX_M=18 m cap active)

3. **CCRS matrix:** run `TEST_MODE=original python run_matrix_ccrs.py` after updating approach_d. Confirm:
   - All 50 cells produce AVOIDED or COLLISION or NO BRAKE (no spawn failures)
   - Console shows `[SPAWN] BLOCKED ...` for any coordinate still on the baked obstacle

*Last updated: 2026-07-23.*

---

### Spawn Hardening — July 2026 (Round 3: warn→hard-block + diagnostic log)

**Problem:** Round 2 added `ground_projection_z` but left the `_proj_z > SPAWN_SURFACE_Z_MAX` path as a **warning-only**: the code printed `[SPAWN] WARNING:` and then still spawned at the suspect z. A vehicle placed on a baked-obstacle roof gets pushed sideways by CARLA's physics during the 20 settle ticks — the observed "CCRS target visibly relocates" during actual runs. The `try_spawn_actor → None` path was already loud (hard fail with `result_txt = "TARGET spawn failed"`), but the surface-z check was not.

**Root cause of run-time target shift:**
1. `approach_d = 60.0` in `APPROACH_DISTANCES` maps to `(-43.636, -32.741)` — the baked static vehicle.
2. `ground_projection_z` returns the baked car's roof z (≈ 1.5–2.0 m). The value may be ≤ `SPAWN_SURFACE_Z_MAX = 2.0`, so the existing `if _proj_z > _surf_z_max` guard may not even fire. When it does fire, the code only warned and then spawned at `roof_z + 0.5 m`.
3. CARLA accepts the spawn (vehicle sitting on top of baked car). During `SETTLE_TICKS = 20`, physics pushes the vehicle off the side → visible relocation.

**Fix:**
- `_proj_z > SPAWN_SURFACE_Z_MAX` → **hard abort** (returns `SPAWN_BLOCKED`, no spawn attempted).
- `try_spawn_actor → None` → **hard abort** (already was, now uses uniform `SPAWN_BLOCKED` result_txt for both failure modes).
- One structured `[SPAWN][CCRS]` / `[SPAWN][CUTOUT]` diagnostic line emitted for every TARGET / LEAD spawn attempt (success and failure) — see §Spawn Safety in README.
- After a successful spawn, `target.get_location().x,y` are asserted within 0.1 m of the requested `x,y`. A `RuntimeError` is raised immediately if drift is detected. Ground projection may only change z.
- x,y of the spawn transform are **never modified** by the ground-projection path. `dict(target_spawn, z=_spawn_z)` overrides only z; the assertion makes this machine-verifiable.

**Result-txt values after this round:**

| `result_txt` | Meaning |
|---|---|
| `SPAWN_BLOCKED` | Hard-blocked before or during spawn (surface z too high, or CARLA overlap). Probe and fix coordinate. |
| `SCENARIO_INFEASIBLE` | Cut-out cell where the lead cannot complete the manoeuvre (headway clamp Round 1). |
| `AVOIDED` / `COLLISION` / `NO BRAKE / passed` | Normal run outcomes. |

**Modified files (Round 3):**

| File | Change |
|---|---|
| `core/runner_ccrs.py` | TARGET spawn: `_proj_z > _surf_z_max` → hard-fail `SPAWN_BLOCKED` (was warn-and-spawn). Structured `[SPAWN][CCRS]` diagnostic log every spawn. x,y assertion after successful spawn. Unified result_txt `"SPAWN_BLOCKED"` for both failure modes. |
| `core/runner_cutout.py` | LEAD spawn: same hard-fail and diagnostic log. Unified `"SPAWN_BLOCKED"`. |
| `README.md` | New §Spawn Safety section: log format, BLOCKED/OK example lines, known blocked coordinate, x,y invariant. |

**Untouched:** `runner.py`, `runner_lead_brake.py`, all scene03_2 spawns, `conflict.py`, `metrics.py`, CSV schema, matrix dimensions (still 5×5×2=50).

#### How to read the console log after this round

OK line (inspect that x,y match req, and final.z = surf_z + 0.5):
```
[SPAWN][CCRS] case=ego30_ad40_mu0.85  req=(-27.817,-21.718,z_nom=0.250)  surf_z=0.122  dz=-0.128  blocked=N  status=OK  final=(-27.817,-21.718,0.622)
```

BLOCKED line (note the exact coordinate and approach_d to fix):
```
[SPAWN][CCRS] case=ego30_ad60_mu0.85  req=(-43.636,-32.741,z_nom=0.250)  surf_z=1.823  dz=+1.573  blocked=Y  status=BLOCKED  final=n/a
[SPAWN] BLOCKED at (-43.636,-32.741): surface z=1.823 exceeds SPAWN_SURFACE_Z_MAX=2.00 — baked 3DGS obstacle at approach_d=60m.
[SPAWN]   Run tools/probe_spawn_points.py to find a clear road coordinate, then replace approach_d=60 in APPROACH_DISTANCES.
```

Any BLOCKED line tells you exactly which case and coordinate to probe. Fix it by running `tools/probe_spawn_points.py` in CARLA, confirm 58 m or 62 m is clear, then replace `60.0` in `config/scenario_ccrs.py:APPROACH_DISTANCES`.

*Last updated: 2026-07-24.*

---

### Spawn Hardening — July 2026 (Round 4: unified spawn path + scene-relative threshold)

**Problem:** Adding `SPAWN_SURFACE_Z_MIN = -1.0` in Round 3 to catch underground mesh hits was miscalibrated for train000. The road sits at z ≈ −1.95 m — below the −1.0 floor — so every road ray hit triggered the `UNDERGROUND_HIT(fallback)` branch. The fallback spawned at the config nominal z = 0.25, which is 2.2 m above the road; at the computed (x, y) positions this z is inside 3DGS scene geometry, causing `try_spawn_actor` to return None → `SPAWN_BLOCKED` for **all 150 rows**.

Additionally, `probe_spawn_points.py` had no Z_MIN check — it cast the ray, used surf_z + 0.5 = −1.45, and CARLA accepted it — creating a divergence between "CLEAR in probe" and "BLOCKED in runner".

**Root cause (precise):**
1. `SPAWN_SURFACE_Z_MIN = -1.0` in config/scenario_ccrs.py and config/scenario_cutout.py.
2. Road ray hit returns surf_z ≈ −1.95; −1.95 < −1.0 → `UNDERGROUND_HIT` branch fires for every road point.
3. Falls back to `_spawn_z = target_spawn["z"]` = 0.25 (wrong for computed x,y positions).
4. CARLA overlap rejection at z = 0.25 → `SPAWN_BLOCKED`.

Verification: The log would show `status=BLOCKED(overlap)` (not `status=BLOCKED(obstacle_roof)`), confirming the failure is at `try_spawn_actor`, not at the z-gate.

**Fix:**

1. **Removed `SPAWN_SURFACE_Z_MIN`** from both scenario configs — no underground fallback. The road IS at negative z; spawning at surf_z + 0.5 = −1.45 is correct (probe-confirmed CLEAR).

2. **Recalibrated `SPAWN_SURFACE_Z_MAX`** from 2.0 → **−0.95** (= train000 road level −1.95 + 1.0 m margin):
   - Road −1.95: −1.95 > −0.95 → **FALSE** → allow (normal spawn path) ✓
   - Obstacle roof +0.9: +0.9 > −0.95 → **TRUE** → `BLOCKED(obstacle_roof)` ✓
   This is a scene-relative threshold. Document it if porting to another scene.

3. **Factored into `actors.spawn_ground_projected()`** — shared by both runners and the probe. No divergent copies. The function: cast ray → gate on SPAWN_SURFACE_Z_MAX → spawn at surf_z + offset → check CARLA overlap → check x,y drift. One [SPAWN] log line per call.

4. **Updated `probe_spawn_points.py`** to import and call `spawn_ground_projected` from `core.actors`. Probe output now matches runner behaviour exactly.

**Threshold verification (offline, using probe's measured surf_z values):**

| approach_d | surf_z | surf_z > −0.95 | Action |
|---|---|---|---|
| 30 m | −1.952 | FALSE | spawn at −1.452 → CLEAR ✓ |
| 40 m | −1.952 | FALSE | spawn at −1.452 → CLEAR ✓ |
| 50 m | −1.952 | FALSE | spawn at −1.452 → CLEAR ✓ |
| 62 m | −1.952 | FALSE | spawn at −1.452 → CLEAR ✓ |
| 70 m | −1.952 | FALSE | spawn at −1.452 → CLEAR ✓ |
| 60 m (obstacle) | +0.901 | **TRUE** | BLOCKED(obstacle_roof) ✓ |

**Modified files (Round 4):**

| File | Change |
|---|---|
| `core/actors.py` | Add `spawn_ground_projected()` shared function; add `_SPAWN_XY_TOL` constant |
| `core/runner_ccrs.py` | Replace ~90-line inline TARGET spawn block with `actors.spawn_ground_projected()` call |
| `core/runner_cutout.py` | Replace ~80-line inline LEAD spawn block with `actors.spawn_ground_projected()` call |
| `config/scenario_ccrs.py` | Remove `SPAWN_SURFACE_Z_MIN`; recalibrate `SPAWN_SURFACE_Z_MAX` 2.0 → −0.95 |
| `config/scenario_cutout.py` | Same |
| `tools/probe_spawn_points.py` | Import `ground_projection_z`, `spawn_ground_projected` from `core.actors`; remove local `_cast_ray`/`_try_spawn`; update `SURFACE_Z_WARN` to −0.95 |
| `README.md` | Update §Spawn Safety: train000 road level note, scene-relative threshold, removed Z_MIN, updated log examples, approach_d table 60→62 |

**Untouched:** `runner.py`, `runner_lead_brake.py`, all scene03_2 spawns, `conflict.py`, `metrics.py`, CSV schema, matrix dimensions (5×5×2=50), `approach_d` values (62 is correct).

*Last updated: 2026-07-24.*
