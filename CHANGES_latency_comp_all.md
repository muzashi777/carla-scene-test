# Predictive-Oracle Compensation Across All Base Controllers — Change Summary

## What was added and why

`latency_comp` measures how much of the latency-induced `Rc` loss the *compensated
controllers* (`enhanced_predictive`, `enhanced_inflation`) recover. It does **not** answer:
under one identical environment (same latency, same perception, same matrix), how much of
**each base controller's** `Rc` can a predictor recover, and does the best *compensated*
controller beat the best *uncompensated* one?

This change adds `TEST_MODE=latency_comp_all`, which reframes predictive-oracle compensation
as a **preprocessing layer** that can sit in front of *any* base controller — the exact
mirror of the existing degradation layer:

```
raw Perception → degrade (delay by L → stale) → predict (extrapolate +L → fresh) → controller
```

*Degrade makes the input worse; predict makes it fresh again.* The layer feeds every base
controller (`baseline`, `proposed`, `proposed_enhanced`) the **same** predicted Perception at
the **same** latency, so `Rc` differences are attributable to the controller (controller-swap
protocol / fairness).

## The prediction layer (`perception/predict.py`)

`PerceptionPredictor.apply(perc, ego)` extrapolates the Perception fields forward by `L`
seconds using the same constant-acceleration model as `control/enhanced_predictive.py`
(closing acceleration `+lead_decel`, since a braking lead makes the gap close faster):

```
lead_speed' = max(0, lead_speed − lead_decel · L)
distance'   = max(0, distance − v_close · L − 0.5 · lead_decel · L²)   # v_close = rel_speed
rel_speed'  = v_close + lead_decel · L
ttc'        = distance' / rel_speed'        (∞ if rel_speed' ≈ 0)
lead_decel' = lead_decel (unchanged) ;  detected / box_h passed through
```

- **`L = 0` identity.** `apply()` returns a copy of the Perception with every field
  unchanged, so `compensated_X(L=0)` equals base controller `X` per tick — for every `X`.
- **Predictive only.** Inflation is distance-based and does not fit the time-based TTC
  controllers (`baseline`, `proposed`), so this mode sweeps the predictor only (oracle `L`).
- **Oracle `L`.** `L = delay_frames · FIXED_DT` (exactly the injected delay), delivered via
  the run spec (`comp_source="oracle"`). This is an idealized upper bound on `Rc` recovery.

## enhanced_predictive consistency (checked, no difference to escalate)

The existing `enhanced_predictive` predicts the *inputs to `required_decel()`*; the new layer
predicts the *Perception fields*. These are **mathematically identical** for the
required-decel controller: `proposed_enhanced` reads only `distance` / `lead_speed` /
`lead_decel` / `detected`, and `distance'` / `lead_speed'` above are term-for-term the
`gap_pred` / `v_l_pred` that `enhanced_predictive` feeds into `required_decel()` (which
re-clamps its arguments idempotently). Both gate on `detected or _engaged` and share the same
`_emit` latch. Therefore:

> **`PerceptionPredictor` + `proposed_enhanced` reproduces `enhanced_predictive` per tick.**

Proven for `L ∈ {0, 4, 8, 16}` on both a cut-in and a lead-brake synthetic sequence in
`tests/test_latency_comp_all.py`. `rel_speed'` / `ttc'` extend the same model to the TTC
controllers. There is no discrepancy to surface.

## Files

### New

| File | Purpose |
|---|---|
| `perception/predict.py` | `PerceptionPredictor` — the feedforward predictor layer (mirror of `PerceptionDegrader`). |
| `tests/test_latency_comp_all.py` | Unit tests: L=0 identity per controller, layer==`enhanced_predictive` consistency, gap/ttc monotonicity, run-count, old modes unchanged. |
| `CHANGES_latency_comp_all.md` | This summary. |

### Modified

| File | Change | Why |
|---|---|---|
| `config/scenario_cutin.py` | `latency_comp_all` branch in `build_matrix_runs` | Build `CONTROLLERS × LATENCY_DELAY_FRAMES`, `comp_source="oracle"`, `comp_L_frames=delay_frames`, `predict=True`. |
| `config/scenario_lead_brake.py` | same | Same branch for the lead-brake matrix. |
| `core/runner.py` | import + build `predictor` when `spec["predict"]`; apply after the degrader | Insert the layer between degrader and controller for `latency_comp_all` only. |
| `core/runner_lead_brake.py` | same | Same wiring for the lead-brake runner. |
| `run_matrix.py` | embed `test_mode` in CSV filename | `matrix_<TEST_MODE>_<stamp>.csv` — self-describing, consistent. |
| `run_matrix_lead.py` | same | `lead_matrix_<TEST_MODE>_<stamp>.csv`. |
| `README.md` | Test Modes row + run commands + filename note | Documentation. |
| `TECHNICAL_DOC.md` | §5 predictive-layer subsection + §12 revision entry | Documentation. |

## Purely additive / nothing broken

- The layer is engaged **only** when a run spec carries `predict=True`, which is set **only**
  by `build_matrix_runs("latency_comp_all")`. Every other mode leaves `predictor=None`, so
  `original` / `latency` / `noise` / `latency_comp` / `latency_mismatch` are byte-for-byte
  unchanged (verified: run specs for old modes carry no `predict`/`comp` keys).
- **No controller decision logic changed.** `baseline_static_ttc.py`,
  `proposed_dynamic_ttc.py`, `proposed_enhanced.py`, `enhanced_predictive.py`,
  `enhanced_inflation.py` are untouched (`git diff` empty). MFDD, the kinematic brake cap,
  `FIXED_DT`, and scene logic are unchanged.
- **No CSV columns renamed/removed** — reuses existing `comp_source` / `comp_L_frames` /
  `test_mode`. Rows are distinguished from `latency_comp` by the base `controller` and by
  `test_mode="latency_comp_all"`.
- The filename change only **inserts** `<TEST_MODE>` before the timestamp; the timestamp
  keeps every run unique, so existing files in `results/` are never overwritten.

## How to verify (no CARLA)

```bash
python3 tests/test_latency_comp_all.py   # L=0 identity, enhanced_predictive consistency, run-count
```

## Run command (CARLA required — run on server)

```bash
TEST_MODE=latency_comp_all python run_matrix.py                    # cut-in
TEST_MODE=latency_comp_all LEAD_DECEL=6.0 python run_matrix_lead.py  # lead-brake (Euro-NCAP CCRb)
```

Each writes `50 cases × 12 runs = 600 rows` to
`results/matrix_latency_comp_all_<stamp>.csv` / `results/lead_matrix_latency_comp_all_<stamp>.csv`.
