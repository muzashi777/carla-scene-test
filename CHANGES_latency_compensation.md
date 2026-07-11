# Latency Compensation — Change Summary

## What was added and why

The required-deceleration controller (`proposed_enhanced`) brakes close to the friction
ceiling (`a_req` up to ~90 % of μg, `REQ_FULL_FRAC=0.9`), leaving only ~0.64 m (cut-in) /
~0.91 m (lead-brake) of stopping clearance. Injecting perception latency `L ≥ 0.2 s`
collapses the avoidance rate `Rc` from 72 %/68 % to 0 %/2 % — there is no margin to absorb
the delay.

This change adds **two controllers that compensate for a known latency `L`** to recover
`Rc`, plus two test modes that (a) measure the upper bound of that recovery when `L` is known
exactly, and (b) probe how fragile the compensation is when `L` is mis-estimated.

### The two methods

1. **`enhanced_predictive` (feedforward predictor — main method).** Extrapolates the target
   state forward by `L` with constant-acceleration kinematics, then feeds the *predicted
   inputs* into the **existing** `required_decel()` — the decision is made on the predicted
   state, not the stale (latency-delayed) one. A discrete analog of a Smith predictor.

   ```
   v_l_pred = max(0, lead_speed - lead_decel · L)
   gap_pred = max(0, distance - v_close · L - 0.5 · lead_decel · L²)   # v_close = rel_speed (closing)
   a_req    = required_decel(ego.speed_ms, v_l_pred, lead_decel, gap_pred)
   urgency  = a_req / (μ·g)     # same REQ_FULL_FRAC / REQ_WARN_FRAC thresholds and latch
   ```

   Closing acceleration is `+lead_decel` (a braking lead makes the gap close faster). Because
   it extrapolates the *inputs* rather than a new closed form, at `L=0` it reduces **exactly**
   to `proposed_enhanced`, tick by tick.

2. **`enhanced_inflation` (threshold inflation — worst-case robust).** Does not predict the
   target; it adds the distance travelled during `L` to the required stopping distance.

   ```
   r_required = v_close² / (2·μ·g) + v_close · L
   urgency    = r_required / max(ε, gap - COMP_R_SAFE)
     urgency ≥ REQ_FULL_FRAC → full brake ; ≥ REQ_WARN_FRAC → partial
   ```

   Needs only an upper bound `L_max`, not exact `L`. The `v_close·L` term is the standard
   delay term in Mazda/Berkeley-PATH safety-distance formulas. At `L=0` the inflation term
   vanishes → non-compensated behaviour.

### Research framing: oracle vs mismatched

The predictor must know `L`. In this simulator the harness *injects* `L`, so it can be known
exactly — an **oracle / idealized upper bound** on how much `Rc` compensation can recover.
Real systems must estimate `L` imperfectly; we emulate that with a **mismatched** sweep.

| `comp_source` | Compensated L | Research question |
|---|---|---|
| `oracle` | `L = delay_frames · dt` (exact injected delay) | Upper bound — if `L` is known exactly, how much `Rc` is recovered? |
| `mismatched` | `L = comp_L_frames · dt` (set independently of `delay_frames`) | Fragility — how does `Rc` degrade under under-/over-compensation? |

> **Why online L-measurement (timestamp) is future work, not implemented here.** In simulation
> a sensor timestamp equals the value we injected, so there is no measurement uncertainty to
> test — nothing to learn from a `timestamp_perfect`/`timestamp_jitter` mode. The `mismatched`
> sweep is the meaningful stand-in (grounded in Richard 2003: predictor-based control is
> sensitive to delay-mismatch). Online `L` estimation belongs to perception-in-the-loop
> testing; MPC is likewise deferred.

---

## New files

| File | Purpose |
|---|---|
| `control/enhanced_predictive.py` | `@register("enhanced_predictive")` — feedforward predictor; extrapolates inputs, reuses `required_decel()` |
| `control/enhanced_inflation.py` | `@register("enhanced_inflation")` — threshold inflation (`v_close·L`); worst-case robust |
| `tests/test_latency_comp.py` | Unit tests (mock `carla`, no CARLA): L=0 reduction, L-monotonicity, mismatched, matrix-mode counts |
| `CHANGES_latency_compensation.md` | This file |

---

## Modified files (minimal footprint)

| File | Change summary | ~Lines changed |
|---|---|---|
| `control/base_controller.py` | `make_controller(name, cfg, run_spec=None)` attaches `ctrl.run_spec`; new `compensation_latency(run_spec, cfg)` helper (frames→seconds, oracle/mismatched). No existing controller reads `run_spec` → behaviour unchanged | +~25 |
| `core/runner.py` | Import 2 controllers; `make_controller(..., run_spec=spec)`; set `rec.comp_source`/`rec.comp_L_frames` | +~7 |
| `core/runner_lead_brake.py` | Same as runner.py + 2 fields on `LeadBrakeRecord` | +~9 |
| `core/metrics.py` | Append `comp_source: str = ""`, `comp_L_frames: int = 0` to `RunRecord` | +2 |
| `config/scenario_cutin.py` | `COMP_CONTROLLERS`, `COMP_MISMATCH_CTRL/DELAY/L_FRAMES`, `COMP_R_SAFE`; `latency_comp` + `latency_mismatch` branches in `build_matrix_runs()` | +~30 |
| `config/scenario_lead_brake.py` | Same as scenario_cutin.py | +~30 |
| `README.md` | Controllers table rows; 2 Test Modes + run commands; CSV columns note | +~25 |
| `TECHNICAL_DOC.md` | §5 latency-comp subsection; §7 new CSV columns; §12 revision entry | +~70 |

**How `L` reaches the controller (no tight coupling):** the run spec carries `comp_source`
and `comp_L_frames`; `make_controller` binds the spec onto the controller; the controller
converts to seconds via `FIXED_DT`. Controllers never read `PerceptionDegrader` internals.

---

## Run commands (for server — not run here)

```bash
# Latency compensation — upper bound (L known exactly): COMP_CONTROLLERS × delay × oracle = 16 run-specs
TEST_MODE=latency_comp python run_matrix.py
TEST_MODE=latency_comp LEAD_DECEL=6.0 python run_matrix_lead.py

# Latency compensation — fragility (mis-estimated L): enhanced_predictive × comp_L_frames = 4 run-specs
TEST_MODE=latency_mismatch python run_matrix.py
TEST_MODE=latency_mismatch LEAD_DECEL=6.0 python run_matrix_lead.py

# Summarise (no CARLA)
python -m core.report results/matrix_*.csv results/lead_matrix_*.csv
```

`latency_comp` = `{proposed_enhanced (control), enhanced_predictive, enhanced_inflation,
proposed (crossover ref)}` × `delay_frames ∈ {0,4,8,16}` with `comp_source=oracle`.
`latency_mismatch` = `enhanced_predictive` at fixed injected `delay_frames=8` (`COMP_MISMATCH_DELAY`)
while sweeping `comp_L_frames ∈ {4,8,12,16}` (under=4, exact=8, over=12,16). Sweep values are
constants at the top of each config file.

---

## How to verify L=0 reduces to the original (no CARLA)

```bash
python3 tests/test_latency_comp.py
```

Key assertions (`tests/test_latency_comp.py`):
- `enhanced_predictive` with an empty/`L=0` run spec produces the **same brake command as
  `proposed_enhanced` on every tick**, over both a cut-in and a lead-brake input sequence.
- `enhanced_inflation` at `L=0` yields `r_required = v_close²/(2·μg)` — the inflation term is 0.
- At `L>0`, `a_req` (predictive) and `r_required` (inflation) grow monotonically with `L`.
- Under `mismatched`, the controller compensates with `comp_L_frames`, not `delay_frames`.
- `proposed_enhanced` is unaffected by any `comp_*` keys in its run spec.
- `build_matrix_runs("latency_comp"/"latency_mismatch")` return the correct specs; the
  `original`/`latency`/`noise` modes are unchanged and carry no `comp_*` keys.

The three original controllers show an empty `git diff`.

---

## What was NOT touched

- Decision logic of `baseline_static_ttc`, `proposed_dynamic_ttc`, `proposed_enhanced`
- `required_decel()`, MFDD formula, kinematic brake cap, collision detection, scenario logic
- `PerceptionDegrader` (`perception/degrade.py`) — unchanged
- Existing CSV columns (only `comp_source`, `comp_L_frames` appended; old CSVs still load)
- `TEST_MODE=original/latency/noise` behaviour and run-spec shape
- Files in `results/` (read-only)
- `FRAME_SYNC`, `FIXED_DT`, `DETECTION_SOURCE`, `BRAKE_MODEL`, `GRAVITY`
- `run_single.py`, `run_single_lead.py` (single-run scripts unchanged)
- No new dependency beyond numpy

---

## References (verified — cite in the paper)

**Tier A (peer-reviewed / standard texts — primary basis):**

- **Xing, H., Ploeg, J. & Nijmeijer, H. (2019).** Smith predictor compensating for vehicle
  actuator delays in cooperative ACC systems. *IEEE Transactions on Vehicular Technology*
  68(2), 1106–1115. — basis for the predictor/extrapolation method (`enhanced_predictive`).
- **Richard, J.-P. (2003).** Time-delay systems: an overview of some recent advances and open
  problems. *Automatica* 39(10), 1667–1694. DOI 10.1016/S0005-1098(03)00167-5. — basis for the
  mismatched sweep (predictor sensitivity to delay-mismatch; robustness to design-delay
  deviation).
- **Rajamani, R. (2012).** *Vehicle Dynamics and Control*, 2nd ed. Springer. — basis for the
  threshold-inflation method (`enhanced_inflation`) + longitudinal/actuation dynamics.
- **Kusano, K.D. & Gabler, H.C. (2012).** *IEEE Transactions on Intelligent Transportation
  Systems* 13(4), 1546–1555. — supports the 0–0.8 s latency range (autonomous braking 0.45 s,
  brake assist 0.8 s).

**Citation caveat:** Xing et al. and Richard are CACC/platoon and general time-delay-control
contexts, **not AEB directly** — describe this work as "borrowing delay-compensation
principles and adapting them to AEB," not as prior AEB work. Online `L` measurement
(timestamp / actuation characterization) is perception-in-the-loop future work.
