# Latency & Noise Degradation Study — Results Summary

Analysis of the perception-**latency** and distance-**noise** sweeps in `results/latency_noise/`.
All numbers below are computed directly from the CSV files (stdlib `csv`, no re-simulation). Every
mechanistic claim is tied to the actual code with `file:line` references. A clean separation is kept
between **[FACT]** (measured from CSV / read from code) and **[INTERPRETATION]**.

---

## 1. Overview

- **Scenarios (2):** *cut-in / dart-out* (`matrix_*.csv`) and *lead-brake / Euro-NCAP CCRb*
  (`lead_matrix_*.csv`).
- **Controllers (3):** `baseline` (static TTC), `proposed` (adaptive/dynamic TTC),
  `proposed_enhanced` (required-deceleration).
- **Cases:** 50 per controller per scenario (cut-in: 5 speeds × 2 μ × 5 trigger_d; lead-brake:
  5 speeds × 5 THW × 2 μ). Every case is a **conflict case** — an unbraked ego collides for sure
  (`core/conflict.py`).
- **Sweeps (this study):**
  - Latency: `delay_frames ∈ {0, 4, 8, 16}` → 3 × 4 × 50 = **600 rows** per scenario.
  - Noise: `noise_sigma_m ∈ {0.0, 0.5, 1.0, 2.0} m` → **600 rows** per scenario.
  - Original reference: 3 × 50 = **150 rows** per scenario.
- **Perception source:** ground-truth from CARLA (`DETECTION_SOURCE="groundtruth"`); degradation is
  an *abstracted surrogate* injected on top of ground truth, **not** a real sensor pipeline.

Row counts and sweep values match the config (`config/scenario_cutin.py`,
`config/scenario_lead_brake.py`, lines 120–147). **[FACT]**

---

## 2. Method — how degradation is injected

All degradation lives in `perception/degrade.py` (`PerceptionDegrader.apply`) and is applied to the
`Perception` object **before** the controller decides:
`core/runner.py:225-228` (cut-in) and `core/runner_lead_brake.py:268-271` (lead-brake). **[FACT]**

### 2.1 Latency
`degrade.py:68-69`:
```python
self._buf.append(perc)                 # deque(maxlen=delay_frames + 1)
perc_out = copy.copy(self._buf[0])     # the frame from delay_frames ticks ago
```
A FIFO buffer of length `delay_frames+1` delays the **entire** `Perception` (distance, rel_speed,
ttc, lead_speed, lead_decel, detected). At `FIXED_DT=0.05 s` (20 FPS):

| delay_frames | 0 | 4 | 8 | 16 |
|---|---|---|---|---|
| seconds | 0.0 | 0.2 | 0.4 | 0.8 |

The runner's own detection buffer is `deque(maxlen=1)` — a no-op — so latency is counted exactly
once (`core/runner.py:116-117`). **[FACT]**

### 2.2 Noise
`degrade.py:72-82`:
```python
perc_out.distance = max(0.0, perc_out.distance + rng.normal(0.0, noise_sigma_m))  # zero-mean, clamped ≥0
...
perc_out.ttc = (perc_out.distance / rs) if rs > 1e-3 else inf                     # ttc recomputed
```
Noise is **zero-mean Gaussian** added to `distance` only, clamped to ≥0; `ttc` is then recomputed
from the noisy distance. **[FACT]**

**Coverage caveat [FACT]:** across all noise rows, `noise_sigma_vr = 0.0` and `dropout_p = 0.0` for
every row. Only *distance* noise was actually exercised. Speed noise (`noise_sigma_vr`, which feeds
`required_decel` via `v_l`/`a_l`) and dropout remain untested (see §7).

### 2.3 Latched braking (key to the noise artifact)
`control/base_controller.py:30-45`:
```python
if desired > 0.0: self._engaged = True          # never cleared
if self._engaged:
    self._brake_held = max(self._brake_held, desired)   # monotonic — never decreases
    return self._brake_held
```
Once any controller commands brake, it stays engaged for the rest of the run and brake force never
decreases. All three gate identically on `(perc.detected or self._engaged)`
(`baseline_static_ttc.py:28`, `proposed_dynamic_ttc.py:39`, `proposed_enhanced.py:52`). **[FACT]**

**[INTERPRETATION]** Latch + threshold-crossing decisions + zero-mean noise → a **one-sided
early-trigger bias**: a negative distance sample makes the object *look closer*, trips the threshold
early, and latches permanently; the offsetting positive samples arrive too late to matter because
the latch cannot be released.

---

## 3. Sanity checks

**[FACT]** Using the in-file `delay_frames=0` / `noise_sigma_m=0.0` rows as baseline:

| Check | baseline rows compared | mismatches vs `matrix_*`/`lead_matrix_*` original |
|---|---|---|
| cut-in latency | 150 | 0 |
| cut-in noise | 150 | 0 |
| lead-brake latency | 150 | **1** |
| lead-brake noise | 150 | **1** |

The single lead-brake discrepancy is one borderline case (v=60 km/h, μ=0.85, THW=1.0): the original
file *avoided* with 1.5 m clearance, the zero-degradation re-run *collided* at 21.8 km/h. Seeds
differ (they are derived from the run label) but are irrelevant at zero noise/dropout — the RNG is
never drawn (`degrade.py:72,77,85`). **[INTERPRETATION]** This is CARLA physics / collision-timing
nondeterminism across separate sessions on a single marginal case (1/150 = 0.7%); it does not change
any conclusion. Per the analysis protocol, helped/hurt counts below use the **in-file** zero-degradation
rows as baseline.

---

## 4. Results — Latency

### 4.1 Rc (collision-avoidance rate) vs delay

**Cut-in** — Rc = avoided / N (overall = 50, dry = wet = 25): **[FACT]**

| controller | delay 0 | 4 (0.2s) | 8 (0.4s) | 16 (0.8s) |
|---|---|---|---|---|
| baseline | 44% (dry 68 / wet 20) | 26% (52 / 0) | 18% (36 / 0) | 0% (0 / 0) |
| proposed | 56% (68 / 44) | 44% (52 / 36) | 36% (36 / 36) | 0% (0 / 0) |
| proposed_enhanced | **72%** (76 / 68) | **0%** (0 / 0) | **0%** | **0%** |

**Lead-brake:** **[FACT]**

| controller | delay 0 | 4 | 8 | 16 |
|---|---|---|---|---|
| baseline | 34% (dry 56 / wet 12) | 16% (32 / 0) | 10% (20 / 0) | 2% (4 / 0) |
| proposed | 40% (56 / 24) | 26% (32 / 20) | 16% (20 / 12) | 2% (4 / 0) |
| proposed_enhanced | **68%** (72 / 64) | **2%** (0 / 4) | 2% (4 / 0) | 0% |

### 4.2 Finding (A): required-decel is best at 0 delay but collapses fastest — CONFIRMED

**[FACT]** `proposed_enhanced` leads at delay 0 (72% cut-in, 68% lead) but drops to **0% / 2%** at
just delay=4 (0.2 s), while `proposed` degrades gracefully (56→44→36 cut-in; 40→26→16 lead). A
**crossover** occurs at delay ≥ 4: both `proposed` and even `baseline` overtake `proposed_enhanced`.

**Mechanism — near-limit braking with no margin. [FACT + code]**
At delay 0, mean clearance of *avoided* cases (`s_clearance`) is smallest for enhanced:

| controller | mean s_clearance @delay0 (cut-in / lead) |
|---|---|
| baseline | 2.13 m / 1.36 m |
| proposed | 2.59 m / 1.75 m |
| proposed_enhanced | **0.64 m / 0.91 m** |

And mean `a_req_at_brake / a_max` at brake onset (cut-in): baseline 0.77, proposed 0.72,
**enhanced 0.90**. This equals `REQ_FULL_FRAC = 0.9` (`proposed_enhanced.py:25,44`) — by construction
it fires full braking only when the required deceleration reaches ~90% of the friction ceiling μ·g.

**[INTERPRETATION]** Because `proposed_enhanced` triggers right at the friction limit, it stops with
almost no spatial margin (~0.6–0.9 m). A 0.2 s latency shifts every decision one 0.2 s-window late;
that late distance is exactly the missing margin, so nearly every avoided case flips to a collision.
The TTC-based `proposed` triggers with 2–2.6 m of slack, which absorbs a couple of delayed frames —
hence its graceful decline. This is a **fragility-vs-optimality trade-off**, not a bug.

### 4.3 Impact-speed mitigation (collision_speed_kmh)

**[FACT]** Reported two ways. *Pooled* = mean over each controller's own collided rows (biased —
each controller collides on a different case set). *Same-collided-set* = restricted to cases where
**all three** controllers collide (apples-to-apples).

At delay 0:

| | cut-in pooled | cut-in same-set (n=11) | lead pooled | lead same-set (n=13) |
|---|---|---|---|---|
| baseline | 15.5 | 22.9 | 10.2 | 7.2 |
| proposed | 12.0 | 16.6 | 8.3 | 6.1 |
| proposed_enhanced | 13.0 | **13.6** | 3.5 | **3.6** |

**[INTERPRETATION]** Pooled means understate baseline's severity (it "wins" easy cases it also
avoids). On the same set, `proposed_enhanced` gives the lowest impact speed at delay 0
(13.6 vs 22.9 km/h cut-in; 3.6 vs 7.2 lead). But at delay 16 the ordering collapses — cut-in
same-set (n=50): baseline 18.9, proposed 17.0, **enhanced 19.4 (worst)** — consistent with §4.2.

---

## 5. Results — Noise

### 5.1 Rc vs σ (distance noise)

**Cut-in:** **[FACT]**

| controller | σ=0 | 0.5 | 1.0 | 2.0 |
|---|---|---|---|---|
| baseline | 44% | 46% | 48% | 54% |
| proposed | 56% | 58% | 62% | 64% |
| proposed_enhanced | 72% | 72% | 68% | 74% |

**Lead-brake:** **[FACT]**

| controller | σ=0 | 0.5 | 1.0 | 2.0 |
|---|---|---|---|---|
| baseline | 34% | 36% | 38% | 46% |
| proposed | 40% | 42% | 46% | 52% |
| proposed_enhanced | 68% | 78% | 78% | **80%** |

### 5.2 Finding (B): adding distance noise *raises* Rc — CONFIRMED (with a code-truthful nuance)

**[FACT]** Rc rises (near-)monotonically with σ for every controller in both scenarios.

**Evidence of earlier triggering — `t_c_warn` (TTC at brake onset) vs σ. [FACT]**

| scenario / controller | σ=0 | 0.5 | 1.0 | 2.0 |
|---|---|---|---|---|
| cut-in baseline | 1.38 | 1.39 | 1.41 | 1.50 |
| cut-in proposed | 1.45 | 1.46 | 1.48 | 1.54 |
| cut-in enhanced | 1.18 | 1.19 | 1.21 | 1.31 |
| lead baseline | 1.52 | 4.32 | 7.93 | 25.29 |
| lead proposed | 1.62 | 13.57 | 8.61 | 16.91 |
| lead enhanced | 5.78 | 9.28 | 9.46 | 48.57 |

Braking starts at higher TTC as σ grows (= earlier). In lead-brake the values inflate enormously
because, during the car-following phase before the lead brakes, closing speed ≈ 0; a negative
distance sample trips the threshold and `ttc = distance/rel_speed` is near-infinite → a **spurious
early trigger** that then latches.

**Helped (avoided 0→1) / Hurt (1→0) vs each controller's own σ=0 baseline. [FACT]**

| scenario | baseline | proposed | proposed_enhanced |
|---|---|---|---|
| cut-in (σ 0.5/1/2 summed) | +8 / −0 | +8 / −0 | **+3 / −4** |
| lead-brake | +11 / −2 | +12 / −2 | +16 / −0 |

**[INTERPRETATION — the artifact]** Because the test matrix is **100% conflict cases**
(`core/conflict.py`: every case collides if unbraked), braking *earlier* — even for the wrong reason
— can only help or be neutral; it is **never penalized**. Combined with the latch (§2.3), zero-mean
distance noise produces a one-sided benefit: negative samples cause early latched braking (helps);
positive samples cannot un-latch (no harm). So the observed "noise improves robustness" is an
**evaluation artifact**, not a real property. In the real world this behaviour is *nuisance /
false-positive braking*, which carries a cost this matrix never measures.

**[INTERPRETATION — code-truthful nuance]** The pre-registered hypothesis "required-decel is hurt
*most* by noise (it works at the limit)" holds in **cut-in** — enhanced is the only controller where
noise hurts as much as it helps (+3/−4), because its razor-thin margin (§4.2) lets cases flip *both*
ways. But it does **not** hold in **lead-brake** (enhanced +16/−0): there enhanced already triggers
very early (`t_c_warn` 5.78 s vs 1.52 s for baseline), so a negative distance sample only reinforces
already-early braking and cannot flip an avoided case to a collision. The nuance is fully explained
by margin size × latch, and is reported as the code/data show it rather than forced to the hypothesis.

### 5.3 Impact speed vs noise
**[FACT]** Noise barely changes impact speed. Lead same-set at σ=2.0 (n=10): baseline 6.4, proposed
5.3, enhanced 3.1 km/h (vs σ=0: 7.2 / 6.1 / 3.6). Cut-in same-set σ=2.0 (n=10): 23.5 / 17.1 / 14.6.
The controller ranking on severity is unchanged by distance noise.

---

## 6. Anomalies & mechanisms (summary)

| # | Observation | Mechanism (code) |
|---|---|---|
| A | `proposed_enhanced` best at 0 delay (72/68%), ~0% by delay 4; `proposed` overtakes it | Triggers at μ·g limit (`REQ_FULL_FRAC=0.9`, `proposed_enhanced.py:44`) → ~0.6–0.9 m clearance → 0.2 s latency erases the margin |
| B | Distance noise *raises* Rc for all controllers | Zero-mean noise + latch (`base_controller.py:30-45`) + threshold crossing + **conflict-only matrix** (`core/conflict.py`) → one-sided early-trigger bias, never penalized |
| B′ | Enhanced hurt by noise in cut-in (+3/−4) but not lead (+16/−0) | Margin size × latch: thin-margin cut-in flips both ways; early-triggering lead only reinforces |
| C | Lead-brake `t_c_warn` explodes under noise (up to ~48 s) | During following phase rel_speed≈0 → noisy `ttc=distance/rel_speed`→∞ = spurious triggers (`degrade.py:82`) |
| D | Only distance noise tested | `noise_sigma_vr` and `dropout_p` = 0 in all rows (config sweep restricted to `NOISE_SIGMA_M_SWEEP`) |

---

## 7. Methodological implications

- **[INTERPRETATION] Conflict-only blind spot.** With 100% conflict cases, the evaluation can only
  reward braking and never punishes early/false braking. This is why noise looks beneficial. Add
  **non-conflict cases** (unbraked ego would *not* collide) and report a **false-positive /
  unnecessary-braking rate** so that early triggering carries a cost.
- **[INTERPRETATION] Report impact speed, not only Rc.** Rc is binary; on the same-collided-set,
  impact speed cleanly separates the controllers (e.g. enhanced 3.6 vs baseline 7.2 km/h in lead at
  delay 0) and avoids the pooled-mean selection bias.
- **[INTERPRETATION] Paper cautions.**
  - Do **not** claim "distance noise improves robustness" — it is a conflict-only + latch artifact.
  - Do **not** present `proposed_enhanced` as simply "best" without stating its **latency fragility**
    (72%→0% at 0.2 s): it is optimal at zero delay and brittle under any perception lag.
  - `proposed` (adaptive TTC) is the most **latency-robust** of the three.

---

## 8. Limitations & future work

- **[FACT/INTERPRETATION] Abstracted surrogate.** Degradation is injected on ground-truth
  `Perception`, not a real sensor/detector pipeline; real perception errors are correlated and
  state-dependent, unlike the i.i.d. zero-mean noise here.
- **[FACT] Untested degradations.** `noise_sigma_vr` (speed noise) and `dropout_p` were 0 throughout.
  Speed noise is especially relevant to `proposed_enhanced`, which consumes `v_l`/`a_l` through
  `required_decel` — future sweeps should exercise `NOISE_SIGMA_VR_SWEEP` and `DROPOUT_P_SWEEP`.
- **[INTERPRETATION] Latch biases noise results positive.** A releasable / debounced brake policy
  would remove the free lunch from negative distance noise and give a fairer robustness picture.
- **[INTERPRETATION] Add non-conflict cases** (see §7) to expose false-positive braking.
- **[FACT] One nondeterministic case** (1/150 lead-brake, §3) — consider fixing the CARLA seed /
  running the zero-degradation baseline in the same session for exact reproducibility.
