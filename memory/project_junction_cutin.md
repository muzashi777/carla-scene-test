---
name: project-junction-cutin
description: Junction cut-in scenario (5th) on train105 — files added, conflict counts, and values still to tune
metadata:
  type: project
---

Fifth AEB test scenario added on map `train105` (Rev 2026-07-29).

**Files created:** `config/scenario_junction_cutin.py`, `core/scenario_junction_cutin.py`, `core/runner_junction_cutin.py`, `run_single_junction_cutin.py`, `run_matrix_junction_cutin.py`.  
**Files edited additively:** `core/conflict.py`, `tools/check_conflict.py`, `README.md`, `TECHNICAL_DOC.md`.

Conflict check: 50/50, grand total 250/250.

**Why:** extend the harness to a junction intersection scenario (intruder emerging from the left) using existing physics-steering infrastructure from cut-out.

**Values the user must tune in CARLA before running the matrix:**
- `INTRUDER_STOP` — placeholder at (39.56, −165.0, 11.12); set to observed stop position
- `SPECTATOR_TF` — currently `None`; set once tuned
- `TURN_TRIGGER_D` — default 30 m; adjust from live observation
- `TURN_HEADING_DEG` — default −90°; negate if intruder turns the wrong way
- `JCUTIN_STEER_K`, `JCUTIN_STEER_MAX`, `JCUTIN_SETTLE_DEG`, `JCUTIN_SPEED_K`, `JCUTIN_MAX_THROTTLE` — P-controller gains

**How to apply:** when the user starts running junction_cutin on train105, remind them of these TO BE TUNED values and suggest starting with `run_single_junction_cutin.py` to observe the intruder behaviour before running the matrix.
