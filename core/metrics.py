# -*- coding: utf-8 -*-
"""
Stores the 5 CPEIM indices per case, aggregates them into R_c, and computes per-controller statistics.
  s    = clearance distance at full stop (m)  — surface-to-surface gap when ego comes to rest
  a_b  = MFDD: mean fully-developed deceleration over the 0.8v→0.1v interval (m/s²)
  T_c  = warning lead time (s) = TTC at the moment braking begins
  dv   = speed variation (km/h) = v_initial − v_impact (or v_initial if ego stops before impact)
  Rc   = proportion of cases where collision was successfully avoided (reported separately for conflict-only cases and all cases)

Index aggregation notes:
  mean_ab  = average of rows where a_b_mfdd > 0 (braking occurred and the 0.8v→0.1v interval was captured)
  mean_sc  = average of cases where avoided=True (s_clearance=0 in collision cases is not meaningful clearance data)
  mean_tc  = average of rows where t_c_warn > 0 (braking occurred)
  mean_dv  = average over all rows (both avoided and collision)
"""
import csv
import os
from dataclasses import dataclass, asdict


# ── CPEIM weights ──────────────────────────────────────────────────────────────
# WARNING: these values are from the V-VRU scenario (vertical, pedestrian) in liu2025 Table 10,
# NOT from the car-to-car scenario (CCRb / cut-in) used in our tests.
# Do NOT use them to compute a composite score for our cases directly.
# Kept for reference / comparison with the paper only.
W_S, W_AB, W_TC, W_DV, W_RC = 0.1447, 0.0901, 0.2962, 0.0603, 0.4087


@dataclass
class RunRecord:
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    mu: float
    trigger_d: float
    dart_speed_kmh: float
    avoided: bool = False
    collision_with: str = ""
    s_clearance: float = 0.0       # m (>0 = surface-to-surface gap at full stop; 0 = collision)
    a_b_mfdd: float = 0.0          # m/s²
    t_c_warn: float = 0.0          # s
    dv_speed_var: float = 0.0      # km/h
    collision_speed_kmh: float = 0.0
    peak_decel: float = 0.0        # m/s² peak deceleration measured during braking
    brake_distance: float = 0.0    # m distance travelled from brake onset to full stop
    min_dist: float = 0.0
    a_req_at_brake: float = 0.0    # m/s² deceleration "required" at brake onset (indicates how close it is to the μ·g limit)
    a_max: float = 0.0             # m/s² deceleration ceiling = μ·g for this case
    is_conflict: bool = True       # True = ego would collide in this case (kinematic, no-brake) — see core/conflict.py
    result_txt: str = ""
    noise_sigma_m: float = 0.0
    noise_sigma_vr: float = 0.0
    dropout_p: float = 0.0
    dropout_mode: str = "freeze"
    test_mode: str = "original"
    seed: int = 0
    comp_source: str = ""          # source of the compensation latency value L (''=no compensation, 'oracle', 'mismatched')
    comp_L_frames: int = 0         # actual L used for compensation (frames) — may differ from delay_frames in mismatched mode


def score_clearance(s, avoided):
    """Score s according to the i-VISTA criteria in the paper (Table 12)."""
    if not avoided:
        return 0.0
    if s <= 0.6:
        return 1.0
    if s <= 1.2:
        return 0.8
    if s <= 1.8:
        return 0.6
    if s <= 2.4:
        return 0.3
    return 0.0


class MfddTracker:
    """Track the distances at which speed drops to 0.8v0 and 0.1v0 for MFDD calculation."""
    def __init__(self, v0_kmh):
        self.vb = 0.8 * v0_kmh
        self.ve = 0.1 * v0_kmh
        self.s_b = None
        self.s_e = None

    def update(self, v_kmh, dist_travelled):
        if self.s_b is None and v_kmh <= self.vb:
            self.s_b = dist_travelled
        if self.s_e is None and v_kmh <= self.ve:
            self.s_e = dist_travelled

    def mfdd(self):
        if self.s_b is None or self.s_e is None or self.s_e <= self.s_b:
            return 0.0
        return (self.vb ** 2 - self.ve ** 2) / (25.92 * (self.s_e - self.s_b))


def write_csv(records, path):
    if not records:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = list(asdict(records[0]).keys())
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in records:
            w.writerow(asdict(r))


def summarize(records):
    """
    Aggregate results per label: returns dict {label: {rc, rc_all, rc_conflict, n, n_conflict, avoided,
                                                        mean_ab, n_ab, mean_sc, n_sc, mean_tc, n_tc, mean_dv}}

    Aggregation rules:
      rc_all      = n_avoid / n (all cases)
      rc_conflict = n_avoid_conflict / n_conflict (conflict-only cases where is_conflict=True)
      mean_ab     = average a_b_mfdd for rows > 0 (the 0.8v→0.1v interval was captured)
      mean_sc     = average s_clearance for avoided=True only (s=0 in collision cases is not clearance data)
      mean_tc     = average t_c_warn for rows > 0 (braking occurred)
      mean_dv     = average dv_speed_var across all rows

    NOTE: CPEIM composite (mean_score) has been removed because the weights W_S/W_AB/W_TC/W_DV/W_RC
    come from the V-VRU scenario in liu2025, not the car-to-car scenario used in our tests. Applying
    the wrong scenario weights distorts the composite score; the 5 raw indices + Rc are reported instead.
    """
    by_label = {}
    for r in records:
        by_label.setdefault(r.label, []).append(r)
    out = {}
    for label, rs in by_label.items():
        n = len(rs)
        n_avoid = sum(1 for r in rs if r.avoided)
        rc_all = n_avoid / n if n else 0.0

        # Conflict-only Rc (is_conflict is set in the runner file before the simulation starts — see core/conflict.py)
        conflict_rs = [r for r in rs if getattr(r, "is_conflict", True)]
        n_conflict = len(conflict_rs)
        n_conflict_avoid = sum(1 for r in conflict_rs if r.avoided)
        rc_conflict = n_conflict_avoid / n_conflict if n_conflict else 0.0

        # MFDD: average only rows where a value was measured (the 0.8v→0.1v interval must be captured for the value to be > 0)
        ab_vals = [r.a_b_mfdd for r in rs if r.a_b_mfdd > 0]
        mean_ab = sum(ab_vals) / len(ab_vals) if ab_vals else 0.0

        # s_clearance: average only cases where ego stopped successfully (s=0 in collision cases is not true clearance data)
        sc_vals = [r.s_clearance for r in rs if r.avoided]
        mean_sc = sum(sc_vals) / len(sc_vals) if sc_vals else 0.0

        # T_c warning lead time: average only rows where braking occurred
        tc_vals = [r.t_c_warn for r in rs if r.t_c_warn > 0]
        mean_tc = sum(tc_vals) / len(tc_vals) if tc_vals else 0.0

        # Δv speed variation: average across all rows
        mean_dv = sum(r.dv_speed_var for r in rs) / n if n else 0.0

        out[label] = dict(
            rc=rc_all, rc_all=rc_all, rc_conflict=rc_conflict,
            n=n, n_conflict=n_conflict, avoided=n_avoid,
            mean_ab=mean_ab, n_ab=len(ab_vals),
            mean_sc=mean_sc, n_sc=len(sc_vals),
            mean_tc=mean_tc, n_tc=len(tc_vals),
            mean_dv=mean_dv,
        )
    return out
