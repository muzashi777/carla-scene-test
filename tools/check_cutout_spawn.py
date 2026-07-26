#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/check_cutout_spawn.py — offline spawn-feasibility checker for the Cut-out scenario.
No CARLA required.

For every (ego_speed_kmh, reveal_ttc) cell in the cut-out matrix, computes:
  1. headway_d = max(FIXED_HEADWAY_THW × ego_ms, MIN_HEADWAY_M)
  2. spawn_gap = headway_d − 2×half_len  (positive = clear)
  3. cutout_trigger_d = max(CUTOUT_TRIGGER_TTC × ego_ms,
                            reveal_ttc × ego_ms + GAP_OFFSET − headway_d)
  4. lead-to-target surface gap at trigger  (≈ trigger_d − 2×half_len)
  5. actual_reveal_ttc = (headway_d + trigger_d − GAP_OFFSET) / ego_ms
     (may differ from matrix when TTC floor clips)
  6. lead TTC at trigger = trigger_d / ego_ms  (time budget for lane-change arc)
  7. estimated forward clearance distance needed (simple arc model):
       arc_fwd ≈ CUTOUT_LANE_WIDTH / tan(CUTOUT_HEADING_DEG) + steer ramp ≈ 5.2 m constant

VERDICT column:
  OK                  — no clamp needed; spawns and triggers cleanly; arc clears.
  TTC_FLOOR_CLIPS     — trigger_d raised by TTC floor; actual_reveal_ttc > matrix value
                         (scenario is easier than intended but safe — lead has more time).
  SPAWN_FIX           — needs headway clamp but trigger gap is viable.
  TIGHT/INFEASIBLE    — trigger gap < trigger_margin; lead cannot clear before hitting target.
  TRIGGER_D_NEGATIVE  — trigger distance turns negative (unreachable geometry).

Usage:
    python tools/check_cutout_spawn.py
    python tools/check_cutout_spawn.py --half-len 2.07 --clearance 0.5 --trigger-margin 1.0

All values are centre-to-centre metres unless noted as 'surf' (surface gap).
half_len default = GAP_OFFSET / 2 (proxy; replace with CARLA bounding-box readout).
"""
import argparse
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import config.scenario_cutout as cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--half-len", type=float, default=cfg.GAP_OFFSET / 2,
                        help="Vehicle half-length (extent.x, m). Default: GAP_OFFSET/2 = %(default).2f m")
    parser.add_argument("--clearance", type=float, default=getattr(cfg, "SPAWN_CLEARANCE_M", 0.5),
                        help="Minimum surface clearance at spawn (m). Default: %(default)s m")
    parser.add_argument("--trigger-margin", type=float,
                        default=getattr(cfg, "MIN_TRIGGER_SURF_GAP_M", 1.0),
                        help="Min lead-to-target surface gap at trigger for safe manoeuvre (m). "
                             "Default: %(default)s m")
    args = parser.parse_args()

    half_len       = args.half_len
    clearance      = args.clearance
    trigger_margin = args.trigger_margin
    thw            = cfg.FIXED_HEADWAY_THW
    min_hw_m       = getattr(cfg, "MIN_HEADWAY_M", 5.0)
    trigger_ttc    = getattr(cfg, "CUTOUT_TRIGGER_TTC", 1.5)
    gap_offset     = cfg.GAP_OFFSET
    lead_tgt_ext   = 2.0 * half_len   # lead.extent.x + target.extent.x (same model)

    # Estimated forward distance for lead to clear CUTOUT_LANE_WIDTH laterally.
    # Simple arc model: heading offset = CUTOUT_HEADING_DEG, so lateral velocity ≈ v×sin(H).
    # Forward dist = lateral_target / tan(H).  Additionally add approx half-arc for ramp-up.
    heading_deg = getattr(cfg, "CUTOUT_HEADING_DEG", 30.0)
    lane_width  = getattr(cfg, "CUTOUT_LANE_WIDTH", 1.5)
    _h = math.radians(heading_deg)
    # Full steer+straighten arc: 2 × (lateral / tan(heading)) when heading is maintained
    arc_fwd_est = 2.0 * lane_width / math.tan(_h) if math.tan(_h) > 1e-6 else 10.0

    bbox_min_hw = 2.0 * half_len + clearance
    # Effective minimum headway: max of bbox constraint and MIN_HEADWAY_M
    eff_min_hw  = max(bbox_min_hw, min_hw_m)

    speeds   = cfg.MATRIX["ego_speed_kmh"]
    ttc_vals = cfg.MATRIX["reveal_ttc"]

    W = 115
    print("=" * W)
    print("Cut-out spawn feasibility (TTC-based trigger) — no CARLA required")
    print(f"  FIXED_HEADWAY_THW={thw}s  MIN_HEADWAY_M={min_hw_m}m  "
          f"CUTOUT_TRIGGER_TTC={trigger_ttc}s  GAP_OFFSET={gap_offset}m")
    print(f"  vehicle half-len={half_len:.3f}m  "
          f"bbox_min_hw={bbox_min_hw:.3f}m  eff_min_hw={eff_min_hw:.3f}m")
    print(f"  trigger_margin={trigger_margin}m  arc_fwd_est≈{arc_fwd_est:.1f}m "
          f"(CUTOUT_HEADING_DEG={heading_deg}°, CUTOUT_LANE_WIDTH={lane_width}m)")
    print()

    hdr = (f"{'spd':>5} {'ttc':>5} | {'hw_d':>6} {'spwn_gap':>9} | "
           f"{'trig_d':>7} {'tsurf':>7} {'act_rtc':>8} {'lead_ttc':>9} | "
           f"{'arc_ok':>6} | VERDICT")
    print(hdr)
    print("-" * W)

    n_ok, n_clip, n_fix, n_infeas, n_neg = 0, 0, 0, 0, 0
    clipped_cells = []
    infeasible_cells = []

    for speed in speeds:
        ego_ms    = speed / 3.6
        hw_nom    = thw * ego_ms
        hw_d      = max(hw_nom, eff_min_hw)
        spawn_gap = hw_d - 2.0 * half_len

        for ttc in ttc_vals:
            formula_d = ttc * ego_ms + gap_offset - hw_d
            floor_d   = trigger_ttc * ego_ms
            trig_d    = max(formula_d, floor_d)
            floor_clips = floor_d > formula_d + 1e-6

            trig_surf  = trig_d - lead_tgt_ext
            actual_rtc = (hw_d + trig_d - gap_offset) / ego_ms if ego_ms > 1e-6 else 0.0
            lead_ttc   = trig_d / ego_ms if ego_ms > 1e-6 else 0.0
            arc_ok     = "YES" if trig_d >= arc_fwd_est else "NO"

            if trig_d <= 0:
                verdict = "TRIGGER_D_NEGATIVE"
                n_neg += 1
                infeasible_cells.append((speed, ttc, "negative_trigger_d", trig_surf))
            elif trig_surf < trigger_margin:
                verdict = "TIGHT/INFEASIBLE"
                n_infeas += 1
                infeasible_cells.append((speed, ttc, "trigger_too_close", trig_surf))
            elif floor_clips:
                verdict = "TTC_FLOOR_CLIPS"
                n_clip += 1
                clipped_cells.append((speed, ttc, actual_rtc))
            elif hw_d > hw_nom + 1e-6:
                verdict = "SPAWN_FIX"
                n_fix += 1
            else:
                verdict = "OK"
                n_ok += 1

            clip_note = f"  [floor clips; actual_rtc={actual_rtc:.2f}s]" if floor_clips else ""
            print(f"{speed:>5.0f} {ttc:>5.1f} | {hw_d:>6.3f} {spawn_gap:>9.3f} | "
                  f"{trig_d:>7.3f} {trig_surf:>7.3f} {actual_rtc:>8.3f} {lead_ttc:>9.3f} | "
                  f"{arc_ok:>6} | {verdict}{clip_note}")

    print("-" * W)
    total = len(speeds) * len(ttc_vals)
    print(f"\nSUMMARY ({total} speed×ttc cells, μ-independent):")
    print(f"  OK              : {n_ok}")
    print(f"  TTC_FLOOR_CLIPS : {n_clip}  — trigger_d raised by TTC floor; "
          f"actual reveal_ttc > matrix value (easier but safe)")
    print(f"  SPAWN_FIX       : {n_fix}  — headway clamp needed; trigger gap OK")
    print(f"  TIGHT/INFEASIBLE: {n_infeas}  — trigger gap < {trigger_margin} m (physics concern)")
    print(f"  TRIGGER_NEGATIVE: {n_neg}")

    if clipped_cells:
        print()
        print("TTC_FLOOR_CLIPS detail (lead gets ≥ CUTOUT_TRIGGER_TTC s; actual_reveal_ttc > matrix):")
        print(f"  {'speed':>5}  {'matrix_ttc':>10}  {'actual_rtc':>10}")
        for s, t, art in clipped_cells:
            print(f"  {s:>5.0f}  {t:>10.1f}  {art:>10.3f}")

    if infeasible_cells:
        print()
        print("INFEASIBLE cells (need design change):")
        for s, t, reason, sg in infeasible_cells:
            print(f"  speed={s:.0f} km/h  reveal_ttc={t:.1f} s  "
                  f"trig_surf_gap={sg:.3f} m  [{reason}]")

    print()
    print(f"arc_fwd_est≈{arc_fwd_est:.1f}m is the estimated forward distance for the lead to reach")
    print(f"  {lane_width}m lateral offset (CUTOUT_LANE_WIDTH) given CUTOUT_HEADING_DEG={heading_deg}°.")
    print("  All cells with arc_ok=YES have trig_d > arc_fwd_est → lead clears before reaching target.")
    print()
    print("Note: half_len is a proxy from GAP_OFFSET/2. Run with --half-len <actual value>")
    print("      once you have the CARLA bounding-box readout (ego.bounding_box.extent.x).")


if __name__ == "__main__":
    main()
