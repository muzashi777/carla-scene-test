#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/check_cutout_spawn.py — offline spawn-feasibility checker for the Cut-out scenario.
No CARLA required.

For every (ego_speed_kmh, reveal_ttc) cell in the cut-out matrix, computes:
  1. headway_d = FIXED_HEADWAY_THW × ego_ms   (spawn distance, centre-to-centre)
  2. spawn_gap = headway_d − 2×half_len        (positive = clear, negative = bounding-box overlap)
  3. clamped headway_d (enforced minimum = 2×half_len + clearance)
  4. cutout_trigger_d after clamping           (distance from lead to target when cut-out fires)
  5. lead-to-target surface gap at trigger     (≈ trigger_d − 2×half_len)

VERDICT column:
  OK                  — no clamp needed; spawns and triggers cleanly.
  SPAWN_FIX           — needs headway clamp but trigger gap is physically viable (> trigger_margin).
  ⚠ TIGHT/INFEASIBLE  — after clamping, trigger gap is < trigger_margin; the lead may collide
                         with the target before completing the lane-change manoeuvre.
  TRIGGER_D_NEGATIVE  — trigger distance turns negative; cannot satisfy reveal_ttc at all.

Usage:
    python tools/check_cutout_spawn.py
    python tools/check_cutout_spawn.py --half-len 2.07 --clearance 0.5 --trigger-margin 1.0

All values are centre-to-centre metres unless noted as 'surf' (surface gap).
half_len default = GAP_OFFSET / 2 (proxy derived from config: ego.extent.x + target.extent.x ≈ GAP_OFFSET).
Replace with the actual CARLA bounding-box readout when available.
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
    parser.add_argument("--trigger-margin", type=float, default=1.0,
                        help="Min lead-to-target surface gap at trigger needed for a safe manoeuvre (m). "
                             "Default: %(default)s m")
    args = parser.parse_args()

    half_len       = args.half_len
    clearance      = args.clearance
    trigger_margin = args.trigger_margin
    min_headway    = 2.0 * half_len + clearance
    thw            = cfg.FIXED_HEADWAY_THW
    gap_offset     = cfg.GAP_OFFSET
    lead_tgt_ext   = 2.0 * half_len   # lead.extent.x + target.extent.x (same model)

    speeds   = cfg.MATRIX["ego_speed_kmh"]
    ttc_vals = cfg.MATRIX["reveal_ttc"]

    print("=" * 90)
    print("Cut-out spawn feasibility — no CARLA required")
    print(f"  FIXED_HEADWAY_THW = {thw} s  |  GAP_OFFSET = {gap_offset} m")
    print(f"  vehicle half-length = {half_len:.3f} m  (2× = {2*half_len:.3f} m; proxy from GAP_OFFSET/2)")
    print(f"  spawn clearance    = {clearance} m  → min_headway = {min_headway:.3f} m")
    print(f"  trigger margin     = {trigger_margin} m  (min lead↔target surface gap for physics manoeuvre)")
    print()

    hdr = (f"{'speed':>6} {'ttc':>5} | {'hw_d':>6} {'spawn_gap':>10} | "
           f"{'clamp_hw':>8} {'trig_d':>7} {'trig_surf_gap':>14} | VERDICT")
    print(hdr)
    print("-" * len(hdr))

    n_ok, n_fix, n_infeasible, n_negative = 0, 0, 0, 0
    infeasible_cells = []

    for speed in speeds:
        ego_ms   = speed / 3.6
        hw_d     = thw * ego_ms
        spawn_gap = hw_d - 2.0 * half_len
        clamped  = max(hw_d, min_headway)
        changed  = clamped > hw_d + 1e-6
        trig_d   = ego_ms * ttc_vals[0] + gap_offset - clamped   # compute with first ttc for the speed header row
        for ttc in ttc_vals:
            trig_d       = ttc * ego_ms + gap_offset - clamped
            trig_surf_gap = trig_d - lead_tgt_ext

            if spawn_gap >= clearance and not changed:
                verdict = "OK"
                n_ok += 1
            elif trig_d <= 0:
                verdict = "TRIGGER_D_NEGATIVE"
                n_negative += 1
                infeasible_cells.append((speed, ttc, "negative_trigger_d"))
            elif trig_surf_gap < trigger_margin:
                verdict = "TIGHT/INFEASIBLE"
                n_infeasible += 1
                infeasible_cells.append((speed, ttc, "trigger_too_close"))
            else:
                verdict = "SPAWN_FIX"
                n_fix += 1

            clamp_note = f"  [clamp {hw_d:.3f}→{clamped:.3f}]" if changed else ""
            print(f"{speed:>6.0f} {ttc:>5.1f} | {hw_d:>6.3f} {spawn_gap:>10.3f} | "
                  f"{clamped:>8.3f} {trig_d:>7.3f} {trig_surf_gap:>14.3f} | {verdict}{clamp_note}")

    print("-" * len(hdr))
    total = len(speeds) * len(ttc_vals)
    print(f"\nSUMMARY ({total} speed×ttc cells, μ-independent):")
    print(f"  OK              : {n_ok}")
    print(f"  SPAWN_FIX       : {n_fix}  (headway clamp will resolve spawn failure; trigger gap OK)")
    print(f"  TIGHT/INFEASIBLE: {n_infeasible}  (spawn fixable but trigger gap < {trigger_margin} m — physics concern)")
    print(f"  TRIGGER_NEGATIVE: {n_negative}")

    if infeasible_cells:
        print()
        print("Cells requiring user decision (matrix cannot be kept 5×5×2 without a design change):")
        for s, t, reason in infeasible_cells:
            ego_ms = s / 3.6
            hw_c   = max(thw * ego_ms, min_headway)
            td     = t * ego_ms + gap_offset - hw_c
            sg     = td - lead_tgt_ext
            t_avail = sg / ego_ms if ego_ms > 1e-6 else float("inf")
            print(f"    speed={s:.0f} km/h  reveal_ttc={t:.1f} s  "
                  f"trigger_surf_gap={sg:.3f} m  time_avail={t_avail:.2f} s  [{reason}]")

    print()
    print("Note: half_len is a proxy from GAP_OFFSET/2. Run with --half-len <actual value>")
    print("      once you have the CARLA bounding-box readout (ego.bounding_box.extent.x).")


if __name__ == "__main__":
    main()
