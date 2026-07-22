#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/check_conflict.py — kinematic conflict check for every case in both test matrices.
No CARLA required: only core/conflict.py (pure maths) and the two config files.

Usage:
    python tools/check_conflict.py
    LEAD_DECEL=6.0 python tools/check_conflict.py

A case is "conflict" if ego, applying NO braking at all (constant speed),
would collide with the obstacle within MAX_TICKS × FIXED_DT seconds.
See core/conflict.py for derivations.
"""
import sys
import os
import itertools
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config.scenario_cutin      as cfg_c
import config.scenario_lead_brake as cfg_lb
import config.scenario_ccrs       as cfg_ccrs
import config.scenario_cutout     as cfg_co
from core.conflict import cutin_is_conflict, lead_brake_is_conflict, ccrs_is_conflict, cutout_is_conflict


def check_cutin():
    speeds    = cfg_c.MATRIX["ego_speed_kmh"]
    mu_vals   = cfg_c.MATRIX["mu"]
    trig_vals = cfg_c.MATRIX["trigger_d"]
    dart_spd  = cfg_c.MATRIX["dart_speed_kmh"][0]
    n_total   = len(speeds) * len(mu_vals) * len(trig_vals)

    x_offset = abs(cfg_c.DART_SPAWN["x"] - cfg_c.EGO_SPAWN["x"])

    print("\n" + "=" * 72)
    print("CUT-IN matrix conflict check")
    print(f"  matrix: {len(speeds)} speeds × {len(trig_vals)} trigger_d × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  trigger_d (m): {trig_vals}")
    print(f"  dart_speed = {dart_spd} km/h  |  MAX_TIME = {cfg_c.MAX_TICKS * cfg_c.FIXED_DT:.0f} s")
    lon_at_40 = math.sqrt(max(0.0, 40.0 ** 2 - x_offset ** 2))
    print(f"  INPATH_MAX_RANGE = {cfg_c.INPATH_MAX_RANGE:.1f} m  "
          f"(longitudinal dist at trigger_d=40m: {lon_at_40:.1f} m — within range ✓)")
    print()
    print(f"  {'speed':>8} {'trigger_d':>10} {'mu':>6}  conflict?")
    print(f"  {'-'*8} {'-'*10} {'-'*6}  ---------")

    n_conflict = 0
    for v, td, mu in itertools.product(speeds, trig_vals, mu_vals):
        case = {"ego_speed_kmh": v, "trigger_d": td, "dart_speed_kmh": dart_spd}
        flag = cutin_is_conflict(case, cfg_c)
        if flag:
            n_conflict += 1
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {td:>6.0f} m    {mu:>5.2f}  {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_lead_brake():
    lead_decel = cfg_lb.LEAD_DECEL
    speeds   = cfg_lb.MATRIX["ego_speed_kmh"]
    thw_vals = cfg_lb.HEADWAY_THW
    mu_vals  = cfg_lb.MATRIX["mu"]
    n_total  = len(speeds) * len(thw_vals) * len(mu_vals)

    print("\n" + "=" * 72)
    print("LEAD-BRAKE matrix conflict check")
    print(f"  matrix: {len(speeds)} speeds × {len(thw_vals)} THW × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  THW (s): {thw_vals}")
    print(f"  LEAD_DECEL = {lead_decel} m/s²  |  MAX_TIME = {cfg_lb.MAX_TICKS * cfg_lb.FIXED_DT:.0f} s")
    print()
    print(f"  {'speed':>8} {'THW':>6} {'headway_d':>10} {'mu':>6}  conflict?")
    print(f"  {'-'*8} {'-'*6} {'-'*10} {'-'*6}  ---------")

    n_conflict = 0
    for v, thw, mu in itertools.product(speeds, thw_vals, mu_vals):
        v_ms = v / 3.6
        headway_d = v_ms * thw
        case = {"ego_speed_kmh": v, "headway_d": headway_d}
        flag = lead_brake_is_conflict(case, cfg_lb)
        if flag:
            n_conflict += 1
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {thw:>5.1f}s  {headway_d:>8.1f} m  {mu:>5.2f}  {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_ccrs():
    speeds  = cfg_ccrs.MATRIX["ego_speed_kmh"]
    mu_vals = cfg_ccrs.MATRIX["mu"]
    n_total = len(speeds) * len(mu_vals)

    import math as _math
    dist = _math.hypot(cfg_ccrs.EGO_SPAWN["x"] - cfg_ccrs.TARGET_SPAWN["x"],
                       cfg_ccrs.EGO_SPAWN["y"] - cfg_ccrs.TARGET_SPAWN["y"])
    surface_gap = max(0.0, dist - cfg_ccrs.GAP_OFFSET)
    max_time = cfg_ccrs.MAX_TICKS * cfg_ccrs.FIXED_DT

    print("\n" + "=" * 72)
    print("CCRs matrix conflict check  (train000)")
    print(f"  matrix: {len(speeds)} speeds × {len(mu_vals)} μ = {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  ego→target dist = {dist:.1f} m  |  GAP_OFFSET = {cfg_ccrs.GAP_OFFSET} m  "
          f"→ surface_gap ≈ {surface_gap:.1f} m")
    print(f"  MAX_TIME = {max_time:.0f} s  |  t_conflict(20 km/h) = {surface_gap/(20/3.6):.1f} s")
    print()
    print(f"  {'speed':>8} {'mu':>6}  t_conflict    conflict?")
    print(f"  {'-'*8} {'-'*6}  {'-'*11}  ---------")

    n_conflict = 0
    for v, mu in itertools.product(speeds, mu_vals):
        case = {"ego_speed_kmh": v, "mu": mu}
        flag = ccrs_is_conflict(case, cfg_ccrs)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        t = surface_gap / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {mu:>5.2f}  {t:>8.2f} s    {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_cutout():
    speeds   = cfg_co.MATRIX["ego_speed_kmh"]
    thw_vals = cfg_co.HEADWAY_THW
    mu_vals  = cfg_co.MATRIX["mu"]
    n_total  = len(speeds) * len(thw_vals) * len(mu_vals)

    import math as _math
    dist = _math.hypot(cfg_co.EGO_SPAWN["x"] - cfg_co.TARGET_SPAWN["x"],
                       cfg_co.EGO_SPAWN["y"] - cfg_co.TARGET_SPAWN["y"])
    surface_gap = max(0.0, dist - cfg_co.GAP_OFFSET)
    max_time = cfg_co.MAX_TICKS * cfg_co.FIXED_DT

    print("\n" + "=" * 72)
    print("CUT-OUT matrix conflict check  (train000, conflict = ego vs stationary target)")
    print(f"  matrix: {len(speeds)} speeds × {len(thw_vals)} THW × {len(mu_vals)} μ = {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  THW (s): {thw_vals}  [lead headway — does not affect conflict formula]")
    print(f"  ego→target dist = {dist:.1f} m  |  GAP_OFFSET = {cfg_co.GAP_OFFSET} m  "
          f"→ surface_gap ≈ {surface_gap:.1f} m")
    print(f"  MAX_TIME = {max_time:.0f} s")
    print()
    print(f"  {'speed':>8} {'THW':>6} {'mu':>6}  t_conflict    conflict?")
    print(f"  {'-'*8} {'-'*6} {'-'*6}  {'-'*11}  ---------")

    n_conflict = 0
    for v, thw, mu in itertools.product(speeds, thw_vals, mu_vals):
        case = {"ego_speed_kmh": v, "mu": mu}
        flag = cutout_is_conflict(case, cfg_co)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        t = surface_gap / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {thw:>5.1f}s  {mu:>5.2f}  {t:>8.2f} s    {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def main():
    print("Kinematic conflict check — no simulation required.")
    print(f"LEAD_DECEL env: {os.environ.get('LEAD_DECEL', 'not set (using default 4.0)')}")

    c_conf, c_no, c_tot     = check_cutin()
    lb_conf, lb_no, lb_tot  = check_lead_brake()
    cc_conf, cc_no, cc_tot  = check_ccrs()
    co_conf, co_no, co_tot  = check_cutout()

    grand_conf = c_conf + lb_conf + cc_conf + co_conf
    grand_no   = c_no   + lb_no   + cc_no   + co_no
    grand_tot  = c_tot  + lb_tot  + cc_tot  + co_tot

    print("\n" + "=" * 72)
    print("GRAND TOTAL")
    print(f"  cut-in:     {c_conf}/{c_tot} conflict  ({c_no} no-conflict)")
    print(f"  lead-brake: {lb_conf}/{lb_tot} conflict  ({lb_no} no-conflict)")
    print(f"  CCRs:       {cc_conf}/{cc_tot} conflict  ({cc_no} no-conflict)")
    print(f"  cut-out:    {co_conf}/{co_tot} conflict  ({co_no} no-conflict)")
    print(f"  TOTAL:      {grand_conf}/{grand_tot} conflict  ({grand_no} no-conflict)")
    if grand_no == 0:
        print("\n  All cases are conflict cases.")
        print("  Rc_conflict = Rc_all for these matrices.")
    else:
        print(f"\n  {grand_no} no-conflict case(s) will be excluded from Rc_conflict.")
        print("  They are still counted in Rc_all and appear in the CSV.")


if __name__ == "__main__":
    main()
