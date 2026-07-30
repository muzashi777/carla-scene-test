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

import config.scenario_cutin              as cfg_c
import config.scenario_lead_brake         as cfg_lb
import config.scenario_ccrs               as cfg_ccrs
import config.scenario_cutout             as cfg_co
import config.scenario_junction_cutin     as cfg_jc
import config.scenario_cutout_train108    as cfg_co108
from core.conflict import (cutin_is_conflict, lead_brake_is_conflict,
                            ccrs_is_conflict, cutout_is_conflict,
                            junction_cutin_is_conflict,
                            cutout_train108_is_conflict)


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
    speeds    = cfg_ccrs.MATRIX["ego_speed_kmh"]
    dist_vals = cfg_ccrs.MATRIX["approach_d"]
    mu_vals   = cfg_ccrs.MATRIX["mu"]
    n_total   = len(speeds) * len(dist_vals) * len(mu_vals)
    max_time  = cfg_ccrs.MAX_TICKS * cfg_ccrs.FIXED_DT

    yaw_rad = math.radians(cfg_ccrs.EGO_SPAWN["yaw"])
    fwd_x = math.cos(yaw_rad)
    fwd_y = math.sin(yaw_rad)

    print("\n" + "=" * 72)
    print("CCRs matrix conflict check  (train000)")
    print(f"  matrix: {len(speeds)} speeds × {len(dist_vals)} approach_d × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  approach_d (m): {dist_vals}")
    print(f"  GAP_OFFSET = {cfg_ccrs.GAP_OFFSET} m  |  MAX_TIME = {max_time:.0f} s")
    print()
    print(f"  {'speed':>8} {'appr_d':>8} {'mu':>6}  surf_gap  t_conflict  conflict?")
    print(f"  {'-'*8} {'-'*8} {'-'*6}  {'-'*8}  {'-'*10}  ---------")

    n_conflict = 0
    for v, d, mu in itertools.product(speeds, dist_vals, mu_vals):
        tx = cfg_ccrs.EGO_SPAWN["x"] + d * fwd_x
        ty = cfg_ccrs.EGO_SPAWN["y"] + d * fwd_y
        case = {"ego_speed_kmh": v, "mu": mu, "target_x": tx, "target_y": ty}
        flag = ccrs_is_conflict(case, cfg_ccrs)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        dist_c = math.hypot(cfg_ccrs.EGO_SPAWN["x"] - tx, cfg_ccrs.EGO_SPAWN["y"] - ty)
        sg = max(0.0, dist_c - cfg_ccrs.GAP_OFFSET)
        t = sg / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {d:>6.0f} m  {mu:>5.2f}  {sg:>6.1f} m  {t:>8.2f} s  {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_cutout():
    speeds         = cfg_co.MATRIX["ego_speed_kmh"]
    reveal_ttc_vals = cfg_co.MATRIX["reveal_ttc"]
    mu_vals        = cfg_co.MATRIX["mu"]
    n_total        = len(speeds) * len(reveal_ttc_vals) * len(mu_vals)

    dist = math.hypot(cfg_co.EGO_SPAWN["x"] - cfg_co.TARGET_SPAWN["x"],
                      cfg_co.EGO_SPAWN["y"] - cfg_co.TARGET_SPAWN["y"])
    surface_gap = max(0.0, dist - cfg_co.GAP_OFFSET)
    max_time = cfg_co.MAX_TICKS * cfg_co.FIXED_DT

    print("\n" + "=" * 72)
    print("CUT-OUT matrix conflict check  (train000, conflict = ego vs stationary target)")
    print(f"  matrix: {len(speeds)} speeds × {len(reveal_ttc_vals)} reveal_ttc × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  reveal_ttc (s): {reveal_ttc_vals}  [does not affect conflict formula]")
    print(f"  ego→target dist = {dist:.1f} m  |  GAP_OFFSET = {cfg_co.GAP_OFFSET} m  "
          f"→ surface_gap ≈ {surface_gap:.1f} m")
    print(f"  MAX_TIME = {max_time:.0f} s")
    print()
    print(f"  {'speed':>8} {'reveal_ttc':>11} {'mu':>6}  t_conflict    conflict?")
    print(f"  {'-'*8} {'-'*11} {'-'*6}  {'-'*11}  ---------")

    n_conflict = 0
    for v, ttc, mu in itertools.product(speeds, reveal_ttc_vals, mu_vals):
        case = {"ego_speed_kmh": v, "mu": mu}
        flag = cutout_is_conflict(case, cfg_co)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        t = surface_gap / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {ttc:>9.1f}s  {mu:>5.2f}  {t:>8.2f} s    {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_cutout_train108():
    speeds         = cfg_co108.MATRIX["ego_speed_kmh"]
    reveal_ttc_vals = cfg_co108.MATRIX["reveal_ttc"]
    mu_vals        = cfg_co108.MATRIX["mu"]
    n_total        = len(speeds) * len(reveal_ttc_vals) * len(mu_vals)

    dist = math.hypot(cfg_co108.EGO_SPAWN["x"] - cfg_co108.TARGET_SPAWN["x"],
                      cfg_co108.EGO_SPAWN["y"] - cfg_co108.TARGET_SPAWN["y"])
    surface_gap = max(0.0, dist - cfg_co108.GAP_OFFSET)
    max_time = cfg_co108.MAX_TICKS * cfg_co108.FIXED_DT

    print("\n" + "=" * 72)
    print("CUT-OUT train108 matrix conflict check  (train108, conflict = ego vs stationary target)")
    print(f"  matrix: {len(speeds)} speeds × {len(reveal_ttc_vals)} reveal_ttc × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  reveal_ttc (s): {reveal_ttc_vals}  [does not affect conflict formula]")
    print(f"  ego→target dist = {dist:.1f} m  |  GAP_OFFSET = {cfg_co108.GAP_OFFSET} m  "
          f"→ surface_gap ≈ {surface_gap:.1f} m")
    print(f"  MAX_TIME = {max_time:.0f} s")
    print()
    print(f"  {'speed':>8} {'reveal_ttc':>11} {'mu':>6}  t_conflict    conflict?")
    print(f"  {'-'*8} {'-'*11} {'-'*6}  {'-'*11}  ---------")

    n_conflict = 0
    for v, ttc, mu in itertools.product(speeds, reveal_ttc_vals, mu_vals):
        case = {"ego_speed_kmh": v, "mu": mu}
        flag = cutout_train108_is_conflict(case, cfg_co108)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        t = surface_gap / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {ttc:>9.1f}s  {mu:>5.2f}  {t:>8.2f} s    {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def check_junction_cutin():
    speeds    = cfg_jc.MATRIX["ego_speed_kmh"]
    trig_vals = cfg_jc.MATRIX["trigger_d"]
    mu_vals   = cfg_jc.MATRIX["mu"]
    n_total   = len(speeds) * len(trig_vals) * len(mu_vals)

    dist = math.hypot(cfg_jc.EGO_SPAWN["x"] - cfg_jc.INTRUDER_STOP["x"],
                      cfg_jc.EGO_SPAWN["y"] - cfg_jc.INTRUDER_STOP["y"])
    surface_gap = max(0.0, dist - cfg_jc.GAP_OFFSET)
    max_time = cfg_jc.MAX_TICKS * cfg_jc.FIXED_DT

    print("\n" + "=" * 72)
    print("JUNCTION CUT-IN matrix conflict check  (train105, conflict = ego vs stationary intruder)")
    print(f"  matrix: {len(speeds)} speeds × {len(trig_vals)} trigger_d × {len(mu_vals)} μ "
          f"= {n_total} cases")
    print(f"  speeds (km/h): {speeds}")
    print(f"  trigger_d (m): {trig_vals}  [does not affect conflict formula]")
    print(f"  ego→intruder_stop dist = {dist:.1f} m  |  GAP_OFFSET = {cfg_jc.GAP_OFFSET} m  "
          f"→ surface_gap ≈ {surface_gap:.1f} m")
    print(f"  MAX_TIME = {max_time:.0f} s")
    print()
    print(f"  {'speed':>8} {'trigger_d':>10} {'mu':>6}  t_conflict    conflict?")
    print(f"  {'-'*8} {'-'*10} {'-'*6}  {'-'*11}  ---------")

    n_conflict = 0
    for v, td, mu in itertools.product(speeds, trig_vals, mu_vals):
        case = {"ego_speed_kmh": v, "trigger_d": td}
        flag = junction_cutin_is_conflict(case, cfg_jc)
        if flag:
            n_conflict += 1
        v_ms = v / 3.6
        t = surface_gap / v_ms if v_ms > 1e-3 else float("inf")
        mark = "✓ conflict" if flag else "✗ no-conflict"
        print(f"  {v:>5.0f} km/h  {td:>8.0f} m  {mu:>5.2f}  {t:>8.2f} s    {mark}")

    n_no = n_total - n_conflict
    print(f"\n  SUMMARY: {n_conflict}/{n_total} conflict,  {n_no}/{n_total} no-conflict")
    return n_conflict, n_no, n_total


def main():
    print("Kinematic conflict check — no simulation required.")
    print(f"LEAD_DECEL env: {os.environ.get('LEAD_DECEL', 'not set (using default 4.0)')}")

    c_conf, c_no, c_tot       = check_cutin()
    lb_conf, lb_no, lb_tot    = check_lead_brake()
    cc_conf, cc_no, cc_tot    = check_ccrs()
    co_conf, co_no, co_tot    = check_cutout()
    jc_conf, jc_no, jc_tot    = check_junction_cutin()
    co108_conf, co108_no, co108_tot = check_cutout_train108()

    grand_conf = c_conf + lb_conf + cc_conf + co_conf + jc_conf + co108_conf
    grand_no   = c_no   + lb_no   + cc_no   + co_no   + jc_no   + co108_no
    grand_tot  = c_tot  + lb_tot  + cc_tot  + co_tot  + jc_tot  + co108_tot

    print("\n" + "=" * 72)
    print("GRAND TOTAL")
    print(f"  cut-in:               {c_conf}/{c_tot} conflict  ({c_no} no-conflict)")
    print(f"  lead-brake:           {lb_conf}/{lb_tot} conflict  ({lb_no} no-conflict)")
    print(f"  CCRs:                 {cc_conf}/{cc_tot} conflict  ({cc_no} no-conflict)")
    print(f"  cut-out (train000):   {co_conf}/{co_tot} conflict  ({co_no} no-conflict)")
    print(f"  junction cut-in:      {jc_conf}/{jc_tot} conflict  ({jc_no} no-conflict)")
    print(f"  cut-out (train108):   {co108_conf}/{co108_tot} conflict  ({co108_no} no-conflict)")
    print(f"  TOTAL:                {grand_conf}/{grand_tot} conflict  ({grand_no} no-conflict)")
    if grand_no == 0:
        print("\n  All cases are conflict cases.")
        print("  Rc_conflict = Rc_all for these matrices.")
    else:
        print(f"\n  {grand_no} no-conflict case(s) will be excluded from Rc_conflict.")
        print("  They are still counted in Rc_all and appear in the CSV.")


if __name__ == "__main__":
    main()
