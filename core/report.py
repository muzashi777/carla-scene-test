#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone CPEIM summary from a results CSV — does NOT run any simulation.

Reads a CSV produced by write_csv() (from core/metrics.py) and prints a
formatted per-controller table with the 5 CPEIM indices and Rc values.

Usage:
  python core/report.py results/matrix_*.csv
  python core/report.py results/lead_matrix_*.csv
  python core/report.py results/matrix_*.csv results/lead_matrix_*.csv   # multiple files

Aggregation rules (must match the paper's METHOD section):
  Rc_all      = n_avoided / n_total  (all cases)
  Rc_conflict = n_avoided_conflict / n_conflict  (is_conflict==True rows only)
  mean_ab     = mean(a_b_mfdd) where a_b_mfdd > 0  (braked cases that hit 0.8v→0.1v window)
  mean_sc     = mean(s_clearance) where avoided==True  (collision → s=0 is not a clearance)
  mean_tc     = mean(t_c_warn) where t_c_warn > 0  (braking was triggered)
  mean_dv     = mean(dv_speed_var) over all cases
  n_collision = count of rows where avoided==False
"""
import csv
import sys
import os
from collections import defaultdict


def _to_bool(val):
    if isinstance(val, bool):
        return val
    return str(val).strip().lower() in ("true", "1", "yes")


def _to_float(val, default=0.0):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def load_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def aggregate(rows):
    """
    Returns dict: {label: {rc_all, rc_conflict, n, n_conflict, n_collision, avoided,
                            mean_ab, n_ab, mean_sc, n_sc, mean_tc, n_tc, mean_dv,
                            n_collision}}
    """
    by_label = defaultdict(list)
    for r in rows:
        by_label[r["label"]].append(r)

    out = {}
    for label, rs in by_label.items():
        n = len(rs)
        avoided_rs = [r for r in rs if _to_bool(r.get("avoided", False))]
        n_avoid = len(avoided_rs)
        rc_all = n_avoid / n if n else 0.0

        conflict_rs = [r for r in rs if _to_bool(r.get("is_conflict", True))]
        n_conflict = len(conflict_rs)
        n_conflict_avoid = sum(1 for r in conflict_rs if _to_bool(r.get("avoided", False)))
        rc_conflict = n_conflict_avoid / n_conflict if n_conflict else 0.0

        ab_vals = [_to_float(r.get("a_b_mfdd", 0)) for r in rs
                   if _to_float(r.get("a_b_mfdd", 0)) > 0]
        mean_ab = sum(ab_vals) / len(ab_vals) if ab_vals else 0.0

        sc_vals = [_to_float(r.get("s_clearance", 0)) for r in avoided_rs]
        mean_sc = sum(sc_vals) / len(sc_vals) if sc_vals else 0.0

        tc_vals = [_to_float(r.get("t_c_warn", 0)) for r in rs
                   if _to_float(r.get("t_c_warn", 0)) > 0]
        mean_tc = sum(tc_vals) / len(tc_vals) if tc_vals else 0.0

        dv_vals = [_to_float(r.get("dv_speed_var", 0)) for r in rs]
        mean_dv = sum(dv_vals) / n if n else 0.0

        n_collision = n - n_avoid

        out[label] = dict(
            rc_all=rc_all, rc_conflict=rc_conflict,
            n=n, n_conflict=n_conflict, n_collision=n_collision, avoided=n_avoid,
            mean_ab=mean_ab, n_ab=len(ab_vals),
            mean_sc=mean_sc, n_sc=len(sc_vals),
            mean_tc=mean_tc, n_tc=len(tc_vals),
            mean_dv=mean_dv,
        )
    return out


def print_summary(path, label_order=None):
    rows = load_csv(path)
    if not rows:
        print(f"[WARN] {path}: no rows")
        return

    agg = aggregate(rows)
    has_conflict_col = "is_conflict" in rows[0]
    n_total = len(rows)

    print(f"\n{'=' * 76}")
    print(f"CSV: {os.path.basename(path)}  ({n_total} rows, {len(agg)} controllers)")
    if not has_conflict_col:
        print("  [NOTE] is_conflict column absent — treating all rows as conflict (Rc_conflict = Rc_all)")
    print(f"{'=' * 76}")
    print(f"  {'Controller':<22} {'Rc_all':>7} {'Rc_conf':>8} {'Coll':>5} | "
          f"{'MFDD':>6}(n) {'s_c':>5}(n) {'Tc':>5}(n) {'Δv':>6}")
    print(f"  {'-'*22} {'-'*7} {'-'*8} {'-'*5}   {'-'*6}    {'-'*5}    {'-'*5}    {'-'*6}")

    labels = label_order if label_order else list(agg.keys())
    for label in labels:
        if label not in agg:
            continue
        s = agg[label]
        print(f"  {label:<22} {s['rc_all']*100:6.1f}% {s['rc_conflict']*100:7.1f}% "
              f"{s['n_collision']:5d} | "
              f"{s['mean_ab']:5.2f}({s['n_ab']:2d}) "
              f"{s['mean_sc']:4.2f}({s['n_sc']:2d}) "
              f"{s['mean_tc']:4.2f}({s['n_tc']:2d}) "
              f"{s['mean_dv']:6.1f}")

    print(f"  {'':22}   all-N   conflict  #col   MFDD      s_c       Tc        Δv km/h")

    # Delta-Rc vs first controller
    label_list = [l for l in labels if l in agg]
    if len(label_list) >= 2:
        base = label_list[0]
        print(f"\n  Δ Rc vs '{base}':")
        for prop in label_list[1:]:
            delta = (agg[prop]["rc_all"] - agg[base]["rc_all"]) * 100
            print(f"    {prop:<22} {delta:+.1f}%")

    print()


def main():
    paths = sys.argv[1:]
    if not paths:
        print(f"Usage: python {sys.argv[0]} results/matrix_*.csv [results/lead_matrix_*.csv ...]")
        sys.exit(1)
    for p in paths:
        if not os.path.isfile(p):
            print(f"[ERROR] File not found: {p}")
            continue
        print_summary(p)


if __name__ == "__main__":
    main()
