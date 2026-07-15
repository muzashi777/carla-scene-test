# -*- coding: utf-8 -*-
"""
Iterate through all TEST MATRIX cases for the lead-brake scenario (headless, no display) for each controller in MATRIX_RUNS
then write CSV (results/lead_matrix_*.csv) + summarize R_c comparison → check +20% target (Achievement 2)
Separated from the cut-in scenario (run_matrix.py) for easier per-scenario testing/debugging
Usage:  python run_matrix_lead.py
"""
import sys, os, itertools, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_lead_brake as cfg
from core.carla_session import CarlaSession
from core.runner_lead_brake import run_case
from core.metrics import write_csv, summarize
from perception.yolo_detector import YoloDetector


def build_cases():
    """Build case list from MATRIX
    Gap: USE_THW=True → iterate headway_thw (seconds) / USE_THW=False → iterate headway_d (meters, legacy mode)
    Speed: LEAD_SAME_AS_EGO=True → lead vehicle same speed as ego / False → lead_speed_kmh is a separate variable
    runner will convert THW→actual meters automatically (based on ego speed)
    """
    same = getattr(cfg, "LEAD_SAME_AS_EGO", True)
    use_thw = getattr(cfg, "USE_THW", False)
    gap_key = "headway_thw" if use_thw else "headway_d"
    gap_vals = cfg.MATRIX["headway_thw"] if use_thw else cfg.MATRIX["headway_d"]
    speeds, mus = cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["mu"]

    cases = []
    if same:
        for v, g, m in itertools.product(speeds, gap_vals, mus):
            cases.append({"ego_speed_kmh": v, "lead_speed_kmh": v, gap_key: g, "mu": m})
    else:
        for ve, vl, g, m in itertools.product(speeds, cfg.MATRIX["lead_speed_kmh"], gap_vals, mus):
            cases.append({"ego_speed_kmh": ve, "lead_speed_kmh": vl, gap_key: g, "mu": m})
    return cases


def main():
    test_mode = os.environ.get("TEST_MODE", "original")
    matrix_runs = cfg.build_matrix_runs(test_mode)
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX/lead-brake] {len(cases)} cases/controller × {len(matrix_runs)} runs "
          f"= {len(cases)*len(matrix_runs)} runs  TEST_MODE={test_mode}")

    records = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        for run in matrix_runs:
            for i, case in enumerate(cases):
                print(f"\n--- [{run['label']}] case {i+1}/{len(cases)} {case} ---")
                rec, _ = run_case(sess, cfg, case, run["controller"],
                                  run.get("delay_frames", 0), detector,
                                  run_spec=run, case_idx=i, test_mode=test_mode, viz=None)
                rec.label = run["label"]
                records.append(rec)

    # ── Write CSV ──
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = getattr(cfg, "RESULTS_PREFIX", "lead_matrix")
    # embed TEST_MODE in the filename so each result file states what it tested
    # (e.g. lead_matrix_original_*.csv, lead_matrix_latency_comp_all_*.csv); timestamp keeps it unique
    csv_path = os.path.join(cfg.RESULTS_DIR, f"{prefix}_{test_mode}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] wrote results to {csv_path}")

    # ── Summarize + check 20% ──
    summary = summarize(records)
    print("\n" + "=" * 72)
    print("Summary per controller (lead-brake scenario):")
    labels = [r["label"] for r in matrix_runs]
    for label in labels:
        if label not in summary:
            continue
        s = summary[label]
        print(f"  {label:20s} | Rc_all={s['rc_all']*100:5.1f}% ({s['avoided']}/{s['n']}) "
              f"| Rc_conflict={s['rc_conflict']*100:5.1f}% ({s['n_conflict']} conflict)")
        print(f"  {'':20s}   MFDD={s['mean_ab']:5.2f} m/s² (n={s['n_ab']}) "
              f"| s_c={s['mean_sc']:.2f}m (n={s['n_sc']}) "
              f"| Tc={s['mean_tc']:.2f}s (n={s['n_tc']}) "
              f"| Δv={s['mean_dv']:.1f} km/h")

    if labels and labels[0] in summary:
        base = labels[0]
        print("-" * 72)
        for prop in labels[1:]:
            if prop not in summary:
                continue
            delta = (summary[prop]["rc"] - summary[base]["rc"]) * 100
            status = "✓ Pass" if delta >= 20 else "✗ Not yet — tune DYN_K_* / REQ_*_FRAC or DELAY_FRAMES"
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f}%  | target +20%: {status}")
    print("=" * 72)
    print(f"[TEST_MODE={test_mode}]  CSV: {csv_path}")


if __name__ == "__main__":
    main()
