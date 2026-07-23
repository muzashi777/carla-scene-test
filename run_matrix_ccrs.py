# -*- coding: utf-8 -*-
"""
Iterate through all TEST MATRIX cases for the CCRs scenario (headless, no display by default).
Writes results/ccrs_matrix_<TEST_MODE>_<stamp>.csv + prints CPEIM summary.

Before running: load the train000 scene in CARLA.
Usage:
    python run_matrix_ccrs.py                   # original controllers, no viz
    TEST_MODE=latency python run_matrix_ccrs.py
    MATRIX_VIZ=1 python run_matrix_ccrs.py      # opt-in display (all runs)
"""
import sys, os, itertools, datetime, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_ccrs as cfg
from core import actors
from core.carla_session import CarlaSession
from core.runner_ccrs import run_case
from core.metrics import write_csv, summarize
from core.viz import Viz
from perception.yolo_detector import YoloDetector


def build_cases():
    """5 speeds × 5 approach_d × 2 μ = 50 cases/controller."""
    yaw_rad = math.radians(cfg.EGO_SPAWN["yaw"])
    fwd_x = math.cos(yaw_rad)
    fwd_y = math.sin(yaw_rad)
    cases = []
    for v, d, m in itertools.product(
            cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["approach_d"], cfg.MATRIX["mu"]):
        target_x = cfg.EGO_SPAWN["x"] + d * fwd_x
        target_y = cfg.EGO_SPAWN["y"] + d * fwd_y
        cases.append({
            "ego_speed_kmh": v,
            "approach_d":    d,
            "mu":            m,
            "target_x":      round(target_x, 3),
            "target_y":      round(target_y, 3),
        })
    return cases


def main():
    test_mode = os.environ.get("TEST_MODE", "original")
    matrix_runs = cfg.build_matrix_runs(test_mode)

    viz_enabled = os.environ.get("MATRIX_VIZ", "0") == "1"

    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX/ccrs] {len(cases)} cases/controller × {len(matrix_runs)} runs "
          f"= {len(cases)*len(matrix_runs)} runs  TEST_MODE={test_mode}"
          f"  MATRIX_VIZ={'on' if viz_enabled else 'off'}")

    records = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        actors.check_scene(sess.world, cfg.EXPECTED_SCENE)
        actors.set_spectator(sess.world, cfg.SPECTATOR_TF)
        viz = Viz(cfg) if viz_enabled else None
        for run in matrix_runs:
            for i, case in enumerate(cases):
                print(f"\n--- [{run['label']}] case {i+1}/{len(cases)} {case} ---")
                rec, _ = run_case(sess, cfg, case, run["controller"],
                                  run.get("delay_frames", 0), detector,
                                  run_spec=run, case_idx=i, test_mode=test_mode, viz=viz)
                rec.label = run["label"]
                records.append(rec)
        if viz is not None:
            viz.close()

    # ── Write CSV ──
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = getattr(cfg, "RESULTS_PREFIX", "ccrs_matrix")
    csv_path = os.path.join(cfg.RESULTS_DIR, f"{prefix}_{test_mode}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] wrote results to {csv_path}")

    # ── Summarize ──
    summary = summarize(records)
    print("\n" + "=" * 72)
    print("Summary per controller (CCRs scenario):")
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
            status = "✓ Pass" if delta >= 20 else "✗ Not yet — tune DYN_K_* / REQ_*_FRAC"
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f}%  | target +20%: {status}")
    print("=" * 72)
    print(f"[TEST_MODE={test_mode}]  CSV: {csv_path}")


if __name__ == "__main__":
    main()
