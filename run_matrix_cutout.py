# -*- coding: utf-8 -*-
"""
Iterate through all TEST MATRIX cases for the Cut-out scenario (headless, no display by default).
Writes results/cutout_matrix_<TEST_MODE>_<stamp>.csv + prints CPEIM summary.

Before running: load the train000 scene in CARLA.
Usage:
    python run_matrix_cutout.py                   # original controllers, no viz
    TEST_MODE=latency python run_matrix_cutout.py
    MATRIX_VIZ=1 python run_matrix_cutout.py      # opt-in display
"""
import sys, os, itertools, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_cutout as cfg
from core import actors
from core.carla_session import CarlaSession
from core.runner_cutout import run_case
from core.metrics import write_csv, summarize
from core.viz import Viz
from perception.yolo_detector import YoloDetector


def build_cases():
    """5 speeds × 5 reveal_ttc × 2 μ = 50 cases/controller."""
    fixed_thw = getattr(cfg, "FIXED_HEADWAY_THW", 1.5)
    gap_offset = getattr(cfg, "GAP_OFFSET", 4.5)
    cases = []
    for v, ttc, m in itertools.product(
            cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["reveal_ttc"], cfg.MATRIX["mu"]):
        ego_ms = v / 3.6
        headway_d = fixed_thw * ego_ms
        cutout_trigger_d = (ttc - fixed_thw) * ego_ms + gap_offset
        cases.append({
            "ego_speed_kmh":    v,
            "reveal_ttc":       ttc,
            "mu":               m,
            "headway_d":        headway_d,
            "cutout_trigger_d": cutout_trigger_d,
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
    print(f"[MATRIX/cutout] {len(cases)} cases/controller × {len(matrix_runs)} runs "
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
    prefix = getattr(cfg, "RESULTS_PREFIX", "cutout_matrix")
    csv_path = os.path.join(cfg.RESULTS_DIR, f"{prefix}_{test_mode}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] wrote results to {csv_path}")

    # ── Summarize ──
    summary = summarize(records)
    print("\n" + "=" * 72)
    print("Summary per controller (Cut-out scenario):")
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
