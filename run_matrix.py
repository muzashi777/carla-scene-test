# -*- coding: utf-8 -*-
"""
ไล่ TEST MATRIX ทั้งหมด (headless ไม่มีภาพ) สำหรับแต่ละสมองกลใน MATRIX_RUNS
แล้วเขียน CSV + สรุป R_c เทียบกัน → เช็กว่าได้เป้า +20% (成果2) ไหม
วิธีใช้:  python run_matrix.py
"""
import sys, os, itertools, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_cutin as cfg
from core.carla_session import CarlaSession
from core.runner import run_case
from core.metrics import write_csv, summarize
from perception.yolo_detector import YoloDetector


def build_cases():
    keys = ["ego_speed_kmh", "mu", "trigger_d", "dart_speed_kmh"]
    combos = itertools.product(
        cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["mu"],
        cfg.MATRIX["trigger_d"], cfg.MATRIX["dart_speed_kmh"],
    )
    return [dict(zip(keys, c)) for c in combos]


def main():
    test_mode = os.environ.get("TEST_MODE", "original")
    matrix_runs = cfg.build_matrix_runs(test_mode)
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX] {len(cases)} เคส/สมองกล × {len(matrix_runs)} runs "
          f"= {len(cases)*len(matrix_runs)} รัน  TEST_MODE={test_mode}")

    records = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        for run in matrix_runs:
            for i, case in enumerate(cases):
                print(f"\n--- [{run['label']}] เคส {i+1}/{len(cases)} {case} ---")
                rec, _ = run_case(sess, cfg, case, run["controller"],
                                  run.get("delay_frames", 0), detector,
                                  run_spec=run, case_idx=i, test_mode=test_mode, viz=None)
                rec.label = run["label"]
                records.append(rec)

    # ── เขียน CSV ──
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # embed TEST_MODE in the filename so each result file states what it tested
    # (e.g. matrix_original_*.csv, matrix_latency_comp_all_*.csv); timestamp keeps it unique
    csv_path = os.path.join(cfg.RESULTS_DIR, f"matrix_{test_mode}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] เขียนผลที่ {csv_path}")

    # ── สรุป + เช็ก 20% ──
    summary = summarize(records)
    print("\n" + "=" * 72)
    print("สรุปต่อสมองกล (ฉาก cut-in):")
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
            status = "✓ ผ่าน" if delta >= 20 else "✗ ยังไม่ถึง — จูน DYN_K_* / REQ_*_FRAC หรือ DELAY_FRAMES"
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f}%  | เป้า +20%: {status}")
    print("=" * 72)
    print(f"[TEST_MODE={test_mode}]  CSV: {csv_path}")


if __name__ == "__main__":
    main()
