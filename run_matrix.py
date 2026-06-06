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
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX] {len(cases)} เคส/สมองกล × {len(cfg.MATRIX_RUNS)} สมองกล "
          f"= {len(cases)*len(cfg.MATRIX_RUNS)} รัน")

    records = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        for run in cfg.MATRIX_RUNS:
            for i, case in enumerate(cases):
                print(f"\n--- [{run['label']}] เคส {i+1}/{len(cases)} {case} ---")
                rec, _ = run_case(sess, cfg, case, run["controller"],
                                  run["delay_frames"], detector, viz=None)
                rec.label = run["label"]
                records.append(rec)

    # ── เขียน CSV ──
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(cfg.RESULTS_DIR, f"matrix_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] เขียนผลที่ {csv_path}")

    # ── สรุป + เช็ก 20% ──
    summary = summarize(records)
    print("\n" + "=" * 60)
    print("สรุปต่อสมองกล:")
    for label, s in summary.items():
        print(f"  {label:10s} | Rc={s['rc']*100:5.1f}% "
              f"({s['avoided']}/{s['n']}) | คะแนนเฉลี่ย={s['mean_score']:.4f}")

    labels = [r["label"] for r in cfg.MATRIX_RUNS]
    if len(labels) >= 2:
        base, prop = labels[0], labels[-1]
        if base in summary and prop in summary:
            delta = (summary[prop]["rc"] - summary[base]["rc"]) * 100
            print("-" * 60)
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f} เปอร์เซ็นต์")
            print(f"เป้า 成果2 (+20%): {'✓ ผ่าน' if delta >= 20 else '✗ ยังไม่ถึง — จูน DYN_K_* หรือ DELAY_FRAMES'}")
    print("=" * 60)


if __name__ == "__main__":
    main()
