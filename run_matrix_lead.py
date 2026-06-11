# -*- coding: utf-8 -*-
"""
ไล่ TEST MATRIX ทั้งหมดของฉาก lead-brake (headless ไม่มีภาพ) สำหรับแต่ละสมองกลใน MATRIX_RUNS
แล้วเขียน CSV (results/lead_matrix_*.csv) + สรุป R_c เทียบกัน → เช็กเป้า +20% (成果2)
แยกขาดจากฉาก cut-in (run_matrix.py) เพื่อง่ายต่อการทดสอบ/ดีบักทีละฉาก
วิธีใช้:  python run_matrix_lead.py
"""
import sys, os, itertools, datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_lead_brake as cfg
from core.carla_session import CarlaSession
from core.runner_lead_brake import run_case
from core.metrics import write_csv, summarize
from perception.yolo_detector import YoloDetector


def build_cases():
    """สร้างรายการเคสจาก MATRIX
    LEAD_SAME_AS_EGO=True  → ความเร็วรถนำ = ความเร็ว ego ทุกเคส (speed × headway × μ)
    LEAD_SAME_AS_EGO=False → lead_speed_kmh เป็นตัวแปรแยก (ego × lead × headway × μ)
    """
    same = getattr(cfg, "LEAD_SAME_AS_EGO", True)
    cases = []
    if same:
        for v, h, m in itertools.product(
                cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["headway_d"], cfg.MATRIX["mu"]):
            cases.append(dict(ego_speed_kmh=v, lead_speed_kmh=v, headway_d=h, mu=m))
    else:
        for ve, vl, h, m in itertools.product(
                cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["lead_speed_kmh"],
                cfg.MATRIX["headway_d"], cfg.MATRIX["mu"]):
            cases.append(dict(ego_speed_kmh=ve, lead_speed_kmh=vl, headway_d=h, mu=m))
    return cases


def main():
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX/lead-brake] {len(cases)} เคส/สมองกล × {len(cfg.MATRIX_RUNS)} สมองกล "
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
    prefix = getattr(cfg, "RESULTS_PREFIX", "lead_matrix")
    csv_path = os.path.join(cfg.RESULTS_DIR, f"{prefix}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] เขียนผลที่ {csv_path}")

    # ── สรุป + เช็ก 20% ──
    summary = summarize(records)
    print("\n" + "=" * 60)
    print("สรุปต่อสมองกล (ฉาก lead-brake):")
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
