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
    ระยะห่าง: USE_THW=True → ไล่ headway_thw (วินาที) / USE_THW=False → ไล่ headway_d (เมตร, โหมดเดิม)
    ความเร็ว: LEAD_SAME_AS_EGO=True → รถนำเร็วเท่า ego / False → lead_speed_kmh เป็นตัวแปรแยก
    runner จะแปลง THW→เมตรจริง เอง (อิงความเร็ว ego)
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
    # เรียงตามลำดับใน MATRIX_RUNS เพื่ออ่านง่าย (baseline → proposed → proposed_enhanced)
    labels = [r["label"] for r in cfg.MATRIX_RUNS]
    for label in labels:
        if label not in summary:
            continue
        s = summary[label]
        print(f"  {label:18s} | Rc={s['rc']*100:5.1f}% "
              f"({s['avoided']}/{s['n']}) | คะแนนเฉลี่ย={s['mean_score']:.4f}")

    # เทียบทุกสมองกล (ที่ไม่ใช่ baseline) กับ baseline → เช็กเป้า +20% ทีละตัว
    if labels and labels[0] in summary:
        base = labels[0]
        print("-" * 60)
        for prop in labels[1:]:
            if prop not in summary:
                continue
            delta = (summary[prop]["rc"] - summary[base]["rc"]) * 100
            status = "✓ ผ่าน" if delta >= 20 else "✗ ยังไม่ถึง — จูน DYN_K_* / REQ_*_FRAC หรือ DELAY_FRAMES"
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f} เปอร์เซ็นต์  | เป้า 成果2 (+20%): {status}")
    print("=" * 60)


if __name__ == "__main__":
    main()
