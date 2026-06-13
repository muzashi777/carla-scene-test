# -*- coding: utf-8 -*-
"""
รัน perception-quality logging pass บนฉาก 3DGS แล้ว export CSV (READ-ONLY)
─────────────────────────────────────────────────────────────────────────
วิธีใช้:  เปิด CARLA (โหลดฉาก 3DGS), วาง yolov8n.pt ไว้โฟลเดอร์นี้ แล้ว
          python run_perception_log.py

ทำอะไร:
  - ขับ 'กล้อง ego' ไปตาม trajectory ตรงเดิม (EGO_SPAWN.y → END_Y)
  - pass "background": ไม่ spawn รถเลย → log detection ของฉาก 3DGS ที่ reconstruct ล้วน ๆ
  - pass "with_actor" (ออปชัน): + spawn รถเป้าหมาย (dart) เพื่อเทียบ actor แทรก vs ฉาก
  - ทุก M เมตร (หรือ N เฟรม) รัน YOLO แล้วเขียน 1 แถว/1 detection ลง CSV
  - เขียนไฟล์ sidecar (.meta.json) บันทึก conf threshold, ความละเอียด, สรุปต่อคลาส

ไม่แตะ controller / scenario / runner / braking / CSV ผลของ AEB เดิมใด ๆ
ใช้ YOLO/ความละเอียด/conf threshold เดียวกับเกต AEB (อ่านจาก config.scenario_cutin)

DO-NOT: ไม่คำนวณ precision/recall, ไม่เรียก confidence ว่า accuracy
        (ฉาก reconstruct ไม่มี ground-truth label — log แค่ raw detection + confidence)
"""
import sys, os, csv, json, datetime, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_cutin as cfg     # ใช้ trajectory/กล้อง/YOLO ร่วมกับฉาก cut-in
from core.carla_session import CarlaSession
from perception.yolo_detector import YoloDetector
from perception import scene_logger


# ══════════════════════════════════════════════════════════════════
#  ตั้งค่า perception-logging (ปรับได้ — แยกจาก config ฉาก AEB)
# ══════════════════════════════════════════════════════════════════
# pass ที่จะรัน: "background" = ฉาก 3DGS ล้วน (ไม่มีรถ),
#               "with_actor" = + รถเป้าหมาย (dart) แทรกเข้าไป (เทียบกับ background)
PERCEP_PASSES = ["background", "with_actor"]

# โหมด sampling: ถ้า _M > 0 → สุ่มทุก ๆ M เมตรของระยะวิ่ง; ไม่งั้นใช้ทุก ๆ N เฟรม
PERCEP_SAMPLE_EVERY_M      = 1.0     # เมตร
PERCEP_SAMPLE_EVERY_FRAMES = 5       # ใช้เมื่อ PERCEP_SAMPLE_EVERY_M <= 0

PERCEP_DRIVE_KMH = 30.0              # ความเร็วเลื่อนกล้อง (ไม่กระทบตำแหน่งที่ sample แบบเมตร)
PERCEP_SETTLE_TICKS = 20

RESULTS_DIR = cfg.RESULTS_DIR        # เขียนที่ results/perception_log_*.csv (แยกจาก matrix_*)


def _summary_per_class(per_class):
    """คืน dict {class_name: {count, mean_conf, median_conf, min_conf, max_conf}}"""
    out = {}
    for name, confs in sorted(per_class.items()):
        out[name] = dict(
            count=len(confs),
            mean_conf=round(statistics.fmean(confs), 4) if confs else 0.0,
            median_conf=round(statistics.median(confs), 4) if confs else 0.0,
            min_conf=round(min(confs), 4) if confs else 0.0,
            max_conf=round(max(confs), 4) if confs else 0.0,
        )
    return out


def main():
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )

    all_rows = []
    pass_stats = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        print(f"[SIM] sync ON dt={cfg.FIXED_DT}s | perception-logging passes={PERCEP_PASSES}")
        for pass_type in PERCEP_PASSES:
            rows, stats = scene_logger.run_pass(
                sess, cfg, detector, pass_type,
                sample_every_m=PERCEP_SAMPLE_EVERY_M,
                sample_every_frames=PERCEP_SAMPLE_EVERY_FRAMES,
                drive_kmh=PERCEP_DRIVE_KMH,
                settle_ticks=PERCEP_SETTLE_TICKS,
            )
            all_rows.extend(rows)
            pass_stats.append(stats)

    # ── เขียน CSV (1 แถว/1 detection) ──
    os.makedirs(RESULTS_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(RESULTS_DIR, f"perception_log_{stamp}.csv")
    fields = ["pass_type", "frame_index", "timestamp", "ego_x", "ego_y", "ego_yaw",
              "class_id", "class_name", "confidence",
              "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2", "bbox_area_px",
              "img_width", "img_height"]
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    print(f"\n[CSV] เขียน {len(all_rows)} detection rows ที่ {csv_path}")

    # ── สรุป + sidecar ──
    class_names = {cid: detector._class_name(cid) for cid in cfg.TARGET_CLASSES}
    meta = dict(
        generated=stamp,
        detector=dict(
            model=cfg.YOLO_MODEL,
            device=cfg.YOLO_DEVICE,
            conf_threshold=cfg.CONF_THRESH,
            iou_threshold=cfg.IOU_THRESH,
            target_classes=cfg.TARGET_CLASSES,
            target_class_names=class_names,
            img_width=cfg.CAM_W,
            img_height=cfg.CAM_H,
            fov_deg=cfg.CAM_FOV_DEG,
        ),
        sampling=dict(
            mode="meters" if PERCEP_SAMPLE_EVERY_M > 0 else "frames",
            every_m=PERCEP_SAMPLE_EVERY_M,
            every_frames=PERCEP_SAMPLE_EVERY_FRAMES,
            drive_kmh=PERCEP_DRIVE_KMH,
            trajectory=dict(start_y=cfg.EGO_SPAWN["y"], end_y=cfg.END_Y,
                            ego_x=cfg.EGO_SPAWN["x"], ego_yaw=cfg.EGO_SPAWN["yaw"]),
        ),
        note=("raw detections + confidence only; no ground-truth labels in the "
              "reconstructed 3DGS scene, so NO precision/recall/accuracy is computed."),
        passes=[],
    )

    print("\n" + "=" * 64)
    print("สรุป perception-logging (raw detections + confidence ดิบ ไม่ใช่ accuracy)")
    for st in pass_stats:
        per_cls = _summary_per_class(st["per_class"])
        meta["passes"].append(dict(
            pass_type=st["pass_type"],
            n_frames_sampled=st["n_frames_sampled"],
            n_detections=st["n_detections"],
            per_class=per_cls,
        ))
        print("-" * 64)
        print(f"pass = {st['pass_type']}")
        print(f"  เฟรมที่ sample = {st['n_frames_sampled']} | detections รวม = {st['n_detections']}")
        if per_cls:
            print(f"  {'class':<16}{'count':>7}{'mean':>9}{'median':>9}{'min':>8}{'max':>8}")
            for name, s in per_cls.items():
                print(f"  {name:<16}{s['count']:>7}{s['mean_conf']:>9.3f}"
                      f"{s['median_conf']:>9.3f}{s['min_conf']:>8.3f}{s['max_conf']:>8.3f}")
        else:
            print("  (ไม่มี detection)")
    print("=" * 64)

    meta_path = csv_path.rsplit(".", 1)[0] + ".meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[META] เขียน sidecar (conf threshold/ความละเอียด/สรุปต่อคลาส) ที่ {meta_path}")


if __name__ == "__main__":
    main()
