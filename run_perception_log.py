# -*- coding: utf-8 -*-
"""
Run a perception-quality logging pass on a 3DGS scene and export to CSV (READ-ONLY)
─────────────────────────────────────────────────────────────────────────
Usage:  Launch CARLA (load 3DGS scene), place yolov8n.pt in this folder, then
          python run_perception_log.py

What it does:
  - Drives the 'ego camera' along the same straight trajectory (EGO_SPAWN.y → END_Y)
  - pass "background": spawns no vehicle → logs detections from the purely reconstructed 3DGS scene
  - pass "with_actor" (optional): + spawns the target vehicle (dart) to compare inserted actor vs. scene
  - Every M metres (or N frames) runs YOLO and writes 1 row per detection to CSV
  - Writes a sidecar file (.meta.json) recording conf threshold, resolution, and per-class summary

Does not touch any existing controller / scenario / runner / braking / AEB result CSV.
Uses the same YOLO model / resolution / conf threshold as the AEB gate (read from config.scenario_cutin).

DO-NOT: Do not compute precision/recall; do not label confidence as accuracy.
        (the reconstructed scene has no ground-truth labels — log raw detections + confidence only)
"""
import sys, os, csv, json, datetime, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_cutin as cfg     # shares trajectory / camera / YOLO settings with the cut-in scenario
from core.carla_session import CarlaSession
from perception.yolo_detector import YoloDetector
from perception import scene_logger


# ══════════════════════════════════════════════════════════════════
#  Perception-logging settings (tunable — separate from AEB scenario config)
# ══════════════════════════════════════════════════════════════════
# passes to run: "background" = pure 3DGS scene (no vehicle),
#               "with_actor" = + target vehicle (dart) inserted (compared against background)
PERCEP_PASSES = ["background", "with_actor"]

# sampling mode: if _M > 0 → sample every M metres of travel distance; otherwise sample every N frames
PERCEP_SAMPLE_EVERY_M      = 1.0     # metres
PERCEP_SAMPLE_EVERY_FRAMES = 5       # used when PERCEP_SAMPLE_EVERY_M <= 0

PERCEP_DRIVE_KMH = 30.0              # camera drive speed (does not affect sample positions in metre mode)
PERCEP_SETTLE_TICKS = 20

# open an OpenCV window to view live detections (drawn on sampled frames = matches logged frames); 'q' = quit
# set to False to run headless (server without display / cv2 not installed)
PERCEP_SHOW_WINDOW = True

RESULTS_DIR = cfg.RESULTS_DIR        # writes to results/perception_log_*.csv (separate from matrix_*)


def _summary_per_class(per_class):
    """Return a dict {class_name: {count, mean_conf, median_conf, min_conf, max_conf}}"""
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

    viz = None
    if PERCEP_SHOW_WINDOW:
        from perception.percep_viz import PercepViz   # import cv2 only when the display window is enabled
        viz = PercepViz(cfg)

    all_rows = []
    pass_stats = []
    try:
        with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
            print(f"[SIM] sync ON dt={cfg.FIXED_DT}s | perception-logging passes={PERCEP_PASSES}")
            for pass_type in PERCEP_PASSES:
                rows, stats = scene_logger.run_pass(
                    sess, cfg, detector, pass_type,
                    sample_every_m=PERCEP_SAMPLE_EVERY_M,
                    sample_every_frames=PERCEP_SAMPLE_EVERY_FRAMES,
                    drive_kmh=PERCEP_DRIVE_KMH,
                    settle_ticks=PERCEP_SETTLE_TICKS,
                    viz=viz,
                )
                all_rows.extend(rows)
                pass_stats.append(stats)
    finally:
        if viz is not None:
            viz.close()

    # ── write CSV (1 row per detection) ──
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
    print(f"\n[CSV] wrote {len(all_rows)} detection rows to {csv_path}")

    # ── summary + sidecar ──
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
    print("Perception-logging summary (raw detections + raw confidence — NOT accuracy)")
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
        print(f"  sampled frames = {st['n_frames_sampled']} | total detections = {st['n_detections']}")
        if per_cls:
            print(f"  {'class':<16}{'count':>7}{'mean':>9}{'median':>9}{'min':>8}{'max':>8}")
            for name, s in per_cls.items():
                print(f"  {name:<16}{s['count']:>7}{s['mean_conf']:>9.3f}"
                      f"{s['median_conf']:>9.3f}{s['min_conf']:>8.3f}{s['max_conf']:>8.3f}")
        else:
            print("  (no detections)")
    print("=" * 64)

    meta_path = csv_path.rsplit(".", 1)[0] + ".meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[META] wrote sidecar (conf threshold / resolution / per-class summary) to {meta_path}")


if __name__ == "__main__":
    main()
