# -*- coding: utf-8 -*-
"""
Perception-quality logger สำหรับฉาก 3DGS (READ-ONLY pass — แยกจาก AEB ทั้งหมด)
──────────────────────────────────────────────────────────────────────────────
เป้าหมาย (สำหรับเปเปอร์): บอกว่า YOLO ตรวจจับวัตถุที่ 'เป็นส่วนหนึ่งของฉาก 3DGS ที่สร้างใหม่'
(รถจอด, ไฟจราจร, ป้าย ฯลฯ) ได้ดีแค่ไหน → สนับสนุนข้อสรุปว่าฉากจริงที่ reconstruct มาให้
อินพุต perception สมจริง

หลักการ:
  - ขับ 'กล้อง ego' ไปตามแนววิ่งตรงเดิม (จาก EGO_SPAWN.y → END_Y ตามแกน −Y)
  - pass "background": ไม่ spawn รถใด ๆ เลย — กล้องเดี่ยว ๆ (sensor) เลื่อนไปตาม trajectory
    → detection ทั้งหมดมาจากฉาก 3DGS ที่ reconstruct เท่านั้น
  - pass "with_actor": แนว trajectory เดิมเป๊ะ + spawn 'รถเป้าหมาย' (dart) ที่จุด DART_SPAWN
    (จอดนิ่ง) → เทียบการตรวจจับ 'actor ที่แทรกเข้าไป' กับ 'วัตถุในฉากที่ reconstruct'
  - ทุก ๆ N เฟรม หรือ M เมตรของระยะวิ่ง → รัน YOLO บนเฟรม RGB แล้ว log 'ทุก detection'

ไม่ยุ่งกับ controller / scenario / runner / braking / CSV ผลเดิมใด ๆ ทั้งสิ้น
ใช้ YOLO (yolov8n.pt), ความละเอียดภาพ, conf threshold ชุดเดียวกับเกต AEB เป๊ะ
(ดู detector.detect_all) → ตัวเลขสะท้อนตัวตรวจจับตัวเดียวกัน

หมายเหตุ DO-NOT: ไม่คำนวณ precision/recall และไม่เรียก confidence ว่า "accuracy" —
ฉาก reconstruct ไม่มี ground-truth label เรา log แค่ raw detection + confidence ดิบ
"""
import math
import queue

import carla

from core import actors


# ── องค์ประกอบเวกเตอร์ของกรอบพิกัด ego (yaw องศา) ──
def _basis(yaw_deg):
    """คืน (forward, right) unit vectors บนระนาบ XY ตามแบบ CARLA (left-handed)
    yaw=0   → forward=(1,0)  right=(0,1)
    yaw=-90 → forward=(0,-1) right=(1,0)  (ego ของเราวิ่งไปทาง −Y)
    """
    y = math.radians(yaw_deg)
    fwd = (math.cos(y), math.sin(y))
    right = (-math.sin(y), math.cos(y))
    return fwd, right


def _camera_world_transform(ego_spawn, cam_tf, ego_y):
    """คำนวณ transform โลกของกล้องหน้า ให้ตรงกับตอนกล้องติดบน ego ที่ (x0, ego_y, z0, yaw0)
    โดย cam_tf เป็น local offset (เฟรม ego) เหมือนที่ใช้ตอน attach กล้องเข้า ego จริง
    คืน (camera_world_transform, ego_world_location)
    """
    yaw0 = ego_spawn["yaw"]
    fwd, right = _basis(yaw0)
    cx = cam_tf.get("x", 0.0)   # ไปข้างหน้า (เฟรม ego)
    cy = cam_tf.get("y", 0.0)   # ออกข้าง
    cz = cam_tf.get("z", 0.0)   # ขึ้นบน
    ego_x = ego_spawn["x"]
    ego_z = ego_spawn["z"]
    wx = ego_x + fwd[0] * cx + right[0] * cy
    wy = ego_y + fwd[1] * cx + right[1] * cy
    wz = ego_z + cz
    rot = carla.Rotation(
        pitch=cam_tf.get("pitch", 0.0),     # ego pitch/roll = 0 → ใช้ค่ากล้องตรง ๆ
        yaw=yaw0 + cam_tf.get("yaw", 0.0),
        roll=cam_tf.get("roll", 0.0),
    )
    loc = carla.Location(x=wx, y=wy, z=wz)
    return carla.Transform(loc, rot), carla.Location(x=ego_x, y=ego_y, z=ego_z)


def _spawn_world_camera(world, w, h, fov, world_tf, sink):
    """spawn กล้อง RGB เดี่ยว ๆ (ไม่ attach กับรถ) ที่ transform โลกที่กำหนด"""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    if fov is not None:
        bp.set_attribute("fov", str(fov))
    cam = world.spawn_actor(bp, world_tf)
    cam.listen(sink)
    return cam


def run_pass(sess, cfg, detector, pass_type,
             sample_every_m=1.0, sample_every_frames=0,
             drive_kmh=30.0, settle_ticks=20, max_ticks=4000):
    """
    ขับกล้องไปตาม trajectory ตรง (EGO_SPAWN.y → END_Y) แล้ว log ทุก detection
      pass_type            : "background" (ไม่ spawn รถ) | "with_actor" (spawn dart)
      sample_every_m  > 0  : สุ่มตัวอย่างทุก ๆ M เมตรของระยะวิ่ง (โหมดหลัก)
      sample_every_frames  : ถ้า sample_every_m<=0 ให้ใช้ทุก ๆ N เฟรมแทน
    คืน (rows, stats)
      rows  : list ของ dict (หนึ่งแถวต่อหนึ่ง detection)
      stats : dict สรุป pass นี้ (n_frames_sampled, n_detections, per_class{name:[conf,...]})
    """
    world = sess.world
    actor_list = []
    cam_q = queue.Queue()
    rows = []
    per_class = {}
    n_frames = 0
    n_dets = 0

    y0 = cfg.EGO_SPAWN["y"]
    y_end = cfg.END_Y
    step = actors.kmh_to_ms(drive_kmh) * cfg.FIXED_DT     # ระยะที่เลื่อนต่อ tick (m, ไปทาง −Y)

    try:
        # ── (with_actor) spawn รถเป้าหมาย dart ที่จุดเดิม จอดนิ่ง = "actor ที่แทรกเข้าฉาก" ──
        if pass_type == "with_actor":
            dart = actors.spawn_vehicle(world, **cfg.DART_SPAWN)
            if dart is None:
                print("[PERCEP] ⚠ spawn dart (ego target) ไม่สำเร็จ — ข้าม pass with_actor")
                return rows, dict(pass_type=pass_type, n_frames_sampled=0,
                                  n_detections=0, per_class={})
            actor_list.append(dart)
            actors.hold(dart)

        # ── spawn กล้องเดี่ยวที่จุดเริ่ม trajectory ──
        cam_tf0, _ = _camera_world_transform(cfg.EGO_SPAWN, cfg.CAM_FRONT_TF, y0)
        cam = _spawn_world_camera(world, cfg.CAM_W, cfg.CAM_H, cfg.CAM_FOV_DEG,
                                  cam_tf0, lambda i: cam_q.put(i))
        actor_list.append(cam)

        # ── ปล่อยให้ฉากนิ่ง ──
        for _ in range(settle_ticks):
            wf = world.tick()
            try:
                actors.grab_synced(cam_q, wf)
            except queue.Empty:
                pass

        print(f"[PERCEP] pass='{pass_type}' ขับกล้อง y {y0:.1f} → {y_end:.1f} "
              f"(step {step:.3f} m/tick); "
              f"sample {'every %.2f m' % sample_every_m if sample_every_m > 0 else 'every %d frames' % sample_every_frames}")

        y = y0
        tick = 0
        frame_index = 0
        last_sample_y = None

        while y > y_end and tick < max_ticks:
            cam_tf, ego_loc = _camera_world_transform(cfg.EGO_SPAWN, cfg.CAM_FRONT_TF, y)
            cam.set_transform(cam_tf)
            wf = world.tick()
            try:
                img = actors.grab_synced(cam_q, wf)
            except queue.Empty:
                y -= step; tick += 1
                continue

            # ── ตัดสินใจว่าจะ sample เฟรมนี้ไหม ──
            if sample_every_m > 0:
                do_sample = (last_sample_y is None) or ((last_sample_y - y) >= sample_every_m - 1e-9)
            else:
                n = sample_every_frames if sample_every_frames > 0 else 1
                do_sample = (tick % n == 0)

            if do_sample:
                last_sample_y = y
                frame = detector.carla_image_to_bgr(img)
                dets = detector.detect_all(frame)
                sim_t = round(tick * cfg.FIXED_DT, 3)
                for d in dets:
                    area = max(0.0, d["x2"] - d["x1"]) * max(0.0, d["y2"] - d["y1"])
                    rows.append(dict(
                        pass_type=pass_type,
                        frame_index=frame_index,
                        timestamp=sim_t,
                        ego_x=round(ego_loc.x, 3),
                        ego_y=round(ego_loc.y, 3),
                        ego_yaw=round(cfg.EGO_SPAWN["yaw"], 3),
                        class_id=d["class_id"],
                        class_name=d["class_name"],
                        confidence=round(d["confidence"], 4),
                        bbox_x1=round(d["x1"], 2),
                        bbox_y1=round(d["y1"], 2),
                        bbox_x2=round(d["x2"], 2),
                        bbox_y2=round(d["y2"], 2),
                        bbox_area_px=round(area, 1),
                        img_width=cfg.CAM_W,
                        img_height=cfg.CAM_H,
                    ))
                    per_class.setdefault(d["class_name"], []).append(d["confidence"])
                    n_dets += 1
                n_frames += 1
                frame_index += 1
                if n_frames % 10 == 0:
                    print(f"[PERCEP]   y={y:6.1f} frames={n_frames} dets={n_dets}")

            y -= step
            tick += 1

        print(f"[PERCEP] pass='{pass_type}' จบ: {n_frames} เฟรม, {n_dets} detections")
        stats = dict(pass_type=pass_type, n_frames_sampled=n_frames,
                     n_detections=n_dets, per_class=per_class)
        return rows, stats

    finally:
        for a in actor_list:
            try:
                if hasattr(a, "stop"):
                    a.stop()
            except Exception:
                pass
            try:
                a.destroy()
            except Exception:
                pass
