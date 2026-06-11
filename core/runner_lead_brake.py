# -*- coding: utf-8 -*-
"""
เครื่องรันเคสเดียวของฉาก lead-brake — ผูกทุกโมดูลเข้าด้วยกัน คืน LeadBrakeRecord
ใช้ร่วมกันทั้ง run_single_lead.py (ส่ง viz = มีภาพ) และ run_matrix_lead.py (viz=None = headless)
แยกขาดจาก core/runner.py (ฉาก cut-in) เพื่อความง่ายในการทดสอบ/ดีบักทีละฉาก
ต่างจาก runner.py ตรง: รถนำ (lead) จอดข้างหน้าในเลนเดียวกัน + ระยะผิวถึงผิวเป็นแบบท้ายชนหน้า
"""
import queue
import math
from collections import deque
from dataclasses import dataclass, asdict  # noqa: F401  (asdict ใช้ผ่าน metrics.write_csv)

import carla

from core import actors
from core.scenario_lead_brake import LeadBrakeScenario
from core.types import Perception, EgoState
from core.metrics import MfddTracker
from control.base_controller import make_controller
# import เพื่อให้ register() ทำงาน (ขึ้นทะเบียนชื่อ controller)
import control.baseline_static_ttc   # noqa: F401
import control.proposed_dynamic_ttc  # noqa: F401


@dataclass
class LeadBrakeRecord:
    """RunRecord เฉพาะฉาก lead-brake — ชื่อฟิลด์เมตริกตรงกับที่ metrics.summarize/write_csv ใช้"""
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    lead_speed_kmh: float          # ความเร็วรถนำก่อนเบรก (km/h)
    mu: float
    headway_thw: float             # ระยะห่างเป็นเวลา THW (s) — 0 ถ้าใช้โหมดระยะคงที่
    headway_d: float               # ระยะห่างรถนำ-ego ตอนเริ่ม "เมตรจริง" (คำนวณจาก THW×speed หรือค่าคงที่)
    avoided: bool = False
    collision_with: str = ""
    s_clearance: float = 0.0       # m (>0 = ระยะเหลือตอนหยุด)
    a_b_mfdd: float = 0.0          # m/s²
    t_c_warn: float = 0.0          # s (TTC ตอนเริ่มเบรก)
    dv_speed_var: float = 0.0      # km/h
    collision_speed_kmh: float = 0.0
    peak_decel: float = 0.0        # m/s²
    brake_distance: float = 0.0    # m
    min_dist: float = 0.0
    result_txt: str = ""


def run_case(sess, cfg, case, controller_name, delay_frames, detector, viz=None):
    world = sess.world
    actor_list = []
    front_q = queue.Queue()
    top_q = queue.Queue()
    collision = {"hit": False, "with": None}

    # ── ระยะห่างรถนำ: THW (วินาที) → เมตรจริง  หรือ  ระยะคงที่ (โหมดเดิม) ──
    # THW อิงความเร็ว "ego" เสมอ (เราเป็นคนตามรถนำ) แม้เปิด lead_speed ต่างจาก ego
    ego_ms = case["ego_speed_kmh"] / 3.6
    if getattr(cfg, "USE_THW", False):
        headway_thw = case["headway_thw"]
        headway_d = ego_ms * headway_thw          # THW → เมตรจริง
    else:
        headway_thw = 0.0
        headway_d = case["headway_d"]             # ระยะคงที่ (พฤติกรรมเดิม)
    if headway_d > cfg.INPATH_MAX_RANGE:
        print(f"[WARN] headway_d={headway_d:.1f}m > INPATH_MAX_RANGE={cfg.INPATH_MAX_RANGE}m "
              f"— เกตตรวจจับ ground-truth อาจไม่เห็นรถนำตอนเริ่ม")

    rec = LeadBrakeRecord(
        label=controller_name, controller=controller_name, delay_frames=delay_frames,
        ego_speed_kmh=case["ego_speed_kmh"], lead_speed_kmh=case["lead_speed_kmh"],
        mu=case["mu"], headway_thw=headway_thw, headway_d=headway_d,
    )

    try:
        # ── EGO ──
        ego = actors.spawn_vehicle(world, **cfg.EGO_SPAWN)
        if not ego:
            rec.result_txt = "EGO spawn failed"; return rec, None
        actor_list.append(ego)
        ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        actors.set_friction(ego, case["mu"])

        # ── LEAD (จอดข้างหน้า ego ในเลนเดียวกัน; y = ego.y − headway_d เพราะ ego วิ่งไป -Y) ──
        lead_y = cfg.EGO_SPAWN["y"] - headway_d
        lead = actors.spawn_vehicle(
            world, x=cfg.LEAD_SPAWN["x"], y=lead_y, z=cfg.LEAD_SPAWN["z"],
            yaw=cfg.LEAD_SPAWN["yaw"], model=cfg.LEAD_SPAWN["model"])
        if not lead:
            rec.result_txt = "LEAD spawn failed"; return rec, None
        actor_list.append(lead)
        lead.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

        # ── กล้องหน้า (ให้ YOLO) ──
        cam = actors.attach_rgb_camera(world, ego, cfg.CAM_FRONT_TF,
                                       cfg.CAM_W, cfg.CAM_H, lambda i: front_q.put(i),
                                       fov=cfg.CAM_FOV_DEG)
        actor_list.append(cam)
        # ── กล้อง top view (เฉพาะตอนโชว์ภาพ) ──
        if viz is not None:
            cam_top = actors.attach_rgb_camera(world, ego, cfg.CAM_TOP_TF,
                                               cfg.CAM_W, cfg.CAM_H, lambda i: top_q.put(i))
            actor_list.append(cam_top)

        # ── collision ──
        def on_col(e):
            if not collision["hit"]:
                collision["hit"] = True
                collision["with"] = e.other_actor.type_id
        col = actors.attach_collision_sensor(world, ego, on_col)
        actor_list.append(col)

        # ── ปล่อยให้ฉากนิ่ง ──
        for _ in range(cfg.SETTLE_TICKS):
            wf = world.tick()
            try:
                actors.grab_synced(front_q, wf)
                if viz is not None:
                    actors.grab_synced(top_q, wf)
            except queue.Empty:
                pass

        # ── เตรียมสมองกล + ฉาก ──
        controller = make_controller(controller_name, cfg)
        controller.reset()
        scen = LeadBrakeScenario(ego, lead, cfg, case)
        scen.start()

        det_buffer = deque(maxlen=delay_frames + 1)
        mfdd = MfddTracker(case["ego_speed_kmh"])
        ego_y0 = ego.get_location().y

        # ── ระยะ "ผิวถึงผิว" = ระยะศูนย์กลาง − ขนาดตัวรถ ──
        # ฉากนี้ท้ายชนหน้า รถสองคันหันทางเดียวกัน → หักครึ่งความยาวทั้งสองคัน (extent.x)
        if cfg.AUTO_GAP_OFFSET:
            try:
                gap_offset = ego.bounding_box.extent.x + lead.bounding_box.extent.x
            except Exception:
                gap_offset = cfg.GAP_OFFSET
        else:
            gap_offset = cfg.GAP_OFFSET
        print(f"[GAP] gap_offset = {gap_offset:.2f} m (หักครึ่งความยาว ego + lead ออกจากระยะศูนย์กลาง)")

        def surface_gap(dc):
            return max(0.0, dc - gap_offset)

        prev_gap = surface_gap(actors.dist2d(ego, lead))

        brake_engaged = False
        brake_info = None        # (tick, gap, v_kmh, ego_y, ttc)
        stopped = False
        min_dist = 1e9
        min_gap = 1e9
        peak_decel = 0.0
        prev_v_ms = actors.speed_ms(ego)
        result_txt = "TIMEOUT"
        last_frame = None
        quit_flag = False

        for tick in range(cfg.MAX_TICKS):
            wf = world.tick()
            try:
                if cfg.FRAME_SYNC:
                    img = actors.grab_synced(front_q, wf)
                    img_top = actors.grab_synced(top_q, wf) if viz is not None else None
                else:
                    img = front_q.get(timeout=2.0)
                    img_top = top_q.get(timeout=2.0) if viz is not None else None
            except queue.Empty:
                continue

            ego_y = ego.get_location().y
            v_kmh = actors.speed_kmh(ego)
            d = actors.dist2d(ego, lead)
            gap = surface_gap(d)
            min_dist = min(min_dist, d)
            min_gap = min(min_gap, gap)

            # ── เช็กชน 'ทันที' หลังอ่านสถานะ (ก่อนเสีย YOLO/วาดภาพ) ──
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh

            scen.update()   # ตรึง lead ให้จอดนิ่ง

            # ── PERCEPTION ──
            frame = detector.carla_image_to_bgr(img)
            yolo_now, in_band, box_h = detector.detect(frame)
            # ground-truth: lead อยู่ในทางเดินข้างหน้า ego ไหม (lead นิ่ง → ไม่ต้อง predict)
            gt_now, lon, lat = actors.inpath_hazard(
                ego, lead, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH,
                cfg.INPATH_LOOKAHEAD if cfg.INPATH_PREDICT else 0.0)

            src = cfg.DETECTION_SOURCE
            if src == "yolo":
                detected_now = yolo_now
            elif src == "both_or":
                detected_now = gt_now or yolo_now
            else:  # "groundtruth"
                detected_now = gt_now

            # ── ระยะ/ความเร็วสัมพัทธ์ ground-truth (ผิวถึงผิว) → TTC ──
            rel_speed = max(0.0, (prev_gap - gap) / cfg.FIXED_DT)
            prev_gap = gap
            ttc = (gap / rel_speed) if rel_speed > 1e-3 else math.inf

            # ── หน่วงเฟรมเฉพาะ detected (perception latency) ──
            det_buffer.append(detected_now)
            perceived = det_buffer[0]

            perc = Perception(detected=perceived, distance=gap,
                              rel_speed=rel_speed, ttc=ttc, box_h=box_h)
            ego_state = EgoState(speed_ms=actors.speed_ms(ego),
                                 speed_kmh=v_kmh, mu=case["mu"])

            # ── สมองกลตัดสินใจ ──
            ctrl = controller.decide(perc, ego_state)
            is_braking = ctrl.brake > 0.0

            if is_braking:
                if not brake_engaged:
                    brake_engaged = True
                    brake_info = (tick, gap, v_kmh, ego_y, ttc)
                if cfg.BRAKE_MODEL == "kinematic":
                    decel_cmd = ctrl.brake * case["mu"] * cfg.GRAVITY
                    new_v = max(0.0, actors.speed_ms(ego) - decel_cmd * cfg.FIXED_DT)
                    f = ego.get_transform().get_forward_vector()
                    ego.set_target_velocity(carla.Vector3D(f.x * new_v, f.y * new_v, 0.0))
                else:
                    ego.apply_control(ctrl)
            elif not brake_engaged:
                scen.cruise_ego()   # รักษาความเร็ว (เฉพาะตอนยังไม่เริ่มเบรก)

            # ── ติดตาม MFDD + ความหน่วงทันที/สูงสุด ──
            v_ms = actors.speed_ms(ego)
            inst_decel = (prev_v_ms - v_ms) / cfg.FIXED_DT if tick > 0 else 0.0
            prev_v_ms = v_ms
            if brake_engaged and inst_decel > peak_decel:
                peak_decel = inst_decel
            mfdd.update(v_kmh, abs(ego_y - ego_y0))

            if tick % cfg.LOG_EVERY == 0 or brake_engaged:
                print(f"t={tick*cfg.FIXED_DT:5.2f}s | ego_y={ego_y:6.1f} "
                      f"v={v_kmh:5.1f} gap={gap:5.1f}m (d={d:4.1f}) ttc={ttc:5.2f} "
                      f"bk={ctrl.brake:.2f} decel={inst_decel:5.2f} "
                      f"{'BRAKE' if brake_engaged else 'cruise'} det={perceived}")

            # ── โชว์ภาพ (ถ้ามี viz) ──
            if viz is not None:
                ov = [f"{controller_name} delay={delay_frames}f | v {v_kmh:.0f} | gap {gap:.1f}m ttc {ttc:.2f}",
                      f"{'BRAKE!' if brake_engaged else 'cruising'} | det[{src}]: {perceived} (lon {lon:.1f} lat {lat:.1f})"]
                hazard = {"camera": cam, "actor": lead, "in_path": gt_now,
                          "engaged": brake_engaged, "lon": lon, "lat": lat, "ttc": ttc}
                last_frame, q = viz.frame(frame, detector.last_results, ov, img_top, hazard)
                if q:
                    result_txt = "QUIT"; quit_flag = True; break

            # ── เงื่อนไขจบ ──
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh
                break
            if brake_engaged and v_kmh < cfg.STOP_KMH and not stopped:
                stopped = True
                result_txt = "AVOIDED"
                rec.avoided = True
                rec.s_clearance = gap
                if brake_info:
                    rec.brake_distance = abs(brake_info[3] - ego_y)
                break
            if ego_y < cfg.END_Y:
                result_txt = "NO BRAKE / passed"
                break

        # ── สรุปดัชนี ──
        rec.result_txt = result_txt
        rec.min_dist = min_dist
        rec.a_b_mfdd = mfdd.mfdd()
        rec.peak_decel = peak_decel
        if brake_info:
            rec.t_c_warn = 0.0 if math.isinf(brake_info[4]) else brake_info[4]
        rec.dv_speed_var = case["ego_speed_kmh"] - rec.collision_speed_kmh

        print("=" * 60)
        hw_txt = (f"THW={headway_thw:.1f}s→{headway_d:.1f}m" if getattr(cfg, "USE_THW", False)
                  else f"headway={headway_d:.0f}m")
        print(f"RESULT [{controller_name} delay={delay_frames}f "
              f"v={case['ego_speed_kmh']:.0f} vlead={case['lead_speed_kmh']:.0f} "
              f"mu={case['mu']} {hw_txt}] : {result_txt}")
        if brake_info:
            print(f"  เริ่มเบรก tick={brake_info[0]} gap={brake_info[1]:.1f}m "
                  f"v={brake_info[2]:.1f} ttc={rec.t_c_warn:.2f}s")
        print(f"  s={rec.s_clearance:.2f}m a_b(MFDD)={rec.a_b_mfdd:.2f} "
              f"peak_decel={rec.peak_decel:.2f} brake_dist={rec.brake_distance:.2f}m")
        print(f"  μ={case['mu']} → ความหน่วงสูงสุดที่ทฤษฎีให้ได้ ≈ μ·g = {case['mu']*cfg.GRAVITY:.2f} m/s²")
        print(f"  Δv={rec.dv_speed_var:.1f} min_gap={min_gap:.1f}m (min_dist={min_dist:.1f}m)")
        print("=" * 60)

        return rec, (last_frame, result_txt, quit_flag)

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
