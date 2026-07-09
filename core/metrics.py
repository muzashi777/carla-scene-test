# -*- coding: utf-8 -*-
"""
เก็บ 5 ดัชนี CPEIM ต่อเคส แล้วรวมเป็น R_c และสถิติต่อสมองกล
  s    = clearance distance ตอนหยุดสนิท (m)  — ระยะผิวถึงผิวเมื่อ ego หยุดสนิท (surface gap)
  a_b  = MFDD ความหน่วงเฉลี่ยช่วง 0.8v→0.1v (m/s²)
  T_c  = warning lead time (s) = TTC ตอนเริ่มเบรก
  dv   = speed variation (km/h) = v_เริ่ม − v_ปะทะ (หรือ v_เริ่ม ถ้าหยุดสนิท)
  Rc   = สัดส่วนเคสที่หลบชนสำเร็จ (เฉพาะเคส conflict และเฉพาะทุกเคส รายงานแยก)

หมายเหตุการรวมดัชนี:
  mean_ab  = เฉลี่ยเฉพาะแถวที่ a_b_mfdd > 0 (มีการเบรกและวัดช่วง 0.8v→0.1v ได้)
  mean_sc  = เฉลี่ยเฉพาะเคสที่ avoided=True (กรณีชน s_clearance=0 ไม่ใช่ข้อมูลที่มีความหมาย)
  mean_tc  = เฉลี่ยเฉพาะแถวที่ t_c_warn > 0 (มีการเบรก)
  mean_dv  = เฉลี่ยทุกแถว (ทั้ง avoided และ collision)
"""
import csv
import os
from dataclasses import dataclass, asdict


# ── น้ำหนัก CPEIM ──────────────────────────────────────────────────────────────
# WARNING: ค่าเหล่านี้มาจากฉาก V-VRU (vertical, คนเดินถนน) ใน liu2025 Table 10
# ไม่ใช่ฉาก car-to-car (CCRb / cut-in) ที่เราทดสอบ
# ห้ามใช้คำนวณคะแนนรวม (composite score) สำหรับเคสของเราโดยตรง
# เก็บไว้เพื่ออ้างอิง/เทียบกับเปเปอร์เท่านั้น
W_S, W_AB, W_TC, W_DV, W_RC = 0.1447, 0.0901, 0.2962, 0.0603, 0.4087


@dataclass
class RunRecord:
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    mu: float
    trigger_d: float
    dart_speed_kmh: float
    avoided: bool = False
    collision_with: str = ""
    s_clearance: float = 0.0       # m (>0 = ระยะผิวถึงผิวเมื่อหยุดสนิท; 0 = ชน)
    a_b_mfdd: float = 0.0          # m/s²
    t_c_warn: float = 0.0          # s
    dv_speed_var: float = 0.0      # km/h
    collision_speed_kmh: float = 0.0
    peak_decel: float = 0.0        # m/s² ความหน่วงสูงสุดที่วัดได้ขณะเบรก
    brake_distance: float = 0.0    # m ระยะตั้งแต่เริ่มเบรกจนหยุด
    min_dist: float = 0.0
    a_req_at_brake: float = 0.0    # m/s² ความหน่วงที่ "จำเป็น" ตอนเริ่มเบรก (ดูว่าใกล้ขีด μ·g แค่ไหน)
    a_max: float = 0.0             # m/s² เพดานความหน่วง = μ·g ของเคสนี้
    is_conflict: bool = True       # True = เคสนี้ ego ชน (kinematic, no-brake) — ดู core/conflict.py
    result_txt: str = ""
    noise_sigma_m: float = 0.0
    noise_sigma_vr: float = 0.0
    dropout_p: float = 0.0
    dropout_mode: str = "freeze"
    test_mode: str = "original"
    seed: int = 0


def score_clearance(s, avoided):
    """ให้คะแนน s ตามเกณฑ์ i-VISTA ในเปเปอร์ (Table 12)"""
    if not avoided:
        return 0.0
    if s <= 0.6:
        return 1.0
    if s <= 1.2:
        return 0.8
    if s <= 1.8:
        return 0.6
    if s <= 2.4:
        return 0.3
    return 0.0


class MfddTracker:
    """ติดตามระยะทางที่ความเร็วตกถึง 0.8v0 และ 0.1v0 เพื่อคำนวณ MFDD"""
    def __init__(self, v0_kmh):
        self.vb = 0.8 * v0_kmh
        self.ve = 0.1 * v0_kmh
        self.s_b = None
        self.s_e = None

    def update(self, v_kmh, dist_travelled):
        if self.s_b is None and v_kmh <= self.vb:
            self.s_b = dist_travelled
        if self.s_e is None and v_kmh <= self.ve:
            self.s_e = dist_travelled

    def mfdd(self):
        if self.s_b is None or self.s_e is None or self.s_e <= self.s_b:
            return 0.0
        return (self.vb ** 2 - self.ve ** 2) / (25.92 * (self.s_e - self.s_b))


def write_csv(records, path):
    if not records:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = list(asdict(records[0]).keys())
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in records:
            w.writerow(asdict(r))


def summarize(records):
    """
    รวมผลต่อ label: คืน dict {label: {rc, rc_all, rc_conflict, n, n_conflict, avoided,
                                        mean_ab, n_ab, mean_sc, n_sc, mean_tc, n_tc, mean_dv}}

    กฎการรวม:
      rc_all      = n_avoid / n (ทุกเคส)
      rc_conflict = n_avoid_conflict / n_conflict (เฉพาะเคสที่ is_conflict=True)
      mean_ab     = เฉลี่ย a_b_mfdd เฉพาะแถวที่ > 0 (ช่วง 0.8v→0.1v บันทึกได้)
      mean_sc     = เฉลี่ย s_clearance เฉพาะ avoided=True (กรณีชน s=0 ไม่ใช่ข้อมูล clearance)
      mean_tc     = เฉลี่ย t_c_warn เฉพาะ > 0 (มีการเบรก)
      mean_dv     = เฉลี่ย dv_speed_var ทุกแถว

    NOTE: CPEIM composite (mean_score) ถูกลบออก เพราะน้ำหนัก W_S/W_AB/W_TC/W_DV/W_RC
    มาจากฉาก V-VRU ของ liu2025 ไม่ใช่ฉาก car-to-car ที่เราทดสอบ การใช้ผิดฉาก
    ทำให้คะแนนรวมบิดเบือน รายงาน 5 ดัชนีดิบ + Rc แทน
    """
    by_label = {}
    for r in records:
        by_label.setdefault(r.label, []).append(r)
    out = {}
    for label, rs in by_label.items():
        n = len(rs)
        n_avoid = sum(1 for r in rs if r.avoided)
        rc_all = n_avoid / n if n else 0.0

        # Conflict-only Rc (is_conflict ถูกตั้งในไฟล์ runner ก่อนเปิดซิม — ดู core/conflict.py)
        conflict_rs = [r for r in rs if getattr(r, "is_conflict", True)]
        n_conflict = len(conflict_rs)
        n_conflict_avoid = sum(1 for r in conflict_rs if r.avoided)
        rc_conflict = n_conflict_avoid / n_conflict if n_conflict else 0.0

        # MFDD: เฉลี่ยเฉพาะแถวที่วัดได้ (ต้องผ่านช่วง 0.8v→0.1v จึงมีค่า > 0)
        ab_vals = [r.a_b_mfdd for r in rs if r.a_b_mfdd > 0]
        mean_ab = sum(ab_vals) / len(ab_vals) if ab_vals else 0.0

        # s_clearance: เฉลี่ยเฉพาะเคสที่หยุดสำเร็จ (กรณีชน s=0 ไม่ใช่ข้อมูล clearance จริง)
        sc_vals = [r.s_clearance for r in rs if r.avoided]
        mean_sc = sum(sc_vals) / len(sc_vals) if sc_vals else 0.0

        # T_c warning lead time: เฉลี่ยเฉพาะที่มีการเบรก
        tc_vals = [r.t_c_warn for r in rs if r.t_c_warn > 0]
        mean_tc = sum(tc_vals) / len(tc_vals) if tc_vals else 0.0

        # Δv speed variation: เฉลี่ยทุกแถว
        mean_dv = sum(r.dv_speed_var for r in rs) / n if n else 0.0

        out[label] = dict(
            rc=rc_all, rc_all=rc_all, rc_conflict=rc_conflict,
            n=n, n_conflict=n_conflict, avoided=n_avoid,
            mean_ab=mean_ab, n_ab=len(ab_vals),
            mean_sc=mean_sc, n_sc=len(sc_vals),
            mean_tc=mean_tc, n_tc=len(tc_vals),
            mean_dv=mean_dv,
        )
    return out
