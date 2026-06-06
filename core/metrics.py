# -*- coding: utf-8 -*-
"""
เก็บ 5 ดัชนี CPEIM ต่อเคส แล้วรวมเป็น R_c และคะแนนต่อสมองกล
  s    = clearance distance ตอนหยุดสนิท (m)        — ผ่านเต็มถ้า 0 < s ≤ 0.6
  a_b  = MFDD ความหน่วงเฉลี่ยช่วง 0.8v→0.1v (m/s²)
  T_c  = warning lead time (s) = TTC ตอนเริ่มเบรก
  dv   = speed variation (km/h) = v_เริ่มเบรก − v_ปะทะ (หรือ v_เริ่ม ถ้าหยุดสนิท)
  Rc   = สัดส่วนเคสที่หลบชนสำเร็จ ต่อสมองกล
หมายเหตุ: CPEIM เต็มสูตรในเปเปอร์รวม 4 ฉาก; ที่นี่โฟกัสฉาก cut-in ฉากเดียว
จึงรายงาน 5 ดัชนีดิบ + Rc + คะแนน scenario-level (น้ำหนักดัชนีฉากที่ 2)
"""
import csv
import os
from dataclasses import dataclass, field, asdict


# น้ำหนักดัชนีของฉากที่ 2 (vertical V-VRU) จากเปเปอร์ — ใกล้เคียงฉาก cut-in สุด
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
    s_clearance: float = 0.0       # m (>0 = ระยะเหลือตอนหยุด, 0 = ชน/ไม่หยุด)
    a_b_mfdd: float = 0.0          # m/s²
    t_c_warn: float = 0.0          # s
    dv_speed_var: float = 0.0      # km/h
    collision_speed_kmh: float = 0.0
    min_dist: float = 0.0
    result_txt: str = ""


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
    """รวมผลต่อ label: คืน dict {label: {rc, n, avoided, mean_score}}"""
    by_label = {}
    for r in records:
        by_label.setdefault(r.label, []).append(r)
    out = {}
    for label, rs in by_label.items():
        n = len(rs)
        n_avoid = sum(1 for r in rs if r.avoided)
        rc = n_avoid / n if n else 0.0
        # คะแนน scenario-level เฉลี่ย (normalize a_b, dv แบบหยาบเพื่อเทียบเชิงสัมพัทธ์)
        scores = []
        for r in rs:
            s_sc = score_clearance(r.s_clearance, r.avoided)
            rc_sc = 1.0 if r.avoided else 0.0
            tc_sc = 1.0 if r.t_c_warn >= 1.0 else (0.5 if r.t_c_warn > 0 else 0.0)
            dv_sc = 1.0 if r.dv_speed_var >= 20 else 0.5
            ab_sc = 1.0 if 2.0 < r.a_b_mfdd <= 9.0 else 0.5
            scores.append(W_S*s_sc + W_AB*ab_sc + W_TC*tc_sc + W_DV*dv_sc + W_RC*rc_sc)
        out[label] = dict(rc=rc, n=n, avoided=n_avoid,
                          mean_score=sum(scores)/n if n else 0.0)
    return out
