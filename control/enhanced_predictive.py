# -*- coding: utf-8 -*-
"""
สมองกล enhanced_predictive — Required-Deceleration + Feedforward Predictor ชดเชย latency (วิธีหลัก)
----------------------------------------------------------------------
ต่อยอดจาก proposed_enhanced (required-decel): ก่อนคำนวณ a_req จะ "พยากรณ์สถานะเป้า
ไปข้างหน้าเท่ากับ latency L" ด้วยจลนศาสตร์ constant-acceleration แล้วตัดสินใจบนค่าที่
พยากรณ์ (ไม่ใช่ค่าที่ล้าสมัยเพราะ latency) — เป็น discrete analog ของ Smith predictor
(ยืมหลักการ delay compensation จาก Xing, Ploeg & Nijmeijer 2019, IEEE T-VT ที่ชดเชย
 actuator delay ใน CACC มาปรับใช้กับ AEB — ไม่ใช่งาน AEB โดยตรง)

การพยากรณ์ (ช่วงก่อน ego เบรก ego accel ≈ 0):
  v_l_pred = max(0, lead_speed − lead_decel·L)                 # คันหน้าชะลอต่อ
  gap_pred = max(0, distance − v_close·L − 0.5·lead_decel·L²)  # gap หดเร็วขึ้นเพราะคันหน้าเบรก
  a_req    = required_decel(ego.speed_ms, v_l_pred, lead_decel, gap_pred)   # เรียกสูตรเดิม
  urgency  = a_req / (μ·g)                                     # เกณฑ์ full/partial เดิม

* พยากรณ์ 'อินพุต' ที่ป้อน required_decel() เดิม (ไม่เขียนสูตรใหม่) → ที่ L=0 ลดรูปเป็น
  proposed_enhanced เป๊ะทุก tick (v_l_pred=lead_speed, gap_pred=distance) *
closing acceleration = +lead_decel: คันหน้าเบรก → closing เร็วขึ้น → gap หดเร็วขึ้น
* ต้องรู้ L (ในซิม harness ฉีดเอง จึงรู้เป๊ะ = oracle/idealized upper bound) *
* ไม่แตะ baseline/proposed/proposed_enhanced — เพิ่มตัวนี้เพื่อกู้ Rc ที่เสียไปเพราะ latency *
"""
import math
from control.base_controller import BaseController, register, compensation_latency
from core.actors import required_decel, REQ_GAP_EPS


@register("enhanced_predictive")
class EnhancedPredictive(BaseController):
    def __init__(self, cfg):
        self.cfg = cfg
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency ≥ ค่านี้ → เบรกเต็ม
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency ≥ ค่านี้ → เบรกบางส่วน
        self.g0 = 9.81
        self.L = 0.0              # วินาทีที่ชดเชย (ตั้งใน reset จาก run_spec)
        self.last_a_req = 0.0     # ไว้ดีบัก/ตรวจสอบ
        self.last_a_max = 0.0
        self.reset()

    def reset(self):
        # อ่านค่า L จาก run_spec ที่ make_controller ผูกไว้ (ไม่มี → L=0 = ลดรูปเป็น proposed_enhanced)
        self.L, self.comp_frames, self.comp_source = compensation_latency(
            getattr(self, "run_spec", {}), self.cfg)
        super().reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        L = self.L
        # ── พยากรณ์อินพุตไปข้างหน้า L วินาที (L=0 → ค่าเดิมเป๊ะ) ──
        lead_decel = max(0.0, perc.lead_decel)
        v_close = max(0.0, perc.rel_speed)                       # closing speed (>0 = gap หด)
        v_l_pred = max(0.0, perc.lead_speed - lead_decel * L)    # คันหน้าชะลอต่อ (ไม่ต่ำกว่า 0)
        gap_pred = max(0.0, perc.distance - v_close * L - 0.5 * lead_decel * L * L)
        # จวนชนตามที่พยากรณ์ (gap_pred ≤ eps) → เบรกเต็ม; ที่ L=0 คือ distance ≤ eps เหมือน proposed_enhanced
        if gap_pred <= REQ_GAP_EPS:
            self.last_a_req, self.last_a_max = a_max, a_max
            return 1.0
        # เรียกสูตร required-decel เดิม ด้วย 'อินพุตที่พยากรณ์แล้ว' — logic/guard เดิมทั้งหมด
        a_req = required_decel(ego.speed_ms, v_l_pred, lead_decel, gap_pred)
        self.last_a_req, self.last_a_max = a_req, a_max
        if a_max <= 1e-6:
            urgency = math.inf if a_req > 0.0 else 0.0
        else:
            urgency = a_req / a_max
        if urgency >= self.req_full:
            return 1.0
        if urgency >= self.req_warn:
            return self.partial
        return 0.0

    def decide(self, perc, ego):
        # เกตเดียวกับ proposed_enhanced: ตัดสินใจเมื่อ 'ตรวจเจอ' หรือ latch เบรกไปแล้ว
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
