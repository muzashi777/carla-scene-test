# -*- coding: utf-8 -*-
"""
สมองกล proposed_enhanced — Required-Deceleration (รู้ทั้งแรงเสียดทาน + การเบรกของรถข้างหน้า)
----------------------------------------------------------------------
ปัญหาของ TTC ล้วน: TTC = gap / closing_speed สมมติรถข้างหน้าวิ่งความเร็วคงที่
  → ในฉาก CCRb (รถข้างหน้าเบรกจนหยุด) TTC ยังสูงอยู่จนระยะเหลือน้อยมาก เบรกไม่ทัน
แนวคิดใหม่: คำนวณ "ความหน่วงที่ ego จำเป็นต้องใช้" (a_req) เทียบกับเพดานแรงเสียดทาน (μ·g)
  a_max = μ·g                         เพดานความหน่วงที่ทำได้จริง (ขึ้นกับถนน)
  d_lead = v_l²/2a_l                  ระยะที่รถข้างหน้ายังวิ่งต่อก่อนหยุด
  a_req = v_e² / (2·(gap + d_lead))   ความหน่วงต่ำสุดที่ต้องใช้เพื่อหยุดทัน
  urgency = a_req / a_max             ยิ่งเข้าใกล้ 1 = ยิ่งใกล้ขีดจำกัดถนน
สูตรนี้รู้ทั้ง μ และการเบรกของรถข้างหน้า และทำงานถูกทั้งฉาก cut-in (dart จอด: v_l=0,d_lead=0
→ a_req=v_e²/2gap = เคสสิ่งกีดขวางนิ่ง) และฉาก lead-brake โดยอัตโนมัติ
* ไม่แตะ baseline และ proposed เดิม — เพิ่มตัวนี้เพื่อเทียบ 3 ทาง *
"""
import math
from control.base_controller import BaseController, register
from core.actors import required_decel, REQ_GAP_EPS


@register("proposed_enhanced")
class ProposedEnhancedReqDecel(BaseController):
    def __init__(self, cfg):
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency ≥ ค่านี้ → เบรกเต็ม
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency ≥ ค่านี้ → เบรกบางส่วน
        self.g0 = 9.81
        self.last_a_req = 0.0     # ไว้ดีบัก/ตรวจสอบ (runner บันทึกเองผ่าน core.actors.required_decel)
        self.last_a_max = 0.0
        self.reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        # จวนชน (gap ≤ eps) → urgency สูงสุด เบรกเต็มทันที (กันค่า a_req เพี้ยน/ระเบิดตอน gap→0)
        if perc.distance <= REQ_GAP_EPS:
            self.last_a_req, self.last_a_max = a_max, a_max
            return 1.0
        a_req = required_decel(ego.speed_ms, perc.lead_speed, perc.lead_decel, perc.distance)
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
        # ตัดสินใจเฉพาะเมื่อ "ตรวจเจอ" (หรือ latch เบรกไปแล้ว) — เกตเดียวกับ proposed
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
