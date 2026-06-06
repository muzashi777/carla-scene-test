# -*- coding: utf-8 -*-
"""
สมองกล baseline — TTC คงที่ (อ้างอิงระบบเบรกฉุกเฉินสากลในเปเปอร์ Sensors)
  TTC ≤ 1.6s → เบรกบางส่วน
  TTC ≤ 0.6s → เบรกเต็ม
ไม่ปรับตามความเร็ว/ความลื่น → คาดว่าจะเบรกไม่ทันในเคสเร็ว+ลื่น (นี่คือจุดอ่อนที่ proposed จะแก้)
"""
from control.base_controller import BaseController, register


@register("baseline")
class BaselineStaticTTC(BaseController):
    def __init__(self, cfg):
        self.ttc_warn = cfg.TTC_WARN_FULL
        self.ttc_full = cfg.TTC_BRAKE_FULL
        self.partial = cfg.PARTIAL_BRAKE
        self._latched = False     # เบรกแล้วเบรกค้าง (ไม่ปล่อย)

    def reset(self):
        self._latched = False

    def decide(self, perc, ego):
        if self._latched:
            return self.control(brake=1.0)
        if not perc.detected:
            return self.control(throttle=0.6)   # ขับต่อ (run loop เป็นคนรักษาความเร็วจริง)
        if perc.ttc <= self.ttc_full:
            self._latched = True
            return self.control(brake=1.0)
        if perc.ttc <= self.ttc_warn:
            return self.control(brake=self.partial)
        return self.control(throttle=0.6)
