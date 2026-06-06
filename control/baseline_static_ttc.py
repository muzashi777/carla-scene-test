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
        self.reset()

    def _desired(self, perc):
        if perc.ttc <= self.ttc_full:
            return 1.0
        if perc.ttc <= self.ttc_warn:
            return self.partial
        return 0.0

    def decide(self, perc, ego):
        # คิด desired เฉพาะเมื่อเห็นอันตราย หรือเคยเริ่มเบรกแล้ว (latch ค้าง)
        desired = self._desired(perc) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
