# -*- coding: utf-8 -*-
"""
สมองกล proposed — Dynamic TTC: ปรับ threshold ตามความเร็ว + ความลื่นถนน
  เร็วขึ้น หรือ ลื่นขึ้น  → ยก threshold ให้สูงขึ้น → เบรกล่วงหน้าเร็วขึ้น → ระยะหยุดพอ
  thr_full = TTC_BRAKE_FULL + K_SPEED*max(0,(v-V0)/100) + K_MU*max(0,(MU0-mu))
นี่คือกลยุทธ์ที่ Master Plan แนะนำ ("if speed>50 and friction==0.40: trigger_brake_TTC=1.2")
แต่ทำเป็นต่อเนื่องเพื่อให้ปรับได้นุ่มนวลและจูนง่าย
"""
from control.base_controller import BaseController, register


@register("proposed")
class ProposedDynamicTTC(BaseController):
    def __init__(self, cfg):
        self.base_full = cfg.TTC_BRAKE_FULL
        self.base_warn = cfg.TTC_WARN_FULL
        self.v0 = cfg.DYN_V0
        self.mu0 = cfg.DYN_MU0
        self.k_speed = cfg.DYN_K_SPEED
        self.k_mu = cfg.DYN_K_MU
        self.partial = cfg.PARTIAL_BRAKE
        self._latched = False

    def reset(self):
        self._latched = False

    def _dynamic_full(self, ego):
        bump = (self.k_speed * max(0.0, (ego.speed_kmh - self.v0) / 100.0)
                + self.k_mu * max(0.0, (self.mu0 - ego.mu)))
        return self.base_full + bump

    def decide(self, perc, ego):
        if self._latched:
            return self.control(brake=1.0)
        if not perc.detected:
            return self.control(throttle=0.6)
        thr_full = self._dynamic_full(ego)
        thr_warn = max(self.base_warn, thr_full + 0.5)
        if perc.ttc <= thr_full:
            self._latched = True
            return self.control(brake=1.0)
        if perc.ttc <= thr_warn:
            return self.control(brake=self.partial)
        return self.control(throttle=0.6)
