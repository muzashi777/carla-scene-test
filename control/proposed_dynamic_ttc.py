# -*- coding: utf-8 -*-
"""
Proposed controller — Dynamic TTC: adjusts thresholds based on speed + road friction
  Higher speed or lower friction → raises the threshold → brakes earlier → sufficient stopping distance
  thr_full = TTC_BRAKE_FULL + K_SPEED*max(0,(v-V0)/100) + K_MU*max(0,(MU0-mu))
This is the strategy recommended by the Master Plan ("if speed>50 and friction==0.40: trigger_brake_TTC=1.2")
but implemented continuously for smoother adaptation and easier tuning
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
        self.reset()

    def _dynamic_full(self, ego):
        bump = (self.k_speed * max(0.0, (ego.speed_kmh - self.v0) / 100.0)
                + self.k_mu * max(0.0, (self.mu0 - ego.mu)))
        return self.base_full + bump

    def _desired(self, perc, ego):
        thr_full = self._dynamic_full(ego)
        thr_warn = max(self.base_warn, thr_full + 0.5)
        if perc.ttc <= thr_full:
            return 1.0
        if perc.ttc <= thr_warn:
            return self.partial
        return 0.0

    def decide(self, perc, ego):
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
