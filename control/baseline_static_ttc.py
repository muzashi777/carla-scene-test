# -*- coding: utf-8 -*-
"""
Baseline controller — fixed TTC thresholds (reference: international AEB standard from the Sensors paper)
  TTC ≤ 1.6s → partial brake
  TTC ≤ 0.6s → full brake
No adaptation to speed / friction → expected to fail in fast + slippery cases (the weakness that proposed fixes)
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
        # compute desired only when a hazard is detected or braking has already started (latch held)
        desired = self._desired(perc) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
