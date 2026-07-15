# -*- coding: utf-8 -*-
"""
enhanced_inflation controller — Required-Deceleration + Threshold Inflation for latency compensation
----------------------------------------------------------------------
Extends proposed_enhanced (required-decel) but *does not predict the target* — simply adds the
distance the vehicle travels during latency L to the required stopping distance, then brakes with
margin (worst-case robust)

  r_required = v_close² / (2·μ·g)     physics-based stopping distance (treats obstacle as stationary, uses closing speed)
  r_required += v_close · L            + distance travelled during perception latency (worst-case margin)
  urgency     = r_required / (gap − r_safe)   ≥1 = insufficient remaining distance → mapped to existing full/partial thresholds

Advantage: no need to know L precisely — designing with upper bound L_max alone gives a conservative safe design
Reference: the delay term (v_close·L) is a standard component of the safety-distance formula family used by Mazda/
           Berkeley-PATH, covered by Rajamani (2012, *Vehicle Dynamics and Control*)
L=0 → inflation term vanishes → reduces to uncompensated required-decel (stationary obstacle)
* Does not touch baseline/proposed/proposed_enhanced — this controller is added solely for Rc recovery comparison *
"""
import math
from control.base_controller import BaseController, register, compensation_latency
from core.actors import REQ_GAP_EPS


@register("enhanced_inflation")
class EnhancedInflation(BaseController):
    def __init__(self, cfg):
        self.cfg = cfg
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency ≥ this value → full brake
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency ≥ this value → partial brake
        self.r_safe = getattr(cfg, "COMP_R_SAFE", 0.0)       # standstill safety margin (default 0 = bumper contact)
        self.g0 = 9.81
        self.L = 0.0                 # compensation duration in seconds (set in reset from run_spec)
        self.last_r_required = 0.0   # for debugging
        self.reset()

    def reset(self):
        # read L from run_spec bound by make_controller (absent → L=0 = no compensation)
        self.L, self.comp_frames, self.comp_source = compensation_latency(
            getattr(self, "run_spec", {}), self.cfg)
        super().reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        # imminent collision (gap ≤ eps) → full brake immediately (same gate as proposed_enhanced)
        if perc.distance <= REQ_GAP_EPS:
            self.last_r_required = 0.0
            return 1.0
        v_close = max(0.0, perc.rel_speed)          # closing speed (>0 = gap shrinking)
        avail = perc.distance - self.r_safe         # effective braking distance remaining
        if a_max <= 1e-6:
            self.last_r_required = math.inf
            return 1.0 if v_close > 0.0 else 0.0
        # required stopping distance = physics-based distance + distance travelled during latency
        r_required = (v_close * v_close) / (2.0 * a_max) + v_close * self.L
        self.last_r_required = r_required
        if avail <= 1e-6:
            return 1.0 if r_required > 0.0 else 0.0
        urgency = r_required / avail
        if urgency >= self.req_full:
            return 1.0
        if urgency >= self.req_warn:
            return self.partial
        return 0.0

    def decide(self, perc, ego):
        # same gate as proposed_enhanced: decide only when 'detected' or brake already latched
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
