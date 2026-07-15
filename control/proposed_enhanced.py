# -*- coding: utf-8 -*-
"""
proposed_enhanced controller — Required-Deceleration (aware of both friction and lead-vehicle braking)
----------------------------------------------------------------------
Problem with pure TTC: TTC = gap / closing_speed assumes the lead vehicle travels at constant speed.
  → In the CCRb scenario (lead vehicle brakes to a stop), TTC remains high until very little gap is left, too late to brake.
New concept: compute the "deceleration ego must apply" (a_req) against the friction ceiling (μ·g).
  a_max = μ·g                         maximum achievable deceleration (depends on road surface)
  d_lead = v_l²/2a_l                  distance the lead vehicle will still travel before stopping
  a_req = v_e² / (2·(gap + d_lead))   minimum deceleration needed to stop in time
  urgency = a_req / a_max             closer to 1 = closer to the road traction limit
This formula accounts for both μ and lead-vehicle braking, and works correctly for both the cut-in scenario
(dart parked: v_l=0, d_lead=0 → a_req=v_e²/2gap = stationary-obstacle case) and the lead-brake scenario automatically.
* Does not touch the original baseline and proposed — this controller is added for a 3-way comparison *
"""
import math
from control.base_controller import BaseController, register
from core.actors import required_decel, REQ_GAP_EPS


@register("proposed_enhanced")
class ProposedEnhancedReqDecel(BaseController):
    def __init__(self, cfg):
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency ≥ this value → full brake
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency ≥ this value → partial brake
        self.g0 = 9.81
        self.last_a_req = 0.0     # for debug/inspection (runner logs via core.actors.required_decel)
        self.last_a_max = 0.0
        self.reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        # Imminent collision (gap ≤ eps) → max urgency, full brake immediately (prevent a_req diverging/blowing up as gap→0)
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
        # Decide only when "detected" (or brake latch already active) — same gate as proposed
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
