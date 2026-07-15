# -*- coding: utf-8 -*-
"""
enhanced_predictive controller — Required-Deceleration + Feedforward Predictor for latency compensation (primary method)
----------------------------------------------------------------------
Extends proposed_enhanced (required-decel): before computing a_req, "predicts the target state
forward by latency L" using constant-acceleration kinematics, then makes decisions on the
predicted values (not stale values due to latency) — a discrete analog of the Smith predictor
(borrowing the delay compensation principle from Xing, Ploeg & Nijmeijer 2019, IEEE T-VT which compensates
 actuator delay in CACC, adapted for AEB — not an AEB paper directly)

Prediction (phase before ego brakes, ego accel ≈ 0):
  v_l_pred = max(0, lead_speed − lead_decel·L)                 # lead vehicle continues to decelerate
  gap_pred = max(0, distance − v_close·L − 0.5·lead_decel·L²)  # gap closes faster because lead vehicle is braking
  a_req    = required_decel(ego.speed_ms, v_l_pred, lead_decel, gap_pred)   # calls the original formula
  urgency  = a_req / (μ·g)                                     # original full/partial threshold

* predicts the 'inputs' fed to the original required_decel() (no new formula) → at L=0 reduces to
  proposed_enhanced exactly every tick (v_l_pred=lead_speed, gap_pred=distance) *
closing acceleration = +lead_decel: lead vehicle brakes → closing faster → gap closes faster
* requires knowing L (the sim harness injects it directly, so it is exact = oracle/idealized upper bound) *
* does not touch baseline/proposed/proposed_enhanced — this is added to recover Rc lost due to latency *
"""
import math
from control.base_controller import BaseController, register, compensation_latency
from core.actors import required_decel, REQ_GAP_EPS


@register("enhanced_predictive")
class EnhancedPredictive(BaseController):
    def __init__(self, cfg):
        self.cfg = cfg
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency >= this value → full brake
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency >= this value → partial brake
        self.g0 = 9.81
        self.L = 0.0              # seconds of compensation (set in reset from run_spec)
        self.last_a_req = 0.0     # for debugging/inspection
        self.last_a_max = 0.0
        self.reset()

    def reset(self):
        # read L from the run_spec bound by make_controller (absent → L=0 = reduces to proposed_enhanced)
        self.L, self.comp_frames, self.comp_source = compensation_latency(
            getattr(self, "run_spec", {}), self.cfg)
        super().reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        L = self.L
        # ── predict inputs forward by L seconds (L=0 → identical to original values) ──
        lead_decel = max(0.0, perc.lead_decel)
        v_close = max(0.0, perc.rel_speed)                       # closing speed (>0 = gap closing)
        v_l_pred = max(0.0, perc.lead_speed - lead_decel * L)    # lead vehicle continues to decelerate (not below 0)
        gap_pred = max(0.0, perc.distance - v_close * L - 0.5 * lead_decel * L * L)
        # predicted imminent collision (gap_pred <= eps) → full brake; at L=0 this is distance <= eps same as proposed_enhanced
        if gap_pred <= REQ_GAP_EPS:
            self.last_a_req, self.last_a_max = a_max, a_max
            return 1.0
        # call the original required-decel formula with 'predicted inputs' — all original logic/guards intact
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
        # same gate as proposed_enhanced: decide when 'detected' or brake already latched
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
