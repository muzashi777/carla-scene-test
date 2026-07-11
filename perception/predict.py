# -*- coding: utf-8 -*-
"""
PerceptionPredictor — feedforward predictor applied to Perception BEFORE it
reaches any controller. It is the mirror image of PerceptionDegrader:

    degrade  makes the perception input stale (delays it by L)
    predict  makes it fresh again (extrapolates it forward by L)

so that ANY base controller (baseline / proposed / proposed_enhanced) can be
run "with predictive-oracle compensation" simply by inserting this layer in
front of it — no change to the controller's decision logic.

Prediction model (constant-acceleration, identical to control/enhanced_predictive.py):
  Before the ego brakes a_ego ≈ 0, so the gap closes with closing acceleration
  +lead_decel (the lead braking makes the gap close faster):

    lead_decel   = max(0, perc.lead_decel)                         # unchanged (const accel)
    v_close      = max(0, perc.rel_speed)                          # closing speed now
    lead_speed'  = max(0, lead_speed − lead_decel·L)              # lead keeps decelerating
    distance'    = max(0, distance − v_close·L − 0.5·lead_decel·L²)
    rel_speed'   = v_close + lead_decel·L                          # d(-gap)/dt at t=L
    ttc'         = distance' / rel_speed'   (∞ if rel_speed' ≈ 0)
    detected, box_h : passed through unchanged

Why these exact terms: `distance'` and `lead_speed'` are the SAME expressions
`enhanced_predictive` feeds into required_decel(). Because `proposed_enhanced`
reads only distance / lead_speed / lead_decel / detected, wrapping it with this
layer reproduces `enhanced_predictive` per tick (proven in
tests/test_latency_comp_all.py). rel_speed'/ttc' extend the same model to the
TTC-based controllers (baseline / proposed) so every base controller receives
the SAME predicted Perception (fairness, controller-swap protocol).

L = 0 (identity):  apply() is a strict no-op — it returns a copy of the input
Perception with every field unchanged (all extrapolation terms are 0). This
guarantees compensated_X(L=0) == base controller X per tick, for every X.

Oracle:  L is provided by the harness (L = delay_frames · FIXED_DT, known
exactly), matching the `latency_comp` upper-bound framing (comp_source="oracle").
"""
import copy
import math


class PerceptionPredictor:

    def __init__(self, l_seconds=0.0):
        self.L = max(0.0, float(l_seconds))

    def reset(self):
        """Kept for API symmetry with PerceptionDegrader (predictor is stateless)."""
        pass

    def apply(self, perc, ego):
        """
        Extrapolate Perception forward by L seconds (latency compensation).
        Returns (perc_predicted, ego). ego is passed through unchanged.
        L = 0 → exact identity (a copy with all fields unchanged).
        """
        L = self.L
        out = copy.copy(perc)
        if L <= 0.0:
            return out, ego   # identity: predicted Perception == original Perception

        lead_decel = max(0.0, perc.lead_decel)
        v_close = max(0.0, perc.rel_speed)                       # closing speed (>0 = gap shrinking)
        out.lead_decel = lead_decel
        out.lead_speed = max(0.0, perc.lead_speed - lead_decel * L)
        out.distance = max(0.0, perc.distance - v_close * L - 0.5 * lead_decel * L * L)
        rel_pred = v_close + lead_decel * L                      # closing speed at t=L
        out.rel_speed = rel_pred
        out.ttc = (out.distance / rel_pred) if rel_pred > 1e-3 else math.inf
        # detected / box_h: passed through unchanged (prediction does not create detections)
        return out, ego
