#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for latency-compensated controllers — no CARLA required.
Mocks the `carla` module (only VehicleControl is used by BaseController).
Run from project root: python tests/test_latency_comp.py

Covers acceptance criteria (AI_INSTRUCTION §7):
  3. enhanced_predictive @ L=0  == proposed_enhanced  (brake per tick, exact)
     enhanced_inflation  @ L=0  → no compensation (inflation term = 0)
     @ L>0 both brake earlier (a_req / r_required grow with L)
     mismatched: comp_L_frames ≠ delay_frames → controller uses comp_L_frames
  4. build_matrix_runs("latency_comp"/"latency_mismatch") correct; old modes unchanged
"""
import sys
import os
import math
import types as _pytypes

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── mock carla (BaseController.control uses carla.VehicleControl only) ──
if "carla" not in sys.modules:
    class _VehicleControl:
        def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
            self.throttle = float(throttle)
            self.brake = float(brake)
            self.steer = float(steer)
    _carla = _pytypes.ModuleType("carla")
    _carla.VehicleControl = _VehicleControl
    sys.modules["carla"] = _carla

from core.types import Perception, EgoState
from control.base_controller import make_controller, compensation_latency
import control.proposed_enhanced    # noqa: F401  (register proposed_enhanced)
import control.enhanced_predictive  # noqa: F401  (register enhanced_predictive)
import control.enhanced_inflation   # noqa: F401  (register enhanced_inflation)
import config.scenario_cutin as cutin_cfg
import config.scenario_lead_brake as lead_cfg


class _Cfg:
    """cfg-like stub with the attributes the controllers read."""
    PARTIAL_BRAKE = 0.4
    REQ_FULL_FRAC = 0.9
    REQ_WARN_FRAC = 0.6
    FIXED_DT = 0.05
    COMP_R_SAFE = 0.0


CFG = _Cfg()


def _perc(distance, rel_speed, lead_speed=0.0, lead_decel=0.0, detected=True):
    ttc = distance / rel_speed if rel_speed > 1e-3 else math.inf
    return Perception(detected=detected, distance=distance, rel_speed=rel_speed,
                      ttc=ttc, lead_speed=lead_speed, lead_decel=lead_decel)


def _ego(v_ms=13.9, mu=0.85):
    return EgoState(speed_ms=v_ms, speed_kmh=v_ms * 3.6, mu=mu)


# ── approach scenario sequences fed to both controllers for tick-by-tick comparison ──
def _cutin_sequence():
    """dart blocking: lead_speed=0, lead_decel=0, gap shrinks steadily"""
    seq = []
    dist = 40.0
    for _ in range(60):
        seq.append((_perc(dist, rel_speed=13.9, lead_speed=0.0, lead_decel=0.0), _ego()))
        dist = max(0.0, dist - 13.9 * CFG.FIXED_DT)
    return seq


def _leadbrake_sequence():
    """lead vehicle braking: lead_speed>0 decreasing, lead_decel>0, closing speed grows"""
    seq = []
    dist = 25.0
    v_lead = 13.9
    for _ in range(60):
        vclose = max(0.0, 13.9 - v_lead)
        seq.append((_perc(dist, rel_speed=vclose + 0.5, lead_speed=v_lead, lead_decel=4.0),
                    _ego()))
        dist = max(0.0, dist - (vclose + 0.5) * CFG.FIXED_DT)
        v_lead = max(0.0, v_lead - 4.0 * CFG.FIXED_DT)
    return seq


def test_predictive_L0_equals_proposed_enhanced():
    """enhanced_predictive @ L=0 gives brake identical to proposed_enhanced on every tick."""
    for seq_name, seq in (("cutin", _cutin_sequence()), ("leadbrake", _leadbrake_sequence())):
        base = make_controller("proposed_enhanced", CFG, run_spec={})
        pred = make_controller("enhanced_predictive", CFG, run_spec={})  # {} → oracle, delay 0 → L=0
        base.reset()
        pred.reset()
        assert pred.L == 0.0, f"L should be 0 for empty run_spec (got {pred.L})"
        for i, (p, e) in enumerate(seq):
            b = base.decide(p, e).brake
            q = pred.decide(p, e).brake
            assert b == q, f"[{seq_name}] tick {i}: proposed_enhanced={b} != predictive={q}"
    print("PASS test_predictive_L0_equals_proposed_enhanced")


def test_inflation_L0_no_compensation():
    """enhanced_inflation @ L=0: inflation term (v_close·L) = 0 → r_required = v_close²/(2·μg)."""
    inf = make_controller("enhanced_inflation", CFG, run_spec={})
    inf.reset()
    assert inf.L == 0.0
    ego = _ego()
    a_max = ego.mu * inf.g0
    for dist in (30.0, 15.0, 8.0):
        p = _perc(dist, rel_speed=13.9)
        inf.reset()
        inf.decide(p, ego)
        expected = (13.9 ** 2) / (2.0 * a_max)   # no latency term
        assert abs(inf.last_r_required - expected) < 1e-9, \
            f"L=0 r_required={inf.last_r_required} != {expected}"
    print("PASS test_inflation_L0_no_compensation")


def test_predictive_L_grows_urgency():
    """enhanced_predictive: larger L → larger computed a_req (brakes earlier)."""
    ego = _ego()
    p = _perc(distance=20.0, rel_speed=13.9, lead_speed=8.0, lead_decel=4.0)
    a_reqs = []
    for L_frames in (0, 4, 8, 16):
        pred = make_controller("enhanced_predictive", CFG,
                               run_spec={"comp_source": "oracle", "delay_frames": L_frames})
        pred.reset()
        pred.decide(p, ego)
        a_reqs.append(pred.last_a_req)
    for a, b in zip(a_reqs, a_reqs[1:]):
        assert b >= a, f"a_req must not decrease as L grows: {a_reqs}"
    assert a_reqs[-1] > a_reqs[0], f"max L must have a_req > L=0: {a_reqs}"
    print("PASS test_predictive_L_grows_urgency")


def test_inflation_L_grows_required_distance():
    """enhanced_inflation: larger L → larger r_required (brakes with more margin)."""
    ego = _ego()
    p = _perc(distance=20.0, rel_speed=13.9)
    r_reqs = []
    for L_frames in (0, 4, 8, 16):
        inf = make_controller("enhanced_inflation", CFG,
                              run_spec={"comp_source": "oracle", "delay_frames": L_frames})
        inf.reset()
        inf.decide(p, ego)
        r_reqs.append(inf.last_r_required)
    for a, b in zip(r_reqs, r_reqs[1:]):
        assert b > a, f"r_required must grow as L grows: {r_reqs}"
    print("PASS test_inflation_L_grows_required_distance")


def test_mismatched_uses_comp_L_frames():
    """mismatched: comp_L_frames ≠ delay_frames → uses comp_L_frames for compensation."""
    # test helper directly
    L_sec, frames, src = compensation_latency(
        {"comp_source": "mismatched", "delay_frames": 8, "comp_L_frames": 4}, CFG)
    assert src == "mismatched"
    assert frames == 4, f"mismatched must use comp_L_frames=4 (got {frames})"
    assert abs(L_sec - 4 * CFG.FIXED_DT) < 1e-12
    # oracle ignores comp_L_frames, uses delay_frames
    L_sec2, frames2, src2 = compensation_latency(
        {"comp_source": "oracle", "delay_frames": 8, "comp_L_frames": 4}, CFG)
    assert frames2 == 8 and src2 == "oracle"
    # at controller: mismatched L=4f vs oracle L=8f must yield different compensation
    pred_mis = make_controller("enhanced_predictive", CFG,
                               run_spec={"comp_source": "mismatched", "delay_frames": 8,
                                         "comp_L_frames": 4})
    pred_mis.reset()
    assert abs(pred_mis.L - 4 * CFG.FIXED_DT) < 1e-12
    assert pred_mis.comp_frames == 4
    print("PASS test_mismatched_uses_comp_L_frames")


def test_existing_controllers_ignore_run_spec():
    """existing controller (proposed_enhanced) ignores run_spec — behavior does not change with comp_*."""
    ego = _ego()
    p = _perc(distance=15.0, rel_speed=13.9)
    a = make_controller("proposed_enhanced", CFG, run_spec={})
    b = make_controller("proposed_enhanced", CFG,
                        run_spec={"comp_source": "oracle", "delay_frames": 16, "comp_L_frames": 16})
    a.reset(); b.reset()
    assert a.decide(p, ego).brake == b.decide(p, ego).brake
    print("PASS test_existing_controllers_ignore_run_spec")


def _count(mode, cfg):
    return cfg.build_matrix_runs(mode)


def test_build_matrix_runs_new_modes():
    """latency_comp / latency_mismatch return complete correct run specs; existing modes unchanged."""
    for cfg in (cutin_cfg, lead_cfg):
        # existing modes unchanged
        assert len(_count("original", cfg)) == len(cfg.CONTROLLERS)
        assert len(_count("latency", cfg)) == len(cfg.CONTROLLERS) * len(cfg.LATENCY_DELAY_FRAMES)
        assert len(_count("noise", cfg)) == len(cfg.CONTROLLERS) * len(cfg.NOISE_SIGMA_M_SWEEP)
        for r in _count("original", cfg):
            assert "comp_source" not in r  # existing modes have no comp key

        # latency_comp: COMP_CONTROLLERS × delay, comp_source=oracle, comp_L_frames=delay
        comp = _count("latency_comp", cfg)
        assert len(comp) == len(cfg.COMP_CONTROLLERS) * len(cfg.LATENCY_DELAY_FRAMES)
        for r in comp:
            assert r["comp_source"] == "oracle"
            assert r["comp_L_frames"] == r["delay_frames"]
            assert r["controller"] in cfg.COMP_CONTROLLERS

        # latency_mismatch: enhanced_predictive × comp_L_frames, delay fixed
        mis = _count("latency_mismatch", cfg)
        assert len(mis) == len(cfg.COMP_MISMATCH_L_FRAMES)
        for r in mis:
            assert r["controller"] == cfg.COMP_MISMATCH_CTRL
            assert r["comp_source"] == "mismatched"
            assert r["delay_frames"] == cfg.COMP_MISMATCH_DELAY
            assert r["comp_L_frames"] in cfg.COMP_MISMATCH_L_FRAMES
    print("PASS test_build_matrix_runs_new_modes")


if __name__ == "__main__":
    test_predictive_L0_equals_proposed_enhanced()
    test_inflation_L0_no_compensation()
    test_predictive_L_grows_urgency()
    test_inflation_L_grows_required_distance()
    test_mismatched_uses_comp_L_frames()
    test_existing_controllers_ignore_run_spec()
    test_build_matrix_runs_new_modes()
    print("\nAll latency-compensation tests passed.")
