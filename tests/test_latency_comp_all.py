#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for TEST_MODE=latency_comp_all — the predictive-oracle compensation
LAYER (perception/predict.py) applied in front of every base controller.
No CARLA required (mocks the `carla` module, as in tests/test_latency_comp.py).
Run from project root:  python tests/test_latency_comp_all.py

Covers the task's correctness criteria:
  1. L=0 identity — PerceptionPredictor(L=0) returns the Perception unchanged, so
     compensated_X(L=0) == base controller X per tick, for every base controller X.
  2. enhanced_predictive consistency — PerceptionPredictor + proposed_enhanced
     reproduces the existing control/enhanced_predictive.py per tick, for L in
     {0,4,8,16}, on both a cut-in and a lead-brake synthetic sequence.
  3. Run-count check — build_matrix_runs("latency_comp_all") == |CONTROLLERS| ×
     |LATENCY_DELAY_FRAMES| runs, all comp_source="oracle", comp_L_frames==delay_frames,
     predict=True, controller in CONTROLLERS. Existing modes are unchanged.
"""
import sys
import os
import math
import types as _pytypes

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── mock carla (BaseController.control ใช้ carla.VehicleControl เท่านั้น) ──
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
from control.base_controller import make_controller
from perception.predict import PerceptionPredictor
import control.baseline_static_ttc   # noqa: F401  (register baseline)
import control.proposed_dynamic_ttc  # noqa: F401  (register proposed)
import control.proposed_enhanced     # noqa: F401  (register proposed_enhanced)
import control.enhanced_predictive   # noqa: F401  (register enhanced_predictive)
import config.scenario_cutin as cutin_cfg
import config.scenario_lead_brake as lead_cfg


class _Cfg:
    """cfg-like stub with the attributes the controllers read."""
    PARTIAL_BRAKE = 0.4
    REQ_FULL_FRAC = 0.9
    REQ_WARN_FRAC = 0.6
    FIXED_DT = 0.05
    # baseline / proposed (TTC) params
    TTC_WARN_FULL = 1.6
    TTC_BRAKE_FULL = 0.6
    DYN_V0 = 40.0
    DYN_MU0 = 0.85
    DYN_K_SPEED = 1.2
    DYN_K_MU = 1.5


CFG = _Cfg()
BASE_CONTROLLERS = ["baseline", "proposed", "proposed_enhanced"]


def _perc(distance, rel_speed, lead_speed=0.0, lead_decel=0.0, detected=True):
    ttc = distance / rel_speed if rel_speed > 1e-3 else math.inf
    return Perception(detected=detected, distance=distance, rel_speed=rel_speed,
                      ttc=ttc, lead_speed=lead_speed, lead_decel=lead_decel)


def _ego(v_ms=13.9, mu=0.85):
    return EgoState(speed_ms=v_ms, speed_kmh=v_ms * 3.6, mu=mu)


def _cutin_sequence():
    """dart จอดขวาง: lead_speed=0, lead_decel=0, gap หดลงเรื่อย ๆ"""
    seq = []
    dist = 40.0
    for _ in range(60):
        seq.append((_perc(dist, rel_speed=13.9, lead_speed=0.0, lead_decel=0.0), _ego()))
        dist = max(0.0, dist - 13.9 * CFG.FIXED_DT)
    return seq


def _leadbrake_sequence():
    """รถนำเบรก: lead_speed>0 ลดลง, lead_decel>0, closing โตขึ้น"""
    seq = []
    dist = 25.0
    v_lead = 13.9
    for _ in range(60):
        vclose = max(0.0, 13.9 - v_lead)
        seq.append((_perc(dist, rel_speed=vclose + 0.5, lead_speed=v_lead, lead_decel=4.0), _ego()))
        dist = max(0.0, dist - (vclose + 0.5) * CFG.FIXED_DT)
        v_lead = max(0.0, v_lead - 4.0 * CFG.FIXED_DT)
    return seq


def test_predictor_L0_is_identity():
    """PerceptionPredictor(L=0) คืน Perception ที่ทุกฟิลด์เหมือนเดิม (identity)."""
    pred = PerceptionPredictor(l_seconds=0.0)
    for p, e in (_cutin_sequence() + _leadbrake_sequence()):
        out, out_ego = pred.apply(p, e)
        assert out == p, f"L=0 must be identity: {out} != {p}"
        assert out_ego is e
    print("PASS test_predictor_L0_is_identity")


def test_compensated_L0_equals_base_per_tick():
    """compensated_X(L=0) == base controller X ทุก tick สำหรับ base controller ทุกตัว."""
    pred = PerceptionPredictor(l_seconds=0.0)
    for name in BASE_CONTROLLERS:
        for seq_name, seq in (("cutin", _cutin_sequence()), ("leadbrake", _leadbrake_sequence())):
            base = make_controller(name, CFG, run_spec={})
            comp = make_controller(name, CFG, run_spec={})
            base.reset()
            comp.reset()
            for i, (p, e) in enumerate(seq):
                b = base.decide(p, e).brake
                pp, ee = pred.apply(p, e)         # L=0 → identity
                q = comp.decide(pp, ee).brake
                assert b == q, f"[{name}/{seq_name}] tick {i}: base={b} != compensated={q}"
    print("PASS test_compensated_L0_equals_base_per_tick")


def test_predictor_wrap_matches_enhanced_predictive():
    """PerceptionPredictor + proposed_enhanced == control/enhanced_predictive per tick.

    ยืนยันว่า layer ที่พยากรณ์ 'ฟิลด์ Perception' ให้ผลตรงกับ enhanced_predictive เดิม
    ที่พยากรณ์ 'อินพุตของ required_decel' (proposed_enhanced อ่านแค่ distance/lead_speed/
    lead_decel/detected → สองทางเท่ากันเป๊ะ) สำหรับ L in {0,4,8,16} ทั้งสองฉาก
    """
    for L_frames in (0, 4, 8, 16):
        L_sec = L_frames * CFG.FIXED_DT
        pred = PerceptionPredictor(l_seconds=L_sec)
        for seq_name, seq in (("cutin", _cutin_sequence()), ("leadbrake", _leadbrake_sequence())):
            wrapped = make_controller("proposed_enhanced", CFG, run_spec={})  # base, no self-comp
            existing = make_controller("enhanced_predictive", CFG,
                                       run_spec={"comp_source": "oracle", "delay_frames": L_frames})
            wrapped.reset()
            existing.reset()
            assert abs(existing.L - L_sec) < 1e-12
            for i, (p, e) in enumerate(seq):
                pp, ee = pred.apply(p, e)
                w = wrapped.decide(pp, ee).brake       # predictor-layer + proposed_enhanced
                x = existing.decide(p, e).brake         # existing internal-predictor controller
                assert w == x, (f"[L={L_frames}f/{seq_name}] tick {i}: "
                                f"layer+proposed_enhanced={w} != enhanced_predictive={x}")
    print("PASS test_predictor_wrap_matches_enhanced_predictive")


def test_predictor_L_shrinks_gap_and_ttc():
    """L มากขึ้น → distance/ttc ที่พยากรณ์เล็กลง (เบรกเร็วขึ้น) — sanity ของ layer เอง."""
    p = _perc(distance=20.0, rel_speed=8.0, lead_speed=6.0, lead_decel=4.0)
    e = _ego()
    prev_dist, prev_ttc = math.inf, math.inf
    for L_frames in (0, 4, 8, 16):
        pred = PerceptionPredictor(l_seconds=L_frames * CFG.FIXED_DT)
        out, _ = pred.apply(p, e)
        assert out.distance <= prev_dist + 1e-9, f"distance ต้องไม่โตเมื่อ L โต ({out.distance})"
        assert out.ttc <= prev_ttc + 1e-9, f"ttc ต้องไม่โตเมื่อ L โต ({out.ttc})"
        prev_dist, prev_ttc = out.distance, out.ttc
    print("PASS test_predictor_L_shrinks_gap_and_ttc")


def test_build_matrix_runs_latency_comp_all():
    """latency_comp_all: |CONTROLLERS| × |LATENCY_DELAY_FRAMES| runs; โหมดเดิมไม่เปลี่ยน."""
    for cfg in (cutin_cfg, lead_cfg):
        runs = cfg.build_matrix_runs("latency_comp_all")
        assert len(runs) == len(cfg.CONTROLLERS) * len(cfg.LATENCY_DELAY_FRAMES), \
            f"count {len(runs)} != {len(cfg.CONTROLLERS)}×{len(cfg.LATENCY_DELAY_FRAMES)}"
        for r in runs:
            assert r["controller"] in cfg.CONTROLLERS, f"unexpected controller {r['controller']}"
            assert r["comp_source"] == "oracle"
            assert r["comp_L_frames"] == r["delay_frames"], "oracle → comp_L_frames == delay_frames"
            assert r["predict"] is True, "layer flag must be set"
            assert r["delay_frames"] in cfg.LATENCY_DELAY_FRAMES

        # existing modes untouched (purely additive)
        assert len(cfg.build_matrix_runs("original")) == len(cfg.CONTROLLERS)
        assert len(cfg.build_matrix_runs("latency")) == len(cfg.CONTROLLERS) * len(cfg.LATENCY_DELAY_FRAMES)
        assert len(cfg.build_matrix_runs("noise")) == len(cfg.CONTROLLERS) * len(cfg.NOISE_SIGMA_M_SWEEP)
        assert len(cfg.build_matrix_runs("latency_comp")) == \
            len(cfg.COMP_CONTROLLERS) * len(cfg.LATENCY_DELAY_FRAMES)
        assert len(cfg.build_matrix_runs("latency_mismatch")) == len(cfg.COMP_MISMATCH_L_FRAMES)
        for r in cfg.build_matrix_runs("original"):
            assert "predict" not in r and "comp_source" not in r, "old modes carry no comp/predict keys"
        for r in cfg.build_matrix_runs("latency_comp"):
            assert "predict" not in r, "latency_comp must NOT set the predictor-layer flag"
    print("PASS test_build_matrix_runs_latency_comp_all")


if __name__ == "__main__":
    test_predictor_L0_is_identity()
    test_compensated_L0_equals_base_per_tick()
    test_predictor_wrap_matches_enhanced_predictive()
    test_predictor_L_shrinks_gap_and_ttc()
    test_build_matrix_runs_latency_comp_all()
    print("\nAll latency_comp_all tests passed.")
