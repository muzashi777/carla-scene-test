#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for PerceptionDegrader — no CARLA required.
Run from project root: python tests/test_degrade.py
"""
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.types import Perception, EgoState
from perception.degrade import PerceptionDegrader


def _make_perc(distance=10.0, rel_speed=5.0, detected=True, lead_speed=0.0):
    ttc = distance / rel_speed if rel_speed > 1e-3 else math.inf
    return Perception(detected=detected, distance=distance, rel_speed=rel_speed,
                      ttc=ttc, lead_speed=lead_speed)


def _ego():
    return EgoState(speed_ms=10.0, speed_kmh=36.0, mu=0.85)


def test_noop():
    """All params = 0 → returned Perception equals input exactly."""
    d = PerceptionDegrader()
    d.reset()
    p = _make_perc(distance=15.0, rel_speed=3.0, detected=True)
    p_out, _ = d.apply(p, _ego())
    assert p_out.detected == p.detected
    assert p_out.distance == p.distance
    assert p_out.rel_speed == p.rel_speed
    assert p_out.ttc == p.ttc
    assert p_out.lead_speed == p.lead_speed
    print("PASS test_noop")


def test_delay_frames():
    """delay_frames=n → output is the perception from n ticks ago."""
    N = 3
    d = PerceptionDegrader(delay_frames=N)
    d.reset()
    ego = _ego()

    inputs = [_make_perc(distance=float(i)) for i in range(10)]
    outputs = [d.apply(p, ego)[0] for p in inputs]

    # For ticks 0..N-1 the buffer isn't full yet — all return inputs[0]
    for t in range(N):
        assert outputs[t].distance == inputs[0].distance, \
            f"tick {t}: expected {inputs[0].distance}, got {outputs[t].distance}"
    # From tick N onward, output lags by N ticks
    for t in range(N, len(inputs)):
        assert outputs[t].distance == inputs[t - N].distance, \
            f"tick {t}: expected {inputs[t-N].distance}, got {outputs[t].distance}"
    print("PASS test_delay_frames")


def test_noise_deterministic():
    """Same seed → identical noisy output on two independent runs."""
    def _run_with_seed(seed):
        d = PerceptionDegrader(noise_sigma_m=1.0, noise_sigma_vr=0.5, seed=seed)
        d.reset()
        results = []
        for i in range(20):
            p = _make_perc(distance=10.0, rel_speed=5.0)
            p_out, _ = d.apply(p, _ego())
            results.append((p_out.distance, p_out.rel_speed))
        return results

    r1 = _run_with_seed(42)
    r2 = _run_with_seed(42)
    assert r1 == r2, "Same seed must produce identical outputs"
    print("PASS test_noise_deterministic")


def test_noise_different_seeds():
    """Different seeds → different outputs (with overwhelming probability)."""
    def _run(seed):
        d = PerceptionDegrader(noise_sigma_m=1.0, seed=seed)
        d.reset()
        p_out, _ = d.apply(_make_perc(), _ego())
        return p_out.distance

    assert _run(0) != _run(1), "Different seeds should produce different outputs"
    print("PASS test_noise_different_seeds")


def test_dropout_freeze():
    """dropout_p=1.0, freeze mode: output is always the last valid frame."""
    d = PerceptionDegrader(dropout_p=1.0, dropout_mode="freeze", seed=0)
    d.reset()
    ego = _ego()

    # First frame: dropout but no prior valid → pass through unchanged
    p0 = _make_perc(distance=10.0, detected=True)
    out0, _ = d.apply(p0, ego)
    assert out0.distance == 10.0

    # Second frame: dropout → freeze → return first (which became _last_valid on tick 0)
    # Wait — on tick 0, dropout triggers (p=1.0), and _last_valid is None → pass through
    # But _last_valid is NOT set on dropout. So after tick 0, _last_valid is still None.
    # On tick 1: dropout triggers again, _last_valid is None → pass through again.
    p1 = _make_perc(distance=20.0, detected=True)
    out1, _ = d.apply(p1, ego)
    assert out1.distance == 20.0, "No valid frame saved yet; pass through"
    print("PASS test_dropout_freeze")


def test_dropout_freeze_with_valid():
    """dropout_mode=freeze: after a non-dropout tick, freezes on subsequent dropouts."""
    d = PerceptionDegrader(dropout_p=0.0, dropout_mode="freeze", seed=0)
    d.reset()
    ego = _ego()

    # Tick 0: no dropout (p=0) → _last_valid = p0
    p0 = _make_perc(distance=5.0, detected=True)
    d.apply(p0, ego)

    # Now switch to p=1.0 to force dropout
    d.dropout_p = 1.0
    p1 = _make_perc(distance=99.0, detected=False)
    out1, _ = d.apply(p1, ego)
    # Should return a copy of _last_valid (p0)
    assert out1.distance == 5.0
    assert out1.detected == True
    print("PASS test_dropout_freeze_with_valid")


def test_dropout_miss():
    """dropout_mode=miss: detected=False on dropout, numerics unchanged."""
    d = PerceptionDegrader(dropout_p=1.0, dropout_mode="miss", seed=0)
    d.reset()
    p = _make_perc(distance=10.0, rel_speed=5.0, detected=True)
    out, _ = d.apply(p, _ego())
    assert out.detected == False
    assert out.distance == 10.0
    assert out.rel_speed == 5.0
    print("PASS test_dropout_miss")


def test_distance_clamped():
    """Noise cannot push distance below 0."""
    d = PerceptionDegrader(noise_sigma_m=1000.0, seed=0)
    d.reset()
    for _ in range(50):
        p = _make_perc(distance=1.0)
        out, _ = d.apply(p, _ego())
        assert out.distance >= 0.0, f"distance={out.distance} went negative"
    print("PASS test_distance_clamped")


def test_ttc_recomputed():
    """After noise, ttc is recomputed from noisy distance/rel_speed."""
    d = PerceptionDegrader(noise_sigma_m=0.1, seed=7)
    d.reset()
    p = _make_perc(distance=10.0, rel_speed=2.0)
    out, _ = d.apply(p, _ego())
    if out.rel_speed > 1e-3:
        expected_ttc = out.distance / out.rel_speed
        assert abs(out.ttc - expected_ttc) < 1e-9
    else:
        assert math.isinf(out.ttc)
    print("PASS test_ttc_recomputed")


def test_ego_passthrough():
    """EgoState is never modified."""
    d = PerceptionDegrader(noise_sigma_m=1.0, noise_sigma_vr=1.0, dropout_p=0.5, seed=0)
    d.reset()
    ego = _ego()
    for _ in range(20):
        _, ego_out = d.apply(_make_perc(), ego)
    assert ego_out is ego
    assert ego.speed_ms == 10.0
    print("PASS test_ego_passthrough")


if __name__ == "__main__":
    test_noop()
    test_delay_frames()
    test_noise_deterministic()
    test_noise_different_seeds()
    test_dropout_freeze()
    test_dropout_freeze_with_valid()
    test_dropout_miss()
    test_distance_clamped()
    test_ttc_recomputed()
    test_ego_passthrough()
    print("\nAll tests passed.")
