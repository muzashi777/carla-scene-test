# -*- coding: utf-8 -*-
"""
Unit tests for the cut-out occlusion gate.

Tests the pure-geometry helper `actors.sight_line_occluded` without any CARLA
connection or simulation.  All coordinates are in the ego's forward/right frame
(lon = distance ahead, lat = rightward offset, both in metres).

Scenario picture (top view, ego at origin, target directly ahead):

    ego(0,0) ── forward ──► lead(20, ~0) ── forward ──► target(40, 0)

The lead starts in the same lane as the target (lat ≈ 0) and progressively
cuts out to the RIGHT (lat increases) during the manoeuvre.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.occlusion import sight_line_occluded

LAT_CLEAR  = 1.5   # m
LON_MARGIN = 2.0   # m


# ── Blocked cases ─────────────────────────────────────────────────────────────

def test_blocked_lead_in_lane():
    """Lead directly in front of ego in the same lane as the target → blocked."""
    assert sight_line_occluded(
        lead_lon=20.0, lead_lat=0.0,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_blocked_lead_slightly_offset():
    """Lead 1 m to the right but still within lat_clear → still blocked."""
    assert sight_line_occluded(
        lead_lon=20.0, lead_lat=1.0,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_blocked_lead_just_inside_lat_threshold():
    """Lead at lat = lat_clear - ε → still blocked (strict <)."""
    eps = 0.01
    assert sight_line_occluded(
        lead_lon=20.0, lead_lat=LAT_CLEAR - eps,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


# ── Clear cases ───────────────────────────────────────────────────────────────

def test_clear_lead_moved_far_right():
    """Lead has cut out well past lat_clear → sight line open."""
    assert not sight_line_occluded(
        lead_lon=20.0, lead_lat=2.0,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_clear_lead_exactly_at_lat_threshold():
    """Lead at exactly lat_clear → gate open (not strictly <)."""
    assert not sight_line_occluded(
        lead_lon=20.0, lead_lat=LAT_CLEAR,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_clear_lead_past_target_lon():
    """Lead has driven ahead of (target_lon - lon_margin) → no longer between."""
    assert not sight_line_occluded(
        lead_lon=39.0, lead_lat=0.0,   # 39 >= 40 - 2 = 38
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_clear_lead_behind_ego():
    """Lead behind ego (lead_lon <= 0) → cannot block."""
    assert not sight_line_occluded(
        lead_lon=-5.0, lead_lat=0.0,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


def test_clear_lead_at_zero_lon():
    """Lead exactly at ego position (lon=0) → not between (0 < lead_lon required)."""
    assert not sight_line_occluded(
        lead_lon=0.0, lead_lat=0.0,
        target_lon=40.0, target_lat=0.0,
        lat_clear=LAT_CLEAR, lon_margin=LON_MARGIN
    )


# ── Transition: gate opens as lead cuts out ───────────────────────────────────

def test_gate_opens_progressively():
    """Simulate the lead moving right in 0.5 m steps; gate should open at lat_clear."""
    steps = [i * 0.5 for i in range(6)]   # 0.0, 0.5, 1.0, 1.5, 2.0, 2.5
    results = [
        sight_line_occluded(20.0, lat, 40.0, 0.0, LAT_CLEAR, LON_MARGIN)
        for lat in steps
    ]
    # blocked at 0.0, 0.5, 1.0; clear at 1.5, 2.0, 2.5
    assert results == [True, True, True, False, False, False], results


if __name__ == "__main__":
    tests = [
        test_blocked_lead_in_lane,
        test_blocked_lead_slightly_offset,
        test_blocked_lead_just_inside_lat_threshold,
        test_clear_lead_moved_far_right,
        test_clear_lead_exactly_at_lat_threshold,
        test_clear_lead_past_target_lon,
        test_clear_lead_behind_ego,
        test_clear_lead_at_zero_lon,
        test_gate_opens_progressively,
    ]
    passed = failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}  {e}")
            failed += 1
    print(f"\n{passed}/{passed+failed} passed")
    if failed:
        sys.exit(1)
