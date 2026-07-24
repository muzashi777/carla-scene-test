#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Probe spawn-point validity on the train000 3DGS scene.

PURPOSE
-------
Before running either matrix (cut-out or CCRS) on train000, run this script
with CARLA live to confirm that all actor spawn coordinates land on clear road
and not on baked scene obstacles.

For each candidate point the script:
  1. Attempts to spawn a test vehicle using spawn_fixed_road_z (same path as
     the runners — no cast_ray, fixed road z from config).
  2. Immediately destroys it if spawned.

"CLEAR in probe" == "spawns OK in runner" — both use spawn_fixed_road_z.

OUTPUT: a table showing which coordinates are clear (OK) and which are blocked.

USAGE
-----
  # Load train000 in CARLA, then:
  python tools/probe_spawn_points.py

WHAT TO DO WITH THE RESULTS
----------------------------
CCRS approach_d=60 m row shows BLOCKED(overlap) — the baked static vehicle at
that position causes CARLA to reject the spawn.  62 m and other approach_d
values show OK on clear road.

Cut-out lead-path rows (labelled "CUT_LEAD_PATH_60m_*") show whether the
post-cutout lead corridor at ~60 m forward/1.5 m lateral is clear.  If BLOCKED,
CUTOUT_STOP_MAX_M = 18.0 in config/scenario_cutout.py guards those cells.
"""

import math
import os as _os
import sys
import time

# Allow running from any directory — add project root to path.
sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))

try:
    import carla
except ImportError:
    sys.exit("carla module not found — activate the CARLA Python virtualenv first")

from core.actors import spawn_fixed_road_z

# ── Scene geometry constants ──────────────────────────────────────────────────
EGO_X, EGO_Y     = 6.42, 0.34
EGO_YAW_DEG      = -146.54
_yaw_rad          = math.radians(EGO_YAW_DEG)
FWD_X             = math.cos(_yaw_rad)   # ≈ -0.8343
FWD_Y             = math.sin(_yaw_rad)   # ≈ -0.5514
# Right vector: CARLA convention (sin(yaw), -cos(yaw))
RGT_X             = math.sin(_yaw_rad)   # ≈ -0.5514
RGT_Y             = -math.cos(_yaw_rad)  # ≈  0.8343

# Fixed road z — empirically measured from train000 (old probe: −1.94 to −1.97,
# 3 cm spread across all car positions; matches SPAWN_ROAD_Z in both scenario configs).
ROAD_Z        = -1.95
VEHICLE_MODEL = "vehicle.ue4.audi.tt"
HOST          = "localhost"
PORT          = 2000
TIMEOUT       = 10.0


def _fwd_point(d_m, lat_m=0.0):
    """World (x, y) at d_m forward + lat_m lateral from ego spawn."""
    return (
        EGO_X + d_m * FWD_X + lat_m * RGT_X,
        EGO_Y + d_m * FWD_Y + lat_m * RGT_Y,
    )


def probe_point(world, label, x, y):
    """Probe one (x, y) coordinate using the same spawn_fixed_road_z path as the runners.

    'CLEAR in probe' == 'spawns OK in runner': both call spawn_fixed_road_z with
    the same ROAD_Z.  No cast_ray, no surface-z threshold — only try_spawn_actor
    overlap is tested.
    """
    actor, status = spawn_fixed_road_z(
        world, x=x, y=y, road_z=ROAD_Z, yaw=EGO_YAW_DEG,
        label=f"PROBE {label}",
        model=VEHICLE_MODEL,
    )
    spawn_ok = actor is not None
    if actor is not None:
        try:
            actor.destroy()
        except Exception:
            pass
    return dict(label=label, x=x, y=y, spawn_ok=spawn_ok, status=status)


def build_probe_list():
    """Return list of (label, x, y) tuples to probe."""
    points = []

    # ── CCRS approach_d candidates ──
    for d in [30.0, 40.0, 50.0, 58.0, 60.0, 62.0, 70.0]:
        x, y = _fwd_point(d)
        points.append((f"CCRS_d={d:.0f}m", x, y))

    # ── Cut-out fixed actors ──
    points.append(("CUTOUT_EGO",    EGO_X, EGO_Y))
    points.append(("CUTOUT_TARGET", -34.0, -26.28))

    # ── Cut-out lead spawn positions (headway_d per speed) ──
    speeds_kmh = [20.0, 30.0, 40.0, 50.0, 60.0]
    for spd in speeds_kmh:
        ego_ms  = spd / 3.6
        hw_d    = max(0.8 * ego_ms, 5.0)   # after SPAWN_CLEARANCE clamp at 20 km/h
        x, y = _fwd_point(hw_d)
        points.append((f"CUTOUT_LEAD_{spd:.0f}kmh", x, y))

    # ── Post-cutout lead path at the 60 m obstacle zone ──
    # The lead is in the right lane (1.5 m lateral) when passing through 60 m forward.
    # Probe a sweep around that position to understand clearance.
    for d_fwd in [55.0, 57.0, 59.0, 60.0, 61.0, 63.0]:
        x, y = _fwd_point(d_fwd, lat_m=1.5)
        points.append((f"CUT_LEAD_PATH_60m_{d_fwd:.0f}m_fwd_1.5m_lat", x, y))

    return points


def main():
    print("=" * 70)
    print("  probe_spawn_points.py — train000 spawn coordinate survey")
    print("=" * 70)
    print(f"  EGO_SPAWN: ({EGO_X}, {EGO_Y})  yaw={EGO_YAW_DEG}°")
    print(f"  Forward:   ({FWD_X:.4f}, {FWD_Y:.4f})")
    print(f"  ROAD_Z (fixed, no cast_ray): {ROAD_Z} m")
    print(f"  Spawn z = ROAD_Z + 1.0 = {ROAD_Z + 1.0} m for all points")
    print(f"  Obstacle gate: try_spawn_actor overlap only")
    print()

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world  = client.get_world()
    print(f"  Connected to CARLA at {HOST}:{PORT}")
    print()

    points = build_probe_list()

    # Header
    hdr = f"{'Label':<40} {'x':>8} {'y':>8} {'status':>20}"
    print(hdr)
    print("-" * len(hdr))

    clear_labels   = []
    blocked_labels = []

    for label, x, y in points:
        r = probe_point(world, label, x, y)
        ok_str = "OK" if r['spawn_ok'] else r['status']
        if not r['spawn_ok']:
            blocked_labels.append(label)
        else:
            clear_labels.append(label)
        print(f"  {label:<38} {x:8.3f} {y:8.3f}   {ok_str}")
        time.sleep(0.05)   # small pause to let CARLA settle between destroy/spawn

    print()
    print("=" * 70)
    print(f"  CLEAR : {len(clear_labels)}")
    print(f"  BLOCKED: {len(blocked_labels)}")
    if blocked_labels:
        print()
        print("  Blocked coordinates:")
        for lbl in blocked_labels:
            print(f"    - {lbl}")
    print()
    print("  EXPECTED RESULTS:")
    print("  CCRS_d=60m  → BLOCKED(overlap) — baked car at that position")
    print("  CCRS_d=30/40/50/62/70m → OK (clear road)")
    print("  CUTOUT_EGO / CUTOUT_TARGET / CUTOUT_LEAD_* → OK (clear road)")
    print("  CUT_LEAD_PATH_60m at lat=1.5 m → BLOCKED (obstacle zone)")
    print("    → CUTOUT_STOP_MAX_M=18.0 in config/scenario_cutout.py guards this")
    print("=" * 70)


if __name__ == "__main__":
    main()
