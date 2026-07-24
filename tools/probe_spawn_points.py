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
  1. Casts a vertical ray (world.cast_ray) and reports the surface z.
  2. Attempts to spawn a test vehicle at surface_z + 0.5 m.
  3. Immediately destroys it if spawned.

OUTPUT: a table showing which coordinates are clear and which are blocked.

USAGE
-----
  # Load train000 in CARLA, then:
  python tools/probe_spawn_points.py

WHAT TO DO WITH THE RESULTS
----------------------------
CCRS approach_d=60 m row will show BLOCKED (surface_z suspiciously high or
spawn rejected).  Pick whichever of 58 m or 62 m shows CLEAR + reasonable
surface_z, then update APPROACH_DISTANCES in config/scenario_ccrs.py.

Cut-out lead-path rows (labelled "CUT_LEAD_PATH_60m_*") show whether the
post-cutout lead corridor at ~60 m forward/1.5 m lateral is clear.  If BLOCKED,
CUTOUT_STOP_MAX_M = 18.0 in config/scenario_cutout.py is already guarding
those cells.  If CLEAR, you may raise or remove CUTOUT_STOP_MAX_M.
"""

import math
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla module not found — activate the CARLA Python virtualenv first")

# ── Scene geometry constants ──────────────────────────────────────────────────
EGO_X, EGO_Y     = 6.42, 0.34
EGO_YAW_DEG      = -146.54
_yaw_rad          = math.radians(EGO_YAW_DEG)
FWD_X             = math.cos(_yaw_rad)   # ≈ -0.8343
FWD_Y             = math.sin(_yaw_rad)   # ≈ -0.5514
# Right vector: CARLA convention (sin(yaw), -cos(yaw))
RGT_X             = math.sin(_yaw_rad)   # ≈ -0.5514
RGT_Y             = -math.cos(_yaw_rad)  # ≈  0.8343

SPAWN_Z_OFFSET    = 0.5    # m above projected surface for vehicle centre
PROBE_Z_START     = 20.0   # m — ray cast from this height downward
VEHICLE_MODEL     = "vehicle.ue4.audi.tt"
HOST              = "localhost"
PORT              = 2000
TIMEOUT           = 10.0
SURFACE_Z_WARN    = 2.0    # m — surface above this warns of possible obstacle


def _fwd_point(d_m, lat_m=0.0):
    """World (x, y) at d_m forward + lat_m lateral from ego spawn."""
    return (
        EGO_X + d_m * FWD_X + lat_m * RGT_X,
        EGO_Y + d_m * FWD_Y + lat_m * RGT_Y,
    )


def _cast_ray(world, x, y):
    """Return surface z at (x, y) via downward ray, or None."""
    try:
        hits = world.cast_ray(
            carla.Location(x=x, y=y, z=PROBE_Z_START),
            carla.Location(x=x, y=y, z=-10.0),
        )
        return hits[0].location.z if hits else None
    except Exception as e:
        return None


def _try_spawn(world, x, y, surface_z):
    """Attempt to spawn + immediately destroy a test vehicle. Returns True if OK."""
    if surface_z is None:
        return False
    spawn_z = surface_z + SPAWN_Z_OFFSET
    bp = world.get_blueprint_library().filter(VEHICLE_MODEL)
    if not bp:
        bp = world.get_blueprint_library().filter("vehicle.*")
    tf = carla.Transform(
        carla.Location(x=x, y=y, z=spawn_z),
        carla.Rotation(yaw=EGO_YAW_DEG),
    )
    actor = world.try_spawn_actor(bp[0], tf)
    if actor:
        try:
            actor.destroy()
        except Exception:
            pass
        return True
    return False


def probe_point(world, label, x, y):
    """Probe one (x, y) coordinate. Returns a result dict."""
    surface_z = _cast_ray(world, x, y)
    spawn_ok   = _try_spawn(world, x, y, surface_z)
    z_flag     = (surface_z is not None and surface_z > SURFACE_Z_WARN)
    return dict(label=label, x=x, y=y, surface_z=surface_z,
                spawn_ok=spawn_ok, z_flag=z_flag)


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
    print(f"  SURFACE_Z_WARN threshold: z > {SURFACE_Z_WARN} m")
    print()

    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world  = client.get_world()
    print(f"  Connected to CARLA at {HOST}:{PORT}")
    print()

    points = build_probe_list()

    # Header
    hdr = f"{'Label':<40} {'x':>8} {'y':>8} {'surf_z':>8} {'spawn':>6} {'note'}"
    print(hdr)
    print("-" * len(hdr))

    clear_labels   = []
    blocked_labels = []

    for label, x, y in points:
        r = probe_point(world, label, x, y)
        sz   = f"{r['surface_z']:6.2f}" if r['surface_z'] is not None else "  None"
        ok   = "OK   " if r['spawn_ok'] else "FAIL "
        note = ""
        if r['z_flag']:
            note = f"  *** surface z={r['surface_z']:.2f} > {SURFACE_Z_WARN} — possible obstacle ***"
        if not r['spawn_ok']:
            note = note or "  BLOCKED"
            blocked_labels.append(label)
        else:
            clear_labels.append(label)
        print(f"  {label:<38} {x:8.3f} {y:8.3f} {sz} {ok}{note}")
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
    print("  NEXT STEPS:")
    print("  1. Check CCRS_d=58m and CCRS_d=62m rows.")
    print("     Whichever shows CLEAR + reasonable surf_z → set that as")
    print("     the replacement for 60.0 in config/scenario_ccrs.py APPROACH_DISTANCES.")
    print("  2. Check CUT_LEAD_PATH_60m_* rows (lat=1.5 m, right lane).")
    print("     If they are CLEAR, CUTOUT_STOP_MAX_M in config/scenario_cutout.py")
    print("     may be raised or set to None.  If BLOCKED, keep 18.0 m.")
    print("=" * 70)


if __name__ == "__main__":
    main()
