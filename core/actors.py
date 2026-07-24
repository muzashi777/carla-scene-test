# -*- coding: utf-8 -*-
"""Actor management helpers: spawn vehicles, set wheel friction μ, attach sensors, state functions"""
import math
import carla
from core.occlusion import sight_line_occluded  # noqa: F401 — re-exported for callers


def kmh_to_ms(k):
    return k / 3.6


def speed_kmh(actor):
    v = actor.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def speed_ms(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def dist2d(a, b):
    la, lb = a.get_location(), b.get_location()
    return math.hypot(la.x - lb.x, la.y - lb.y)


# ── Constants for required_decel (guard against erratic/exploding values in cut-in scenes where target appears at close range) ──
REQ_GAP_EPS  = 0.05    # m   below this distance = imminent collision/overlap → maximum emergency (request full brake)
REQ_VL_STILL = 0.5     # m/s 'along ego forward direction' speed below this = treat obstacle as stationary → d_lead=0
REQ_AREQ_CAP = 50.0    # m/s² clamp maximum value, guard against explosion (high enough above μ·g ceiling that urgency = full brake guaranteed)


def required_decel(v_e, v_l, a_l, gap):
    """Minimum deceleration ego must apply to stop before the rear of the vehicle ahead (m/s²)
    Accounts for 'distance the vehicle ahead still travels before stopping' (d_lead = v_l²/2a_l)
      v_e ≤ v_l                 → not closing in → 0
      gap ≤ REQ_GAP_EPS         → imminent collision → return REQ_AREQ_CAP (guard divide-by-zero/explosion) = request full brake
      v_l ≤ REQ_VL_STILL        → d_lead = 0 (obstacle is 'stationary/not moving forward', e.g. dart-out that cut in and stopped)
                                   → reduces to stationary-obstacle case: a_req = v_e²/(2·gap)
      a_l > 0.1                 → d_lead = v_l²/2a_l (vehicle ahead is braking)
      no braking (a_l≈0, v_l>REQ_VL_STILL) → d_lead = inf (not yet critical; vehicle ahead is pulling away)
    Always returns a value capped at REQ_AREQ_CAP (never returns inf/NaN) for stable log/urgency computation
    v_l supplied should be 'speed along ego's direction of travel' (longitudinal) — see long_speed_along
    Shared by both the controller (decision-making) and the runner (recording a_req at braking onset) so they stay consistent
    """
    v_e = max(0.0, v_e); v_l = max(0.0, v_l); a_l = max(0.0, a_l); gap = max(0.0, gap)
    if v_e <= v_l:
        return 0.0
    if gap <= REQ_GAP_EPS:               # imminent collision/overlap → maximum emergency
        return REQ_AREQ_CAP
    if v_l <= REQ_VL_STILL:
        d_lead = 0.0                     # stationary/cut-in → stationary obstacle (reduced formula v_e²/2·gap)
    elif a_l > 0.1:
        d_lead = (v_l * v_l) / (2.0 * a_l)
    else:
        d_lead = math.inf                # moving forward at constant speed, not decelerating → not yet critical
    D = gap + d_lead
    if D <= 1e-6:
        return REQ_AREQ_CAP
    return min((v_e * v_e) / (2.0 * D), REQ_AREQ_CAP)


def long_speed_along(ego, other):
    """Speed of 'other' along ego's direction of travel (longitudinal, m/s, returns only the ≥0 component)
    Used to supply v_l to required_decel: a vehicle that 'cuts in sideways' (cut-in/dart-out) has a near-zero forward component along ego
      → treated as a stationary obstacle (not a lead vehicle pulling away), preventing d_lead from exploding to inf then a_req=0
    In the lead-brake scene (lead vehicle drives in the same direction as ego) this value is the full speed, so the definition is consistent across both scenes
    Returns max(0,·): if other is approaching ego (negative forward component) → 0 (does not help increase stopping distance)
    """
    f = ego.get_transform().get_forward_vector()
    ov = other.get_velocity()
    return max(0.0, ov.x * f.x + ov.y * f.y)


def apply_kinematic_brake(ego, v_model, brake_cmd, mu, dt, g=9.81):
    """Kinematic braking step that 'caps achievable deceleration' so it never exceeds the friction ceiling a_max = μ·g
    ──────────────────────────────────────────────────────────────────
    Rationale: achievable deceleration is limited by road friction, so a ≤ μ·g must always hold
      (consistent with the kinematic model assumption throughout the system — see README section 'Physics cap on braking deceleration')
    Tracks speed via 'v_model' (not the readback from CARLA) to prevent the engine's internal physics from
      applying deceleration beyond the ceiling, which would stop the vehicle sooner than reality = overestimating collision avoidance
    Shared path by both runner.py (cut-in) and runner_lead_brake.py → all 3 controllers are clamped equally
    Returns (v_new, a_applied):
      v_new     = new speed (m/s) set for ego via set_target_velocity
      a_applied = actual deceleration after clamping (m/s²) → used to record peak_decel (always ≤ a_max)
    """
    a_max = max(0.0, mu) * g
    a_cmd = max(0.0, brake_cmd) * a_max      # brake command 0..1 → requested deceleration (within range 0..a_max)
    a_applied = min(a_cmd, a_max)            # hard clamp to prevent exceeding friction ceiling μ·g
    v_new = max(0.0, v_model - a_applied * dt)
    f = ego.get_transform().get_forward_vector()
    ego.set_target_velocity(carla.Vector3D(f.x * v_new, f.y * v_new, 0.0))
    a_real = (v_model - v_new) / dt if dt > 0 else 0.0   # may be less than a_applied when speed reaches 0
    return v_new, a_real


def grab_synced(q, frame_id, timeout=2.0):
    """Read images from the queue until image.frame matches or is newer than frame_id (prevents frame drift in sync mode)"""
    while True:
        img = q.get(timeout=timeout)
        if img.frame >= frame_id:
            return img


def inpath_hazard(ego, other, max_range, half_width, lookahead=0.0):
    """
    Check from ground-truth whether 'other' is in or is entering ego's forward path
    lookahead>0 = predictive mode enabled: use other's lateral velocity to predict lane entry
    Returns (in_path, lon, lat): lon=longitudinal distance (>0=ahead), lat=lateral offset
    """
    e = ego.get_transform()
    f = e.get_forward_vector()
    r = e.get_right_vector()
    le, lo = e.location, other.get_location()
    dx, dy = lo.x - le.x, lo.y - le.y
    lon = dx * f.x + dy * f.y
    lat = dx * r.x + dy * r.y
    ahead = (0.0 < lon <= max_range)
    cur = ahead and (abs(lat) <= half_width)        # currently in lane
    pred = False
    if lookahead > 0.0 and ahead:
        ov = other.get_velocity()
        lat_vel = ov.x * r.x + ov.y * r.y           # lateral velocity of other
        entering = (lat * lat_vel < 0.0)            # moving toward the lane centre
        lat_future = lat + lat_vel * lookahead
        pred = entering and (abs(lat_future) <= half_width)
    return (cur or pred), lon, lat


def spawn_vehicle(world, x, y, z, yaw, model="vehicle.*"):
    """Spawn one vehicle, return actor (None if failed). Replaces the old actor_spawner"""
    bp_lib = world.get_blueprint_library()
    candidates = bp_lib.filter(model)
    if not candidates:
        candidates = bp_lib.filter("vehicle.*")
    bp = candidates[0]
    tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
    return world.try_spawn_actor(bp, tf)


_SPAWN_XY_TOL = 0.1  # m — max allowed x,y drift after spawn


def spawn_ground_projected(world, x, y, z_nom, yaw, cfg, label="", model="vehicle.*"):
    """Unified ground-projected vehicle spawn used by runner_ccrs, runner_cutout, and probe_spawn_points.

    Ensures 'CLEAR in probe' == 'spawns OK in runner' — no divergent copies.

    Steps:
      1. Cast a downward ray to find the road-surface z.
      2. Hard-block if surf_z > SPAWN_SURFACE_Z_MAX (baked obstacle roof above road level).
      3. Spawn at surf_z + SPAWN_Z_OFFSET (or z_nom when no ray hit).
      4. Hard-block if CARLA overlap-rejects the spawn.
      5. Hard-block if the placed actor drifted in x,y (CARLA (0,0,0) placement on bad z).

    Returns (actor_or_None, status_str).
      status_str == "OK"  → actor is valid and in place.
      status_str starts with "BLOCKED" → actor is None, reason appended.
    Always prints one [SPAWN] log line.

    Scene note (train000): road sits at z ≈ −1.95 m.  SPAWN_SURFACE_Z_MAX in config is set
    relative to this scene (road_level + margin) so it blocks the baked obstacle roof (+0.9 m)
    while accepting road hits.  There is no SPAWN_SURFACE_Z_MIN fallback — that logic was the
    regression source (road z = −1.95 fell below the old −1.0 floor, triggering a wrong fallback).
    """
    surf_z_max = getattr(cfg, "SPAWN_SURFACE_Z_MAX", 2.0)
    z_offset   = getattr(cfg, "SPAWN_Z_OFFSET", 0.5)

    proj_z = ground_projection_z(world, x, y)

    if proj_z is None:
        spawn_z  = z_nom
        proj_str = "n/a"
        dz_str   = "n/a"
    elif proj_z > surf_z_max:
        print(f"[SPAWN] {label}  req=({x:.3f},{y:.3f},z_nom={z_nom:.3f})  "
              f"surf_z={proj_z:.3f}  dz={proj_z - z_nom:+.3f}  "
              f"blocked=Y  status=BLOCKED(obstacle_roof)  final=n/a")
        return None, (f"BLOCKED: surf_z={proj_z:.3f} > SPAWN_SURFACE_Z_MAX={surf_z_max} "
                      f"— baked obstacle roof, not road")
    else:
        spawn_z  = proj_z + z_offset
        proj_str = f"{proj_z:.3f}"
        dz_str   = f"{proj_z - z_nom:+.3f}"

    actor = spawn_vehicle(world, x=x, y=y, z=spawn_z, yaw=yaw, model=model)
    if actor is None:
        print(f"[SPAWN] {label}  req=({x:.3f},{y:.3f},z_nom={z_nom:.3f})  "
              f"surf_z={proj_str}  dz={dz_str}  "
              f"blocked=Y  status=BLOCKED(overlap)  final=n/a")
        return None, f"BLOCKED: CARLA overlap-rejected spawn at z={spawn_z:.3f} (proj_z={proj_str})"

    loc = actor.get_location()
    xy_ok = abs(loc.x - x) <= _SPAWN_XY_TOL and abs(loc.y - y) <= _SPAWN_XY_TOL
    print(f"[SPAWN] {label}  req=({x:.3f},{y:.3f},z_nom={z_nom:.3f})  "
          f"surf_z={proj_str}  dz={dz_str}  "
          f"blocked={'N' if xy_ok else 'Y'}  "
          f"status={'OK' if xy_ok else 'BLOCKED(drift)'}  "
          f"final=({loc.x:.3f},{loc.y:.3f},{loc.z:.3f})")
    if not xy_ok:
        try:
            actor.destroy()
        except Exception:
            pass
        return None, (f"BLOCKED: x,y drifted ({x:.3f},{y:.3f}) → ({loc.x:.3f},{loc.y:.3f}); "
                      f"spawn_z={spawn_z:.3f} may be underground")

    return actor, "OK"


def ground_projection_z(world, x, y, probe_z=20.0):
    """Cast a vertical ray at (x, y) and return the z of the first surface hit, or None.

    Uses world.cast_ray() which works on any collision mesh including 3DGS scenes.
    The returned z is whatever surface is highest at that (x,y) — road, baked vehicle
    roof, raised kerb, etc.  Callers should validate the result against SPAWN_SURFACE_Z_MAX
    to flag suspiciously high hits that indicate a baked scene obstacle rather than road.

    Returns None if cast_ray is unavailable (CARLA < 0.9.10) or hits nothing.
    """
    try:
        hits = world.cast_ray(
            carla.Location(x=float(x), y=float(y), z=float(probe_z)),
            carla.Location(x=float(x), y=float(y), z=-10.0),
        )
        return float(hits[0].location.z) if hits else None
    except AttributeError:
        return None   # cast_ray not available in this CARLA build
    except Exception:
        return None


def _wheel_friction_attr(wheel):
    """Find the friction attribute name on the wheel (varies by CARLA version)"""
    for name in ("tire_friction", "friction", "lateral_friction", "longitudinal_friction"):
        if hasattr(wheel, name):
            return name
    for name in dir(wheel):
        if "friction" in name.lower() and not name.startswith("_"):
            return name
    return None


def set_friction(vehicle, mu, verbose=True):
    """Set road friction μ at the wheels; auto-detect attr name by CARLA version, read back to confirm"""
    pc = vehicle.get_physics_control()
    wheels = pc.wheels
    if not wheels:
        if verbose:
            print("[FRICTION] ⚠ Vehicle has no wheel data in physics control")
        return None
    attr = _wheel_friction_attr(wheels[0])
    if attr is None:
        avail = [a for a in dir(wheels[0]) if not a.startswith("_")]
        if verbose:
            print("[FRICTION] ⚠ Friction attribute not found on wheel — μ was not set!")
            print(f"[FRICTION] Available wheel attributes: {avail}")
        return None
    for w in wheels:
        setattr(w, attr, float(mu))
    pc.wheels = wheels
    vehicle.apply_physics_control(pc)
    readback = [round(getattr(w, attr), 3) for w in vehicle.get_physics_control().wheels]
    if verbose:
        print(f"[FRICTION] Using attr '{attr}' to set μ={mu} → per-wheel readback = {readback}")
        target = round(float(mu), 3)
        if not all(abs(r - target) < 1e-3 for r in readback):
            print("[FRICTION] ⚠ Value not applied/mismatched — physics engine did not accept μ via this path")
            print("[FRICTION]   Recommend setting BRAKE_MODEL='kinematic' in config (controls μ directly)")
    return readback


def cruise(vehicle, target_ms):
    """Maintain constant speed along the forward vector (open-loop because there is no waypoint)"""
    f = vehicle.get_transform().get_forward_vector()
    vehicle.set_target_velocity(carla.Vector3D(f.x * target_ms, f.y * target_ms, 0.0))


def hold(vehicle):
    """Lock the vehicle to a standstill"""
    vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))


def attach_rgb_camera(world, parent, tf_dict, w, h, sink, fov=None):
    """Attach an RGB camera to parent and forward frames to sink (e.g. queue.put)"""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    if fov is not None:
        bp.set_attribute("fov", str(fov))
    rot = carla.Rotation(
        pitch=tf_dict.get("pitch", 0.0),
        yaw=tf_dict.get("yaw", 0.0),
        roll=tf_dict.get("roll", 0.0),
    )
    tf = carla.Transform(
        carla.Location(x=tf_dict.get("x", 0.0), y=tf_dict.get("y", 0.0), z=tf_dict.get("z", 0.0)),
        rot,
    )
    cam = world.spawn_actor(bp, tf, attach_to=parent)
    cam.listen(sink)
    return cam


def attach_collision_sensor(world, parent, on_hit):
    bp = world.get_blueprint_library().find("sensor.other.collision")
    sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent)
    sensor.listen(on_hit)
    return sensor


def set_spectator(world, tf_dict):
    """Position the CARLA spectator camera (cosmetic only — no effect on simulation results).

    tf_dict keys: x, y, z (location, metres), pitch, yaw, roll (rotation, degrees).
    Call after CarlaSession.__enter__ and before the test loop.  If tf_dict is None
    or empty the function is a no-op, so passing cfg.SPECTATOR_TF = None is safe.
    """
    if not tf_dict:
        return
    spectator = world.get_spectator()
    loc = carla.Location(
        x=float(tf_dict.get("x", 0.0)),
        y=float(tf_dict.get("y", 0.0)),
        z=float(tf_dict.get("z", 0.0)),
    )
    rot = carla.Rotation(
        pitch=float(tf_dict.get("pitch", 0.0)),
        yaw=float(tf_dict.get("yaw", 0.0)),
        roll=float(tf_dict.get("roll", 0.0)),
    )
    spectator.set_transform(carla.Transform(loc, rot))
    print(f"[SPECTATOR] Camera set to loc=({loc.x:.2f},{loc.y:.2f},{loc.z:.2f}) "
          f"yaw={rot.yaw:.1f}°")


def check_scene(world, expected_scene):
    """Verify the currently-loaded CARLA map matches the expected scene name.

    Uses a substring check: if 'expected_scene' appears anywhere in the full map
    path returned by world.get_map().name the check passes.  This handles both
    short names ('scene03_2') and full UE paths ('/Game/Maps/scene03_2').

    Raises RuntimeError immediately with a clear message if the map does not match,
    so the tester does not waste time running the wrong scene.

    If expected_scene is empty or None the check is skipped (opt-in).
    """
    if not expected_scene:
        return
    try:
        actual = world.get_map().name
    except Exception as e:
        # Scenes imported from 3DGS/UE5.5 have no OpenDRIVE road network,
        # so get_map() raises "unable to parse the OpenDRIVE XML string".
        # Skip the check rather than aborting — the scene name cannot be verified this way.
        print(f"[SCENE CHECK] SKIP — world.get_map() unavailable ({e}). "
              f"Ensure the correct scene ('{expected_scene}') is loaded in CARLA.")
        return
    if expected_scene not in actual:
        raise RuntimeError(
            f"\n[SCENE CHECK] Wrong scene loaded!\n"
            f"  Expected (substring): '{expected_scene}'\n"
            f"  Loaded map name:      '{actual}'\n"
            f"  Load the correct scene in CARLA and restart."
        )
    print(f"[SCENE CHECK] OK — '{actual}' contains '{expected_scene}'")
