# -*- coding: utf-8 -*-
"""
Kinematic conflict-case determination — no CARLA simulation required.

Definition
----------
A case is a **conflict case** if ego, applying NO braking at all (constant cruise
speed throughout the scenario), would collide with the target obstacle within the
simulation window (MAX_TICKS × FIXED_DT seconds).

This definition is:
  - Computed ONCE per (case, scenario-params) tuple, independent of which controller runs.
  - Deterministic from kinematics only — no actor spawn needed.
  - Identical across all three controllers (baseline, proposed, proposed_enhanced).
  - Not post-hoc (does not depend on any controller's output).

For both scenarios with the current MATRIX parameters the functions return True for
every matrix case (derivations below).  The full formula is retained so that it
returns False correctly if parameters change (e.g. very large THW, very long MAX_TICKS
that ego never traverses before the window closes, etc.).

Run tools/check_conflict.py for per-case conflict counts (no CARLA required).

Public API
----------
  lead_brake_is_conflict(case, cfg) -> bool   # case has 'ego_speed_kmh', 'headway_d'
  cutin_is_conflict(case, cfg)      -> bool   # case has 'ego_speed_kmh', 'trigger_d', 'dart_speed_kmh'

Both are thin wrappers that call the documented formula functions below.
"""
import math


# ── Lead-brake conflict formula ─────────────────────────────────────────────────

def _kinematic_conflict_lead_brake(
    ego_speed_kmh: float,
    headway_d_m: float,
    lead_decel: float,
    lead_brake_after_m: float,
    max_ticks: int,
    fixed_dt: float,
) -> bool:
    """
    Determines whether an unbraked ego (constant speed v_e) collides with the lead
    vehicle within max_ticks × fixed_dt seconds.

    Scenario: both vehicles start at v_e.  Lead travels lead_brake_after_m before
    braking at lead_decel m/s² to a stop.  headway_d_m is the initial centre-to-centre
    separation (metres, positive = lead is ahead of ego).

    Derivation (all in SI units):
      Phase 1 — both at v_e, duration t1 = lead_brake_after_m / v_e.
        Gap unchanged because both vehicles move the same distance.
      Phase 2 — lead decelerates at 'lead_decel', ego constant v_e.
        Relative velocity at braking onset = v_e - v_e = 0, then grows at lead_decel.
        Relative displacement gained by ego = ½ lead_decel × t₂²
        Lead stops after t₂ = v_e / lead_decel.
        Gap at lead stop = headway_d_m − v_e² / (2 × lead_decel).
          If this is ≤ 0 → collision already in Phase 2 → conflict.
      Phase 3 — lead stationary, ego at v_e.
        Remaining surface gap (approximate, gap_offset ignored here — bounding-box
        corrections are small and only delay collision by a fraction of a second).
        Time to collision ≈ gap_at_stop / v_e.
      Total time = t1 + t₂ + t₃ ≤ max_ticks × fixed_dt → conflict.

    For LEAD_DECEL=4.0 and the current MATRIX (THW ∈ {1.0,1.5,2.0,2.5,3.0} s,
    v_e ∈ {20,30,40,50,60} km/h) total collision time is always well below
    MAX_TICKS×FIXED_DT=20 s (worst case: 20 km/h + THW=3.0s → t_total ≈ 5.1 s).
    → All 50 lead-brake matrix cases are conflict cases (verified by tools/check_conflict.py).
    """
    v_e = ego_speed_kmh / 3.6
    if v_e < 1e-3:
        return False

    t1 = lead_brake_after_m / v_e          # Phase 1 duration (both at v_e)
    t2 = v_e / lead_decel                  # Phase 2 duration (lead decelerates to stop)
    gap_at_stop = headway_d_m - (v_e ** 2) / (2.0 * lead_decel)

    if gap_at_stop <= 0.0:
        # Collision occurs before lead stops — definitely within window.
        return True

    # Phase 3: ego closes remaining gap at v_e (bounding-box offset ignored here;
    # it delays collision by gap_offset/v_e ≈ 0.5 s at worst, still within 20 s window).
    t3 = gap_at_stop / v_e
    total_time = t1 + t2 + t3
    return total_time <= max_ticks * fixed_dt


# ── Cut-in conflict formula ──────────────────────────────────────────────────────

def _kinematic_conflict_cutin(
    ego_speed_kmh: float,
    trigger_d_m: float,
    dart_speed_kmh: float,
    dart_spawn_x: float,
    dart_spawn_y: float,
    dart_stop_x: float,
    ego_spawn_x: float,
    max_ticks: int,
    fixed_dt: float,
) -> bool:
    """
    Determines whether an unbraked ego collides with the dart that blocks its lane.

    Key geometry (all positions in CARLA world coordinates; ego travels in −Y direction):
      dart_x_span = |dart_spawn_x − dart_stop_x|   lateral distance dart traverses
      x_offset    = |dart_spawn_x − ego_spawn_x|   initial lateral separation
      At trigger, ego is at y = dart_spawn_y + y_gap  where
        y_gap = sqrt(trigger_d_m² − x_offset²)       (ego still behind dart in Y)

    Timeline after trigger fires:
      t_dart_to_block = dart_x_span / dart_speed_ms  dart enters ego lane fully
      t_ego_to_dart_y = y_gap / v_e                  ego reaches dart's Y position

    After dart_stop_x ≈ ego_spawn_x, dart fully blocks ego's path.  Once dart is in
    lane, ego (unbraked) will always collide; conflict iff this happens within window.

    Conservative conflict time = max(t_dart_to_block, t_ego_to_dart_y).

    For trigger_d ∈ {20,25,30,35,40} m, ego_speed ∈ {20,30,40,50,60} km/h, dart_speed=20 km/h:
      t_dart_to_block ≈ 1.35 s (dart traverses 7.5 m at 5.56 m/s, independent of ego).
      t_ego_to_dart_y ranges from 0.8 s to 7.1 s (worst case: 20 km/h + trigger_d=40 m).
      Both are well below MAX_TICKS×FIXED_DT=20 s.
    → All 50 cut-in matrix cases are conflict cases (verified by tools/check_conflict.py).
    """
    v_e = ego_speed_kmh / 3.6
    dart_speed_ms = dart_speed_kmh / 3.6
    if v_e < 1e-3 or dart_speed_ms < 1e-3:
        return False

    x_offset = abs(dart_spawn_x - ego_spawn_x)
    if trigger_d_m < x_offset:
        # Trigger distance smaller than lateral offset — trigger never fires from behind.
        return False

    y_gap_at_trigger = math.sqrt(max(0.0, trigger_d_m ** 2 - x_offset ** 2))
    dart_x_span = abs(dart_spawn_x - dart_stop_x)

    t_dart_to_block = dart_x_span / dart_speed_ms
    t_ego_to_dart_y = y_gap_at_trigger / v_e

    # Conservative: conflict event (dart in lane AND ego at dart's Y) by max of both times.
    t_conflict = max(t_dart_to_block, t_ego_to_dart_y)
    return t_conflict <= max_ticks * fixed_dt


# ── Public wrappers (accept cfg module) ─────────────────────────────────────────

def lead_brake_is_conflict(case: dict, cfg) -> bool:
    """
    Return True if this lead-brake matrix case is a conflict case.

    'case' must contain 'ego_speed_kmh' and 'headway_d' (metres, already
    computed from THW × v_e or fixed value in the runner before calling this).

    Uses cfg.LEAD_DECEL, cfg.LEAD_BRAKE_AFTER_M, cfg.MAX_TICKS, cfg.FIXED_DT.
    """
    return _kinematic_conflict_lead_brake(
        ego_speed_kmh=case["ego_speed_kmh"],
        headway_d_m=case["headway_d"],
        lead_decel=cfg.LEAD_DECEL,
        lead_brake_after_m=cfg.LEAD_BRAKE_AFTER_M,
        max_ticks=cfg.MAX_TICKS,
        fixed_dt=cfg.FIXED_DT,
    )


# ── Stationary-target conflict formula (CCRs and Cut-out) ──────────────────────

def _kinematic_conflict_stationary_target(
    ego_speed_kmh: float,
    ego_spawn_x: float,
    ego_spawn_y: float,
    target_spawn_x: float,
    target_spawn_y: float,
    gap_offset_m: float,
    max_ticks: int,
    fixed_dt: float,
) -> bool:
    """
    Determines whether an unbraked ego (constant speed v_e) collides with a
    STATIONARY target within max_ticks × fixed_dt seconds.

    The target never moves.  Time to collision = surface_gap / v_e, where:
      surface_gap = max(0, centre_to_centre_distance − gap_offset_m)

    gap_offset_m is an approximation of the vehicle extents sum (ego half-length +
    target half-length) used only for the pre-simulation check; the runner computes
    it exactly from bounding boxes at runtime.  The approximation is conservative
    and only delays t_conflict by gap_offset_m / v_e (≤ ~0.5 s at 60 km/h),
    well within the 20 s window.

    Used by both CCRs (always stationary target) and Cut-out (conflict measured
    against the revealed stationary target, not the cutting-out lead).
    """
    v_e = ego_speed_kmh / 3.6
    if v_e < 1e-3:
        return False
    dist = math.hypot(ego_spawn_x - target_spawn_x, ego_spawn_y - target_spawn_y)
    surface_gap = max(0.0, dist - gap_offset_m)
    if surface_gap <= 0.0:
        return True   # already overlapping — definitely a conflict
    t_conflict = surface_gap / v_e
    return t_conflict <= max_ticks * fixed_dt


def ccrs_is_conflict(case: dict, cfg) -> bool:
    """
    Return True if this CCRs matrix case is a conflict case.

    Target position is taken from 'case' if 'target_x'/'target_y' are present
    (set by run_matrix_ccrs.py for the approach-distance axis); otherwise falls
    back to cfg.TARGET_SPAWN (single-case / legacy behaviour).
    'case' must contain 'ego_speed_kmh'.
    Uses cfg.EGO_SPAWN, cfg.GAP_OFFSET, cfg.MAX_TICKS, cfg.FIXED_DT.
    """
    return _kinematic_conflict_stationary_target(
        ego_speed_kmh=case["ego_speed_kmh"],
        ego_spawn_x=cfg.EGO_SPAWN["x"],
        ego_spawn_y=cfg.EGO_SPAWN["y"],
        target_spawn_x=case.get("target_x", cfg.TARGET_SPAWN["x"]),
        target_spawn_y=case.get("target_y", cfg.TARGET_SPAWN["y"]),
        gap_offset_m=cfg.GAP_OFFSET,
        max_ticks=cfg.MAX_TICKS,
        fixed_dt=cfg.FIXED_DT,
    )


def cutout_is_conflict(case: dict, cfg) -> bool:
    """
    Return True if this Cut-out matrix case is a conflict case.

    Conflict is defined against the STATIONARY revealed target (not the lead
    that cuts out — the lead is not a collision object for the AEB test).
    Ego at constant speed would collide with the stationary target within window.
    'case' must contain 'ego_speed_kmh'.
    Uses cfg.EGO_SPAWN, cfg.TARGET_SPAWN, cfg.GAP_OFFSET, cfg.MAX_TICKS, cfg.FIXED_DT.
    """
    return _kinematic_conflict_stationary_target(
        ego_speed_kmh=case["ego_speed_kmh"],
        ego_spawn_x=cfg.EGO_SPAWN["x"],
        ego_spawn_y=cfg.EGO_SPAWN["y"],
        target_spawn_x=cfg.TARGET_SPAWN["x"],
        target_spawn_y=cfg.TARGET_SPAWN["y"],
        gap_offset_m=cfg.GAP_OFFSET,
        max_ticks=cfg.MAX_TICKS,
        fixed_dt=cfg.FIXED_DT,
    )


def cutin_is_conflict(case: dict, cfg) -> bool:
    """
    Return True if this cut-in matrix case is a conflict case.

    'case' must contain 'ego_speed_kmh', 'trigger_d', 'dart_speed_kmh'.
    Reads DART_SPAWN, DART_STOP_X, EGO_SPAWN, MAX_TICKS, FIXED_DT from cfg.
    """
    return _kinematic_conflict_cutin(
        ego_speed_kmh=case["ego_speed_kmh"],
        trigger_d_m=case["trigger_d"],
        dart_speed_kmh=case["dart_speed_kmh"],
        dart_spawn_x=cfg.DART_SPAWN["x"],
        dart_spawn_y=cfg.DART_SPAWN["y"],
        dart_stop_x=cfg.DART_STOP_X,
        ego_spawn_x=cfg.EGO_SPAWN["x"],
        max_ticks=cfg.MAX_TICKS,
        fixed_dt=cfg.FIXED_DT,
    )


def junction_cutin_is_conflict(case: dict, cfg) -> bool:
    """
    Return True if this Junction Cut-in matrix case is a conflict case.

    Conflict is defined against the STATIONARY intruder at INTRUDER_STOP
    (the point where the intruder brakes to a full stop in the ego lane after
    completing its right turn).  An unbraked ego at constant speed would collide
    with that stationary blocker within MAX_TICKS × FIXED_DT seconds.

    This formula is independent of trigger_d (when the turn starts) — the
    intruder always ends up at INTRUDER_STOP regardless of trigger timing.
    All 50 matrix cases are expected to be conflicts with the current parameters.

    'case' must contain 'ego_speed_kmh'.
    Uses cfg.EGO_SPAWN, cfg.INTRUDER_STOP, cfg.GAP_OFFSET, cfg.MAX_TICKS, cfg.FIXED_DT.
    """
    return _kinematic_conflict_stationary_target(
        ego_speed_kmh=case["ego_speed_kmh"],
        ego_spawn_x=cfg.EGO_SPAWN["x"],
        ego_spawn_y=cfg.EGO_SPAWN["y"],
        target_spawn_x=cfg.INTRUDER_STOP["x"],
        target_spawn_y=cfg.INTRUDER_STOP["y"],
        gap_offset_m=cfg.GAP_OFFSET,
        max_ticks=cfg.MAX_TICKS,
        fixed_dt=cfg.FIXED_DT,
    )
