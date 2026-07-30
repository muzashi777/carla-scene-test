# -*- coding: utf-8 -*-
"""
Central configuration file for the Cut-out scenario on scene train108
----------------------------------------------------------------------
Scenario: a lead vehicle drives ahead of the ego in the same lane at the
same speed, occluding a stationary target vehicle ahead.  As the lead
approaches the stationary target it cuts out to the RIGHT, revealing the
target.  The test measures whether the ego's AEB detects the newly-revealed
stationary obstacle and brakes in time.

Three actors:
  ego         — cruises at constant speed; AEB controller decides braking.
  lead        — drives ahead at ego speed, then cuts out to the right.
  target      — stationary from t=0; never moves (the actual collision object).

Conflict definition: an unbraked ego would collide with the STATIONARY TARGET
(not the lead that cuts out).  Conflict is computed before simulation using
only the ego-to-target kinematics (see core/conflict.py cutout_train108_is_conflict).

Scene:  train108   (load manually in CARLA before running)
----------------------------------------------------------------------
Port of config/scenario_cutout.py (train000) to train108.
Only map, spawn points, and z values differ from the original.
Behaviour is identical: scenario logic reuses core/scenario_cutout.py +
core/runner_cutout.py unchanged (config-swap reuse path).
"""
import os

# ── CARLA Connection ─────────────────────────────────────────────
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# ── Simulation Parameters ─────────────────────────────────────────────────
FIXED_DT  = 0.05
MAX_TICKS = 400
LOG_EVERY = 10
SETTLE_TICKS = 20
STOP_KMH  = 0.6
GRAVITY   = 9.81

# ── Scene-name check (Section 5b) ─────────────────────────────────
EXPECTED_SCENE = "train108"

# ── Spectator camera for train108 ─────────────────────────────────
SPECTATOR_TF = None  # [TO BE TUNED in CARLA]

# ── Vehicle Positions (world coordinates, train108 scene) ─────────────────
# Ego yaw=-90.39°; forward ≈ (cos(-90.39°), sin(-90.39°)) ≈ (-0.007, -1.000)
# i.e. ego travels roughly in the −y direction.
# ego→target: Δy ≈ 81.5 m, Δx ≈ 0.45 m — parked car is straight ahead. ✓
# z values are fixed from measured/provided coordinates — do NOT use cast_ray()
# or ground-projection (3DGS mesh returns unstable z; see TECHNICAL_DOC.md Rev 2026-07-22).
EGO_SPAWN = dict(x=40.10, y=-116.54, z=10.70, yaw=-90.39)

# Target vehicle: stationary from t=0; same heading as ego.
# The lead vehicle will start between ego and target and later cut out to the right.
TARGET_SPAWN = dict(x=39.65, y=-198.00, z=12.14, yaw=-90.14,
                    model="vehicle.ue4.audi.tt")

# Lead vehicle (the "cut-out" actor): spawned at runtime in the runner by computing
#   lead = EGO_SPAWN + headway_d * forward_vector(EGO_SPAWN.yaw)
# so it starts directly ahead of ego in the same lane at headway_d metres.
# Only z, yaw, and model are specified here; x/y are derived at run time.
# z matches ego spawn (terrain is ~10.7 m here); [TO BE TUNED in CARLA].
LEAD_SPAWN = dict(z=10.70, yaw=-90.39, model="vehicle.ue4.audi.tt")  # z [TO BE TUNED]

# ── Cut-out physics-steering parameters (all values TO BE TUNED in CARLA) ─────
# The lead is driven entirely under CARLA's vehicle physics (VehicleControl).
# No set_transform / set_target_velocity is used during the manoeuvre.
#
# Trigger (TTC-based):
# The lane-change fires when TTC(lead→target) ≤ CUTOUT_TRIGGER_TTC, giving the
# lead a consistent time budget to clear regardless of speed.
# Per-case trigger_d = max(CUTOUT_TRIGGER_TTC × ego_ms, reveal_ttc-based formula).
# When the floor clips, actual_reveal_ttc > matrix value — noted in feasibility table.
CUTOUT_TRIGGER_TTC   = 1.9   # s — min lead TTC to target at trigger [TO BE TUNED in CARLA]
CUTOUT_TRIGGER_D     = 12.0  # m — legacy single-case fallback (overridden by matrix formula)
#
# Lateral target (when to stop steering right and begin straightening):
CUTOUT_LANE_WIDTH    = 1.5   # m — lateral displacement (in lead's right-frame) that marks
                              #     "successfully in the right lane" [TO BE TUNED]
#
# Heading offset during steer-right phase:
CUTOUT_HEADING_DEG   = -35.0  # degrees — target yaw offset while steering right.
                              # Positive = turns right in CARLA convention (steer > 0).
                              # ⚠ If the lead turns LEFT, set this to -30.0. [TO BE TUNED]
#
# Steering P-controller:
CUTOUT_STEER_K       = 0.05  # steer command per degree of heading error [TO BE TUNED]
                              # Too low → sluggish / doesn't reach target lane.
                              # Too high → oscillates / twitches.
CUTOUT_STEER_MAX     = 0.7   # max |steer| sent to CARLA (0–1) [TO BE TUNED]
                              # 0.4 ≈ moderate lane-change arc; increase for sharper cut-out.
#
# Straighten-complete threshold:
CUTOUT_SETTLE_DEG    = 5.0   # |heading error| (°) below which the lead is considered straight [TO BE TUNED]
#
# Post-manoeuvre behaviour:
CUTOUT_AFTER_STOP    = True  # False = keep cruising in right lane; True = decelerate to stop
#
# Longitudinal speed controller (used in all phases):
LEAD_SPEED_K            = 1.0   # throttle/brake per m/s speed error (P-gain) [TO BE TUNED]
                                 # Too low → drifts from target speed.
                                 # Too high → oscillates throttle/brake.
LEAD_SPEED_MAX_THROTTLE = 0.6   # max throttle command (0–1) [TO BE TUNED]

# ── Lead headway: fixed THW + absolute floor (not swept) ─────────────────────
# The ego↔lead following distance is a single constant, NOT a matrix axis.
# headway_d = max(FIXED_HEADWAY_THW × ego_ms, MIN_HEADWAY_M) per case.
# MIN_HEADWAY_M prevents unrealistically small gaps at low speed.
# Difficulty is controlled instead by REVEAL_TTC (see matrix section below).
#
# MIN_HEADWAY_M in time must remain BELOW the minimum reveal_ttc so the lead
# stays between ego and target at trigger (occlusion geometry valid):
#   At 20 km/h: MIN_HEADWAY_M/ego_ms = 5.0/5.556 = 0.9 s < 1.0 s (min reveal_ttc) ✓
FIXED_HEADWAY_THW = 0.7   # s — fixed following distance [TO BE TUNED in CARLA]
MIN_HEADWAY_M     = 5.0   # m — absolute headway floor; clamps low-speed THW-derived gap
                           # [TO BE TUNED in CARLA]

# ── Spawn geometry safety margins ────────────────────────────────────────────
# SPAWN_CLEARANCE_M: minimum surface gap enforced between ego and lead at spawn.
#   If headway_d < 2×ego_half_len + SPAWN_CLEARANCE_M, headway_d is clamped up.
#   At 20 km/h the nominal headway (0.8×5.556=4.444 m) overlaps the bounding boxes
#   (combined ≈4.5 m); 0.5 m clearance yields a clamped headway of ~5.0 m.
SPAWN_CLEARANCE_M    = 2.5  # m

# MIN_TRIGGER_SURF_GAP_M: minimum surface gap between lead and target at the
#   cut-out trigger point, required for the physics-based lane-change to succeed.
#   If the post-clamp trigger gap falls below this, the run is marked SCENARIO_INFEASIBLE
#   (kept in the CSV for auditing; filtered out in conflict-rate analysis).
#   At 20 km/h + reveal_ttc=1.0 the gap is only 0.556 m (0.10 s); not viable.
MIN_TRIGGER_SURF_GAP_M = 1.0  # m

# CUTOUT_STOP_MAX_M: hard-stop the lead this many metres past the trigger point
# to prevent it entering the baked obstacle zone on train108.
CUTOUT_STOP_MAX_M = 18.0    # m from trigger point; None = unlimited

# CUTOUT_SAFETY_BACKSTOP_M: after the lane is cleared (lateral_offset ≥
# CUTOUT_LANE_WIDTH + CUTOUT_CLEAR_MARGIN_M), if the lead's center-to-center distance
# to the target is below this value it brakes to a stop. This prevents the lead from
# hitting the target AFTER it has already cleared the ego lane.
# *** ONLY fires after lane-cleared — never in-lane (see CUTOUT_CLEAR_MARGIN_M). ***
# [TO BE TUNED in CARLA]
CUTOUT_SAFETY_BACKSTOP_M = 5.0  # m — center-to-center threshold (≈ 0.5 m surface gap)

# CUTOUT_CLEAR_MARGIN_M: additional margin beyond CUTOUT_LANE_WIDTH required before any
# stop mechanism (backstop, CUTOUT_STOP_MAX_M cap, CUTOUT_AFTER_STOP) may fire.
# "Lane cleared" = lateral_offset ≥ CUTOUT_LANE_WIDTH + CUTOUT_CLEAR_MARGIN_M.
# 0.0 = exact threshold; increase if you want the lead fully out before it can stop.
# [TO BE TUNED in CARLA — start at 0.0]
CUTOUT_CLEAR_MARGIN_M    = 10.0  # m

# ── Target reveal-TTC: primary matrix variable ────────────────────────────────
# reveal_ttc = surface gap / ego_ms at the instant the lead cut-out triggers.
# Per case: CUTOUT_TRIGGER_D = (reveal_ttc - FIXED_HEADWAY_THW) × ego_ms + GAP_OFFSET
# All values produce valid (positive) trigger distances for every ego speed.
# Hardest case: reveal_ttc=1.0, 20 km/h → cutout_trigger_d ≈ 5.6 m (lead-to-target
# surface gap ≈ 1.1 m — nearly touching; tune CARLA physics before running this case).
REVEAL_TTC = [1.0, 1.5, 2.0, 2.5, 3.0]   # s  [range limited by scene geometry]

# ── Cameras ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=2.8, y=0.0, z=0.8, pitch=0)              # front camera: bonnet/windshield-base area, confirmed live in CARLA
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)
CAM_FOV_DEG = 90.0

# ── YOLO ──────────────────────────────────────────────────────────
YOLO_MODEL  = "yolov8n.pt"
YOLO_DEVICE = "cpu"
CONF_THRESH = 0.45
IOU_THRESH  = 0.45
TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 9, 11]
VEHICLE_CLS = (2, 3, 5, 7)
CLASS_COLORS = {0:(0,255,0),1:(255,165,0),2:(0,0,255),3:(255,0,255),
                5:(0,165,255),7:(128,0,128),9:(0,255,255),11:(255,255,0)}

LANE_LEFT  = 520
LANE_RIGHT = 760
MIN_BOX_H  = 90

# ── Detection gate source ──
# Tracks the STATIONARY TARGET (not the lead).  Ground-truth is used for the
# AEB decision; the lead is tracked only by the scenario logic, not the controller.
DETECTION_SOURCE = "groundtruth"

INPATH_HALF_WIDTH = 1.8
INPATH_MAX_RANGE  = 100.0   # must cover the full ego-to-target distance (~81.5 m)

# ── Occlusion gate (cut-out only) ────────────────────────────────────────────
# When True, the stationary target is treated as undetected while the lead
# vehicle blocks the ego→target sight line.  Ground-truth range/TTC are still
# used for the braking decision once the target becomes visible, keeping full
# run-to-run repeatability.
#
# The gate opens (target becomes detected) when EITHER:
#   (a) the lead has moved at least OCCLUSION_LAT_CLEAR metres laterally from
#       the target's position in the ego frame  (lead has cut out far enough), OR
#   (b) the lead is no longer between ego and target longitudinally
#       (lead_lon >= target_lon - OCCLUSION_LON_MARGIN).
#
OCCLUSION_GATE      = True   # set False to restore the old always-detected behaviour
OCCLUSION_LAT_CLEAR = 1.5    # m — lateral clearance before target is considered
                              # visible.  Approx. half the lead vehicle width + margin.
                              # [TO BE TUNED in CARLA]
OCCLUSION_LON_MARGIN = 2.0   # m — lead is no longer "in front of" target once
                              # it is within this distance behind target_lon.
                              # [TO BE TUNED in CARLA]

INPATH_PREDICT   = False   # target is already directly ahead; no predictive corridor needed
INPATH_LOOKAHEAD = 0.0

# ── Camera frame synchronisation ──
FRAME_SYNC = True

# ── Surface-to-surface gap (ego front ↔ target rear) ──
AUTO_GAP_OFFSET = True
GAP_OFFSET = 4.5           # fallback / pre-sim conflict-check approximation (m)

# ── Braking model ──
BRAKE_MODEL = "kinematic"

# ── Road friction μ ──
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  Single case for run_single_cutout_train108.py
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh = 50.0,
    mu            = MU_WET,
    reveal_ttc    = 2.0,     # s — target TTC at cut-out trigger moment
    # headway_d and cutout_trigger_d are derived from reveal_ttc in the runner
)
SINGLE_CONTROLLER  = "proposed_enhanced"
SINGLE_DELAY_FRAMES = 0
SHOW_WINDOW = True

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX for run_matrix_cutout_train108.py
#  5 ego_speed × 5 reveal_ttc × 2 μ = 50 cases/controller
#
#  Primary axis: reveal_ttc (intended ego-to-target TTC at trigger).
#  headway_d = max(FIXED_HEADWAY_THW × ego_ms, MIN_HEADWAY_M)  (not swept; derived per case).
#  cutout_trigger_d = max(CUTOUT_TRIGGER_TTC × ego_ms,
#                         reveal_ttc × ego_ms + GAP_OFFSET − headway_d)
#  The TTC floor (CUTOUT_TRIGGER_TTC) ensures the lead always has enough time to
#  complete the arc. Cells where the floor clips have actual_reveal_ttc > matrix value
#  (scenario is easier than intended but safe). See tools/check_cutout_spawn.py for the
#  per-cell feasibility table. No cells are SCENARIO_INFEASIBLE with these parameters.
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh = [20.0, 30.0, 40.0, 50.0, 60.0],
    reveal_ttc    = REVEAL_TTC,
    mu            = [MU_DRY, MU_WET],
)

CONTROLLERS = ["baseline", "proposed", "proposed_enhanced"]

# ── Sweep values for degradation modes ────────────────────────────
LATENCY_DELAY_FRAMES  = [0, 4, 8, 16]
NOISE_SIGMA_M_SWEEP   = [0.0, 0.5, 1.0, 2.0]
NOISE_SIGMA_VR_SWEEP  = [0.0, 0.2, 0.5, 1.0]
DROPOUT_P_SWEEP       = [0.0, 0.05, 0.1, 0.2]

COMP_CONTROLLERS      = ["proposed_enhanced", "enhanced_predictive",
                         "enhanced_inflation", "proposed"]
COMP_MISMATCH_CTRL    = "enhanced_predictive"
COMP_MISMATCH_DELAY   = 8
COMP_MISMATCH_L_FRAMES = [4, 8, 12, 16]
COMP_R_SAFE           = 0.0


def build_matrix_runs(mode="original"):
    """Return MATRIX_RUNS list for the given test mode."""
    if mode == "latency":
        return [
            dict(label=f"{c}_d{d}f", controller=c, delay_frames=d,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in CONTROLLERS for d in LATENCY_DELAY_FRAMES
        ]
    if mode == "noise":
        return [
            dict(label=f"{c}_nm{sigma_m:.1f}", controller=c, delay_frames=0,
                 noise_sigma_m=sigma_m, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in CONTROLLERS for sigma_m in NOISE_SIGMA_M_SWEEP
        ]
    if mode == "latency_comp":
        return [
            dict(label=f"{c}_d{d}f", controller=c, delay_frames=d,
                 comp_source="oracle", comp_L_frames=d,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in COMP_CONTROLLERS for d in LATENCY_DELAY_FRAMES
        ]
    if mode == "latency_mismatch":
        d = COMP_MISMATCH_DELAY
        return [
            dict(label=f"{COMP_MISMATCH_CTRL}_d{d}f_L{cl}f", controller=COMP_MISMATCH_CTRL,
                 delay_frames=d, comp_source="mismatched", comp_L_frames=cl,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for cl in COMP_MISMATCH_L_FRAMES
        ]
    if mode == "latency_comp_all":
        return [
            dict(label=f"{c}_pred_d{d}f", controller=c, delay_frames=d,
                 comp_source="oracle", comp_L_frames=d, predict=True,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in CONTROLLERS for d in LATENCY_DELAY_FRAMES
        ]
    # "original" (default)
    return [
        dict(label=c, controller=c, delay_frames=0,
             noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
        for c in CONTROLLERS
    ]


MATRIX_RUNS = build_matrix_runs(os.environ.get("TEST_MODE", "original"))
RESULTS_DIR = "results"
RESULTS_PREFIX = "cutout108_matrix"    # results/cutout108_matrix_*.csv

# ══════════════════════════════════════════════════════════════════
#  Controller parameters (same values as existing scenarios)
# ══════════════════════════════════════════════════════════════════
TTC_WARN_FULL  = 1.6
TTC_BRAKE_FULL = 0.6

DYN_V0      = 40.0
DYN_MU0     = 0.85
DYN_K_SPEED = 1.2
DYN_K_MU    = 1.5
PARTIAL_BRAKE = 0.4

# proposed_enhanced on cut-out: target is stationary → v_l=0 → a_req=v_e²/(2·gap)
REQ_FULL_FRAC = 0.9
REQ_WARN_FRAC = 0.6
