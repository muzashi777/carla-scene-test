# -*- coding: utf-8 -*-
"""
Central configuration file for the Cut-out scenario on scene train000
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
only the ego-to-target kinematics (see core/conflict.py cutout_is_conflict).

Scene:  train000   (load manually in CARLA before running)
----------------------------------------------------------------------
Edit all values here only; run_single_cutout.py / run_matrix_cutout.py follow.
Structure mirrors config/scenario_lead_brake.py.
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
EXPECTED_SCENE = "train000"

# ── Spectator camera for train000 ─────────────────────────────────
SPECTATOR_TF = dict(x=5.27, y=-0.18, z=1.0, yaw=-143.44)

# ── Vehicle Positions (world coordinates, train000 scene) ─────────────────
# Ego: yaw=-146.54°; forward ≈ (-0.835, -0.550, 0) (NOT axis-aligned).
# All "ahead" / headway distances are computed along this forward vector in the runner.
EGO_SPAWN = dict(x=6.42,   y=0.34,   z=0.25, yaw=-146.54)

# Target vehicle: stationary from t=0; same heading as ego.
# The lead vehicle will start between ego and target and later cut out to the right.
TARGET_SPAWN = dict(x=-34.0, y=-26.28, z=0.25, yaw=-146.54,
                    model="vehicle.ue4.audi.tt")

# Lead vehicle (the "cut-out" actor): spawned at runtime in the runner by computing
#   lead = EGO_SPAWN + headway_d * forward_vector(EGO_SPAWN.yaw)
# so it starts directly ahead of ego in the same lane at headway_d metres.
# Only model is specified here; position is derived at run time.
LEAD_SPAWN = dict(z=0.79, yaw=-146.54, model="vehicle.ue4.audi.tt")

# ── Cut-out physics-steering parameters (all values TO BE TUNED in CARLA) ─────
# The lead is driven entirely under CARLA's vehicle physics (VehicleControl).
# No set_transform / set_target_velocity is used during the manoeuvre.
#
# Trigger:
CUTOUT_TRIGGER_D     = 12.0  # m — start lane-change when lead is this close to target [TO BE TUNED]
#
# Lateral target (when to stop steering right and begin straightening):
CUTOUT_LANE_WIDTH    = 1.5   # m — lateral displacement (in lead's right-frame) that marks
                              #     "successfully in the right lane" [TO BE TUNED]
#
# Heading offset during steer-right phase:
CUTOUT_HEADING_DEG   = 30.0  # degrees — target yaw offset while steering right.
                              # Positive = turns right in CARLA convention (steer > 0).
                              # ⚠ If the lead turns LEFT, set this to -30.0. [TO BE TUNED]
#
# Steering P-controller:
CUTOUT_STEER_K       = 0.05  # steer command per degree of heading error [TO BE TUNED]
                              # Too low → sluggish / doesn't reach target lane.
                              # Too high → oscillates / twitches.
CUTOUT_STEER_MAX     = 0.4   # max |steer| sent to CARLA (0–1) [TO BE TUNED]
                              # 0.4 ≈ moderate lane-change arc; increase for sharper cut-out.
#
# Straighten-complete threshold:
CUTOUT_SETTLE_DEG    = 5.0   # |heading error| (°) below which the lead is considered straight [TO BE TUNED]
#
# Post-manoeuvre behaviour:
CUTOUT_AFTER_STOP    = True  # False = keep cruising in right lane; True = decelerate to stop
#
# Longitudinal speed controller (used in all phases):
LEAD_SPEED_K            = 0.5   # throttle/brake per m/s speed error (P-gain) [TO BE TUNED]
                                 # Too low → drifts from target speed.
                                 # Too high → oscillates throttle/brake.
LEAD_SPEED_MAX_THROTTLE = 0.6   # max throttle command (0–1) [TO BE TUNED]

# ── Lead headway: fixed THW (not swept) ──────────────────────────────────────
# The ego↔lead following distance is a single constant, NOT a matrix axis.
# headway_d = FIXED_HEADWAY_THW × ego_ms is computed per-case in the runner/matrix.
# Difficulty is controlled instead by REVEAL_TTC (see matrix section below).
#
# Set to 0.8 s so that at the hardest case (reveal_ttc=1.0) the lead-to-target
# surface gap at trigger = (1.0 - 0.8) × ego_ms ≥ 1.1 m (at 20 km/h), giving the
# lead enough room to begin the lane change without overlapping the target.
# If CARLA tuning shows the lead still cannot complete the manoeuvre at reveal_ttc=1.0
# and 20 km/h, increase GAP_OFFSET slightly (e.g. 5.0 m) — do NOT change the matrix
# values; GAP_OFFSET is the per-case geometry constant that shifts the trigger distance.
FIXED_HEADWAY_THW = 0.8   # s — fixed following distance [TO BE TUNED in CARLA]

# ── Spawn geometry safety margins ────────────────────────────────────────────
# SPAWN_CLEARANCE_M: minimum surface gap enforced between ego and lead at spawn.
#   If headway_d < 2×ego_half_len + SPAWN_CLEARANCE_M, headway_d is clamped up.
#   At 20 km/h the nominal headway (0.8×5.556=4.444 m) overlaps the bounding boxes
#   (combined ≈4.5 m); 0.5 m clearance yields a clamped headway of ~5.0 m.
SPAWN_CLEARANCE_M    = 0.5  # m

# MIN_TRIGGER_SURF_GAP_M: minimum surface gap between lead and target at the
#   cut-out trigger point, required for the physics-based lane-change to succeed.
#   If the post-clamp trigger gap falls below this, the run is marked SCENARIO_INFEASIBLE
#   (kept in the CSV for auditing; filtered out in conflict-rate analysis).
#   At 20 km/h + reveal_ttc=1.0 the gap is only 0.556 m (0.10 s); not viable.
MIN_TRIGGER_SURF_GAP_M = 1.0  # m

# CUTOUT_STOP_MAX_M: hard-stop the lead this many metres past the trigger point
# to prevent it entering the baked obstacle zone at ~60 m on train000.
CUTOUT_STOP_MAX_M = 18.0    # m from trigger point; None = unlimited

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
INPATH_MAX_RANGE  = 80.0   # must cover the full ego-to-target distance (~49 m)

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
#  Single case for run_single_cutout.py
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
#  TEST MATRIX for run_matrix_cutout.py
#  5 ego_speed × 5 reveal_ttc × 2 μ = 50 cases/controller
#
#  Primary axis: reveal_ttc (TTC at cut-out trigger, same for all speeds).
#  headway_d = FIXED_HEADWAY_THW × ego_ms (not swept; derived per case in runner).
#  cutout_trigger_d = (reveal_ttc - FIXED_HEADWAY_THW) × ego_ms + GAP_OFFSET (derived per case).
#  Min cutout_trigger_d: reveal_ttc=1.0, 20 km/h → ≈ 5.6 m (all 50 cases > 0 ✓).
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
RESULTS_PREFIX = "cutout_matrix"    # results/cutout_matrix_*.csv

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
