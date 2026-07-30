# -*- coding: utf-8 -*-
"""
Central configuration file for the Junction Cut-in scenario on scene train105
------------------------------------------------------------------------------
Scenario: the ego drives straight in its own lane.  An intruder vehicle waits
at a junction entrance on the left.  When the ego closes to TURN_TRIGGER_D,
the intruder turns right using real physics-based steering (P-heading + P-speed
controllers via VehicleControl), cuts into the ego lane, aligns with the ego
heading, then brakes to a full stop — becoming a stationary blocker that forces
the ego's AEB to engage.

Two actors:
  ego       — cruises at constant speed; AEB controller decides braking.
  intruder  — stationary until trigger fires; physics-steered right into ego lane;
              brakes to a stop once aligned (AFTER_TURN_STOP = True).

Conflict definition: an unbraked ego would collide with the stationary intruder
at INTRUDER_STOP within MAX_TICKS × FIXED_DT seconds.  Computed before simulation
from kinematics only (see core/conflict.py junction_cutin_is_conflict).

Steering reuse: the state machine and P-controllers are adapted from
core/scenario_cutout.py (Rev 2026-07-22b).  The trigger condition differs:
cut-out fires on dist(lead, target); junction cut-in fires on dist(ego, intruder).

Scene:  train105   (load manually in CARLA before running)
------------------------------------------------------------------------------
Edit all values here only; run_single_junction_cutin.py / run_matrix_junction_cutin.py follow.
"""
import os

# ── CARLA Connection ─────────────────────────────────────────────
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# ── Simulation Parameters ─────────────────────────────────────────────────
FIXED_DT     = 0.05
MAX_TICKS    = 400
LOG_EVERY    = 10
SETTLE_TICKS = 20
STOP_KMH     = 0.6
GRAVITY      = 9.81

# ── Scene-name check ─────────────────────────────────────────────
EXPECTED_SCENE = "train105"

# ── Spectator camera ──────────────────────────────────────────────
SPECTATOR_TF = dict(x=42.69, y=-79.03, z=12.94, yaw=-97.20)

# ── Vehicle Positions (world coordinates, train105 scene) ─────────────────
# z values are fixed from first working run — do NOT use cast_ray or
# ground-projection (see README §Spawn Safety and TECHNICAL_DOC.md).
# High z (~10–11 m) is expected: this map's ground sits at a high absolute z.
EGO_SPAWN = dict(x=39.56, y=-89.23, z=10.0, yaw=-90.0)

# Intruder: starts at junction entrance (heading +x, yaw≈0); turns right on trigger.
INTRUDER_SPAWN = dict(x=32.34, y=-160.22, z=10.0, yaw=1.51,
                      model="vehicle.ue4.audi.tt")

# Intruder stop position: where the intruder comes to rest blocking the ego lane.
# Used only for the kinematic conflict check (core/conflict.py).
# [TO BE TUNED in CARLA: set to the ego-lane centre where the intruder stops after the turn]
# Placeholder: ego lane x, junction y — adjust once observed in CARLA.
INTRUDER_STOP = dict(x=40.14, y=-144.77, z=10.0)   # [TO BE TUNED]

# ── Turn trigger distance ─────────────────────────────────────────────────
# Intruder begins turning when ego↔intruder distance falls to this value (m).
# Analogous to trigger_d in the cut-in scenario (and swept as a matrix axis).
# [TO BE TUNED in CARLA — start at 30 m]
TURN_TRIGGER_D = 30.0   # m  [TO BE TUNED]

# ── Target heading for the turn ───────────────────────────────────────────
# The intruder steers until its yaw = trigger_yaw + TURN_HEADING_DEG.
# Intruder spawn yaw ≈ 1.51 (heading +x); ego lane yaw ≈ -88.65 (heading -y).
# Offset needed ≈ -90° to align with ego lane.
# ⚠ If the intruder turns the WRONG way in CARLA, negate this value (+90.0).
#   Same sign convention as CUTOUT_HEADING_DEG (positive = right in CARLA world).
TURN_HEADING_DEG = 75.0   # degrees  [TO BE TUNED — flip sign if wrong direction]

# After aligning into the ego lane, intruder brakes to a full stop.
AFTER_TURN_STOP = True   # always True for this scenario (creates a stationary blocker)

# ── Physics-steering parameters (all TO BE TUNED in CARLA) ───────────────
# Intruder driven entirely via apply_control(VehicleControl) — no set_transform /
# set_target_velocity in the update loop (velocity boot in start() only).
#
# Heading P-controller:
JCUTIN_STEER_K    = 0.05   # steer command per degree of heading error
                             # Too low → sluggish; too high → oscillates
JCUTIN_STEER_MAX  = 0.6    # max |steer| (0–1). Higher than cut-out: ~90° turn.
JCUTIN_SETTLE_DEG = 8.0    # |heading error| (°) below which intruder is considered
                             # aligned with ego lane → transitions to SETTLED/STOP.
#
# Speed P-controller (active during the turn):
JCUTIN_SPEED_K        = 1.2   # throttle/brake P-gain per m/s speed error
JCUTIN_MAX_THROTTLE   = 0.6  # max throttle command (0–1)

# ── Cameras ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=2.8, y=0.0, z=0.8, pitch=0)   # front camera, confirmed live in CARLA
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)
CAM_FOV_DEG  = 90.0

# ── YOLO ──────────────────────────────────────────────────────────
YOLO_MODEL     = "yolov8n.pt"
YOLO_DEVICE    = "cpu"
CONF_THRESH    = 0.45
IOU_THRESH     = 0.45
TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 9, 11]
VEHICLE_CLS    = (2, 3, 5, 7)
CLASS_COLORS   = {0:(0,255,0),1:(255,165,0),2:(0,0,255),3:(255,0,255),
                  5:(0,165,255),7:(128,0,128),9:(0,255,255),11:(255,255,0)}

LANE_LEFT  = 520
LANE_RIGHT = 760
MIN_BOX_H  = 90

# ── Detection gate ────────────────────────────────────────────────
# Tracks the intruder throughout (before and after it enters the ego lane).
# Ground-truth gives full run-to-run repeatability.
DETECTION_SOURCE  = "groundtruth"

INPATH_HALF_WIDTH = 1.8    # half lane width (m)
INPATH_MAX_RANGE  = 80.0   # covers ego-spawn to intruder-stop distance (~76 m)

INPATH_PREDICT    = True   # intruder is stationary until it enters the lane
INPATH_LOOKAHEAD  = 1.5

# ── Camera frame synchronisation ──
FRAME_SYNC = True

# ── Surface-to-surface gap (ego front ↔ intruder rear once in lane) ──
AUTO_GAP_OFFSET = True
GAP_OFFSET      = 4.5   # fallback / pre-sim conflict-check (m)

# ── Braking model ──
BRAKE_MODEL = "kinematic"

# ── Road friction μ ──
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  Single case for run_single_junction_cutin.py
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh = 40.0,
    mu            = MU_DRY,
    trigger_d     = 80.0,   # ego↔intruder distance at which the turn begins (m)
)
SINGLE_CONTROLLER   = "proposed_enhanced"
SINGLE_DELAY_FRAMES = 0
SHOW_WINDOW         = True

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX for run_matrix_junction_cutin.py
#  5 ego_speed × 5 trigger_d × 2 μ = 50 cases/controller
#
#  trigger_d: distance at which the intruder begins its right turn into the lane.
#  Larger trigger_d → more warning time for the AEB (intruder starts turning earlier).
#  Conflict is independent of trigger_d (intruder always stops at INTRUDER_STOP).
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh = [20.0,30.0, 40.0, 50.0, 60.0], # [20.0, 30.0, 40.0, 50.0, 60.0]
    mu            = [MU_DRY, MU_WET],
    trigger_d     = [70.0, 80.0, 90.0, 100.0, 110.0] # [60.0, 65.0, 70.0, 75.0, 80.0,]
)

CONTROLLERS = ["baseline", "proposed", "proposed_enhanced"]

# ── Sweep values for degradation modes ────────────────────────────
LATENCY_DELAY_FRAMES   = [0, 4, 8, 16]
NOISE_SIGMA_M_SWEEP    = [0.0, 0.5, 1.0, 2.0]
NOISE_SIGMA_VR_SWEEP   = [0.0, 0.2, 0.5, 1.0]
DROPOUT_P_SWEEP        = [0.0, 0.05, 0.1, 0.2]

COMP_CONTROLLERS       = ["proposed_enhanced", "enhanced_predictive",
                          "enhanced_inflation", "proposed"]
COMP_MISMATCH_CTRL     = "enhanced_predictive"
COMP_MISMATCH_DELAY    = 8
COMP_MISMATCH_L_FRAMES = [4, 8, 12, 16]
COMP_R_SAFE            = 0.0


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


MATRIX_RUNS  = build_matrix_runs(os.environ.get("TEST_MODE", "original"))
RESULTS_DIR  = "results"
RESULTS_PREFIX = "junction_matrix"   # results/junction_matrix_*.csv

# ══════════════════════════════════════════════════════════════════
#  Controller parameters (same values as existing scenarios)
# ══════════════════════════════════════════════════════════════════
TTC_WARN_FULL  = 1.6
TTC_BRAKE_FULL = 0.6

DYN_V0        = 40.0
DYN_MU0       = 0.85
DYN_K_SPEED   = 1.2
DYN_K_MU      = 1.5
PARTIAL_BRAKE = 0.4

# intruder stops → v_l=0 → proposed_enhanced uses a_req = v_e² / (2·gap)
REQ_FULL_FRAC = 0.9
REQ_WARN_FRAC = 0.6
