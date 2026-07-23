# -*- coding: utf-8 -*-
"""
Central configuration file for the lead-brake scenario (lead vehicle driving ahead then braking suddenly)
----------------------------------------------------------------------
Scenario (Euro-NCAP CCRb): a lead vehicle drives ahead of ego in the same lane
           at a speed "equal to ego" (configurable), maintaining a constant gap,
           then the lead vehicle suddenly "brakes hard" to a stop
           → tests whether AEB causes ego to brake in time
Separate from the cut-in scenario: edit values here only,
then run_single_lead.py / run_matrix_lead.py will follow.
Most structure/parameters carried over from config/scenario_cutin.py
(controller/YOLO/CPEIM are identical).
"""
import os

# ── CARLA Connection ─────────────────────────────────────────────
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# ── Simulation Parameters ─────────────────────────────────────────────────
FIXED_DT  = 0.05          # 20 FPS (sync mode)
MAX_TICKS = 400
LOG_EVERY = 10
SETTLE_TICKS = 20         # empty ticks to let the scene settle before releasing vehicles
STOP_KMH  = 0.6           # below this speed the vehicle is considered fully stopped
END_Y     = -60.0         # ego past this point = overshot the expected collision point (prevents hanging)
GRAVITY   = 9.81

# ── Scene-name check (Section 5b) ─────────────────────────────────
# world.get_map().name must contain this substring; set "" to skip.
EXPECTED_SCENE = "scene03_2"

# ── Spectator camera for scene03_2 (cosmetic, no effect on recorded results) ──
SPECTATOR_TF = dict(x=2.07, y=-0.69, z=1.87, yaw=-91.22)

# ── Vehicle Positions (world coordinates, from scene03 prototype) ─────────────────────────
EGO_SPAWN  = dict(x=3.02, y=-8.70, z=1.15, yaw=-90)            # ego starts here, driving in the -Y direction
# Lead vehicle is "same lane, same heading" ahead of ego
#   lead's y at spawn = EGO_SPAWN.y − headway_d (see runner); gap distance is a test variable
LEAD_SPAWN = dict(x=3.02, z=1.15, yaw=-90,
                  model="vehicle.ue4.audi.tt")

# ── Lead vehicle behaviour: drives ahead → after travelling a set distance, brakes hard to a stop ──
LEAD_BRAKE_AFTER_M = 8.0   # lead travels this far (m) from spawn before "braking hard"

# LEAD_DECEL: deceleration of the lead vehicle when braking (m/s²)
# Euro-NCAP CCRb 2023 §3.4 specifies 6 m/s² as the standard case; 4.0 ≈ moderate braking
# Adjustable via environment variable without editing this file:
#   LEAD_DECEL=6.0 python run_matrix_lead.py
LEAD_DECEL = float(os.environ.get("LEAD_DECEL", "4.0"))
LEAD_SAME_AS_EGO   = True  # True = lead speed equals ego speed in every case (per the "equal speed" requirement)
                           # False = use MATRIX["lead_speed_kmh"] list as a separate variable

# ── Lead vehicle headway definition: time-based (THW) instead of fixed distance ──
#   THW (time headway, seconds) is more realistic/standard than fixed distance
#   because actual gap scales with speed
#   actual gap (m) = ego_speed (m/s) × THW   e.g. 50 km/h × 1.5s ≈ 20.8 m
USE_THW     = True                  # True = use THW (seconds), False = use fixed headway_d (legacy behaviour)
HEADWAY_THW = [1.0, 1.5, 2.0, 2.5, 3.0]  # THW values to sweep (seconds) — used when USE_THW=True

# ── Cameras ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=1.0, y=0.0, z=0.5, pitch=0)              # front camera: rear-view mirror / top-of-windshield position
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)  # top-view camera (display only)
CAM_FOV_DEG = 90.0

# ── YOLO (detector for vehicles in the lane) ───────────────────────────────
YOLO_MODEL  = "yolov8n.pt"
YOLO_DEVICE = "cpu"       # "cpu" or "cuda"
CONF_THRESH = 0.45
IOU_THRESH  = 0.45
TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 9, 11]
VEHICLE_CLS = (2, 3, 5, 7)        # car, moto, bus, truck → used to classify as a "vehicle"
CLASS_COLORS = {0:(0,255,0),1:(255,165,0),2:(0,0,255),3:(255,0,255),
                5:(0,165,255),7:(128,0,128),9:(0,255,255),11:(255,255,0)}

# Ego lane band in pixels — lead vehicle is centred in the lane, easier for YOLO to detect than cut-in
LANE_LEFT  = 520
LANE_RIGHT = 760
MIN_BOX_H  = 90

# ── Detection gate source ──
#   "groundtruth" = use true CARLA position to check whether lead is in the forward path (reliable/repeatable)
#   "yolo" / "both_or" = same as cut-in scenario
DETECTION_SOURCE = "groundtruth"

# Ego corridor for the ground-truth gate — measured in ego's coordinate frame
INPATH_HALF_WIDTH = 1.8    # half lane width (m)
INPATH_MAX_RANGE  = 80.0   # maximum forward look-ahead distance (m) — allows for large headway values

# ── predictive corridor ──
# In this scenario the lead vehicle stays directly ahead (it does not cut in from the side), so prediction is disabled
# detected is True from the start while following, but ttc≈inf until the lead brakes → correct brake timing
INPATH_PREDICT   = False
INPATH_LOOKAHEAD = 0.0

# ── Camera frame / world snapshot synchronisation ──
FRAME_SYNC = True

# ── Surface-to-surface gap ──
# This scenario is a rear-end collision; both vehicles face the same direction
# so subtract (half ego length + half lead length) from the centre-to-centre distance
AUTO_GAP_OFFSET = True     # True = computed automatically from bounding box (ego.extent.x + lead.extent.x)
GAP_OFFSET = 4.5           # used when AUTO_GAP_OFFSET=False (m)

# ── Ego braking model ──
BRAKE_MODEL = "kinematic"

# ── Road friction μ ───────────────────────────────────────────────────
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  Single case for run_single_lead.py (debug/demo, with display)
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh  = 50.0,    # ego vehicle speed (v_x)
    lead_speed_kmh = 50.0,    # lead vehicle speed before braking (normally = ego per the "equal speed" requirement)
    mu             = MU_WET,   # road friction coefficient
    headway_thw    = 1.5,     # time-based headway (seconds) — used when USE_THW=True (50km/h×1.5s≈20.8m)
    headway_d      = 20.0,    # fixed headway distance (metres) — used when USE_THW=False
)
SINGLE_CONTROLLER  = "proposed"   # "baseline" | "proposed"
SINGLE_DELAY_FRAMES = 0           # perception delay (frames) 0=immediate, 16≈0.8s
SHOW_WINDOW = True                # whether to show the OpenCV window

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX for run_matrix_lead.py (automated sweep, no display)
#  lead-brake scenario = 5 speed × 5 headway × 2 μ = 50 cases/controller
#  default compares 3 controllers (baseline, proposed, proposed_enhanced) = 150 runs
#  (default: lead speed = ego speed because LEAD_SAME_AS_EGO=True
#   and headway is measured as THW because USE_THW=True → sweeps headway_thw)
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh  = [20.0, 30.0, 40.0, 50.0, 60.0],   # 20 km/h = low urban/parking-area speed
    headway_thw    = HEADWAY_THW,                 # time-based headway (seconds) — used when USE_THW=True
    headway_d      = [10.0, 15.0, 20.0, 25.0],   # fixed headway distance (m) — used when USE_THW=False
    mu             = [MU_DRY, MU_WET],            # road friction
    lead_speed_kmh = [30.0, 50.0],               # used only when LEAD_SAME_AS_EGO=False
)
# Which controllers to compare + per-controller frame delay
#   default for this CCRb scenario: 3-way comparison: baseline (fixed TTC) vs proposed (dynamic-TTC)
#   vs proposed_enhanced (required-decel) — all on the same case set, delay 0
# ── Controllers registered for all test modes ──────────────────────────────────
# Add new controllers here — build_matrix_runs() picks them up in all modes automatically.
CONTROLLERS = ["baseline", "proposed", "proposed_enhanced"]

# ── Sweep values for degradation modes (researcher-adjustable) ────────────────
LATENCY_DELAY_FRAMES = [0, 4, 8, 16]          # = 0, 0.2, 0.4, 0.8 s at FIXED_DT=0.05
NOISE_SIGMA_M_SWEEP  = [0.0, 0.5, 1.0, 2.0]  # distance noise sigma (m)
NOISE_SIGMA_VR_SWEEP = [0.0, 0.2, 0.5, 1.0]  # rel_speed / lead_speed noise sigma (m/s)
DROPOUT_P_SWEEP      = [0.0, 0.05, 0.1, 0.2] # dropout probability per tick

# ── Latency-compensation experiment (latency_comp / latency_mismatch) ─────────
#   comp controllers compensate for perception latency: enhanced_predictive (predictor) /
#   enhanced_inflation (threshold inflation). proposed_enhanced = control (no compensation),
#   proposed = crossover reference. See control/enhanced_*.py and CHANGES_latency_compensation.md
COMP_CONTROLLERS      = ["proposed_enhanced", "enhanced_predictive",
                         "enhanced_inflation", "proposed"]
COMP_MISMATCH_CTRL    = "enhanced_predictive"  # controller swept for mismatch (most sensitive to delay-mismatch)
COMP_MISMATCH_DELAY   = 8                       # fixed injected real delay (frames) in mismatch mode
COMP_MISMATCH_L_FRAMES = [4, 8, 12, 16]         # L' values used to compensate: under(4) / exact(8) / over(12,16)
COMP_R_SAFE           = 0.0                      # stopping margin for enhanced_inflation (m)


def build_matrix_runs(mode="original"):
    """Return MATRIX_RUNS list for the given test mode.

    TEST_MODE=original        — 3 controllers, all degradation params = 0 (original behaviour)
    TEST_MODE=latency         — sweep delay_frames across LATENCY_DELAY_FRAMES × controllers
    TEST_MODE=noise           — sweep noise_sigma_m across NOISE_SIGMA_M_SWEEP × controllers
    TEST_MODE=latency_comp    — COMP_CONTROLLERS × delay sweep × comp_source=oracle
                                (upper bound: knowing L exactly — how much Rc can be recovered)
    TEST_MODE=latency_mismatch— enhanced_predictive × fixed delay × comp_L_frames swept
                                (fragility when L is estimated incorrectly: under/exact/over-compensate)
    TEST_MODE=latency_comp_all— CONTROLLERS (all base controllers) × delay sweep × predictive-oracle
                                layer (predict=True). Places the predictor in front of every base controller
                                (symmetric with the degradation layer) → compares how much Rc the predictor recovers for
                                each base controller at the same latency (upper bound: knowing L exactly)
    """
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
        # oracle: compensate with L = actual injected delay (comp_L_frames = delay_frames)
        return [
            dict(label=f"{c}_d{d}f", controller=c, delay_frames=d,
                 comp_source="oracle", comp_L_frames=d,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in COMP_CONTROLLERS for d in LATENCY_DELAY_FRAMES
        ]
    if mode == "latency_mismatch":
        # mismatched: fixed real delay but compensate with a different L' (under/exact/over)
        d = COMP_MISMATCH_DELAY
        return [
            dict(label=f"{COMP_MISMATCH_CTRL}_d{d}f_L{cl}f", controller=COMP_MISMATCH_CTRL,
                 delay_frames=d, comp_source="mismatched", comp_L_frames=cl,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for cl in COMP_MISMATCH_L_FRAMES
        ]
    if mode == "latency_comp_all":
        # predictive-oracle compensation applied as a LAYER in front of every base
        # controller (predict=True → runner inserts PerceptionPredictor; see perception/predict.py).
        # oracle: compensate with L = actual injected delay (comp_L_frames = delay_frames). Uses CONTROLLERS
        # (all base) same as original/latency mode for fairness (controller-swap protocol)
        return [
            dict(label=f"{c}_pred_d{d}f", controller=c, delay_frames=d,
                 comp_source="oracle", comp_L_frames=d, predict=True,
                 noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
            for c in CONTROLLERS for d in LATENCY_DELAY_FRAMES
        ]
    # "original" (default) — identical to prior hardcoded MATRIX_RUNS
    return [
        dict(label=c, controller=c, delay_frames=0,
             noise_sigma_m=0.0, noise_sigma_vr=0.0, dropout_p=0.0, dropout_mode="freeze")
        for c in CONTROLLERS
    ]


MATRIX_RUNS = build_matrix_runs(os.environ.get("TEST_MODE", "original"))
RESULTS_DIR = "results"
RESULTS_PREFIX = "lead_matrix"     # result file → results/lead_matrix_*.csv (separate from cut-in scenario)

# ══════════════════════════════════════════════════════════════════
#  Controller parameters (same as cut-in scenario for direct comparison)
# ══════════════════════════════════════════════════════════════════
# baseline: fixed TTC
TTC_WARN_FULL = 1.6
TTC_BRAKE_FULL = 0.6

# proposed: TTC adjusted for speed + friction
DYN_V0      = 40.0
DYN_MU0     = 0.85
DYN_K_SPEED = 1.2
DYN_K_MU    = 1.5
PARTIAL_BRAKE = 0.4

# proposed_enhanced: required-deceleration (knows both μ and the lead vehicle's braking)
#   urgency = a_req / (μ·g) ; a_req = v_e² / (2·(gap + v_l²/2a_l))
REQ_FULL_FRAC = 0.9       # urgency ≥ this value → full brake (90% of μ·g ceiling)
REQ_WARN_FRAC = 0.6       # urgency ≥ this value → partial brake (PARTIAL_BRAKE)
