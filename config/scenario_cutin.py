# -*- coding: utf-8 -*-
"""
Central configuration file for the cut-in / dart-out scenario
(a vehicle cuts in from the side and blocks the lane)
----------------------------------------------------------------------
Edit all values here only; both run_single.py and run_matrix.py will follow.
Scene/transform values are taken from the tuned prototype test_spawn_scene03_step3-2.py
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
END_Y     = -56.0         # ego past this point = passed the intersection without braking
GRAVITY   = 9.81

# ── Scene-name check (Section 5b) ─────────────────────────────────
# world.get_map().name must contain this substring; set "" to skip.
EXPECTED_SCENE = "scene03_2"

# ── Spectator camera for scene03_2 (cosmetic, no effect on recorded results) ──
SPECTATOR_TF = dict(x=2.07, y=-0.69, z=1.87, yaw=-91.22)

# ── Vehicle Positions (world coordinates, from scene03 prototype) ─────────────────────────
EGO_SPAWN  = dict(x=3.02, y=-8.70, z=1.15, yaw=-90)            # ego starts here, driving in the -Y direction
DART_SPAWN = dict(x=11.0, y=-51.5, z=0.8,  yaw=180,            # dart vehicle starts parked at the side
                  model="vehicle.ue4.audi.tt")
DART_STOP_X = 3.5         # dart reaches this x (centre of ego's lane) then brakes to a blocking stop

# ── Cameras ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=2.8, y=0.0, z=0.8, pitch=0)              # front camera: bonnet/windshield-base area, confirmed live in CARLA
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)  # top-view camera (display only)
# Pinhole parameter reserved for future use (currently distance uses ground-truth, this value is unused)
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

# Ego lane band in pixels (not lane detection) — only brake for boxes whose "centre x" falls in this band
LANE_LEFT  = 520
LANE_RIGHT = 760
# Box must be at least this tall to count as "detected close enough" (prevents triggering when dart is still far)
MIN_BOX_H  = 90

# ── Detection gate source (used to decide "hazard in path" so the controller brakes) ──
#   "groundtruth" = use true CARLA position to check whether dart is in the forward path (reliable)
#   "yolo"        = use the centre pixel band + box height (fragile with cut-in geometry)
#   "both_or"     = true when either ground-truth or YOLO is true
# "groundtruth" is recommended because dart cuts in from the side and often slips out of the centre band causing a collision
# YOLO is still drawn on screen at all times for realism/debugging, regardless of which source is selected
DETECTION_SOURCE = "groundtruth"

# Ego corridor for the ground-truth gate — measured in ego's coordinate frame
INPATH_HALF_WIDTH = 1.8    # half lane width (m) — |lateral offset| ≤ this = in lane
INPATH_MAX_RANGE  = 40.0   # maximum forward look-ahead distance (m)

# ── Predictive corridor (detects dart while it is "entering the lane", not waiting until fully in) ──
# Uses dart's lateral speed to predict whether it will enter the corridor within lookahead seconds
# Gives both baseline/proposed sufficient detection lead → dynamic TTC of proposed can take effect
INPATH_PREDICT   = True
INPATH_LOOKAHEAD = 1.5     # seconds look-ahead to predict whether dart will enter the lane

# ── Camera frame / world snapshot synchronisation (prevents image drift in sync mode) ──
FRAME_SYNC = True          # True = read frames until frame id matches world.tick()

# ── Surface-to-surface gap ──
# dist2d is the centre-to-centre distance; each vehicle is ~4.5 m long; collision occurs when bumpers touch
# so subtract the vehicle dimensions before computing TTC/clearance; otherwise the controller brakes ~2 m too late
AUTO_GAP_OFFSET = True     # True = computed automatically from bounding box (ego length + dart width)
GAP_OFFSET = 3.2           # used when AUTO_GAP_OFFSET=False (m)

# ── Braking model ──
#   "kinematic" = controls deceleration = brake × μ × g directly via set_target_velocity
#                 → μ is the actual test variable, reproducible, consistent with v²/2μg (recommended)
#   "physics"   = sends apply_control(brake) and lets CARLA's tire model handle it
#                 (works if CARLA 0.9.x accepts tire_friction; 0.10/Chrono often does not accept μ)
BRAKE_MODEL = "kinematic"

# ── Road friction μ (set at wheel tire_friction) ───────────────────
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  Single case for run_single.py (debug/demo, with display)
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh = 50.0,     # ego vehicle speed (v_x)
    mu            = MU_WET,    # road friction coefficient
    trigger_d     = 20.0,     # dart launches when ego-to-dart distance = this value (Δd, metres)
    dart_speed_kmh= 20.0,     # dart speed when it launches (v_ob)
)
SINGLE_CONTROLLER  = "proposed"   # "baseline" | "proposed"
SINGLE_DELAY_FRAMES = 0           # perception delay (frames) 0=immediate, 16≈0.8s
SHOW_WINDOW = True                # whether to show the OpenCV window

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX for run_matrix.py (automated sweep, no display)
#  cut-in scenario = 5 speed × 2 μ × 5 Δd = 50 cases/controller
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh = [20.0, 30.0, 40.0, 50.0, 60.0],   # 20 km/h = low urban/parking-area speed
    mu            = [MU_DRY, MU_WET],
    trigger_d     = [20.0, 25.0, 30.0, 35.0, 40.0],   # 40 m = longer response distance (expands domain)
    dart_speed_kmh= [20.0],
)
# Which controllers to compare + per-controller frame delay
# (switchable in 2 ways: change controller or change delay)
#   default compares 3 controllers (baseline, proposed, proposed_enhanced) on the same case set
#   proposed_enhanced (required-decel) reduces to the stationary-obstacle case in cut-in (v_l=0 → a_req=v_e²/2gap)
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

# ══════════════════════════════════════════════════════════════════
#  Controller parameters (tunable)
# ══════════════════════════════════════════════════════════════════
# baseline: fixed TTC (reference: international AEB standard in paper)
TTC_WARN_FULL = 1.6       # TTC ≤ this value → partial brake
TTC_BRAKE_FULL = 0.6      # TTC ≤ this value → full brake

# proposed: TTC adjusted for speed + friction (brakes earlier when faster/more slippery)
#   thr_full = TTC_BRAKE_FULL + k_speed*max(0,(v-v0)/100) + k_mu*max(0,(mu0-mu))
DYN_V0      = 40.0        # base speed (km/h); above this threshold starts increasing
DYN_MU0     = 0.85        # base μ (dry); below this threshold starts increasing
DYN_K_SPEED = 1.2         # speed effect weight
DYN_K_MU    = 1.5         # friction effect weight
PARTIAL_BRAKE = 0.4       # partial-braking force

# proposed_enhanced: required-deceleration (knows both μ and the lead vehicle's braking)
#   cut-in scenario: dart blocks the lane v_l=0 → a_req = v_e²/2gap (stationary-obstacle case) automatically
#   add controller="proposed_enhanced" to MATRIX_RUNS to include in comparison
REQ_FULL_FRAC = 0.9       # urgency ≥ this value → full brake (90% of μ·g ceiling)
REQ_WARN_FRAC = 0.6       # urgency ≥ this value → partial brake (PARTIAL_BRAKE)
