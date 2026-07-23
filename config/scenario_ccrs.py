# -*- coding: utf-8 -*-
"""
Central configuration file for the CCRs scenario on scene train000
(Car-to-Car Rear Stationary: ego drives toward a stationary target vehicle)
----------------------------------------------------------------------
Euro-NCAP CCRs style: the target vehicle stands stationary in the ego's path
from the very start of the run, never moving. The test measures whether the
ego's AEB brakes in time to avoid the collision.

Scene:  train000   (load manually in CARLA before running)
----------------------------------------------------------------------
Edit all values here only; both run_single_ccrs.py and run_matrix_ccrs.py
will follow.  Structure mirrors config/scenario_lead_brake.py.
"""
import os

# ── CARLA Connection ─────────────────────────────────────────────
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# ── Simulation Parameters ─────────────────────────────────────────────────
FIXED_DT  = 0.05          # 20 FPS (sync mode)  — same as existing scenarios
MAX_TICKS = 400
LOG_EVERY = 10
SETTLE_TICKS = 20         # empty ticks to let scene settle before releasing vehicles
STOP_KMH  = 0.6           # below this speed the vehicle is considered fully stopped
GRAVITY   = 9.81

# ── Scene-name check (Section 5b) ─────────────────────────────────
# world.get_map().name must contain this substring; set "" to skip the check.
EXPECTED_SCENE = "train000"

# ── Spectator camera for train000 (cosmetic, no effect on recorded results) ──
SPECTATOR_TF = dict(x=5.27, y=-0.18, z=0.67, yaw=-143.44)

# ── Vehicle Positions (world coordinates, train000 scene) ─────────────────
# Ego spawn: driving in the direction of Yaw=-146.54° (NOT axis-aligned,
#   forward ≈ (-0.835, -0.550, 0) — spawn geometry computed from ego forward vector).
EGO_SPAWN = dict(x=6.42,   y=0.34,   z=0.79, yaw=-146.54)

# Target vehicle: stationary from t=0; same heading as ego so the collision is
#   a rear-to-front (front-of-ego vs rear-of-target) geometry.
TARGET_SPAWN = dict(x=-34.95, y=-26.28, z=0.25, yaw=-146.54,
                    model="vehicle.ue4.audi.tt")

# ── Cameras ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=2.8, y=0.0, z=0.8, pitch=0)              # front camera: bonnet/windshield-base area, confirmed live in CARLA
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)
CAM_FOV_DEG = 90.0

# ── YOLO (detector for vehicles in the lane) ───────────────────────────────
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
# Target is always stationary in the ego's path; ground-truth is reliable and
# does not require the predictive corridor (INPATH_PREDICT=False as in lead-brake).
DETECTION_SOURCE = "groundtruth"

# Ego corridor for ground-truth gate (in ego's coordinate frame)
INPATH_HALF_WIDTH = 1.8    # half lane width (m)
INPATH_MAX_RANGE  = 80.0   # large enough to detect target from spawn (~49 m away)

INPATH_PREDICT   = False   # target is already in-path; no lateral prediction needed
INPATH_LOOKAHEAD = 0.0

# ── Camera frame synchronisation ──
FRAME_SYNC = True

# ── Surface-to-surface gap ──
# Both vehicles have the same yaw → rear-to-front geometry (same as lead-brake).
# gap_offset ≈ ego.extent.x + target.extent.x (computed from bounding boxes at runtime).
AUTO_GAP_OFFSET = True
GAP_OFFSET = 4.5           # fallback / pre-sim conflict-check approximation (m)

# ── Braking model ──
BRAKE_MODEL = "kinematic"

# ── Road friction μ ──
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  Single case for run_single_ccrs.py (debug / demo with display)
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh = 50.0,
    mu            = MU_WET,
)
SINGLE_CONTROLLER  = "proposed_enhanced"
SINGLE_DELAY_FRAMES = 0
SHOW_WINDOW = True

# ── Initial approach distance (centre-to-centre, ego → target) ──────────────
# The target is spawned at EGO_SPAWN + approach_d × forward_vector per case.
# This controls available reaction time (TTC at spawn = (approach_d - GAP_OFFSET) / ego_ms).
# Matches Euro-NCAP CCRs spirit: varying approach distance = varying available reaction time.
APPROACH_DISTANCES = [30.0, 40.0, 50.0, 60.0, 70.0]   # m  [TO BE TUNED in CARLA]

# ── Robust target spawn: z-sweep ─────────────────────────────────────────────
# Some positions on the train000 3DGS mesh reject the nominal z=0.25 (bad terrain
# patch or obstacle at that world coordinate).  The runner tries each z in order;
# the first successful spawn is used.  A non-nominal z triggers a console warning
# so the user can verify the placement visually in CARLA.
# Known issue: approach_d=60 m fails at z=0.25 but succeeds at a higher z.
SPAWN_Z_SWEEP = [0.25, 0.5, 0.75, 1.0, 1.5]  # m — z values tried in order
# Surface gaps (approx, GAP_OFFSET ≈ 4.5 m): ~25.5 / ~35.5 / ~45.5 / ~55.5 / ~65.5 m
# TTC at spawn examples: 30 m + 60 km/h ≈ 1.5 s (hard); 70 m + 20 km/h ≈ 11.8 s (easy)
# All 5 values are conflict cases: worst-case 70 m @ 20 km/h → t ≈ 11.8 s < 20 s window ✓

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX for run_matrix_ccrs.py  (5 speed × 5 distance × 2 μ = 50 cases/controller)
#  Target spawn is derived per case from approach_d along the ego forward vector.
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh = [20.0, 30.0, 40.0, 50.0, 60.0],
    approach_d    = APPROACH_DISTANCES,
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
    """Return MATRIX_RUNS list for the given test mode (identical logic to lead-brake config)."""
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
RESULTS_PREFIX = "ccrs_matrix"     # results/ccrs_matrix_*.csv

# ══════════════════════════════════════════════════════════════════
#  Controller parameters (same values as existing scenarios for direct comparison)
# ══════════════════════════════════════════════════════════════════
TTC_WARN_FULL  = 1.6
TTC_BRAKE_FULL = 0.6

DYN_V0      = 40.0
DYN_MU0     = 0.85
DYN_K_SPEED = 1.2
DYN_K_MU    = 1.5
PARTIAL_BRAKE = 0.4

# proposed_enhanced: stationary target → v_l=0 → a_req = v_e²/(2·gap) automatically
REQ_FULL_FRAC = 0.9
REQ_WARN_FRAC = 0.6
