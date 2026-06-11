# -*- coding: utf-8 -*-
"""
ไฟล์ตั้งค่ากลางของฉาก lead-brake (รถนำวิ่งนำอยู่ข้างหน้า แล้วเบรกกระทันหัน)
----------------------------------------------------------------------
สถานการณ์ (Euro-NCAP CCRb): มีรถนำวิ่งอยู่ข้างหน้า ego ในเลนเดียวกัน
           ด้วยความเร็ว "เท่ากับ ego" (ตั้งค่าได้) รักษาระยะห่างคงที่
           แล้วจู่ ๆ รถนำ "เบรกกระทันหัน" จนหยุด → ทดสอบ AEB ว่า ego เบรกทันไหม
แยกขาดจากฉาก cut-in: แก้ค่าที่นี่ที่เดียว แล้ว run_single_lead.py / run_matrix_lead.py ใช้ตาม
โครงสร้าง/พารามิเตอร์ส่วนใหญ่ยกมาจาก config/scenario_cutin.py (สมองกล/YOLO/CPEIM เหมือนกัน)
"""

# ── การเชื่อมต่อ CARLA ─────────────────────────────────────────────
HOST = "localhost"
PORT = 2000
TIMEOUT = 10.0

# ── พารามิเตอร์ซิม ─────────────────────────────────────────────────
FIXED_DT  = 0.05          # 20 FPS (sync mode)
MAX_TICKS = 400
LOG_EVERY = 10
SETTLE_TICKS = 20         # tick เปล่าให้ฉากนิ่งก่อนปล่อยรถ
STOP_KMH  = 0.6           # ต่ำกว่านี้ถือว่าหยุดสนิท
END_Y     = -60.0         # ego เลยจุดนี้ = วิ่งทะลุจุดที่ควรชนไปแล้ว (กันค้าง)
GRAVITY   = 9.81

# ── ตำแหน่งรถ (พิกัดโลก จากต้นแบบ scene03) ─────────────────────────
EGO_SPAWN  = dict(x=3.02, y=-8.70, z=1.15, yaw=-90)            # ego เริ่มที่นี่ วิ่งไปทาง -Y
# รถนำ (lead) อยู่ "เลนเดียวกัน หันทางเดียวกัน" ข้างหน้า ego
#   y ของ lead ตอน spawn = EGO_SPAWN.y − headway_d (ดู runner) ส่วนระยะห่างเป็นตัวแปรทดสอบ
LEAD_SPAWN = dict(x=3.02, z=1.15, yaw=-90,
                  model="vehicle.ue4.audi.tt")

# ── พฤติกรรมรถนำ: วิ่งนำ → พอวิ่งได้ระยะหนึ่งก็เบรกกระทันหันจนหยุด ──
LEAD_BRAKE_AFTER_M = 8.0   # รถนำวิ่งไปได้ไกลเท่านี้ (ม.) จากจุด spawn แล้วค่อย "เบรกกระทันหัน"
LEAD_DECEL         = 4.0   # ความหน่วงตอนรถนำเบรก (m/s²) — มาตรฐาน Euro-NCAP CCRb (เดิม 9.0 แรงเกินจริง)
LEAD_SAME_AS_EGO   = True  # True = ความเร็วรถนำ = ความเร็ว ego ในทุกเคส (ตามโจทย์ "เท่ากัน")
                           # False = ใช้รายการ MATRIX["lead_speed_kmh"] เป็นตัวแปรแยกต่างหาก

# ── นิยามระยะห่างรถนำ: เวลา (THW) แทนระยะคงที่ ──
#   THW (time headway, วินาที) สมจริง/มาตรฐานกว่าระยะคงที่ เพราะระยะจริงโตตามความเร็ว
#   ระยะจริง (m) = ego_speed (m/s) × THW   เช่น 50 km/h × 1.5s ≈ 20.8 m
USE_THW     = True                  # True = ใช้ THW (วินาที), False = ใช้ระยะคงที่ headway_d (พฤติกรรมเดิม)
HEADWAY_THW = [1.0, 1.5, 2.0, 2.5]  # ค่า THW ที่ไล่ทดสอบ (วินาที) — ใช้เมื่อ USE_THW=True

# ── กล้อง ───────────────────────────────────────────────────────────
CAM_W, CAM_H = 1280, 720
CAM_FRONT_TF = dict(x=3.5, y=0.2, z=1.60, pitch=8)             # กล้องหน้าติด ego (ใช้กับ YOLO)
CAM_TOP_TF   = dict(x=-2.0, y=-6.0, z=3.5, pitch=-15, yaw=45)  # กล้อง top view (โชว์เฉยๆ)
CAM_FOV_DEG = 90.0

# ── YOLO (ตัวตรวจจับว่ามีรถในเลนไหม) ───────────────────────────────
YOLO_MODEL  = "yolov8n.pt"
YOLO_DEVICE = "cpu"       # "cpu" หรือ "cuda"
CONF_THRESH = 0.45
IOU_THRESH  = 0.45
TARGET_CLASSES = [0, 1, 2, 3, 5, 7, 9, 11]
VEHICLE_CLS = (2, 3, 5, 7)        # car, moto, bus, truck → ใช้ตัดสินว่าเป็น "รถ"
CLASS_COLORS = {0:(0,255,0),1:(255,165,0),2:(0,0,255),3:(255,0,255),
                5:(0,165,255),7:(128,0,128),9:(0,255,255),11:(255,255,0)}

# แถบเลน ego เป็นพิกเซล — รถนำอยู่กลางเลนพอดี YOLO เห็นง่ายกว่าฉาก cut-in
LANE_LEFT  = 520
LANE_RIGHT = 760
MIN_BOX_H  = 90

# ── แหล่งเกตตรวจจับ ──
#   "groundtruth" = ใช้ตำแหน่งจริงจาก CARLA ว่า lead อยู่ในเส้นทางข้างหน้าไหม (เชื่อถือได้/ทำซ้ำได้)
#   "yolo" / "both_or" = ตามฉาก cut-in
DETECTION_SOURCE = "groundtruth"

# ทางเดิน (corridor) ของ ego สำหรับเกต ground-truth — วัดในกรอบพิกัด ego
INPATH_HALF_WIDTH = 1.8    # ครึ่งความกว้างเลน (m)
INPATH_MAX_RANGE  = 80.0   # มองไปข้างหน้าไกลสุด (m) — เผื่อ headway สูง

# ── predictive corridor ──
# ฉากนี้รถนำอยู่ในเลนข้างหน้าตรง ๆ ตลอด (ไม่ได้พุ่งเข้าด้านข้าง) จึงปิด predict
# detected เป็น True ตั้งแต่ยังวิ่งตามกัน แต่ ttc≈inf จนกว่ารถนำจะเบรก → เบรกถูกจังหวะ
INPATH_PREDICT   = False
INPATH_LOOKAHEAD = 0.0

# ── การจับคู่เฟรมกล้องกับ snapshot โลก ──
FRAME_SYNC = True

# ── ระยะผิวถึงผิว (surface gap) ──
# ฉากนี้ชนแบบท้ายชนหน้า (rear-end) รถสองคันหันทางเดียวกัน
# จึงหัก (ครึ่งความยาว ego + ครึ่งความยาว lead) ออกจากระยะศูนย์กลาง
AUTO_GAP_OFFSET = True     # True = คำนวณจาก bounding box อัตโนมัติ (ego.extent.x + lead.extent.x)
GAP_OFFSET = 4.5           # ใช้ค่านี้เมื่อ AUTO_GAP_OFFSET=False (ม.)

# ── โมเดลการเบรกของ ego ──
BRAKE_MODEL = "kinematic"

# ── ความลื่นถนน μ ───────────────────────────────────────────────────
MU_DRY = 0.85
MU_WET = 0.40

# ══════════════════════════════════════════════════════════════════
#  เคสเดี่ยว สำหรับ run_single_lead.py (ดีบัก/พรีเซนต์ มีภาพ)
# ══════════════════════════════════════════════════════════════════
SINGLE_CASE = dict(
    ego_speed_kmh  = 50.0,    # ความเร็วรถเรา (v_x)
    lead_speed_kmh = 50.0,    # ความเร็วรถนำก่อนเบรก (ปกติ = ego ตามโจทย์ "เท่ากัน")
    mu             = MU_WET,   # ความลื่นถนน
    headway_thw    = 1.5,     # ระยะห่างเป็นเวลา (วินาที) — ใช้เมื่อ USE_THW=True (50km/h×1.5s≈20.8m)
    headway_d      = 20.0,    # ระยะห่างคงที่ (เมตร) — ใช้เมื่อ USE_THW=False
)
SINGLE_CONTROLLER  = "proposed"   # "baseline" | "proposed"
SINGLE_DELAY_FRAMES = 0           # หน่วงการรับรู้ (เฟรม) 0=ทันที, 16≈0.8s
SHOW_WINDOW = True                # โชว์หน้าต่าง OpenCV ไหม

# ══════════════════════════════════════════════════════════════════
#  TEST MATRIX สำหรับ run_matrix_lead.py (ไล่อัตโนมัติ ไม่มีภาพ)
#  ฉาก lead-brake = 4 speed × 4 headway × 2 μ = 32 เคส/สมองกล
#  (ค่า default: ความเร็วรถนำ = ความเร็ว ego เพราะ LEAD_SAME_AS_EGO=True
#   และวัดระยะห่างเป็น THW เพราะ USE_THW=True → ไล่ headway_thw)
# ══════════════════════════════════════════════════════════════════
MATRIX = dict(
    ego_speed_kmh  = [30.0, 40.0, 50.0, 60.0],   # ความเร็วทั้งคู่ (รถนำ = ego)
    headway_thw    = HEADWAY_THW,                 # ระยะห่างเป็นเวลา (วินาที) — ใช้เมื่อ USE_THW=True
    headway_d      = [10.0, 15.0, 20.0, 25.0],   # ระยะห่างคงที่ (ม.) — ใช้เมื่อ USE_THW=False
    mu             = [MU_DRY, MU_WET],            # แรงเสียดทานถนน
    lead_speed_kmh = [30.0, 50.0],               # ใช้เฉพาะตอน LEAD_SAME_AS_EGO=False
)
# เทียบสมองกลไหนบ้าง + หน่วงเฟรมของแต่ละตัว
#   default ฉาก CCRb นี้เทียบ baseline (TTC คงที่) vs proposed_enhanced (required-decel)
#   เพิ่มบรรทัด proposed (dynamic-TTC) ได้เพื่อเทียบ 3 ทาง
MATRIX_RUNS = [
    dict(label="baseline",          controller="baseline",          delay_frames=0),
    # dict(label="proposed",        controller="proposed",          delay_frames=0),
    dict(label="proposed_enhanced", controller="proposed_enhanced", delay_frames=0),
]
RESULTS_DIR = "results"
RESULTS_PREFIX = "lead_matrix"     # ไฟล์ผล → results/lead_matrix_*.csv (แยกจากฉาก cut-in)

# ══════════════════════════════════════════════════════════════════
#  พารามิเตอร์สมองกล (เหมือนฉาก cut-in เพื่อเทียบกันได้ตรง ๆ)
# ══════════════════════════════════════════════════════════════════
# baseline: TTC คงที่
TTC_WARN_FULL = 1.6
TTC_BRAKE_FULL = 0.6

# proposed: TTC ปรับตามความเร็ว + ความลื่น
DYN_V0      = 40.0
DYN_MU0     = 0.85
DYN_K_SPEED = 1.2
DYN_K_MU    = 1.5
PARTIAL_BRAKE = 0.4

# proposed_enhanced: required-deceleration (รู้ทั้ง μ และการเบรกของรถข้างหน้า)
#   urgency = a_req / (μ·g) ; a_req = v_e² / (2·(gap + v_l²/2a_l))
REQ_FULL_FRAC = 0.9       # urgency ≥ ค่านี้ → เบรกเต็ม (90% ของเพดาน μ·g)
REQ_WARN_FRAC = 0.6       # urgency ≥ ค่านี้ → เบรกบางส่วน (PARTIAL_BRAKE)
