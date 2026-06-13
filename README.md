# AEB Test Harness (ฉาก 3DGS + UE5.5 → CARLA)

ชุดทดสอบ AEB รองรับ **2 ฉาก** ที่แยกขาดจากกัน (รัน/ดีบักทีละฉากได้):
1. **cut-in / dart-out** — รถพุ่งออกด้านข้างมาจอดขวางเลน
2. **lead-brake** — รถนำวิ่งอยู่ข้างหน้าในเลนเดียวกัน (เร็วเท่า ego) แล้วเบรกกระทันหัน (Euro-NCAP CCRb)

รันบนฉาก 3DGS ที่นำเข้า UE5.5 (ใช้ collision mesh จาก UE5.5 โดยยังไม่มี `.xodr`/waypoint)
ออกแบบให้ตรงเป้า **成果2**: นำฉากเข้า CARLA ทดสอบสมองกลขับขี่ และยกอัตราหลบชน **+20%**

> ทั้งสองฉากใช้โครงสร้างพื้นฐานร่วมกัน (session, actors, สมองกล, YOLO, metric, viz)
> ต่างกันแค่ config + ตรรกะฉาก + runner + สคริปต์รัน → เพิ่ม/แก้ฉากหนึ่งไม่กระทบอีกฉาก

## โครงสร้าง

```
config/
  scenario_cutin.py          ★ ตัวแปรฉาก cut-in (ฉาก, YOLO, μ, test matrix, สมองกล)
  scenario_lead_brake.py     ★ ตัวแปรฉาก lead-brake (โครงเดียวกัน คนละค่า/คนละ matrix)
core/
  carla_session.py           เปิด/ปิด sync mode, คืน settings เดิม                 [ใช้ร่วม]
  actors.py                  spawn รถ, ตั้งความลื่น μ ที่ล้อ, ติดเซ็นเซอร์          [ใช้ร่วม]
  types.py                   Perception / EgoState (ส่งให้สมองกล)                 [ใช้ร่วม]
  metrics.py                 5 ดัชนี CPEIM + CSV + สรุป Rc                        [ใช้ร่วม]
  viz.py                     แสดงภาพ OpenCV (เฉพาะ run_single*)                   [ใช้ร่วม]
  scenario_cutin.py          ตรรกะฉาก cut-in (ego cruise, dart trigger, จอดขวาง)
  runner.py                  เครื่องรันเคสเดียว ฉาก cut-in (คืน RunRecord)
  scenario_lead_brake.py     ตรรกะฉาก lead-brake (รถนำวิ่งนำ เร็วเท่า ego แล้วเบรกกระทันหัน)
  runner_lead_brake.py       เครื่องรันเคสเดียว ฉาก lead-brake (คืน LeadBrakeRecord)
perception/yolo_detector.py  YOLO: บอก "มีรถในเลน ego ใกล้พอไหม"                   [ใช้ร่วม]
control/
  base_controller.py         ★ สัญญา interface ของปลั๊กอินสมองกล (throttle/brake/steer) [ใช้ร่วม]
  baseline_static_ttc.py     สมองกลเก่า: TTC คงที่                                 [ใช้ร่วม]
  proposed_dynamic_ttc.py    สมองกลใหม่: TTC ปรับตาม speed + μ                     [ใช้ร่วม]
run_single.py                ฉาก cut-in: รัน 1 เคส + ภาพ (ดีบัก/พรีเซนต์)
run_matrix.py                ฉาก cut-in: ไล่ matrix ทั้งหมด + CSV + เช็ก +20%
run_single_lead.py           ฉาก lead-brake: รัน 1 เคส + ภาพ
run_matrix_lead.py           ฉาก lead-brake: ไล่ matrix ทั้งหมด + CSV + เช็ก +20%
```

## วิธีรัน

วาง `yolov8n.pt` ไว้โฟลเดอร์เดียวกัน เปิด CARLA (โหลดฉาก 3DGS ของคุณ) แล้ว:

**ฉาก 1 — cut-in / dart-out**
```bash
python run_single.py     # ดีบัก 1 เคส มีภาพ — แก้ SINGLE_* ใน config/scenario_cutin.py
python run_matrix.py     # ไล่ 32 เคส × 3 สมองกล (baseline/proposed/proposed_enhanced) → results/matrix_*.csv + สรุป
```

**ฉาก 2 — lead-brake (รถนำวิ่งนำแล้วเบรกกระทันหัน)**
```bash
python run_single_lead.py   # ดีบัก 1 เคส มีภาพ — แก้ SINGLE_* ใน config/scenario_lead_brake.py
python run_matrix_lead.py   # ไล่ 32 เคส × 3 สมองกล (baseline/proposed/proposed_enhanced) → results/lead_matrix_*.csv + สรุป
```

> ผล CSV ของฉาก lead-brake ขึ้นต้นด้วย `lead_matrix_` (ตั้งใน `RESULTS_PREFIX`) แยกจากฉาก cut-in (`matrix_`) ชัดเจน

## รายละเอียด 2 ฉาก

**ฉาก 1 — cut-in / dart-out** (`scenario_cutin.py` + `runner.py`)
ego วิ่งตรง รถ dart จอดด้านข้าง พอ ego ห่าง dart ≤ `trigger_d` → dart พุ่งออกตั้งฉากมาจอดขวางเลน
ตัวแปรเคส: `ego_speed_kmh × mu × trigger_d × dart_speed_kmh`
ระยะผิวถึงผิวหักแบบ "หน้า ego ชนข้าง dart" (`ego.extent.x + dart.extent.y`); เปิด predictive corridor (เห็น dart ตั้งแต่กำลังเข้าเลน)

**ฉาก 2 — lead-brake** (`scenario_lead_brake.py` + `runner_lead_brake.py`)
มีรถนำ (lead) ถูก spawn ข้างหน้า ego ในเลนเดียวกัน หันทางเดียวกัน **วิ่งนำด้วยความเร็วเท่า ego**
(รักษา headway คงที่) พอวิ่งครบระยะ `LEAD_BRAKE_AFTER_M` → **เบรกกระทันหัน** (หน่วง `LEAD_DECEL`) จนหยุดนิ่ง
ระยะระหว่างรถจึงหดเร็ว → ทดสอบว่า AEB ของ ego เบรกทันไหม (Euro-NCAP CCRb)
ตัวแปรเคส: `ego_speed_kmh × headway_d × mu` (รถนำเร็วเท่า ego เมื่อ `LEAD_SAME_AS_EGO=True`;
ตั้ง `False` เพื่อให้ `lead_speed_kmh` เป็นตัวแปรแยก) — spawn รถนำที่ `EGO_SPAWN.y − headway_d`
ระยะผิวถึงผิวหักแบบ "ท้ายชนหน้า" (`ego.extent.x + lead.extent.x`); ปิด predictive (รถนำอยู่ในเลนตรง ๆ อยู่แล้ว)

> ทั้งสองฉากป้อน `Perception`/`EgoState` รูปแบบเดียวกันให้สมองกล → ใช้ controller ชุดเดียวกันได้ไม่ต้องแก้

## Test cases — ฉาก cut-in / dart-out

**สถานการณ์โดยย่อ:** ego วิ่งตรงด้วยความเร็วคงที่ มีรถ (dart) จอดด้านข้าง พอ ego เข้าใกล้จนระยะ ego–dart ≤ `trigger_d`
รถ dart **พุ่งออกตั้งฉาก** เข้ามาในเลน ego แล้วจอดขวาง (ที่ `DART_STOP_X`) → วัดว่า AEB ของ ego เบรกหลบชนได้ทันไหม

**ตัวแปรที่ไล่ทดสอบ (sweep):**

| ตัวแปร | ค่า | หมายเหตุ |
|---|---|---|
| `ego_speed_kmh` | 30, 40, 50, 60 | ความเร็ว ego |
| `trigger_d` (Δd) | 20, 25, 30, 35 m | ระยะ ego–dart ที่ทำให้ dart เริ่มพุ่งออก (ตัด 10/15 m ที่หินเกินจนไม่มีใครรอด) |
| `mu` | 0.85 (แห้ง), 0.40 (เปียก) | แรงเสียดทานถนน |
| `dart_speed_kmh` | 20 | ความเร็ว dart ขณะพุ่งออก (คงที่ — ขยายเป็น [20, 40] ได้) |

**จำนวนเคส:** 4 speed × 4 Δd × 2 μ × 1 dart_speed = **32 เคส/สมองกล** รันทั้ง `baseline`, `proposed`
และ `proposed_enhanced` บนเคสชุดเดียวกัน รวม **96 รัน** (32 × 3) ต่อการเรียก `run_matrix.py` หนึ่งครั้ง

**พารามิเตอร์คงที่ (ไม่ได้ sweep):**
- `DART_SPAWN`, `DART_STOP_X` — เรขาคณิตจุดเกิด/จุดจอดขวางของ dart (จากต้นแบบ scene03)
- `INPATH_PREDICT = True`, `INPATH_LOOKAHEAD = 1.5` s — เห็น dart ตั้งแต่กำลังเข้าเลน (predictive corridor)
- `BRAKE_MODEL = "kinematic"`, `LANE_LEFT/RIGHT`, `MIN_BOX_H`, สมองกล (`TTC_*`, `DYN_*`) — เหมือนฉาก lead-brake

**คอลัมน์ CSV หลัก (`results/matrix_*.csv`):** เหมือนฉาก lead-brake แต่คอลัมน์ระยะห่างเป็น
`trigger_d` (Δd, m) และ `dart_speed_kmh` แทน `headway_thw`/`headway_d` ส่วนคอลัมน์เมตริก
(`avoided`, `s_clearance`, `a_b_mfdd`, `t_c_warn`, `dv_speed_var`, `peak_decel`, `result_txt`, …) เหมือนกันทุกตัว

## Test cases — ฉาก lead-brake (CCRb)

**สถานการณ์โดยย่อ:** รถนำวิ่งอยู่ข้างหน้า ego ในเลนเดียวกันด้วยความเร็วเท่ากัน (รักษาระยะห่างคงที่)
พอวิ่งครบระยะ `LEAD_BRAKE_AFTER_M` รถนำ **เบรกกระทันหัน** ที่ความหน่วงคงที่ `LEAD_DECEL` จนหยุดนิ่ง
ระยะระหว่างรถจึงหดเร็ว → วัดว่า AEB ของ ego เบรกหลบชนได้ทันไหม (Euro-NCAP CCRb)

**ตัวแปรที่ไล่ทดสอบ (sweep):**

| ตัวแปร | ค่า | หมายเหตุ |
|---|---|---|
| `ego_speed_kmh` | 30, 40, 50, 60 | ความเร็วทั้ง ego และรถนำ (เท่ากัน เพราะ `LEAD_SAME_AS_EGO=True`) |
| `HEADWAY_THW` | 1.0, 1.5, 2.0, 2.5 วินาที | ระยะห่างเป็น **เวลา** (THW) — ระยะจริง = ego_speed(m/s) × THW |
| `mu` | 0.85 (แห้ง), 0.40 (เปียก) | แรงเสียดทานถนน |

**จำนวนเคส:** 4 speed × 4 THW × 2 μ = **32 เคส/สมองกล** รันทั้ง `baseline`, `proposed` และ `proposed_enhanced`
บนเคสชุดเดียวกัน รวม **96 รัน** (32 × 3) ต่อการเรียก `run_matrix_lead.py` หนึ่งครั้ง

**ตาราง THW → ระยะจริง (เมตร) ตามความเร็ว:**

| speed | 1.0 s | 1.5 s | 2.0 s | 2.5 s |
|---|---|---|---|---|
| 30 km/h |  8.3 m | 12.5 m | 16.7 m | 20.8 m |
| 40 km/h | 11.1 m | 16.7 m | 22.2 m | 27.8 m |
| 50 km/h | 13.9 m | **20.8 m** | 27.8 m | 34.7 m |
| 60 km/h | 16.7 m | 25.0 m | 33.3 m | 41.7 m |

(ตรวจสอบ: 50 km/h = 13.89 m/s × 1.5 s ≈ **20.8 m**) ทุกระยะ < `INPATH_MAX_RANGE` (80 m) จึงอยู่ในระยะที่เกตตรวจจับเห็น

**พารามิเตอร์คงที่ (ไม่ได้ sweep):**
- `LEAD_DECEL = 4.0` m/s² — ความหน่วงรถนำตอนเบรก (มาตรฐาน CCRb)
- `LEAD_BRAKE_AFTER_M = 8.0` m — รถนำวิ่งนำได้ไกลเท่านี้ก่อนเบรก
- `LEAD_SAME_AS_EGO = True` — รถนำเร็วเท่า ego (ตั้ง `False` เพื่อให้ `lead_speed_kmh` เป็นตัวแปรแยก)
- `BRAKE_MODEL = "kinematic"`, `LANE_LEFT/RIGHT`, `MIN_BOX_H`, สมองกล (`TTC_*`, `DYN_*`) — เหมือนฉาก cut-in

**สลับกลับเป็นโหมดระยะคงที่:** ตั้ง `USE_THW = False` ใน `config/scenario_lead_brake.py`
ระบบจะกลับไปไล่ `MATRIX["headway_d"]` (เมตร) แทน และคอลัมน์ `headway_thw` จะเป็น 0

**คอลัมน์ CSV หลัก (`results/lead_matrix_*.csv`):**

| คอลัมน์ | ความหมาย |
|---|---|
| `label`, `controller`, `delay_frames` | สมองกลที่ใช้ + เฟรมหน่วงการรับรู้ |
| `ego_speed_kmh`, `lead_speed_kmh`, `mu` | ความเร็ว ego / รถนำ และแรงเสียดทานของเคส |
| `headway_thw` | ระยะห่างที่ตั้งเป็นเวลา (วินาที) — 0 ถ้าใช้โหมดระยะคงที่ |
| `headway_d` | ระยะห่างจริง (เมตร) ที่คำนวณได้ (THW×speed หรือค่าคงที่) |
| `avoided`, `collision_with`, `collision_speed_kmh` | หลบชนสำเร็จไหม / ชนกับอะไร / ความเร็วตอนชน |
| `s_clearance` | ระยะเหลือตอนหยุดสนิท (m) |
| `a_b_mfdd`, `peak_decel`, `brake_distance` | ความหน่วงเฉลี่ย MFDD / ความหน่วงสูงสุด / ระยะเบรกจนหยุด |
| `t_c_warn` | TTC ตอนเริ่มเบรก (warning lead time, s) |
| `dv_speed_var` | ความเร็วที่หายไป (km/h) |
| `a_req_at_brake` | ความหน่วงที่ "จำเป็น" ตอนเริ่มเบรก (m/s²) — ดูว่าทริกใกล้ขีดจำกัดถนนแค่ไหน |
| `a_max` | เพดานความหน่วง = μ·g ของเคสนี้ (m/s²); ถ้า `a_req_at_brake` ใกล้ `a_max` = เบรกแทบไม่ทัน |
| `min_dist`, `result_txt` | ระยะศูนย์กลางใกล้สุด / ข้อความสรุปผล |

## กลไกความต่าง 20% — สลับได้ 2 ทาง

ใน `config.MATRIX_RUNS` กำหนดได้ว่าจะเทียบอะไรกับอะไร:

1. **เปลี่ยนสมองกล** (เชิงอัลกอริทึม): `controller="baseline"` vs `"proposed"` vs `"proposed_enhanced"`
   proposed ยก TTC threshold ขึ้นเมื่อเร็ว/ลื่น → เบรกล่วงหน้า → รอดในเคสวิกฤต
   proposed_enhanced ใช้ required-deceleration (รู้การเบรกของรถข้างหน้า) → ทริกเบรกถูกจังหวะในฉาก CCRb
2. **เปลี่ยนหน่วงเฟรม** (เชิง latency): `delay_frames=16` vs `0`
   เลียนระบบรับรู้ช้า (16f ≈ 0.8s) เทียบกับเร็ว

**ค่า default ฉาก lead-brake** เทียบ 3 สมองกลพร้อมกัน (`baseline`, `proposed`, `proposed_enhanced`)
ที่ delay 0 ทั้งหมด (โชว์ผลของอัลกอริทึมล้วน) — `run_matrix_lead.py` จะรายงาน Δ Rc ของ
`proposed` และ `proposed_enhanced` เทียบกับ `baseline` ทีละตัว แล้วเช็กเป้า +20% (成果2) ให้แต่ละตัว

## Algorithm — การคำนวณต่อเคส (per-case pipeline)

แต่ละเคสในเมทริกซ์รันด้วยลูป sync mode (`FIXED_DT = 0.05`s = 20 FPS) ใน `core/runner_lead_brake.py`
ทุก tick คำนวณค่าด้านล่างจาก **ground-truth ของ CARLA** (ไม่ใช่ค่าจากภาพ) แล้วป้อนให้สมองกลตัดสินใจ
สูตรเหล่านี้คือ "อินพุตร่วม" ที่สมองกลทั้ง 3 ตัวเห็นเหมือนกัน → ความต่างมาจากตรรกะตัดสินใจล้วน ๆ

**1) ตั้งระยะห่างเริ่มต้นของเคส (THW → เมตร)** — `runner_lead_brake.run_case`
```
v_e   = ego_speed_kmh / 3.6                 ความเร็ว ego (m/s)
หาก USE_THW=True:  headway_d = v_e · THW    ระยะห่างจริง (m) โตตามความเร็ว (สมจริงกว่าระยะคงที่)
หาก USE_THW=False: headway_d = ค่าคงที่ (m) (โหมดเดิม)
spawn รถนำที่  y = EGO_SPAWN.y − headway_d  (ego วิ่งไปทาง −Y รถนำจึงอยู่ข้างหน้า)
```
*ปัญหาที่แก้:* ระยะห่างคงที่ (เมตร) ไม่ยุติธรรมข้ามความเร็ว — 20 m ที่ 30 km/h ปลอดภัย แต่ที่ 60 km/h
อันตราย จึงนิยามด้วย **เวลา (THW)** ตามมาตรฐานวิศวกรรมจราจร → ทุกความเร็วได้ "ระยะเป็นวินาที" เท่ากัน

**2) ระยะผิวถึงผิว (surface gap)** — หักขนาดตัวรถออกจากระยะศูนย์กลาง
```
gap = max(0, dist_center − (ego.extent.x + lead.extent.x))   ฉาก CCRb ท้ายชนหน้า หันทางเดียวกัน
```
*ปัญหาที่แก้:* ระยะศูนย์กลาง-ถึง-ศูนย์กลางทำให้ "ชน" ช้ากว่าจริง (รถยังไม่ทันแตะกันก็คิดว่าห่าง)

**3) ความเร็วสัมพัทธ์ + TTC** — finite difference ต่อ tick
```
rel_speed = max(0, (gap_prev − gap) / FIXED_DT)             อัตราที่ช่องว่างหดลง (m/s)
TTC       = gap / rel_speed   (= ∞ ถ้า rel_speed ≈ 0)        เวลาถึงชนถ้าทุกอย่างคงที่
```

**4) ความเร็ว/ความหน่วงของรถนำ (ground-truth + EMA)** — ใช้โดย `proposed_enhanced`
```
raw_decel      = max(0, (lead_v_prev − lead_v) / FIXED_DT)
lead_decel_ema = 0.3·raw_decel + 0.7·lead_decel_ema          กรอง noise ของ finite-difference
```
*ปัญหาที่แก้:* ไม่อ่าน `LEAD_DECEL` จาก config ตรง ๆ แต่ **ประมาณจากการเคลื่อนที่จริง** → ทำซ้ำได้และ
สมจริง (เหมือนเซ็นเซอร์จริงที่ต้องประมาณเอง) EMA กันค่ากระโดดจากการหาร finite-difference

**5) แบบจำลองเบรกของ ego (kinematic, friction-limited)** — แปลงคำสั่งเบรกเป็นความเร็วใหม่
โดย **จำกัดความหน่วงที่ทำได้จริงไว้ที่เพดานแรงเสียดทาน** `a_max = μ·g` (ดูหัวข้อ
"Physics cap on braking deceleration" ด้านล่าง) — โค้ดอยู่ใน `core/actors.apply_kinematic_brake`
```
a_max     = μ · g                            เพดานความหน่วงที่ถนนให้ได้จริง
a_applied = min(brake · a_max, a_max)         hard clamp: ความหน่วงจริงไม่เกิน μ·g
v_new     = max(0, v_model − a_applied · FIXED_DT)   อัปเดตจาก v_model (ไม่ใช่ค่าอ่านกลับ CARLA)
```
*ปัญหาที่แก้:* ใช้โมเดล kinematic แทนฟิสิกส์ล้อของ CARLA → ตัดตัวแปรกวน (ล้อลื่น/ระบบกันสะเทือน) ออก
ให้ผลทำซ้ำได้และเทียบสมองกลได้ตรง ๆ ว่าต่างกันเพราะ "ตัดสินใจเบรกเมื่อไร" ไม่ใช่เพราะพลวัตตัวรถ
อัปเดตความเร็วจาก `v_model` ที่จับไว้ตอนเริ่มเบรก (ไม่ใช่ค่าอ่านกลับของ CARLA ที่อาจถูกฟิสิกส์ภายใน
หน่วงเพิ่ม) → ความหน่วงต่อ tick = `a_applied ≤ a_max` เป๊ะ และค่า `peak_decel` ที่ log = ความหน่วงจริงหลัง clamp

## สมองกล (controllers) — มี 3 ตัวให้เทียบ

ทุกตัวได้รับ `Perception`/`EgoState` ชุดเดียวกัน (รวมถึง `lead_speed`, `lead_decel` ที่ประมาณจาก
การเคลื่อนที่จริงของรถข้างหน้า) ต่างกันแค่ "ตรรกะการตัดสินใจ" เท่านั้น → เทียบกันได้อย่างยุติธรรม
ทุกตัวส่งคำสั่งเบรกผ่าน `_latch_brake` เดียวกัน (เบรกแล้วค้าง ไม่ปล่อยคืน) และเริ่มตัดสินใจเฉพาะเมื่อ
"ตรวจเจอรถนำ" (`perc.detected`) หรือเคยเริ่มเบรกไปแล้ว (latch) → เกตตัดสินใจเหมือนกันทั้ง 3 ตัว

### 1) `baseline` — static TTC (threshold คงที่)
```
TTC ≤ TTC_BRAKE_FULL (0.6s) → เบรกเต็ม (1.0)
TTC ≤ TTC_WARN_FULL  (1.6s) → เบรกบางส่วน (PARTIAL_BRAKE = 0.4)
นอกนั้น → ไม่เบรก
```
*ปัญหา/จุดอ่อน:* threshold คงที่ไม่รู้จักความเร็วหรือความลื่น → ในเคสเร็ว+ถนนเปียก (μ ต่ำ) ระยะหยุดยาว
กว่าที่ TTC=0.6s จะให้ → **เบรกไม่ทัน** นี่คือเส้นฐานที่อีกสองตัวต้องเอาชนะให้ได้ ≥ +20% Rc

### 2) `proposed` — dynamic TTC (ยก threshold ตามความเร็ว+ความลื่น)
```
bump     = K_SPEED · max(0, (v_kmh − V0)/100) + K_MU · max(0, (MU0 − μ))
thr_full = TTC_BRAKE_FULL + bump            เร็วขึ้น/ลื่นขึ้น → threshold สูงขึ้น → เบรกล่วงหน้า
thr_warn = max(TTC_WARN_FULL, thr_full + 0.5)
TTC ≤ thr_full → เบรกเต็ม ; TTC ≤ thr_warn → เบรกบางส่วน
```
(ค่า: `V0=40`, `MU0=0.85`, `K_SPEED=1.2`, `K_MU=1.5`)
*ปัญหาที่แก้:* แก้จุดอ่อนของ baseline — พอเร็วหรือลื่น ระยะหยุดยาวขึ้น สมองกลจึงเลื่อนจุดเบรกให้เร็วขึ้น
อัตโนมัติ (Master Plan แนะนำ "ถ้า speed>50 และ μ=0.40 → ใช้ TTC=1.2") ทำเป็นต่อเนื่องเพื่อจูนง่าย
*ข้อจำกัดที่ยังเหลือ:* ยังอิง TTC ซึ่งสมมติรถข้างหน้าวิ่งความเร็วคงที่ → ในฉาก CCRb ยังพลาดได้ (ดูตัวถัดไป)

### 3) `proposed_enhanced` — required-deceleration (รู้ทั้ง μ และการเบรกของรถข้างหน้า)
**ปัญหาที่แก้:** TTC ล้วน (`gap / closing_speed`) สมมติรถข้างหน้าวิ่งความเร็วคงที่ แต่ในฉาก CCRb
รถข้างหน้า **เบรกจนหยุด** → ขณะรถนำกำลังชะลอ closing_speed ยังต่ำ ทำให้ TTC ยังสูงจนระยะเหลือน้อยมาก
→ baseline/proposed เบรกไม่ทันแม้บนถนนแห้ง ตัวนี้เลิกพึ่ง TTC แล้วถามตรง ๆ ว่า "ต้องหน่วงเท่าไรถึงหยุดทัน":
```
a_max  = μ · g                         เพดานความหน่วงที่ถนนให้ได้จริง
d_lead = v_l² / (2·a_l)                ระยะที่รถข้างหน้ายังวิ่งต่อก่อนหยุด (a_l = lead_decel_ema)
a_req  = v_e² / (2·(gap + d_lead))     ความหน่วงต่ำสุดที่ ego ต้องใช้เพื่อหยุดทันท้ายรถนำ
urgency = a_req / a_max
  urgency ≥ REQ_FULL_FRAC (0.9) → เบรกเต็ม (1.0)
  urgency ≥ REQ_WARN_FRAC (0.6) → เบรกบางส่วน (PARTIAL_BRAKE)
```
เมื่อรถข้างหน้าเริ่มเบรก `a_l` พุ่งขึ้น → `d_lead` หด → `a_req` โตทันที → เบรกถูกจังหวะ (ต่างจาก TTC ล้วน)
และสูตรนี้ generalizes ไปฉาก cut-in อัตโนมัติ: dart จอดขวาง `v_l=0 → d_lead=0 → a_req=v_e²/2gap`
(= เคสสิ่งกีดขวางนิ่ง) เคส guard ไว้หมด (`required_decel` ใน `core/actors.py`): `v_e ≤ v_l` → 0,
รถข้างหน้าไม่เบรก (`a_l≈0, v_l>0`) → `d_lead=∞` (ยังไม่ฉุกเฉิน), ระยะหยุดที่มี ≈ 0 → `∞` (สายเกินไป)

**ประเมินบน 2 ฉาก:** ตอนนี้ `proposed_enhanced` รันใน `MATRIX_RUNS` ของ **ทั้งฉาก cut-in และ lead-brake**
(เทียบ 3 ทางกับ `baseline`/`proposed` บนเคสชุดเดียวกัน) ในฉาก cut-in สูตรลดรูปเป็น **เคสสิ่งกีดขวางนิ่ง**
(`v_l=0 → d_lead=0 → a_req = v_e²/2·gap`) จึงใช้ได้โดยไม่ต้องแก้ตรรกะ และใช้ **ชั้น perception ร่วม**
(`Perception`/`EgoState` ชุดเดียวกัน) กับ **ทางเบรกที่ถูก clamp** (`apply_kinematic_brake`, `peak_decel ≤ μ·g`)
เหมือน `baseline`/`proposed` ทุกประการ → เทียบกันได้อย่างยุติธรรมทั้งสองฉาก

**config knob ใหม่:** `REQ_FULL_FRAC` (เบรกเต็มเมื่อ a_req ถึง 90% ของ μ·g), `REQ_WARN_FRAC` (เบรกบางส่วน)
อยู่ทั้งใน `scenario_lead_brake.py` และ `scenario_cutin.py` — เลือกใช้ตัวไหนผ่าน `MATRIX_RUNS`
**คอลัมน์ CSV ใหม่:** `a_req_at_brake` (a_req ตอนเริ่มเบรก) และ `a_max` (= μ·g) → ดูว่าทริกใกล้ขีดถนนแค่ไหน

## Physics cap on braking deceleration (เพดานความหน่วง a_max = μ·g)

**ทำไมต้องมี:** ความหน่วงที่รถทำได้จริงถูกจำกัดด้วยแรงเสียดทานระหว่างยางกับถนน จึงต้องไม่เกิน
`a_max = μ·g` (เป็นสมมติฐานหลักของโมเดลเบรก kinematic ที่ใช้ทั้งระบบ ตามเปเปอร์ Section III-C)
รถจะเบรกแรงเกินที่ผิวถนนยอมให้ไม่ได้ — ถ้าปล่อยให้เกิน = ประเมินสมรรถนะการหลบชนสูงเกินจริง

**เดิมผิดตรงไหน:** ขั้น kinematic เดิมอัปเดตความเร็วใหม่จาก *ค่าอ่านกลับความเร็วของ CARLA* ทุก tick
(`new_v = speed_ms(ego) − brake·μ·g·dt`) ทำให้ความหน่วงที่ฟิสิกส์ภายในของเอนจินเพิ่มเข้ามา (drag/contact)
ทบเข้าไปสะสม → รถหยุดเร็วกว่าที่แรงเสียดทานยอมให้ ในรันก่อนแก้ (`results/lead_matrix_20260613_104314.csv`)
มี **20 แถวที่ `peak_decel` เกิน `a_max = μ·g`**:
- ถนนเปียก (μ=0.40): `a_max=3.92 m/s²` แต่ `peak_decel` พุ่งถึง ~5.1–5.7 m/s²
- ถนนแห้ง (μ=0.85): `a_max=8.34 m/s²` แต่ `peak_decel` พุ่งถึง ~9.3 m/s²

**แก้อะไร:** เพิ่ม hard clamp ต่อ timestep ที่ฟังก์ชันเบรกร่วม `core/actors.apply_kinematic_brake`
```
a_max     = μ · g
a_applied = min(brake · a_max, a_max)              ความหน่วงจริงไม่เกินเพดานแรงเสียดทาน
v_new     = max(0, v_model − a_applied · dt)        อัปเดตจาก v_model (จับไว้ตอนเริ่มเบรก)
```
- ใช้ผ่าน **ทางเบรกร่วม (shared path)** ทั้ง `core/runner.py` (cut-in) และ `core/runner_lead_brake.py`
  (lead-brake) → สมองกลทั้ง 3 ตัว (`baseline`, `proposed`, `proposed_enhanced`) ถูก clamp **เท่ากันทุกตัว**
- **ไม่แตะตรรกะตัดสินใจ** ของสมองกลใด ๆ (`_desired`/`decide`, threshold ทั้งหมดคงเดิม) — แก้เฉพาะชั้นฟิสิกส์
- อัปเดตความเร็วจาก `v_model` (ไม่ใช่ค่าอ่านกลับ CARLA) → ความหน่วงต่อ tick = `a_applied ≤ a_max` เป๊ะ
  กันฟิสิกส์ภายในเอนจินหน่วงเกินเพดาน
- `peak_decel` ที่เขียนลง CSV ตอนนี้คือ **ความหน่วงจริงหลัง clamp** → ทุกแถวจะมี `peak_decel ≤ a_max`
  (โครงสร้าง/ชื่อคอลัมน์ CSV ไม่เปลี่ยน)

**ผลลัพธ์เก่าถูกแทนที่:** ไฟล์ CSV ที่สร้างก่อนการแก้นี้ — โดยเฉพาะ `results/lead_matrix_20260613_104314.csv`
— ถือว่าใช้ไม่ได้แล้ว (superseded) ต้องรัน `run_matrix_lead.py` ใหม่เพื่อให้ได้ผลที่สอดคล้องทางฟิสิกส์

## บั๊ก a_req ของ proposed_enhanced ในฉาก cut-in (แก้แล้ว)

**อาการ:** ในฉาก cut-in `proposed_enhanced` **ชนในเคสถนนแห้งง่าย ๆ ที่ควรหลบได้** (dry 5/16 เทียบกับ
baseline/proposed ที่ 10/16) — เห็นชัดใน `results/matrix_20260613_125341.csv`: ค่า `a_req_at_brake`
เพี้ยน (ระเบิดถึง 459, 134 m/s² ทั้งที่ `a_max=8.34` บางเคส, เป็น 0.0 บางเคส) และ `t_c_warn` ≈ 0 (เบรกช้าไป)
ส่วนฉาก lead-brake สมองกลตัวนี้ทำงานดีอยู่แล้ว → บั๊กเฉพาะตอนเป้า "โผล่ระยะใกล้/พุ่งตัดเข้าด้านข้าง"

**สาเหตุ:** `a_req = v_e² / (2·(gap + d_lead))` โดย `d_lead = v_l²/2a_l`
รถ dart พุ่งออก **ตั้งฉาก** กับแนววิ่งของ ego — เดิม runner ป้อน `v_l = speed_ms(dart)` ซึ่งเป็น
*ความเร็วด้านข้างเต็ม ๆ* (~5.5 m/s) ไม่ใช่ความเร็วตามแนวการชน ทำให้ระหว่าง dart กำลังพุ่ง
(`v_l>0`, `a_l≈0`) → `d_lead = ∞` → `a_req = 0` → ไม่เบรก พอ dart จอด (`gap→0`) → `a_req` ระเบิด
→ เบรกช้าเกินไป → ชน (ส่วนเคสที่ `a_req` เป็น 0 ตลอด = ไม่เคยเบรกเลย จึงชน) การลดรูปเป็น
"สิ่งกีดขวางนิ่ง" (`v_l=0 → d_lead=0 → a_req=v_e²/2·gap`) ที่ตั้งใจไว้จึงไม่เกิด

**แก้อะไร (logic เท่านั้น — ไม่แตะ baseline/proposed, perception, scenario, matrix, braking cap, CSV schema):**
- **ระยะ (gap) สอดคล้องกับสมองกล TTC:** ป้อน `perc.distance` (surface gap ชุดเดียวกับที่ baseline/proposed
  ใช้) เข้า `required_decel` → ทั้ง 3 สมองกลเห็นระยะตามแนวเลนชุดเดียวกัน (ยุติธรรม/สอดคล้อง)
- **ความเร็วรถข้างหน้าใช้องค์ประกอบตามแนววิ่ง ego (longitudinal):** `core/actors.long_speed_along(ego, dart)`
  ใน `core/runner.py` — dart ที่พุ่งตัดข้างมีองค์ประกอบนี้ ≈ 0 → ถูกมองเป็น **สิ่งกีดขวางนิ่ง** อย่างถูกต้อง
  → `a_req = v_e²/2·gap` เบรกได้ทันเวลาเหมือนสมองกล TTC (ฉาก lead-brake รถนำวิ่งแนวเดียวกับ ego
  ค่านี้ = ความเร็วเต็มอยู่แล้ว จึงไม่เปลี่ยนพฤติกรรม → นิยามตรงกันทั้งสองฉาก)
- **เกตป้องกันใน `required_decel`** (`core/actors.py`): ถ้า `gap ≤ REQ_GAP_EPS (0.05 m)` → คืน
  `REQ_AREQ_CAP` = ขอเบรกเต็ม (แทนค่าระเบิด/ศูนย์); ถือว่าเป้า "นิ่ง" เมื่อ `v_l ≤ REQ_VL_STILL (0.5 m/s)`
  → `d_lead=0` (ลดรูปเป็นสิ่งกีดขวางนิ่ง, ไม่หารศูนย์); และ **clamp `a_req ≤ REQ_AREQ_CAP (50 m/s²)`**
  ไม่คืน `inf`/`NaN` อีก → ค่าที่ log เสถียร
- **สมองกล** (`control/proposed_enhanced.py`): คำนวณ `a_req` เฉพาะเมื่อ "ตรวจเจอเป้า" (เกตเดียวกับ proposed)
  และเพิ่มกรณี `gap ≤ REQ_GAP_EPS → เบรกเต็มทันที` ตรรกะ threshold ที่เหลือคงเดิม

**ผลลัพธ์เก่าถูกแทนที่:** CSV ฉาก cut-in ที่มี `proposed_enhanced` ซึ่งสร้างก่อนการแก้นี้ — โดยเฉพาะ
`results/matrix_20260613_125341.csv` — ถือว่าใช้ไม่ได้แล้ว (superseded) ต้องรัน `run_matrix.py` ใหม่

## 5 ดัชนี CPEIM ที่ log (จากเปเปอร์ Sensors 2025)

`s` (clearance), `a_b` (MFDD), `T_c` (warning lead = TTC ตอนเริ่มเบรก),
`Δv` (speed variation), `R_c` (อัตราหลบชนรวมทั้ง matrix)

> หมายเหตุ: ระยะ/TTC ที่ป้อนสมองกลเป็น **ground-truth จาก CARLA** (สะอาด ทำซ้ำได้)
> YOLO ทำหน้าที่ตัวตรวจจับว่ามีรถในเลนเท่านั้น `s` เป็นระยะ center-to-center

## จุดจูนหลัก (ทั้งหมดอยู่ใน config ของแต่ละฉาก)

ร่วมทั้งสองฉาก:
- `MATRIX` — ตัวแปรทดสอบ (cut-in: speed × μ × Δd × dart_speed / lead-brake: speed × THW × μ)
- `DYN_K_SPEED`, `DYN_K_MU`, `DYN_V0`, `DYN_MU0` — ความก้าวร้าวของ proposed
- `TTC_BRAKE_FULL`, `TTC_WARN_FULL` — threshold ของ baseline
- `LANE_LEFT/RIGHT`, `MIN_BOX_H` — แถบเลนพิกเซล + ระยะวิกฤตของ YOLO
- `EGO_SPAWN` — จุดเกิด ego (จากต้นแบบ scene03)

เฉพาะ cut-in (`scenario_cutin.py`): `DART_SPAWN`, `DART_STOP_X`, `INPATH_PREDICT/LOOKAHEAD`
เฉพาะ lead-brake (`scenario_lead_brake.py`): `LEAD_SPAWN` (x/z/yaw/model), `LEAD_BRAKE_AFTER_M`, `LEAD_DECEL`, `LEAD_SAME_AS_EGO`, `USE_THW` + `HEADWAY_THW` (ระยะห่างเป็นวินาที) หรือ `headway_d` (เมตร, โหมดเดิม), `RESULTS_PREFIX`

## เพิ่มสมองกลใหม่ในอนาคต

สร้างไฟล์ใน `control/` สืบทอด `BaseController` ใส่ `@register("ชื่อ")`
แล้วเพิ่มชื่อใน `MATRIX_RUNS` — ไม่ต้องแตะโค้ดฉากหรือ metric เลย ใช้ได้ทั้งสองฉาก
interface รองรับ `steer` แล้ว (ไว้ทำ evasive maneuver)

## เพิ่มฉากใหม่ในอนาคต (แพตเทิร์นเดียวกับ lead-brake)

1. `config/scenario_<ชื่อ>.py` — คัดลอกค่าฐาน เปลี่ยน spawn/ตัวแปรเคส/`MATRIX`/`RESULTS_PREFIX`
2. `core/scenario_<ชื่อ>.py` — คลาสฉากที่มี `start()` / `update()→just_event` / `cruise_ego()`
3. `core/runner_<ชื่อ>.py` — คัดลอกจาก runner ที่ใกล้สุด เปลี่ยน scenario ที่ import, ตัวแปรเคส,
   สูตร `gap_offset`, และ dataclass record (ชื่อฟิลด์เมตริกต้องตรงที่ `metrics.summarize` ใช้)
4. `run_single_<ชื่อ>.py` / `run_matrix_<ชื่อ>.py` — คัดลอกแล้วชี้ไป config + runner ใหม่

โครงสร้างพื้นฐาน (session, actors, types, metrics, viz, YOLO, control) ใช้ร่วมได้หมด ไม่ต้องแตะ
