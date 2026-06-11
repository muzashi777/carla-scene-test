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
python run_matrix.py     # ไล่ 32 เคส × สมองกล → results/matrix_*.csv + สรุป
```

**ฉาก 2 — lead-brake (รถนำวิ่งนำแล้วเบรกกระทันหัน)**
```bash
python run_single_lead.py   # ดีบัก 1 เคส มีภาพ — แก้ SINGLE_* ใน config/scenario_lead_brake.py
python run_matrix_lead.py   # ไล่ 32 เคส × สมองกล → results/lead_matrix_*.csv + สรุป
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

**จำนวนเคส:** 4 speed × 4 Δd × 2 μ × 1 dart_speed = **32 เคส/สมองกล** รันทั้ง `baseline` และ `proposed` บนเคสชุดเดียวกัน
รวม **64 รัน** ต่อการเรียก `run_matrix.py` หนึ่งครั้ง

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

**จำนวนเคส:** 4 speed × 4 THW × 2 μ = **32 เคส/สมองกล** รันทั้ง `baseline` และ `proposed` บนเคสชุดเดียวกัน
รวม **64 รัน** ต่อการเรียก `run_matrix_lead.py` หนึ่งครั้ง

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

1. **เปลี่ยนสมองกล** (เชิงอัลกอริทึม): `controller="baseline"` vs `"proposed"`
   proposed ยก TTD threshold ขึ้นเมื่อเร็ว/ลื่น → เบรกล่วงหน้า → รอดในเคสวิกฤต
2. **เปลี่ยนหน่วงเฟรม** (เชิง latency): `delay_frames=16` vs `0`
   เลียนระบบรับรู้ช้า (16f ≈ 0.8s) เทียบกับเร็ว

ค่า default เทียบ baseline vs proposed ที่ delay 0 ทั้งคู่ (โชว์ผลของอัลกอริทึมล้วน)

## สมองกล (controllers) — มี 3 ตัวให้เทียบ

ทุกตัวได้รับ `Perception`/`EgoState` ชุดเดียวกัน (รวมถึง `lead_speed`, `lead_decel` ที่ประมาณจาก
การเคลื่อนที่จริงของรถข้างหน้า) ต่างกันแค่ "ตรรกะการตัดสินใจ" เท่านั้น → เทียบกันได้อย่างยุติธรรม
ทุกตัวส่งคำสั่งเบรกผ่าน `_latch_brake` เดียวกัน (เบรกแล้วค้าง ไม่ปล่อยคืน)

1. **`baseline`** — *static TTC*: เบรกเมื่อ `TTC ≤ TTC_BRAKE_FULL` (threshold คงที่)
2. **`proposed`** — *dynamic TTC*: ยก threshold ขึ้นเมื่อเร็ว/ลื่น (`+K_SPEED·…+K_MU·…`) → เบรกล่วงหน้า
3. **`proposed_enhanced`** — *required-deceleration* (รู้ทั้งแรงเสียดทาน **และ** การเบรกของรถข้างหน้า)

**ทำไมต้องมี `proposed_enhanced`:** TTC ล้วน (`gap / closing_speed`) สมมติรถข้างหน้าวิ่งความเร็วคงที่
แต่ในฉาก CCRb รถข้างหน้า **เบรกจนหยุด** → TTC ยังสูงอยู่จนระยะเหลือน้อยมาก ego เบรกไม่ทันแม้บนถนนแห้ง
ตัวนี้คำนวณ "ความหน่วงที่จำเป็น" แล้วเทียบกับเพดานแรงเสียดทาน:

```
a_max  = μ · g                         เพดานความหน่วงที่ทำได้จริง
d_lead = v_l² / (2·a_l)                ระยะที่รถข้างหน้ายังวิ่งต่อก่อนหยุด (a_l = ความหน่วงรถข้างหน้า)
a_req  = v_e² / (2·(gap + d_lead))     ความหน่วงต่ำสุดที่ ego ต้องใช้เพื่อหยุดทัน
urgency = a_req / a_max
  urgency ≥ REQ_FULL_FRAC (0.9) → เบรกเต็ม
  urgency ≥ REQ_WARN_FRAC (0.6) → เบรกบางส่วน (PARTIAL_BRAKE)
```

เมื่อรถข้างหน้าเริ่มเบรก `a_l` พุ่งขึ้น → `d_lead` หด → `a_req` โตทันที → เบรกถูกจังหวะ (ต่างจาก TTC ล้วน)
และสูตรนี้ generalizes ไปฉาก cut-in อัตโนมัติ: dart จอดขวาง `v_l=0 → d_lead=0 → a_req=v_e²/2gap`
(= เคสสิ่งกีดขวางนิ่ง) เคสที่เข้าเงื่อนไขปลอดภัย (`v_e ≤ v_l`, รถข้างหน้าไม่เบรก, gap=0, ค่า inf) ถูก guard ไว้หมด

**config knob ใหม่:** `REQ_FULL_FRAC` (เบรกเต็มเมื่อ a_req ถึง 90% ของ μ·g), `REQ_WARN_FRAC` (เบรกบางส่วน)
อยู่ทั้งใน `scenario_lead_brake.py` และ `scenario_cutin.py` — เลือกใช้ตัวไหนผ่าน `MATRIX_RUNS`
**คอลัมน์ CSV ใหม่:** `a_req_at_brake` (a_req ตอนเริ่มเบรก) และ `a_max` (= μ·g) → ดูว่าทริกใกล้ขีดถนนแค่ไหน

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
