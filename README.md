# AEB Test Harness (ฉาก 3DGS + UE5.5 → CARLA)

ชุดทดสอบ AEB รองรับ **2 ฉาก** ที่แยกขาดจากกัน (รัน/ดีบักทีละฉากได้):
1. **cut-in / dart-out** — รถพุ่งออกด้านข้างมาจอดขวางเลน
2. **lead-brake** — มีรถจอด/เบรกแล้วอยู่ข้างหน้าในเลนเดียวกัน ego วิ่งเข้ามาด้านหลัง (Euro-NCAP CCRs)

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
  scenario_lead_brake.py     ตรรกะฉาก lead-brake (ego cruise เข้าหารถนำที่จอดนิ่ง)
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

**ฉาก 2 — lead-brake (รถจอด/เบรกแล้วข้างหน้า)**
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
มีรถนำ (lead) ถูก spawn ข้างหน้า ego ในเลนเดียวกัน หันทางเดียวกัน และ **เบรกจอดสนิทแล้ว** (ตรึงนิ่งทุก tick)
ego วิ่งเข้ามาด้านหลัง → ทดสอบ AEB ล้วน ๆ ว่าหยุดทันไหม (Euro-NCAP CCRs)
ตัวแปรเคส: `ego_speed_kmh × mu × headway_d` (รถนำจอดห่างข้างหน้า = `EGO_SPAWN.y − headway_d`)
ระยะผิวถึงผิวหักแบบ "ท้ายชนหน้า" (`ego.extent.x + lead.extent.x`); ปิด predictive (รถนำอยู่ในเลนตรง ๆ อยู่แล้ว)

> ทั้งสองฉากป้อน `Perception`/`EgoState` รูปแบบเดียวกันให้สมองกล → ใช้ controller ชุดเดียวกันได้ไม่ต้องแก้

## กลไกความต่าง 20% — สลับได้ 2 ทาง

ใน `config.MATRIX_RUNS` กำหนดได้ว่าจะเทียบอะไรกับอะไร:

1. **เปลี่ยนสมองกล** (เชิงอัลกอริทึม): `controller="baseline"` vs `"proposed"`
   proposed ยก TTD threshold ขึ้นเมื่อเร็ว/ลื่น → เบรกล่วงหน้า → รอดในเคสวิกฤต
2. **เปลี่ยนหน่วงเฟรม** (เชิง latency): `delay_frames=16` vs `0`
   เลียนระบบรับรู้ช้า (16f ≈ 0.8s) เทียบกับเร็ว

ค่า default เทียบ baseline vs proposed ที่ delay 0 ทั้งคู่ (โชว์ผลของอัลกอริทึมล้วน)

## 5 ดัชนี CPEIM ที่ log (จากเปเปอร์ Sensors 2025)

`s` (clearance), `a_b` (MFDD), `T_c` (warning lead = TTC ตอนเริ่มเบรก),
`Δv` (speed variation), `R_c` (อัตราหลบชนรวมทั้ง matrix)

> หมายเหตุ: ระยะ/TTC ที่ป้อนสมองกลเป็น **ground-truth จาก CARLA** (สะอาด ทำซ้ำได้)
> YOLO ทำหน้าที่ตัวตรวจจับว่ามีรถในเลนเท่านั้น `s` เป็นระยะ center-to-center

## จุดจูนหลัก (ทั้งหมดอยู่ใน config ของแต่ละฉาก)

ร่วมทั้งสองฉาก:
- `MATRIX` — ตัวแปรทดสอบ (cut-in: speed × μ × Δd × dart_speed / lead-brake: speed × μ × headway)
- `DYN_K_SPEED`, `DYN_K_MU`, `DYN_V0`, `DYN_MU0` — ความก้าวร้าวของ proposed
- `TTC_BRAKE_FULL`, `TTC_WARN_FULL` — threshold ของ baseline
- `LANE_LEFT/RIGHT`, `MIN_BOX_H` — แถบเลนพิกเซล + ระยะวิกฤตของ YOLO
- `EGO_SPAWN` — จุดเกิด ego (จากต้นแบบ scene03)

เฉพาะ cut-in (`scenario_cutin.py`): `DART_SPAWN`, `DART_STOP_X`, `INPATH_PREDICT/LOOKAHEAD`
เฉพาะ lead-brake (`scenario_lead_brake.py`): `LEAD_SPAWN` (x/z/yaw/model), `headway_d` ใน `MATRIX`/`SINGLE_CASE`, `RESULTS_PREFIX`

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
