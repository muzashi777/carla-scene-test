# AEB Cut-in Test Harness (ฉาก 3DGS + UE5.5 → CARLA)

ชุดทดสอบ AEB สำหรับฉาก **cut-in / dart-out** (รถพุ่งออกด้านข้างมาจอดขวางเลน)
รันบนฉาก 3DGS ที่นำเข้า UE5.5 (ใช้ collision mesh จาก UE5.5 โดยยังไม่มี `.xodr`/waypoint)
ออกแบบให้ตรงเป้า **成果2**: นำฉากเข้า CARLA ทดสอบสมองกลขับขี่ และยกอัตราหลบชน **+20%**

## โครงสร้าง

```
config/scenario_cutin.py     ★ แก้ตัวแปรทุกอย่างที่นี่ (ฉาก, YOLO, μ, test matrix, สมองกล)
core/
  carla_session.py           เปิด/ปิด sync mode, คืน settings เดิม
  actors.py                  spawn รถ, ตั้งความลื่น μ ที่ล้อ, ติดเซ็นเซอร์
  scenario_cutin.py          ตรรกะฉาก (ego cruise, dart trigger ตามระยะ, จอดขวาง)
  types.py                   Perception / EgoState (ส่งให้สมองกล)
  runner.py                  เครื่องรันเคสเดียว (ผูกทุกอย่าง คืน RunRecord)
  metrics.py                 5 ดัชนี CPEIM + CSV + สรุป Rc
  viz.py                     แสดงภาพ OpenCV (เฉพาะ run_single)
perception/yolo_detector.py  YOLO: บอก "มีรถในเลน ego ใกล้พอไหม"
control/
  base_controller.py         ★ สัญญา interface ของปลั๊กอินสมองกล (throttle/brake/steer)
  baseline_static_ttc.py     สมองกลเก่า: TTC คงที่
  proposed_dynamic_ttc.py    สมองกลใหม่: TTC ปรับตาม speed + μ
run_single.py                รัน 1 เคส + ภาพ (ดีบัก/พรีเซนต์)
run_matrix.py                ไล่ matrix ทั้งหมด + CSV + เช็ก +20%
```

## วิธีรัน

วาง `yolov8n.pt` ไว้โฟลเดอร์เดียวกัน เปิด CARLA (โหลดฉาก 3DGS ของคุณ) แล้ว:

```bash
python run_single.py     # ดีบัก 1 เคส มีภาพ — แก้ SINGLE_* ใน config
python run_matrix.py     # ไล่ 32 เคส × สมองกล → results/matrix_*.csv + สรุป
```

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

## จุดจูนหลัก (ทั้งหมดอยู่ใน config)

- `MATRIX` — ตัวแปรทดสอบ (speed × μ × Δd × dart_speed)
- `DYN_K_SPEED`, `DYN_K_MU`, `DYN_V0`, `DYN_MU0` — ความก้าวร้าวของ proposed
- `TTC_BRAKE_FULL`, `TTC_WARN_FULL` — threshold ของ baseline
- `LANE_LEFT/RIGHT`, `MIN_BOX_H` — แถบเลนพิกเซล + ระยะวิกฤตของ YOLO
- `DART_STOP_X`, `DART_SPAWN`, `EGO_SPAWN` — เรขาคณิตฉาก (จากต้นแบบ scene03)

## เพิ่มสมองกลใหม่ในอนาคต

สร้างไฟล์ใน `control/` สืบทอด `BaseController` ใส่ `@register("ชื่อ")`
แล้วเพิ่มชื่อใน `MATRIX_RUNS` — ไม่ต้องแตะโค้ดฉากหรือ metric เลย
interface รองรับ `steer` แล้ว (ไว้ทำ evasive maneuver)
