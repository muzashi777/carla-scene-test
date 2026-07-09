# ขั้นตอนการตรวจจับและการเริ่มเบรก (Detection & Braking Pipeline)

---

## คำตอบสั้น ๆ

**YOLO ไม่ใช่สาเหตุที่ทำให้เบรก**

ในทุกรันที่ใช้สำหรับผลในเปเปอร์ (`DETECTION_SOURCE = "groundtruth"`) YOLO ทำหน้าที่แค่ **วาดภาพบนจอเพื่อดีบัก** เท่านั้น
สิ่งที่กำหนดว่า "สมองกลเริ่มทำงานได้หรือยัง" คือ **ตำแหน่งจริงจาก CARLA** และ
ค่า gap / TTC ที่ส่งให้สมองกลตัดสินใจปริมาณแรงเบรกก็มาจาก **ground-truth** เช่นกัน

---

## ภาพรวมสิ่งที่เกิดขึ้นทุก tick

```
world.tick()                       ← ก้าวซิม 0.05 s (20 FPS)
     │
     ├─ อ่านภาพกล้องหน้า
     │
     ├─ [A] คำนวณ ground-truth state ─────────────────────────────────────
     │       gap      = dist2d(ego, เป้าหมาย) − ขนาดตัวรถ   ← ระยะผิวถึงผิว (เมตร)
     │       rel_speed= (gap_ก่อนหน้า − gap) / 0.05 s        ← ความเร็วปิดช่องว่าง
     │       TTC      = gap / rel_speed                       ← Time-to-Collision
     │       lead_decel= EMA(finite-diff ของ v_lead)          ← ความหน่วงรถข้างหน้า
     │
     ├─ [B] YOLO ────────────────────────────────────────────────────────
     │       รัน YOLOv8n บนภาพ
     │       yolo_now = มีกล่อง vehicle ที่ cx อยู่ในแถบเลน (px 520–760) และสูง ≥ 90 px?
     │       → เก็บผลไว้วาดบนจอเสมอ
     │
     ├─ [C] Ground-truth detection (inpath_hazard) ───────────────────────
     │       gt_now = เป้าหมายอยู่/กำลังเข้า corridor ข้างหน้า ego ไหม?
     │                (lon ∈ (0, INPATH_MAX_RANGE], |lat| ≤ 1.8 m)
     │       cut-in  → INPATH_PREDICT=True  : นับตั้งแต่ dart กำลังพุ่งเข้าเลน (predictive)
     │       lead-brake → INPATH_PREDICT=False: lead อยู่หน้าตรง ๆ → gt_now=True เสมอตั้งแต่เริ่ม
     │
     ├─ [D] เลือก "gate" ตาม DETECTION_SOURCE ────────────────────────────
     │       "groundtruth" (default / ใช้ในเปเปอร์) → detected_now = gt_now
     │       "yolo"                                  → detected_now = yolo_now
     │       "both_or"                               → detected_now = gt_now OR yolo_now
     │
     ├─ [E] หน่วงเฟรม (perception latency) ──────────────────────────────
     │       det_buffer.append(detected_now)
     │       perceived = det_buffer[0]       ← oldest entry (delay_frames=0 → ทันที)
     │
     ├─ [F] สร้าง Perception object ──────────────────────────────────────
     │       Perception(
     │           detected  = perceived,      ← gate (True/False) จาก [D]+[E]
     │           distance  = gap,            ← ground-truth เสมอ
     │           rel_speed = rel_speed,      ← ground-truth เสมอ
     │           ttc       = TTC,            ← ground-truth เสมอ
     │           lead_speed= v_lead,         ← ground-truth เสมอ
     │           lead_decel= lead_decel_ema, ← ground-truth เสมอ
     │       )
     │
     └─ [G] สมองกลตัดสินใจ ───────────────────────────────────────────────
             controller.decide(perc, ego_state)
             เงื่อนไขแรก: if NOT perc.detected → คืน throttle=0.6 (ยังวิ่งปกติ)
             ถ้า detected=True → เอา gap, TTC, lead_decel ไปคำนวณแรงเบรก
```

---

## บทบาทของ YOLO และ Ground-truth

| | YOLO | Ground-truth (CARLA) |
|---|---|---|
| **หน้าที่** | Perception trigger (เลียนการตรวจจับจริง) | แหล่งข้อมูลระยะ/ความเร็ว |
| **ค่าที่คืน** | True/False (มีรถในเลนและใหญ่พอ) | gap, rel_speed, TTC, lead_decel |
| **ใช้กับ** | `DETECTION_SOURCE = "yolo"` เท่านั้น | **ทุกโหมดเสมอ** (ไม่ว่าจะเลือก source อะไร) |
| **ในเปเปอร์** | วาดบนจอ / ทำ perception-quality log | กำหนดว่าเบรกหรือไม่ + ปริมาณเบรก |
| **ข้อจำกัด** | YOLO อาจพลาด dart ที่พุ่งเข้าจากด้านข้าง (bbox เล็ก/กึ่งกลางหลุดแถบ) | ไม่มีสัญญาณรบกวน แต่ไม่เลียนเซ็นเซอร์จริง |

> **ทำไมถึงแยกสองสิ่งนี้ออกจากกัน?**  
> YOLO ตรวจจับ "มีหรือไม่มี" แต่ไม่รู้ระยะ  
> ระยะ/TTC จากภาพเดียวต้องใช้ pinhole model + สมมติฐานหลายอย่าง (ขนาดรถ ฯลฯ)  
> งานนี้จึงแยก: YOLO ทำหน้าที่เป็น "ตาบอก" ส่วน CARLA เป็น "ไม้วัด"

---

## รายละเอียดแต่ละฉาก

### ฉาก Cut-in / Dart-out

**Ground-truth gate (`inpath_hazard` + predictive):**
- ตรวจว่า dart อยู่ใน corridor ข้างหน้า ego: `0 < lon ≤ 40 m`, `|lat| ≤ 1.8 m`
- เปิด `INPATH_PREDICT = True`: ถ้า dart กำลังเคลื่อนเข้าด้านข้าง (`v_lateral`) และ lat ที่ทำนาย
  ใน `1.5 s` จะเข้า corridor → นับว่า detected แล้ว (ก่อน dart เข้าเลนจริง)
- ประโยชน์: สมองกล baseline/proposed ได้ "คำเตือนล่วงหน้า" พอให้ dynamic TTC ของ proposed ทำงาน

**ทำไม YOLO พลาดได้ใน cut-in:**
- dart พุ่งเข้าจากด้านข้างตั้งฉาก → bbox อาจอยู่ขอบซ้าย/ขวาของเฟรม นอกแถบ px 520–760
- ช่วงแรก dart ยังไกล → กล่องเล็กกว่า min_box_h=90 → `yolo_now=False` แม้เห็นรถ
- นี่คือเหตุผลหลักที่ใช้ `DETECTION_SOURCE = "groundtruth"` ในฉากนี้

**เส้นทางระยะ/TTC:**
- dart พุ่งตั้งฉาก → `long_speed_along(ego, dart)` ≈ 0 (ส่วนความเร็วตามแนวหน้า ego)
- `lead_decel_ema` ≈ 0 เช่นกัน
- `proposed_enhanced` จึงใช้สูตรลดรูป `a_req = v_e² / (2 × gap)` (สิ่งกีดขวางนิ่ง)

---

### ฉาก Lead-brake (CCRb)

**Ground-truth gate (`inpath_hazard` ไม่มี predictive):**
- `INPATH_PREDICT = False` — รถนำอยู่หน้าตรง ๆ ตั้งแต่ต้น → `gt_now = True` ตั้งแต่ tick แรก
- แต่ตอนนั้น `rel_speed ≈ 0` (ทั้งคู่วิ่งเร็วเท่ากัน) → `TTC = ∞`
- สมองกล baseline/proposed ยังไม่เบรก เพราะ `TTC > threshold` ทุกตัว
- พอรถนำเบรก: gap เริ่มแคบ → `rel_speed > 0` → TTC ลดลง → สมองกลตอบสนอง

**เส้นทาง lead_decel:**
- `raw_lead_decel = (v_lead_ก่อนหน้า − v_lead) / 0.05`
- `lead_decel_ema = 0.3 × raw + 0.7 × ema_ก่อนหน้า` (EMA กันค่ากระโดด)
- **ไม่อ่าน `LEAD_DECEL` จาก config ตรง ๆ** — ประมาณจาก finite-difference ของความเร็วจริง
- `proposed_enhanced` ใช้ `lead_decel_ema` คำนวณ `d_lead = v_lead²/(2×a_lead)` → `a_req = v_e²/(2×(gap+d_lead))`
- ยิ่งรถนำเบรกแรง → `a_lead ↑ → d_lead ↓ → a_req ↑ → urgency ↑` → เบรกถูกจังหวะ

---

## สรุปขั้นตอนการเริ่มเบรกจากสมองกลแต่ละตัว

### Baseline (Static TTC)

```
ทุก tick ถ้า perceived=True:
    ถ้า TTC ≤ 0.6 s → brake = 1.0 (เต็ม)
    ถ้า TTC ≤ 1.6 s → brake = 0.4 (บางส่วน)
    ไม่เช่นนั้น → throttle = 0.6 (วิ่งต่อ)
```

Threshold คงที่ — ไม่รู้ว่าถนนลื่นหรือเร็วแค่ไหน

---

### Proposed (Adaptive TTC)

```
ทุก tick ถ้า perceived=True:
    bump = 1.2 × max(0, (v_kmh−40)/100) + 1.5 × max(0, (0.85−μ))
    thr_full = 0.6 + bump           ← threshold ขยายขึ้นตามความเร็ว/ความลื่น
    thr_warn = max(1.6, thr_full+0.5)
    ถ้า TTC ≤ thr_full → brake = 1.0
    ถ้า TTC ≤ thr_warn → brake = 0.4
```

เร็ว/ลื่น → bump ใหญ่ → threshold สูง → เบรกล่วงหน้ามากขึ้น

---

### Proposed Enhanced (Required-Deceleration)

```
ทุก tick ถ้า perceived=True:
    a_max = μ × 9.81                               ← เพดานแรงเสียดทาน
    d_lead= v_lead²/(2×lead_decel)                 ← ระยะที่รถข้างหน้าวิ่งต่อก่อนหยุด
    a_req = v_ego²/(2×(gap + d_lead))              ← ความหน่วงที่ ego ต้องใช้
    urgency = a_req / a_max
    ถ้า urgency ≥ 0.9 → brake = 1.0 (เต็ม)
    ถ้า urgency ≥ 0.6 → brake = 0.4 (บางส่วน)
```

ไม่ใช้ TTC เลย — ตอบสนองจากฟิสิกส์จริง (ต้องการความหน่วงเท่าไหร่?)

---

## Brake Latch — เบรกแล้วค้าง ไม่ปล่อย

ทุกสมองกลใช้ `_latch_brake()` ร่วมกัน:
- พอ `brake > 0` ครั้งแรก → ตั้งสถานะ `_engaged = True`
- หลังจากนั้น แรงเบรก **เพิ่มได้** (partial → full) แต่ **ลดไม่ได้**
- กัน TTC เด้งขึ้น (เช่น dart ผ่าน corridor ไปแล้ว) แล้วสมองกลหยุดเบรกกลางคัน

---

## Kinematic Brake Model — ความหน่วงถูก clamp ที่ μ·g

```python
a_max     = μ × 9.81                # เพดานจากแรงเสียดทาน
a_cmd     = brake_cmd × a_max       # 0–1 → 0–a_max
a_applied = min(a_cmd, a_max)       # hard clamp (redundant แต่ชัดเจน)
v_new     = max(0, v_model − a_applied × 0.05)
ego.set_target_velocity(v_new)      # ตั้งความเร็วให้ CARLA ตรง
```

- อัปเดตจาก `v_model` (จับไว้ตอนเริ่มเบรก) ไม่ได้อ่านกลับจาก CARLA
  → ป้องกันไม่ให้ฟิสิกส์ภายในเอนจินหน่วงรถเกินเพดาน μ·g
- `peak_decel` ที่บันทึก = ความหน่วงที่ทำได้จริง → ทุกแถว CSV มี `peak_decel ≤ a_max` เสมอ
- ถนนเปียก (μ=0.40): `a_max ≈ 3.92 m/s²` — ถนนแห้ง (μ=0.85): `a_max ≈ 8.34 m/s²`

---

## คำถามที่พบบ่อย

**Q: ถ้า YOLO ไม่เห็นรถ รถ ego จะเบรกได้ไหม?**  
A: ได้ ถ้า `DETECTION_SOURCE = "groundtruth"` (ค่า default) — YOLO ไม่เกี่ยวกับการเบรกเลย

**Q: ถ้า YOLO เห็นรถผิด (False Positive) รถจะเบรกหรือเปล่า?**  
A: ขึ้นกับ DETECTION_SOURCE:
- `"groundtruth"` → ไม่เบรก (ใช้ ground-truth gate ไม่ใช่ YOLO)
- `"yolo"` → `detected=True` แต่ถ้า `gap` ยัง ∞ (ไม่มีรถจริงข้างหน้า) → TTC=∞ → สมองกลก็ไม่เบรกอยู่ดี
- `"both_or"` → เช่นเดียวกับ "yolo" สำหรับกรณี False Positive

**Q: ระยะและ TTC ที่ส่งให้สมองกลมาจากไหน?**  
A: **จาก ground-truth ของ CARLA เสมอ** ไม่ว่าจะตั้ง DETECTION_SOURCE เป็นอะไร  
(ดูไฟล์ `core/runner.py` บรรทัด 185–187 และ `core/runner_lead_brake.py` บรรทัด 222–225)

**Q: Perception delay ทำงานอย่างไร?**  
A: หน่วงเฉพาะ signal `detected` (True/False) ผ่าน deque  
ระยะ/TTC ยังคงเป็น real-time ground-truth — ไม่หน่วง  
ในทุกรัน production: `delay_frames = 0` → ไม่มีหน่วง

**Q: YOLO ใช้ทำอะไรในโปรเจ็คนี้?**  
A: สองบทบาท:
1. **Perception trigger** (ถ้าตั้ง `DETECTION_SOURCE = "yolo"`) — ใช้ทดสอบว่าถ้าพึ่ง YOLO จริง ๆ AEB จะทำงานได้ไหม
2. **Perception-quality logging** — `run_perception_log.py` ใช้ `detect_all()` บันทึกสถิติ detection บนฉาก 3DGS (ผลอยู่ใน TABLE V ของเปเปอร์)
