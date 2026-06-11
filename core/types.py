# -*- coding: utf-8 -*-
"""โครงสร้างข้อมูลกลางที่ส่งให้สมองกล (ป้องกัน import วน)"""
from dataclasses import dataclass


@dataclass
class Perception:
    """สิ่งที่สมองกล 'มองเห็น' ในเฟรมนี้ (หลังหน่วงเฟรมแล้ว)"""
    detected: bool        # YOLO เจอรถในแถบเลน ego ไหม
    distance: float       # ระยะถึงสิ่งกีดขวาง (m) — ground-truth จาก CARLA
    rel_speed: float      # ความเร็วเข้าหากัน (m/s, >0 = กำลังเข้าใกล้)
    ttc: float            # distance / rel_speed (วินาที, inf ถ้าไม่เข้าใกล้)
    box_h: float = 0.0    # ความสูงกล่อง YOLO (px) — ไว้ดีบัก/วาด
    # ── ข้อมูลรถข้างหน้า (ground-truth, แชร์ให้ทุกสมองกลเท่ากันเพื่อความยุติธรรม) ──
    lead_speed: float = 0.0   # ความเร็วรถข้างหน้า (m/s) — baseline จะไม่ใช้ก็ได้
    lead_decel: float = 0.0   # ความหน่วงรถข้างหน้าที่ "ประมาณจากการเคลื่อนที่จริง" (m/s², ≥0)


@dataclass
class EgoState:
    speed_ms: float
    speed_kmh: float
    mu: float             # ความลื่นถนนปัจจุบัน (รู้จาก config ของเคส)
