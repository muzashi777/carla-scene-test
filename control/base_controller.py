# -*- coding: utf-8 -*-
"""
สัญญา (interface) ของ 'ปลั๊กอินสมองกล' — ทุกโมเดลต้องสืบทอดและคืน VehicleControl
เปลี่ยนสมองกลได้โดยไม่แตะโค้ดฉาก/metric เลย แค่เปลี่ยนคลาสที่โหลด
interface เผื่อ steer ไว้แล้ว (อนาคตทำ evasive maneuver ได้)
"""
import carla
from core.types import Perception, EgoState


class BaseController:
    name = "base"

    def reset(self):
        """เรียกก่อนเริ่มแต่ละเคส — เคลียร์ latch ภายใน"""
        self._engaged = False     # เคยสั่งเบรกแล้วหรือยัง
        self._brake_held = 0.0    # แรงเบรกที่ค้างไว้ (ไม่ลดลง)

    def decide(self, perc: Perception, ego: EgoState) -> carla.VehicleControl:
        """รับสิ่งที่มองเห็น + สถานะรถ → คืนคำสั่งคุมรถ (throttle/brake/steer)"""
        raise NotImplementedError

    # ── ตัวช่วยที่ทุกสมองกลใช้ร่วมกัน ──────────────────────────────
    @staticmethod
    def control(throttle=0.0, brake=0.0, steer=0.0):
        return carla.VehicleControl(
            throttle=float(throttle), brake=float(brake), steer=float(steer)
        )

    def _latch_brake(self, desired):
        """
        latch การเบรก: พอเริ่มเบรกแล้ว 'ค้าง' ไม่กลับไปปล่อยคันเร่งอีก
        และแรงเบรกเพิ่มได้ (partial→full) แต่ไม่ลดลง → กัน ttc เด้งขึ้นแล้วเลิกเบรก
        """
        if desired > 0.0:
            self._engaged = True
        if self._engaged:
            self._brake_held = max(self._brake_held, desired)
            return self._brake_held
        return 0.0

    def _emit(self, desired):
        """แปลง desired brake (ผ่าน latch) เป็น VehicleControl"""
        b = self._latch_brake(desired)
        return self.control(brake=b) if b > 0.0 else self.control(throttle=0.6)


# ── ตัวช่วยชดเชย latency (ใช้ร่วมโดย enhanced_predictive / enhanced_inflation) ──
def compensation_latency(run_spec, cfg):
    """แปลง run_spec → (L วินาที, L เฟรมที่ใช้จริง, comp_source)

    controller ที่ชดเชย latency เรียกฟังก์ชันนี้เพื่อรู้ว่าจะพยากรณ์ไปข้างหน้ากี่วินาที
      comp_source='oracle'      → ชดเชยด้วย L = delay_frames จริงที่ฉีด (upper bound: รู้ L เป๊ะ)
      comp_source='mismatched'  → ชดเชยด้วย comp_L_frames ที่กำหนดแยกจาก delay_frames (ทดสอบความเปราะ)
    run_spec ว่าง/ไม่มี key → default oracle + delay_frames=0 → L=0 → ลดรูปเป็นพฤติกรรมไม่ชดเชย
    ไม่ผูกกับ PerceptionDegrader โดยตรง — รับค่าผ่าน run_spec เท่านั้น (loose coupling)
    """
    spec = run_spec or {}
    delay = spec.get("delay_frames", 0)
    src = spec.get("comp_source", "oracle")
    frames = spec.get("comp_L_frames", delay) if src == "mismatched" else delay
    dt = getattr(cfg, "FIXED_DT", 0.05)
    return max(0.0, frames * dt), frames, src


# ── registry: map ชื่อ → คลาส (run scripts เรียกผ่านนี้) ───────────
_REGISTRY = {}


def register(name):
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def make_controller(name, cfg, run_spec=None):
    if name not in _REGISTRY:
        raise KeyError(f"ไม่รู้จัก controller '{name}' (มี: {list(_REGISTRY)})")
    ctrl = _REGISTRY[name](cfg)
    # ผูก run_spec ไว้ให้ controller ที่ต้องการใช้ (เช่น ชดเชย latency) อ่านได้ —
    # controller เดิมไม่แตะ attribute นี้ พฤติกรรมจึงไม่เปลี่ยน
    ctrl.run_spec = run_spec or {}
    return ctrl
