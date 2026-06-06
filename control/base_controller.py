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
        """เรียกก่อนเริ่มแต่ละเคส — เคลียร์ state ภายใน (เช่น latch)"""
        pass

    def decide(self, perc: Perception, ego: EgoState) -> carla.VehicleControl:
        """รับสิ่งที่มองเห็น + สถานะรถ → คืนคำสั่งคุมรถ (throttle/brake/steer)"""
        raise NotImplementedError

    # ── ตัวช่วยที่ทุกสมองกลใช้ร่วมกัน ──────────────────────────────
    @staticmethod
    def control(throttle=0.0, brake=0.0, steer=0.0):
        return carla.VehicleControl(
            throttle=float(throttle), brake=float(brake), steer=float(steer)
        )


# ── registry: map ชื่อ → คลาส (run scripts เรียกผ่านนี้) ───────────
_REGISTRY = {}


def register(name):
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def make_controller(name, cfg):
    if name not in _REGISTRY:
        raise KeyError(f"ไม่รู้จัก controller '{name}' (มี: {list(_REGISTRY)})")
    return _REGISTRY[name](cfg)
