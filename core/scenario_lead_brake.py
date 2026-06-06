# -*- coding: utf-8 -*-
"""
ตรรกะฉาก lead-brake (รถนำวิ่งนำอยู่ข้างหน้า แล้วเบรกกระทันหัน — Euro-NCAP CCRb):
  - lead ถูก spawn ข้างหน้า ego ในเลนเดียวกัน (หันทางเดียวกัน)
  - ทั้ง ego และ lead วิ่งตรงด้วยความเร็วคงที่ (open-loop เพราะไม่มี waypoint)
    โดยปกติ lead วิ่งเร็วเท่า ego → รักษาระยะห่าง (headway) คงที่
  - พอ lead วิ่งไปได้ระยะ LEAD_BRAKE_AFTER_M → "เบรกกระทันหัน" หน่วง LEAD_DECEL จนหยุดนิ่ง
  - ระยะระหว่างรถจึงหดเร็ว → ทดสอบว่า AEB ของ ego เบรกทันไหม
ตัวฉากไม่ยุ่งกับการตัดสินใจเบรกของ ego เลย (นั่นเป็นหน้าที่ controller)
อินเทอร์เฟซตรงกับ CutInScenario: start() / update()→just_braked / cruise_ego()
"""
import math
import carla
from core.actors import kmh_to_ms, cruise, hold, speed_ms


class LeadBrakeScenario:
    def __init__(self, ego, lead, cfg, case):
        self.ego = ego
        self.lead = lead
        self.cfg = cfg
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])
        self.lead_ms = kmh_to_ms(case["lead_speed_kmh"])
        self.brake_after_m = cfg.LEAD_BRAKE_AFTER_M
        self.lead_decel = cfg.LEAD_DECEL
        self.dt = cfg.FIXED_DT
        self.braking = False
        self._lx0 = self._ly0 = 0.0

    def start(self):
        """ปล่อยทั้ง ego และ lead ออกตัวด้วยความเร็วเป้าหมาย (lead วิ่งนำไปก่อน)"""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        self.lead.apply_control(carla.VehicleControl(hand_brake=False))
        loc = self.lead.get_location()
        self._lx0, self._ly0 = loc.x, loc.y
        cruise(self.ego, self.ego_ms)
        cruise(self.lead, self.lead_ms)

    def update(self):
        """เรียกทุก tick — คุม lead (วิ่งนำ → เบรกกระทันหัน) คืน True เมื่อ lead เพิ่งเริ่มเบรก"""
        just_braked = False
        loc = self.lead.get_location()
        travelled = math.hypot(loc.x - self._lx0, loc.y - self._ly0)

        if not self.braking:
            # รถนำหยุดนิ่งแต่แรก (lead_ms≈0) หรือวิ่งครบระยะแล้ว → เริ่มเบรกกระทันหัน
            if self.lead_ms <= 1e-3 or travelled >= self.brake_after_m:
                self.braking = True
                just_braked = True
            else:
                cruise(self.lead, self.lead_ms)   # ยังวิ่งนำด้วยความเร็วคงที่

        if self.braking:
            v = speed_ms(self.lead)
            if v <= 0.05:
                hold(self.lead)                   # หยุดสนิทแล้ว ตรึงไว้
            else:
                new_v = max(0.0, v - self.lead_decel * self.dt)
                f = self.lead.get_transform().get_forward_vector()
                self.lead.set_target_velocity(
                    carla.Vector3D(f.x * new_v, f.y * new_v, 0.0))
        return just_braked

    def cruise_ego(self):
        """รักษาความเร็ว ego (เรียกเมื่อ controller ยังไม่สั่งเบรก)"""
        cruise(self.ego, self.ego_ms)
