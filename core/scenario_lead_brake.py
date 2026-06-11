# -*- coding: utf-8 -*-
"""
ตรรกะฉาก lead-brake (รถนำเบรกจอดสนิทแล้วอยู่ข้างหน้าในเลนเดียวกัน):
  - lead ถูก spawn ข้างหน้า ego ในเลนเดียวกัน (หันทางเดียวกัน) และ "หยุดสนิทแล้ว"
  - ego วิ่งตรงด้วยความเร็วคงที่ (open-loop เพราะไม่มี waypoint) เข้าหา lead
  - ฉากนี้ lead ไม่ขยับเลย → ทดสอบล้วน ๆ ว่า AEB ของ ego เบรกทันไหม
ตัวฉากไม่ยุ่งกับการตัดสินใจเบรกของ ego เลย (นั่นเป็นหน้าที่ controller)
อินเทอร์เฟซตรงกับ CutInScenario: start() / update()→just_braked / cruise_ego()
"""
import carla
from core.actors import kmh_to_ms, cruise, hold


class LeadBrakeScenario:
    def __init__(self, ego, lead, cfg, case):
        self.ego = ego
        self.lead = lead
        self.cfg = cfg
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])

    def start(self):
        """ปล่อย ego ออกตัว; ตรึง lead ให้จอดนิ่ง (เบรกแล้ว)"""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        hold(self.lead)
        cruise(self.ego, self.ego_ms)

    def update(self):
        """เรียกทุก tick — lead จอดสนิทตลอด (ตรึงไว้กันไหล) คืน False เสมอ (ไม่มี event)"""
        hold(self.lead)
        return False

    def cruise_ego(self):
        """รักษาความเร็ว ego (เรียกเมื่อ controller ยังไม่สั่งเบรก)"""
        cruise(self.ego, self.ego_ms)
