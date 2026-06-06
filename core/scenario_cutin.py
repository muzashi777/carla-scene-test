# -*- coding: utf-8 -*-
"""
ตรรกะฉาก cut-in / dart-out:
  - ego วิ่งตรงด้วยความเร็วคงที่ (open-loop เพราะไม่มี waypoint)
  - เมื่อระยะ ego-dart ≤ trigger_d (Δd) → dart พุ่งออกตั้งฉาก
  - dart พุ่งมาถึงกลางเลน ego (DART_STOP_X) แล้วเบรกจอดขวาง
ตัวฉากไม่ยุ่งกับการตัดสินใจเบรกของ ego เลย (นั่นเป็นหน้าที่ controller)
"""
import carla
from core.actors import kmh_to_ms, dist2d, cruise, hold


class CutInScenario:
    def __init__(self, ego, dart, cfg, case):
        self.ego = ego
        self.dart = dart
        self.cfg = cfg
        self.trigger_d = case["trigger_d"]
        self.dart_ms = kmh_to_ms(case["dart_speed_kmh"])
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])
        self.launched = False

    def start(self):
        """ปล่อย ego ออกตัวด้วยความเร็วเป้าหมาย"""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)

    def update(self):
        """เรียกทุก tick — คุมเฉพาะ dart และ trigger คืน True เมื่อเพิ่งปล่อย dart"""
        just_launched = False
        d = dist2d(self.ego, self.dart)

        if (not self.launched) and d <= self.trigger_d:
            self.launched = True
            just_launched = True
            self.dart.apply_control(carla.VehicleControl(hand_brake=False))
            df = self.dart.get_transform().get_forward_vector()
            self.dart.set_target_velocity(
                carla.Vector3D(df.x * self.dart_ms, df.y * self.dart_ms, 0.0))
        elif self.launched:
            if self.dart.get_location().x > self.cfg.DART_STOP_X:
                df = self.dart.get_transform().get_forward_vector()
                self.dart.set_target_velocity(
                    carla.Vector3D(df.x * self.dart_ms, df.y * self.dart_ms, 0.0))
            else:
                hold(self.dart)
        return just_launched

    def cruise_ego(self):
        """รักษาความเร็ว ego (เรียกเมื่อ controller ยังไม่สั่งเบรก)"""
        cruise(self.ego, self.ego_ms)
