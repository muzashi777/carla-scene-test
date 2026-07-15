# -*- coding: utf-8 -*-
"""
Cut-in / dart-out scenario logic:
  - ego drives straight at constant speed (open-loop — no waypoints)
  - when the ego-to-dart distance ≤ trigger_d (Δd) → dart launches perpendicularly
  - dart travels to the centre of the ego's lane (DART_STOP_X) and brakes to a standstill blocking the path
The scenario does not interfere with the ego's braking decisions (that is the controller's responsibility).
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
        """Release ego at the target speed."""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)

    def update(self):
        """Called every tick — controls only the dart and trigger; returns True on the tick the dart is first launched."""
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
        """Maintain ego speed (called when the controller has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)
