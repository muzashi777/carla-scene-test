# -*- coding: utf-8 -*-
"""
CCRs scenario logic (Car-to-Car Rear Stationary):
  - ego drives straight at constant speed toward a stationary target vehicle
  - the target never moves (held via hold() every tick)
The scenario does not interfere with the ego's braking decisions (that is the controller's responsibility).
Interface matches CutInScenario / LeadBrakeScenario: start() / update() / cruise_ego()
"""
import carla
from core.actors import kmh_to_ms, cruise, hold


class CCRsScenario:
    def __init__(self, ego, target, cfg, case):
        self.ego = ego
        self.target = target
        self.cfg = cfg
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])

    def start(self):
        """Release ego at the target speed; hold the target stationary."""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)
        hold(self.target)

    def update(self):
        """Called every tick — keeps target stationary. Returns False (no special event)."""
        hold(self.target)
        return False   # no trigger event to signal

    def cruise_ego(self):
        """Maintain ego speed (called when controller has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)
