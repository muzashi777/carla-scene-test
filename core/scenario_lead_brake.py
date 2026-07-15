# -*- coding: utf-8 -*-
"""
Lead-brake scenario logic (lead vehicle ahead then brakes suddenly — Euro-NCAP CCRb):
  - lead is spawned ahead of ego in the same lane (same heading)
  - both ego and lead drive straight at constant speed (open-loop — no waypoints);
    normally lead travels at the same speed as ego → maintaining a constant headway
  - once lead has travelled LEAD_BRAKE_AFTER_M metres → "emergency brake" at LEAD_DECEL until fully stopped
  - the inter-vehicle gap closes rapidly → tests whether the ego's AEB brakes in time
The scenario does not interfere with the ego's braking decisions (that is the controller's responsibility).
Interface matches CutInScenario: start() / update()→just_braked / cruise_ego()
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
        """Release both ego and lead at their target speeds (lead moves ahead first)."""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        self.lead.apply_control(carla.VehicleControl(hand_brake=False))
        loc = self.lead.get_location()
        self._lx0, self._ly0 = loc.x, loc.y
        cruise(self.ego, self.ego_ms)
        cruise(self.lead, self.lead_ms)

    def update(self):
        """Called every tick — controls lead (cruise → emergency brake); returns True on the tick lead first begins braking."""
        just_braked = False
        loc = self.lead.get_location()
        travelled = math.hypot(loc.x - self._lx0, loc.y - self._ly0)

        if not self.braking:
            # lead was stationary from the start (lead_ms≈0) or has covered the required distance → trigger emergency brake
            if self.lead_ms <= 1e-3 or travelled >= self.brake_after_m:
                self.braking = True
                just_braked = True
            else:
                cruise(self.lead, self.lead_ms)   # still cruising at constant speed

        if self.braking:
            v = speed_ms(self.lead)
            if v <= 0.05:
                hold(self.lead)                   # fully stopped — hold in place
            else:
                new_v = max(0.0, v - self.lead_decel * self.dt)
                f = self.lead.get_transform().get_forward_vector()
                self.lead.set_target_velocity(
                    carla.Vector3D(f.x * new_v, f.y * new_v, 0.0))
        return just_braked

    def cruise_ego(self):
        """Maintain ego speed (called when the controller has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)
