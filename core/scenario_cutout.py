# -*- coding: utf-8 -*-
"""
Cut-out scenario logic:
  - target vehicle is stationary from t=0 (never moves)
  - lead vehicle drives ahead of ego in the same lane at the same speed as ego
  - when lead-to-target distance ≤ CUTOUT_TRIGGER_D → lead cuts out to the RIGHT
    by applying a rightward velocity (+ optional forward component)
  - after travelling CUTOUT_TRAVEL_M metres the lead stops
The scenario does not interfere with the ego's braking decisions.
The perception/runner tracks the STATIONARY TARGET (not the lead) for gap/TTC.
Interface matches existing scenario classes: start() / update()→just_triggered / cruise_ego()
"""
import math
import carla
from core.actors import kmh_to_ms, dist2d, cruise, hold


class CutOutScenario:
    def __init__(self, ego, lead, target, cfg, case):
        self.ego = ego
        self.lead = lead
        self.target = target
        self.cfg = cfg
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])
        self.cutout_triggered = False
        self._cutout_x0 = self._cutout_y0 = 0.0

    def start(self):
        """Release ego and lead at ego speed; hold target stationary."""
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        self.lead.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)
        cruise(self.lead, self.ego_ms)   # lead matches ego speed → constant headway
        hold(self.target)

    def update(self):
        """Called every tick. Returns True on the tick the cut-out first triggers."""
        hold(self.target)   # target always stationary
        just_triggered = False

        d_lead_to_target = dist2d(self.lead, self.target)

        if not self.cutout_triggered:
            if d_lead_to_target <= self.cfg.CUTOUT_TRIGGER_D:
                self.cutout_triggered = True
                just_triggered = True
                loc = self.lead.get_location()
                self._cutout_x0, self._cutout_y0 = loc.x, loc.y
                self._apply_cutout_velocity()
            else:
                cruise(self.lead, self.ego_ms)   # still following at ego speed
        else:
            # Continue cutting out until CUTOUT_TRAVEL_M lateral metres, then stop
            loc = self.lead.get_location()
            lateral_travelled = math.hypot(loc.x - self._cutout_x0,
                                           loc.y - self._cutout_y0)
            if lateral_travelled >= self.cfg.CUTOUT_TRAVEL_M:
                hold(self.lead)
            else:
                self._apply_cutout_velocity()

        return just_triggered

    def _apply_cutout_velocity(self):
        """Set lead velocity: rightward at CUTOUT_LATERAL_SPEED + forward at CUTOUT_FORWARD_SPEED."""
        f = self.lead.get_transform().get_forward_vector()
        r = self.lead.get_transform().get_right_vector()
        vx = (f.x * self.cfg.CUTOUT_FORWARD_SPEED
              + r.x * self.cfg.CUTOUT_LATERAL_SPEED)
        vy = (f.y * self.cfg.CUTOUT_FORWARD_SPEED
              + r.y * self.cfg.CUTOUT_LATERAL_SPEED)
        self.lead.set_target_velocity(carla.Vector3D(vx, vy, 0.0))

    def cruise_ego(self):
        """Maintain ego speed (called when controller has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)
