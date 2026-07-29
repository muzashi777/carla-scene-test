# -*- coding: utf-8 -*-
"""
Junction cut-in scenario logic — physics-based steering (adapted from cut-out).

The intruder vehicle is driven entirely under CARLA's physics via VehicleControl
(throttle / steer / brake) throughout its lifecycle:

  PHASE_CRUISE   — intruder waits stationary (hand brake) until
                   dist(ego, intruder) ≤ TURN_TRIGGER_D.
  PHASE_STEER    — P-heading controller steers the intruder toward
                   trigger_yaw + TURN_HEADING_DEG (≈ ego lane heading).
                   P-speed controller holds target speed during the turn.
  PHASE_SETTLED  — intruder decelerates to a full stop in the ego lane
                   (AFTER_TURN_STOP = True always for this scenario).

Trigger condition (differs from cut-out):
  Cut-out:          dist(lead, target)   ≤ CUTOUT_TRIGGER_D
  Junction cut-in:  dist(ego, intruder)  ≤ cfg.TURN_TRIGGER_D  [per-case from case["trigger_d"]]

Velocity boot: none — the intruder starts from rest and accelerates from zero
during STEER.  set_simulate_physics(True) is called in start() as in cut-out.

Key invariant: only apply_control(VehicleControl(...)) is used in update();
set_target_velocity and set_transform are not called after start().
"""
import carla
from core.actors import kmh_to_ms, dist2d, cruise, speed_ms


def _norm_angle(deg):
    """Normalise angle to (-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0


# Phase constants
_CRUISE   = 0
_STEER    = 1
_SETTLED  = 2


class JunctionCutInScenario:
    def __init__(self, ego, intruder, cfg, case):
        self.ego      = ego
        self.intruder = intruder
        self.cfg      = cfg
        self.ego_ms   = kmh_to_ms(case["ego_speed_kmh"])

        self._phase = _CRUISE

        # Per-case trigger distance (matrix axis); falls back to config constant.
        self._trigger_d = case.get("trigger_d", cfg.TURN_TRIGGER_D)

        # Recorded at the trigger tick:
        self._trigger_yaw    = 0.0   # intruder yaw (degrees) at trigger
        self._target_heading = 0.0   # trigger_yaw + TURN_HEADING_DEG

        # Diagnostic
        self.turn_tick = -1     # tick index when trigger fired
        self.turn_dist = -1.0   # ego↔intruder distance at trigger

    @property
    def phase(self):
        return self._phase

    # ──────────────────────────────────────────────────────────────────────────
    # Public interface
    # ──────────────────────────────────────────────────────────────────────────

    def start(self):
        """Release ego (cruise at target speed); hold intruder stationary."""
        # Ego — unchanged kinematic cruise
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)

        # Intruder — physics enabled; starts stationary (no velocity boot)
        self.intruder.set_simulate_physics(True)
        self.intruder.apply_control(
            carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))

    def update(self, tick=0):
        """Drive the intruder one tick.  Returns True only on the trigger tick."""
        just_triggered = False
        v = speed_ms(self.intruder)

        # ── CRUISE phase: hold stationary until trigger ────────────────────
        if self._phase == _CRUISE:
            d = dist2d(self.ego, self.intruder)
            if d <= self._trigger_d:
                just_triggered = True
                self.turn_tick  = tick
                self.turn_dist  = d
                tf = self.intruder.get_transform()
                self._trigger_yaw    = tf.rotation.yaw
                self._target_heading = _norm_angle(
                    self._trigger_yaw + self.cfg.TURN_HEADING_DEG)
                self._phase = _STEER
                print(f"[JUNCTION] Turn triggered: d={d:.1f}m ego→intruder "
                      f"trigger_yaw={self._trigger_yaw:.1f}° "
                      f"target_heading={self._target_heading:.1f}°")
                # Fall through to _STEER below on this same tick
            else:
                self.intruder.apply_control(
                    carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))

        # ── STEER phase: P-heading + P-speed until aligned ────────────────
        if self._phase == _STEER:
            cur_yaw = self.intruder.get_transform().rotation.yaw
            herr = abs(_norm_angle(self._target_heading - cur_yaw))
            if herr < self.cfg.JCUTIN_SETTLE_DEG:
                self._phase = _SETTLED
                print(f"[JUNCTION] Intruder aligned: yaw={cur_yaw:.1f}° "
                      f"target={self._target_heading:.1f}° herr={herr:.1f}°")
                # Fall through to _SETTLED below
            else:
                steer = self._heading_steer(self._target_heading)
                self._ctrl_speed_steer(v, steer)

        # ── SETTLED phase: brake to a full stop ───────────────────────────
        if self._phase == _SETTLED:
            self._ctrl_stop(v)

        return just_triggered

    def cruise_ego(self):
        """Maintain ego speed (called while AEB has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _heading_steer(self, target_yaw_deg):
        """P-controller: steer proportional to heading error, clamped."""
        cur = self.intruder.get_transform().rotation.yaw
        err = _norm_angle(target_yaw_deg - cur)
        raw = self.cfg.JCUTIN_STEER_K * err
        return max(-self.cfg.JCUTIN_STEER_MAX,
                   min(self.cfg.JCUTIN_STEER_MAX, raw))

    def _ctrl_speed_steer(self, v_ms, steer):
        """Full longitudinal P-controller with lateral steer.
        Applies active braking on overspeed to prevent rear-end risk mid-turn."""
        err = self.ego_ms - v_ms
        if err > 0.0:
            t = min(self.cfg.JCUTIN_MAX_THROTTLE, self.cfg.JCUTIN_SPEED_K * err)
            self.intruder.apply_control(
                carla.VehicleControl(throttle=t, steer=steer, brake=0.0))
        else:
            b = min(0.3, self.cfg.JCUTIN_SPEED_K * (-err))
            self.intruder.apply_control(
                carla.VehicleControl(throttle=0.0, steer=steer, brake=b))

    def _ctrl_stop(self, v_ms):
        """Decelerate intruder to a full stop in the ego lane."""
        if v_ms > 0.5:
            b = min(0.8, self.cfg.JCUTIN_SPEED_K * v_ms)
            self.intruder.apply_control(
                carla.VehicleControl(brake=b, throttle=0.0, steer=0.0))
        else:
            self.intruder.apply_control(
                carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))
