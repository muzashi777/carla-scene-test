# -*- coding: utf-8 -*-
"""
Cut-out scenario logic — physics-based steering implementation.

The lead vehicle is driven entirely under CARLA's physics via VehicleControl
(throttle / steer / brake) throughout its lifecycle:

  PHASE_CRUISE    — drives straight at ego speed using a P-speed-controller.
  PHASE_STEER     — steers right using a closed-loop heading P-controller until
                    the lateral displacement from the trigger point reaches
                    cfg.CUTOUT_LANE_WIDTH metres (measured in the lead's own
                    right-vector frame at the moment of trigger).
  PHASE_STRAIGHTEN— steers back toward the original heading until |heading error|
                    < cfg.CUTOUT_SETTLE_DEG (the car is pointing straight again
                    in the right lane).
  PHASE_SETTLED   — cruises in the right lane (CUTOUT_AFTER_STOP=False, default)
                    or decelerates to a stop (CUTOUT_AFTER_STOP=True).

Key physics decisions:
  - set_simulate_physics(True) is called explicitly in start() (defensive; default is True).
  - A one-time set_target_velocity() call in start() boots the physics engine with the
    correct initial forward velocity so the car is already at speed when the main loop
    begins; after that every tick uses apply_control(VehicleControl) only.
  - hold(target) is called every tick — the stationary target stays kinematic (unchanged).
  - cruise(ego, ...) / cruise_ego() are unchanged — ego still uses set_target_velocity.

Reveal-timing note:
  A real steered lane-change produces a smooth arc that takes longer to move the lead out
  of the ego's sight-line than the old teleport/velocity-injection approach.  The target will
  be revealed LATER (at a smaller ego-to-target distance) than before.  Compensate by
  DECREASING cfg.CUTOUT_TRIGGER_D or INCREASING cfg.CUTOUT_HEADING_DEG / cfg.CUTOUT_STEER_MAX
  so the manoeuvre is faster/more aggressive.  Alternatively, the earlier reveal distance can
  be restored by tuning CUTOUT_TRIGGER_D.

Interface unchanged: start() / update() → bool (just_triggered) / cruise_ego().
"""
import math
import carla
from core.actors import kmh_to_ms, dist2d, cruise, hold, speed_ms


def _norm_angle(deg):
    """Normalise to (-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0


# Phase constants
_CRUISE     = 0
_STEER      = 1
_STRAIGHTEN = 2
_SETTLED    = 3


class CutOutScenario:
    def __init__(self, ego, lead, target, cfg, case):
        self.ego    = ego
        self.lead   = lead
        self.target = target
        self.cfg    = cfg
        self.ego_ms = kmh_to_ms(case["ego_speed_kmh"])

        self._phase = _CRUISE

        # Per-case trigger distance (derived from reveal_ttc in runner; falls back to config).
        self._trigger_d = case.get("cutout_trigger_d", cfg.CUTOUT_TRIGGER_D)

        # Recorded at the cut-out trigger tick:
        self._trigger_yaw = 0.0   # lead yaw (degrees) at trigger
        self._rx = self._ry = 0.0  # right-vector components at trigger (world frame)
        self._tx = self._ty = 0.0  # trigger position (world frame)

        self._backstop_fired = False  # True once no-hit safety backstop engages
        self._ever_cleared   = False  # latches True the first tick the lead clears the ego lane

    @property
    def phase(self):
        return self._phase

    # ──────────────────────────────────────────────────────────────────────────
    # Public interface
    # ──────────────────────────────────────────────────────────────────────────

    def start(self):
        """Release ego and lead, give lead an initial forward velocity, hold target."""
        # Ego — unchanged (kinematic cruise)
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))
        cruise(self.ego, self.ego_ms)

        # Lead — physics from here onward
        self.lead.set_simulate_physics(True)
        self.lead.apply_control(carla.VehicleControl(hand_brake=False))
        # One-time velocity boot so physics engine starts at the right speed.
        # After this, apply_control(VehicleControl) drives the lead every tick.
        f = self.lead.get_transform().get_forward_vector()
        self.lead.set_target_velocity(
            carla.Vector3D(f.x * self.ego_ms, f.y * self.ego_ms, 0.0))

        # Target — always stationary (kinematic hold, unchanged)
        hold(self.target)

    def update(self):
        """Drive the lead one tick.  Returns True only on the trigger tick."""
        hold(self.target)   # stationary target: held kinematically every tick
        just_triggered = False
        v = speed_ms(self.lead)

        # ── Lane-cleared latch ─────────────────────────────────────────────────
        # Latches True once the lead has moved far enough right to leave the ego
        # lane (lateral ≥ CUTOUT_LANE_WIDTH + CUTOUT_CLEAR_MARGIN_M).  Also
        # latches when entering SETTLED since that phase implies the arc finished.
        # Once True it never resets: brief dips during STRAIGHTEN (P-controller
        # overshoot) cannot re-enable in-lane braking.
        if not self._ever_cleared and self._phase != _CRUISE:
            _cm = getattr(self.cfg, "CUTOUT_CLEAR_MARGIN_M", 0.0)
            if (self._lateral_offset() >= self.cfg.CUTOUT_LANE_WIDTH + _cm
                    or self._phase == _SETTLED):
                self._ever_cleared = True
                print(f"[CUTOUT] Lane cleared: lat={self._lateral_offset():.2f}m "
                      f">= {self.cfg.CUTOUT_LANE_WIDTH + _cm:.2f}m  phase={self._phase}")

        # ── Safety backstop: ONLY fires after lane is cleared (B4) ───────────
        # Stops the lead from hitting the target once it is safely in the right
        # lane.  Must NOT fire while the lead is still in the ego lane — doing
        # so would stop it mid-arc and leave it as an in-path obstacle.
        # If geometry prevents both clearing and stopping short, the cell is
        # SCENARIO_INFEASIBLE (detected offline by check_cutout_spawn.py).
        if self._ever_cleared and not self._backstop_fired:
            _bs = getattr(self.cfg, "CUTOUT_SAFETY_BACKSTOP_M", 5.0)
            if dist2d(self.lead, self.target) < _bs:
                self._backstop_fired = True
                print(f"[SAFETY] Lane cleared; lead dist={dist2d(self.lead, self.target):.2f}m "
                      f"< {_bs}m backstop — brake (lead is out of ego lane)")
        if self._backstop_fired:
            self.lead.apply_control(
                carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))
            return just_triggered

        # ── Hard-stop cap: ONLY fires after lane is cleared ───────────────────
        # Locks the lead once its 2-D displacement from the trigger position
        # exceeds CUTOUT_STOP_MAX_M.  Prevents the lead from entering baked-
        # obstacle zones after cut-out.  Gated on _ever_cleared so it cannot
        # fire mid-arc and stop the lead in the ego lane.
        _stop_max = getattr(self.cfg, "CUTOUT_STOP_MAX_M", None)
        if _stop_max is not None and self._ever_cleared:
            loc = self.lead.get_location()
            if math.hypot(loc.x - self._tx, loc.y - self._ty) >= _stop_max:
                self.lead.apply_control(
                    carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))
                return just_triggered

        # ── CRUISE phase ──────────────────────────────────────────────────────
        if self._phase == _CRUISE:
            if dist2d(self.lead, self.target) <= self._trigger_d:
                just_triggered = True
                tf = self.lead.get_transform()
                self._trigger_yaw = tf.rotation.yaw
                rv = tf.get_right_vector()
                self._rx, self._ry = rv.x, rv.y
                loc = tf.location
                self._tx, self._ty = loc.x, loc.y
                self._phase = _STEER
                # Fall through to _STEER below to apply first control this tick
            else:
                self._ctrl_cruise(v)

        # ── STEER phase ───────────────────────────────────────────────────────
        # Speed held via full P-controller (throttle + small brake on overspeed)
        # so the lead does not decelerate mid-turn and get rear-ended by the ego.
        if self._phase == _STEER:
            if self._lateral_offset() >= self.cfg.CUTOUT_LANE_WIDTH:
                self._phase = _STRAIGHTEN
                # Fall through to _STRAIGHTEN below
            else:
                steer = self._heading_steer(
                    self._trigger_yaw + self.cfg.CUTOUT_HEADING_DEG)
                self._ctrl_speed_steer(v, steer)

        # ── STRAIGHTEN phase ──────────────────────────────────────────────────
        # Speed held via full P-controller through the return arc.
        if self._phase == _STRAIGHTEN:
            herr = abs(_norm_angle(
                self._trigger_yaw - self.lead.get_transform().rotation.yaw))
            if herr < self.cfg.CUTOUT_SETTLE_DEG:
                self._phase = _SETTLED
                # Fall through to _SETTLED below
            else:
                steer = self._heading_steer(self._trigger_yaw)
                self._ctrl_speed_steer(v, steer)

        # ── SETTLED phase ─────────────────────────────────────────────────────
        if self._phase == _SETTLED:
            self._ctrl_settled(v)

        return just_triggered

    def cruise_ego(self):
        """Maintain ego speed (called when AEB controller has not yet commanded braking)."""
        cruise(self.ego, self.ego_ms)

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _lateral_offset(self):
        """Signed lateral displacement from trigger point in the right-vector frame."""
        loc = self.lead.get_location()
        return (loc.x - self._tx) * self._rx + (loc.y - self._ty) * self._ry

    def _heading_steer(self, target_yaw_deg):
        """P-controller: steer proportional to heading error, clamped."""
        cur = self.lead.get_transform().rotation.yaw
        err = _norm_angle(target_yaw_deg - cur)
        raw = self.cfg.CUTOUT_STEER_K * err
        return max(-self.cfg.CUTOUT_STEER_MAX, min(self.cfg.CUTOUT_STEER_MAX, raw))

    def _ctrl_speed_steer(self, v_ms, steer):
        """Full longitudinal P-controller with lateral steer — maintains speed through turns.
        Applies active braking on overspeed (unlike _throttle which only idles on overspeed),
        preventing speed drop from tire drag during the cut-out arc from turning into rear-end risk.
        """
        err = self.ego_ms - v_ms
        if err > 0.0:
            t = min(self.cfg.LEAD_SPEED_MAX_THROTTLE, self.cfg.LEAD_SPEED_K * err)
            self.lead.apply_control(
                carla.VehicleControl(throttle=t, steer=steer, brake=0.0))
        else:
            b = min(0.3, self.cfg.LEAD_SPEED_K * (-err))
            self.lead.apply_control(
                carla.VehicleControl(throttle=0.0, steer=steer, brake=b))

    def _throttle(self, v_ms):
        """P throttle to maintain ego speed; engine-brake only on overshoot (no active brake).
        Kept for reference; STEER/STRAIGHTEN now use _ctrl_speed_steer instead."""
        err = self.ego_ms - v_ms
        if err > 0.0:
            return min(self.cfg.LEAD_SPEED_MAX_THROTTLE,
                       self.cfg.LEAD_SPEED_K * err)
        return 0.0

    def _ctrl_cruise(self, v_ms):
        """Longitudinal P-controller, straight ahead."""
        err = self.ego_ms - v_ms
        if err > 0.0:
            t = min(self.cfg.LEAD_SPEED_MAX_THROTTLE, self.cfg.LEAD_SPEED_K * err)
            self.lead.apply_control(
                carla.VehicleControl(throttle=t, steer=0.0, brake=0.0))
        else:
            # Small brake to prevent runaway (mirrors throttle gain)
            b = min(0.3, self.cfg.LEAD_SPEED_K * (-err))
            self.lead.apply_control(
                carla.VehicleControl(throttle=0.0, steer=0.0, brake=b))

    def _ctrl_settled(self, v_ms):
        """Behaviour after completing the lane change."""
        if getattr(self.cfg, "CUTOUT_AFTER_STOP", False):
            # Decelerate to a full stop in the right lane
            if v_ms > 0.5:
                b = min(0.8, self.cfg.LEAD_SPEED_K * v_ms)
                self.lead.apply_control(
                    carla.VehicleControl(brake=b, throttle=0.0, steer=0.0))
            else:
                self.lead.apply_control(
                    carla.VehicleControl(brake=1.0, hand_brake=True, throttle=0.0))
        else:
            # Keep cruising in the right lane at ego speed
            self._ctrl_cruise(v_ms)
