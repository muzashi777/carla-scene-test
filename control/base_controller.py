# -*- coding: utf-8 -*-
"""
Interface contract for the 'controller plugin' — every model must inherit and return VehicleControl
Controllers can be swapped without touching any scenario/metric code; only the loaded class changes
The interface already accommodates steer (supports evasive maneuvers in the future)
"""
import carla
from core.types import Perception, EgoState


class BaseController:
    name = "base"

    def reset(self):
        """Called before each case — clears the internal latch"""
        self._engaged = False     # whether brake has been commanded before
        self._brake_held = 0.0    # held brake force (never decreases)

    def decide(self, perc: Perception, ego: EgoState) -> carla.VehicleControl:
        """Takes perception + ego state → returns vehicle command (throttle/brake/steer)"""
        raise NotImplementedError

    # ── helper shared by all controllers ──────────────────────────────
    @staticmethod
    def control(throttle=0.0, brake=0.0, steer=0.0):
        return carla.VehicleControl(
            throttle=float(throttle), brake=float(brake), steer=float(steer)
        )

    def _latch_brake(self, desired):
        """
        Brake latch: once braking starts it 'holds' and never releases back to throttle
        Brake force may increase (partial→full) but never decreases → prevents disengaging when TTC rebounds
        """
        if desired > 0.0:
            self._engaged = True
        if self._engaged:
            self._brake_held = max(self._brake_held, desired)
            return self._brake_held
        return 0.0

    def _emit(self, desired):
        """Converts desired brake (through latch) to VehicleControl"""
        b = self._latch_brake(desired)
        return self.control(brake=b) if b > 0.0 else self.control(throttle=0.6)


# ── latency compensation helper (shared by enhanced_predictive / enhanced_inflation) ──
def compensation_latency(run_spec, cfg):
    """Converts run_spec → (L in seconds, L frames actually used, comp_source)

    Latency-compensating controllers call this function to determine how many seconds ahead to predict
      comp_source='oracle'      → compensates with L = actual injected delay_frames (upper bound: L known exactly)
      comp_source='mismatched'  → compensates with comp_L_frames specified separately from delay_frames (robustness test)
    Empty/key-absent run_spec → default oracle + delay_frames=0 → L=0 → reduces to uncompensated behaviour
    Not directly coupled to PerceptionDegrader — values received exclusively via run_spec (loose coupling)
    """
    spec = run_spec or {}
    delay = spec.get("delay_frames", 0)
    src = spec.get("comp_source", "oracle")
    frames = spec.get("comp_L_frames", delay) if src == "mismatched" else delay
    dt = getattr(cfg, "FIXED_DT", 0.05)
    return max(0.0, frames * dt), frames, src


# ── registry: name → class map (run scripts call through this) ───────────
_REGISTRY = {}


def register(name):
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def make_controller(name, cfg, run_spec=None):
    if name not in _REGISTRY:
        raise KeyError(f"Unknown controller '{name}' (available: {list(_REGISTRY)})")
    ctrl = _REGISTRY[name](cfg)
    # bind run_spec so controllers that need it (e.g. latency compensation) can read it —
    # existing controllers do not touch this attribute, so their behaviour is unchanged
    ctrl.run_spec = run_spec or {}
    return ctrl
