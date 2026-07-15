# -*- coding: utf-8 -*-
"""Central data structures passed to controllers (prevents circular imports)."""
from dataclasses import dataclass


@dataclass
class Perception:
    """What the controller 'sees' in this frame (after delay frames are applied)"""
    detected: bool        # Did YOLO detect a vehicle in the ego lane?
    distance: float       # Distance to obstacle (m) — ground-truth from CARLA
    rel_speed: float      # Closing speed (m/s, >0 = approaching)
    ttc: float            # distance / rel_speed (seconds, inf if not closing)
    box_h: float = 0.0    # YOLO bounding box height (px) — for debug/drawing
    # ── lead vehicle info (ground-truth, shared equally across all controllers for fairness) ──
    lead_speed: float = 0.0   # Lead vehicle speed (m/s) — baseline may ignore this
    lead_decel: float = 0.0   # Lead vehicle deceleration "estimated from actual motion" (m/s², ≥0)


@dataclass
class EgoState:
    speed_ms: float
    speed_kmh: float
    mu: float             # Current road friction coefficient (from case config)
