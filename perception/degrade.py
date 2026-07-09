# -*- coding: utf-8 -*-
"""
PerceptionDegrader — configurable latency, Gaussian noise, and dropout
applied to Perception before it reaches any controller.

Pipeline order: latency → noise → dropout

Latency:   FIFO buffer of length (delay_frames + 1); the controller always
           receives the Perception from delay_frames ticks ago.
           All latency for the run is handled here — det_buffer in the
           runner files is set to maxlen=1 (no-op) so latency is counted once.

Noise:     Gaussian N(0, sigma) added to:
             distance   (sigma = noise_sigma_m, metres)
             rel_speed  (sigma = noise_sigma_vr, m/s)
             lead_speed (sigma = noise_sigma_vr, m/s)
           distance is clamped to >= 0.
           ttc is recomputed from the noisy values so TTC-based controllers
           see consistent data.

Dropout:   Each tick independently draws Bernoulli(dropout_p).
           dropout_mode selects behaviour on a dropout event:
             "freeze"  (default) — return last-valid Perception unchanged
                        (entire object, including detected as-is).
                        Edge case: first-frame dropout with no valid frame yet
                        → pass current frame through (no crash, no None).
             "miss"    — force detected=False; leave all numeric fields as-is.
                        The controller's existing 'detected' gate handles gating.
           dropout_p=0 is a strict no-op regardless of mode.

Seeding:   Pass an integer seed to __init__; call reset() before each run.
           Same seed → same noise/dropout sequence every time.
"""
import copy
import math
import numpy as np
from collections import deque


class PerceptionDegrader:

    def __init__(self, delay_frames=0, noise_sigma_m=0.0, noise_sigma_vr=0.0,
                 dropout_p=0.0, dropout_mode="freeze", seed=None):
        self.delay_frames = delay_frames
        self.noise_sigma_m = noise_sigma_m
        self.noise_sigma_vr = noise_sigma_vr
        self.dropout_p = dropout_p
        self.dropout_mode = dropout_mode
        self._seed = seed
        self._buf = None
        self._rng = None
        self._last_valid = None

    def reset(self):
        """Clear latency buffer and reset RNG — call alongside controller.reset()."""
        self._buf = deque(maxlen=self.delay_frames + 1)
        self._rng = np.random.default_rng(self._seed)
        self._last_valid = None

    def apply(self, perc, ego):
        """
        Apply degradation pipeline (latency → noise → dropout).
        Returns (perc_degraded, ego). ego is passed through unchanged.
        All parameters at defaults → exact no-op (original mode).
        """
        # ── 1. Latency: FIFO buffer over full Perception ─────────────────
        # delay_frames=0 → maxlen=1 → buffer[0] is the just-appended item (no delay)
        self._buf.append(perc)
        perc_out = copy.copy(self._buf[0])  # fresh copy of the delayed frame

        # ── 2. Gaussian noise ────────────────────────────────────────────
        if self.noise_sigma_m > 0.0:
            perc_out.distance = max(
                0.0,
                perc_out.distance + float(self._rng.normal(0.0, self.noise_sigma_m))
            )
        if self.noise_sigma_vr > 0.0:
            perc_out.rel_speed  += float(self._rng.normal(0.0, self.noise_sigma_vr))
            perc_out.lead_speed += float(self._rng.normal(0.0, self.noise_sigma_vr))
        if self.noise_sigma_m > 0.0 or self.noise_sigma_vr > 0.0:
            rs = perc_out.rel_speed
            perc_out.ttc = (perc_out.distance / rs) if rs > 1e-3 else math.inf

        # ── 3. Dropout ───────────────────────────────────────────────────
        if self.dropout_p > 0.0 and float(self._rng.random()) < self.dropout_p:
            if self.dropout_mode == "miss":
                perc_out.detected = False
            else:  # "freeze"
                if self._last_valid is not None:
                    perc_out = copy.copy(self._last_valid)
                # else: first-frame dropout, no valid frame yet → pass through unchanged
        else:
            self._last_valid = perc_out

        return perc_out, ego
