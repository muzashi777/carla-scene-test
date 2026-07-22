# -*- coding: utf-8 -*-
"""Pure-geometry occlusion helpers (no CARLA dependency — safe to unit-test)."""


def sight_line_occluded(lead_lon, lead_lat, target_lon, target_lat,
                        lat_clear, lon_margin):
    """Return True if the lead vehicle is blocking the ego→target sight line.

    All arguments are in the ego's forward/right coordinate frame (metres):
      lead_lon/lat    — lead vehicle's longitudinal / lateral position
      target_lon/lat  — target vehicle's longitudinal / lateral position
      lat_clear       — minimum lateral separation (lead − target) before the
                        sight line is considered open (OCCLUSION_LAT_CLEAR)
      lon_margin      — lead is no longer "between" when
                        lead_lon >= target_lon − lon_margin (OCCLUSION_LON_MARGIN)

    The lead blocks the sight line when:
      • it is longitudinally between ego and (target − lon_margin), AND
      • it has not yet moved lat_clear metres away from the target laterally.
    """
    between = (0.0 < lead_lon < target_lon - lon_margin)
    return between and (abs(lead_lat - target_lat) < lat_clear)
