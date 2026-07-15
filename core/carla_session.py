# -*- coding: utf-8 -*-
"""Manage a CARLA session: enable sync mode on entry and restore original settings on exit."""
import carla


class CarlaSession:
    """
    Use as a context manager:
        with CarlaSession(host, port) as sess:
            world = sess.world
            ...
    Sync mode is enabled automatically and restored to its original state on exit.
    """
    def __init__(self, host, port, timeout, fixed_dt):
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()
        self._fixed_dt = fixed_dt
        self._original = None

    def __enter__(self):
        self._original = self.world.get_settings()
        s = self.world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = self._fixed_dt
        self.world.apply_settings(s)
        return self

    def unlock(self):
        """Disable sync mode and return to real-time (useful for manually inspecting the scene in CARLA after a run)."""
        s = self.world.get_settings()
        s.synchronous_mode = False
        self.world.apply_settings(s)

    def __exit__(self, exc_type, exc, tb):
        if self._original is not None:
            self.world.apply_settings(self._original)
        return False
