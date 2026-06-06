# -*- coding: utf-8 -*-
"""จัดการ session กับ CARLA: เปิด sync mode, คืนค่า settings เดิมตอนจบ"""
import carla


class CarlaSession:
    """
    ใช้แบบ context manager:
        with CarlaSession(host, port) as sess:
            world = sess.world
            ...
    sync mode จะถูกเปิดให้อัตโนมัติ และคืนค่าเดิมเมื่อออกจาก with
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
        """ปลด sync mode กลับเป็น real-time (ไว้เดินดูฉากใน CARLA หลังจบ)"""
        s = self.world.get_settings()
        s.synchronous_mode = False
        self.world.apply_settings(s)

    def __exit__(self, exc_type, exc, tb):
        if self._original is not None:
            self.world.apply_settings(self._original)
        return False
