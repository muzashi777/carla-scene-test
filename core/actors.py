# -*- coding: utf-8 -*-
"""ตัวช่วยจัดการ actor: spawn รถ, ตั้งความลื่น μ ที่ล้อ, ติดเซ็นเซอร์, ฟังก์ชันสถานะ"""
import math
import carla


def kmh_to_ms(k):
    return k / 3.6


def speed_kmh(actor):
    v = actor.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def speed_ms(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def dist2d(a, b):
    la, lb = a.get_location(), b.get_location()
    return math.hypot(la.x - lb.x, la.y - lb.y)


def grab_synced(q, frame_id, timeout=2.0):
    """อ่านภาพจากคิวจนกว่า image.frame ตรง/ใหม่กว่า frame_id (กันภาพดริฟต์ใน sync mode)"""
    while True:
        img = q.get(timeout=timeout)
        if img.frame >= frame_id:
            return img


def inpath_hazard(ego, other, max_range, half_width):
    """
    เช็กจาก ground-truth ว่า 'other' อยู่ในทางเดินข้างหน้า ego ไหม
    คืน (in_path, lon, lat): lon=ระยะตามแนวหน้า(>0=ข้างหน้า), lat=ระยะเยื้องข้าง
    """
    e = ego.get_transform()
    f = e.get_forward_vector()
    r = e.get_right_vector()
    le, lo = e.location, other.get_location()
    dx, dy = lo.x - le.x, lo.y - le.y
    lon = dx * f.x + dy * f.y
    lat = dx * r.x + dy * r.y
    in_path = (0.0 < lon <= max_range) and (abs(lat) <= half_width)
    return in_path, lon, lat


def spawn_vehicle(world, x, y, z, yaw, model="vehicle.*"):
    """spawn รถ 1 คัน คืน actor (None ถ้าล้มเหลว). แทน actor_spawner เดิม"""
    bp_lib = world.get_blueprint_library()
    candidates = bp_lib.filter(model)
    if not candidates:
        candidates = bp_lib.filter("vehicle.*")
    bp = candidates[0]
    tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
    return world.try_spawn_actor(bp, tf)


def set_friction(vehicle, mu):
    """ตั้งค่าความลื่นถนน μ ผ่าน tire_friction ของทุกล้อ (ใช้แทนการตั้งใน .xodr)"""
    pc = vehicle.get_physics_control()
    wheels = pc.wheels
    for w in wheels:
        w.tire_friction = float(mu)
    pc.wheels = wheels
    vehicle.apply_physics_control(pc)


def cruise(vehicle, target_ms):
    """รักษาความเร็วคงที่ตาม forward vector (open-loop เพราะไม่มี waypoint)"""
    f = vehicle.get_transform().get_forward_vector()
    vehicle.set_target_velocity(carla.Vector3D(f.x * target_ms, f.y * target_ms, 0.0))


def hold(vehicle):
    """ล็อกรถให้หยุดนิ่ง"""
    vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))


def attach_rgb_camera(world, parent, tf_dict, w, h, sink):
    """ติดกล้อง RGB เข้ากับ parent แล้วส่งภาพไป sink (เช่น queue.put)"""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    rot = carla.Rotation(
        pitch=tf_dict.get("pitch", 0.0),
        yaw=tf_dict.get("yaw", 0.0),
        roll=tf_dict.get("roll", 0.0),
    )
    tf = carla.Transform(
        carla.Location(x=tf_dict.get("x", 0.0), y=tf_dict.get("y", 0.0), z=tf_dict.get("z", 0.0)),
        rot,
    )
    cam = world.spawn_actor(bp, tf, attach_to=parent)
    cam.listen(sink)
    return cam


def attach_collision_sensor(world, parent, on_hit):
    bp = world.get_blueprint_library().find("sensor.other.collision")
    sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent)
    sensor.listen(on_hit)
    return sensor
