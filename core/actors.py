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


def required_decel(v_e, v_l, a_l, gap):
    """ความหน่วงต่ำสุดที่ ego ต้องใช้เพื่อหยุดให้ทันท้ายรถข้างหน้า (m/s²)
    คำนึงถึง 'ระยะที่รถข้างหน้ายังวิ่งต่อก่อนหยุด' (d_lead = v_l²/2a_l)
      v_e ≤ v_l            → ไม่ได้เข้าใกล้ → 0
      v_l ≈ 0              → d_lead = 0 (รถข้างหน้าหยุดแล้ว เช่น dart จอดขวาง)
      a_l > 0.1            → d_lead = v_l²/2a_l (รถข้างหน้ากำลังเบรก)
      ไม่เบรก (a_l≈0,v_l>0) → d_lead = inf (ยังไม่ฉุกเฉิน รถข้างหน้าไม่ได้ชะลอ)
    a_req = v_e² / (2·(gap + d_lead)); คืน inf ถ้าระยะหยุดที่มี ≈ 0 (สายเกินไป)
    ใช้ร่วมทั้งสมองกล (ตัดสินใจ) และ runner (บันทึก a_req ตอนเริ่มเบรก) ให้ตรงกัน
    """
    v_e = max(0.0, v_e); v_l = max(0.0, v_l); a_l = max(0.0, a_l); gap = max(0.0, gap)
    if v_e <= v_l:
        return 0.0
    if v_l <= 1e-3:
        d_lead = 0.0
    elif a_l > 0.1:
        d_lead = (v_l * v_l) / (2.0 * a_l)
    else:
        d_lead = math.inf
    D = gap + d_lead
    if D <= 1e-6:
        return math.inf
    return (v_e * v_e) / (2.0 * D)


def apply_kinematic_brake(ego, v_model, brake_cmd, mu, dt, g=9.81):
    """ขั้นเบรกแบบ kinematic ที่ 'จำกัดความหน่วงที่ทำได้จริง' ไม่ให้เกินเพดานแรงเสียดทาน a_max = μ·g
    ──────────────────────────────────────────────────────────────────
    เหตุผล: ความหน่วงที่ทำได้จริงถูกจำกัดด้วยแรงเสียดทานถนน จึงต้อง a ≤ μ·g เสมอ
      (สอดคล้องสมมติฐานโมเดล kinematic ทั้งระบบ — ดู README หัวข้อ 'Physics cap on braking deceleration')
    ติดตามความเร็วจาก 'v_model' (ไม่ใช่ค่าอ่านกลับจาก CARLA) เพื่อกันไม่ให้ฟิสิกส์ภายในของ
      เอนจินเพิ่มความหน่วงเกินเพดาน ซึ่งจะทำให้รถหยุดเร็วเกินจริง = ประเมินการหลบชนสูงเกินจริง
    ใช้ร่วม (shared path) ทั้ง runner.py (cut-in) และ runner_lead_brake.py → สมองกลทั้ง 3 ตัวถูก clamp เท่ากัน
    คืน (v_new, a_applied):
      v_new     = ความเร็วใหม่ (m/s) ที่ตั้งให้ ego ผ่าน set_target_velocity แล้ว
      a_applied = ความหน่วงที่ทำได้จริงหลัง clamp (m/s²) → ใช้บันทึกเป็น peak_decel (≤ a_max เสมอ)
    """
    a_max = max(0.0, mu) * g
    a_cmd = max(0.0, brake_cmd) * a_max      # คำสั่งเบรก 0..1 → ความหน่วงที่ขอ (อยู่ในช่วง 0..a_max)
    a_applied = min(a_cmd, a_max)            # hard clamp กันเกินเพดานแรงเสียดทาน μ·g
    v_new = max(0.0, v_model - a_applied * dt)
    f = ego.get_transform().get_forward_vector()
    ego.set_target_velocity(carla.Vector3D(f.x * v_new, f.y * v_new, 0.0))
    a_real = (v_model - v_new) / dt if dt > 0 else 0.0   # อาจน้อยกว่า a_applied ตอนความเร็วแตะ 0
    return v_new, a_real


def grab_synced(q, frame_id, timeout=2.0):
    """อ่านภาพจากคิวจนกว่า image.frame ตรง/ใหม่กว่า frame_id (กันภาพดริฟต์ใน sync mode)"""
    while True:
        img = q.get(timeout=timeout)
        if img.frame >= frame_id:
            return img


def inpath_hazard(ego, other, max_range, half_width, lookahead=0.0):
    """
    เช็กจาก ground-truth ว่า 'other' อยู่/กำลังเข้าทางเดินข้างหน้า ego ไหม
    lookahead>0 = เปิด predictive: ใช้ความเร็วด้านข้างของ other ทำนายการเข้าเลน
    คืน (in_path, lon, lat): lon=ระยะตามแนวหน้า(>0=ข้างหน้า), lat=ระยะเยื้องข้าง
    """
    e = ego.get_transform()
    f = e.get_forward_vector()
    r = e.get_right_vector()
    le, lo = e.location, other.get_location()
    dx, dy = lo.x - le.x, lo.y - le.y
    lon = dx * f.x + dy * f.y
    lat = dx * r.x + dy * r.y
    ahead = (0.0 < lon <= max_range)
    cur = ahead and (abs(lat) <= half_width)        # อยู่ในเลนแล้วจริง
    pred = False
    if lookahead > 0.0 and ahead:
        ov = other.get_velocity()
        lat_vel = ov.x * r.x + ov.y * r.y           # ความเร็วด้านข้างของ other
        entering = (lat * lat_vel < 0.0)            # กำลังวิ่งเข้าหากึ่งกลางเลน
        lat_future = lat + lat_vel * lookahead
        pred = entering and (abs(lat_future) <= half_width)
    return (cur or pred), lon, lat


def spawn_vehicle(world, x, y, z, yaw, model="vehicle.*"):
    """spawn รถ 1 คัน คืน actor (None ถ้าล้มเหลว). แทน actor_spawner เดิม"""
    bp_lib = world.get_blueprint_library()
    candidates = bp_lib.filter(model)
    if not candidates:
        candidates = bp_lib.filter("vehicle.*")
    bp = candidates[0]
    tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
    return world.try_spawn_actor(bp, tf)


def _wheel_friction_attr(wheel):
    """หาชื่อ attribute แรงเสียดทานบนล้อ (ต่างกันตามเวอร์ชัน CARLA)"""
    for name in ("tire_friction", "friction", "lateral_friction", "longitudinal_friction"):
        if hasattr(wheel, name):
            return name
    for name in dir(wheel):
        if "friction" in name.lower() and not name.startswith("_"):
            return name
    return None


def set_friction(vehicle, mu, verbose=True):
    """ตั้งความลื่นถนน μ ที่ล้อ; auto-detect ชื่อ attr ตามเวอร์ชัน CARLA, อ่านกลับยืนยัน"""
    pc = vehicle.get_physics_control()
    wheels = pc.wheels
    if not wheels:
        if verbose:
            print("[FRICTION] ⚠ รถไม่มีข้อมูลล้อใน physics control")
        return None
    attr = _wheel_friction_attr(wheels[0])
    if attr is None:
        avail = [a for a in dir(wheels[0]) if not a.startswith("_")]
        if verbose:
            print("[FRICTION] ⚠ ไม่พบ attribute แรงเสียดทานบนล้อ — μ ไม่ถูกตั้ง!")
            print(f"[FRICTION] attribute ที่ล้อมีให้: {avail}")
        return None
    for w in wheels:
        setattr(w, attr, float(mu))
    pc.wheels = wheels
    vehicle.apply_physics_control(pc)
    readback = [round(getattr(w, attr), 3) for w in vehicle.get_physics_control().wheels]
    if verbose:
        print(f"[FRICTION] ใช้ attr '{attr}' ตั้ง μ={mu} → อ่านกลับต่อล้อ = {readback}")
        target = round(float(mu), 3)
        if not all(abs(r - target) < 1e-3 for r in readback):
            print("[FRICTION] ⚠ ค่าไม่ติด/ไม่ตรง — physics ไม่รับ μ ผ่าน path นี้")
            print("[FRICTION]   แนะนำตั้ง BRAKE_MODEL='kinematic' ใน config (คุม μ ตรงๆ)")
    return readback


def cruise(vehicle, target_ms):
    """รักษาความเร็วคงที่ตาม forward vector (open-loop เพราะไม่มี waypoint)"""
    f = vehicle.get_transform().get_forward_vector()
    vehicle.set_target_velocity(carla.Vector3D(f.x * target_ms, f.y * target_ms, 0.0))


def hold(vehicle):
    """ล็อกรถให้หยุดนิ่ง"""
    vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    vehicle.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))


def attach_rgb_camera(world, parent, tf_dict, w, h, sink, fov=None):
    """ติดกล้อง RGB เข้ากับ parent แล้วส่งภาพไป sink (เช่น queue.put)"""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    if fov is not None:
        bp.set_attribute("fov", str(fov))
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
