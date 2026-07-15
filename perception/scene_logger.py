# -*- coding: utf-8 -*-
"""
Perception-quality logger for 3DGS scenes (READ-ONLY pass — separate from AEB entirely)
──────────────────────────────────────────────────────────────────────────────
Goal (for paper): report how well YOLO detects objects that 'are part of the
reconstructed 3DGS scene' (parked cars, traffic lights, signs, etc.) →
supports the conclusion that the reconstructed real scene provides realistic
perception input

Principle:
  - Drive the 'ego camera' along the original straight trajectory (from EGO_SPAWN.y → END_Y along the −Y axis)
  - pass "background": spawn no vehicles at all — a single camera (sensor) moves along the trajectory
    → all detections come from the reconstructed 3DGS scene only
  - pass "with_actor": exact same trajectory + spawn 'target vehicle' (dart) at DART_SPAWN
    (stationary) → compare detection of 'inserted actor' against 'objects in the reconstructed scene'
  - Every N frames or M meters of travel → run YOLO on the RGB frame and log 'all detections'

Does not touch any controller / scenario / runner / braking / existing CSV results
Uses YOLO (yolov8n.pt), image resolution, and conf threshold identical to the AEB gate
(see detector.detect_all) → numbers reflect the same detector

Note DO-NOT: do not compute precision/recall and do not call confidence "accuracy" —
the reconstructed scene has no ground-truth labels; we log only raw detections + raw confidence
"""
import math
import queue

import carla

from core import actors


# ── Ego coordinate frame basis vectors (yaw in degrees) ──
def _basis(yaw_deg):
    """Return (forward, right) unit vectors on the XY plane in CARLA convention (left-handed)
    yaw=0   → forward=(1,0)  right=(0,1)
    yaw=-90 → forward=(0,-1) right=(1,0)  (our ego travels in the −Y direction)
    """
    y = math.radians(yaw_deg)
    fwd = (math.cos(y), math.sin(y))
    right = (-math.sin(y), math.cos(y))
    return fwd, right


def _camera_world_transform(ego_spawn, cam_tf, ego_y):
    """Compute the world transform of the front camera as if it were mounted on ego at (x0, ego_y, z0, yaw0)
    where cam_tf is the local offset (ego frame) identical to what is used when attaching the camera to ego
    Returns (camera_world_transform, ego_world_location)
    """
    yaw0 = ego_spawn["yaw"]
    fwd, right = _basis(yaw0)
    cx = cam_tf.get("x", 0.0)   # forward (ego frame)
    cy = cam_tf.get("y", 0.0)   # lateral
    cz = cam_tf.get("z", 0.0)   # upward
    ego_x = ego_spawn["x"]
    ego_z = ego_spawn["z"]
    wx = ego_x + fwd[0] * cx + right[0] * cy
    wy = ego_y + fwd[1] * cx + right[1] * cy
    wz = ego_z + cz
    rot = carla.Rotation(
        pitch=cam_tf.get("pitch", 0.0),     # ego pitch/roll = 0 → use camera values directly
        yaw=yaw0 + cam_tf.get("yaw", 0.0),
        roll=cam_tf.get("roll", 0.0),
    )
    loc = carla.Location(x=wx, y=wy, z=wz)
    return carla.Transform(loc, rot), carla.Location(x=ego_x, y=ego_y, z=ego_z)


def _spawn_world_camera(world, w, h, fov, world_tf, sink):
    """Spawn a standalone RGB camera (not attached to a vehicle) at the given world transform"""
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    if fov is not None:
        bp.set_attribute("fov", str(fov))
    cam = world.spawn_actor(bp, world_tf)
    cam.listen(sink)
    return cam


def run_pass(sess, cfg, detector, pass_type,
             sample_every_m=1.0, sample_every_frames=0,
             drive_kmh=30.0, settle_ticks=20, max_ticks=4000, viz=None):
    """
    Drive the camera along the straight trajectory (EGO_SPAWN.y → END_Y) and log all detections
      pass_type            : "background" (no vehicle spawned) | "with_actor" (spawn dart)
      sample_every_m  > 0  : sample every M meters of travel (primary mode)
      sample_every_frames  : if sample_every_m<=0, sample every N frames instead
      viz                  : PercepViz (display OpenCV window) or None (headless) — draws on sampled frames
    Returns (rows, stats)
      rows  : list of dicts (one row per detection)
      stats : dict summarising this pass (n_frames_sampled, n_detections, per_class{name:[conf,...]})
    """
    world = sess.world
    actor_list = []
    cam_q = queue.Queue()
    rows = []
    per_class = {}
    n_frames = 0
    n_dets = 0

    y0 = cfg.EGO_SPAWN["y"]
    y_end = cfg.END_Y
    step = actors.kmh_to_ms(drive_kmh) * cfg.FIXED_DT     # distance moved per tick (m, in the −Y direction)

    try:
        # ── (with_actor) spawn target vehicle dart at the original position, stationary = "actor inserted into the scene" ──
        if pass_type == "with_actor":
            dart = actors.spawn_vehicle(world, **cfg.DART_SPAWN)
            if dart is None:
                print("[PERCEP] ⚠ spawn dart (ego target) failed — skipping pass with_actor")
                return rows, dict(pass_type=pass_type, n_frames_sampled=0,
                                  n_detections=0, per_class={})
            actor_list.append(dart)
            actors.hold(dart)

        # ── spawn standalone camera at the start of trajectory ──
        cam_tf0, _ = _camera_world_transform(cfg.EGO_SPAWN, cfg.CAM_FRONT_TF, y0)
        cam = _spawn_world_camera(world, cfg.CAM_W, cfg.CAM_H, cfg.CAM_FOV_DEG,
                                  cam_tf0, lambda i: cam_q.put(i))
        actor_list.append(cam)

        # ── let the scene settle ──
        for _ in range(settle_ticks):
            wf = world.tick()
            try:
                actors.grab_synced(cam_q, wf)
            except queue.Empty:
                pass

        print(f"[PERCEP] pass='{pass_type}' driving camera y {y0:.1f} → {y_end:.1f} "
              f"(step {step:.3f} m/tick); "
              f"sample {'every %.2f m' % sample_every_m if sample_every_m > 0 else 'every %d frames' % sample_every_frames}")

        y = y0
        tick = 0
        frame_index = 0
        last_sample_y = None

        while y > y_end and tick < max_ticks:
            cam_tf, ego_loc = _camera_world_transform(cfg.EGO_SPAWN, cfg.CAM_FRONT_TF, y)
            cam.set_transform(cam_tf)
            wf = world.tick()
            try:
                img = actors.grab_synced(cam_q, wf)
            except queue.Empty:
                y -= step; tick += 1
                continue

            # ── decide whether to sample this frame ──
            if sample_every_m > 0:
                do_sample = (last_sample_y is None) or ((last_sample_y - y) >= sample_every_m - 1e-9)
            else:
                n = sample_every_frames if sample_every_frames > 0 else 1
                do_sample = (tick % n == 0)

            if do_sample:
                last_sample_y = y
                frame = detector.carla_image_to_bgr(img)
                dets = detector.detect_all(frame)
                sim_t = round(tick * cfg.FIXED_DT, 3)
                for d in dets:
                    area = max(0.0, d["x2"] - d["x1"]) * max(0.0, d["y2"] - d["y1"])
                    rows.append(dict(
                        pass_type=pass_type,
                        frame_index=frame_index,
                        timestamp=sim_t,
                        ego_x=round(ego_loc.x, 3),
                        ego_y=round(ego_loc.y, 3),
                        ego_yaw=round(cfg.EGO_SPAWN["yaw"], 3),
                        class_id=d["class_id"],
                        class_name=d["class_name"],
                        confidence=round(d["confidence"], 4),
                        bbox_x1=round(d["x1"], 2),
                        bbox_y1=round(d["y1"], 2),
                        bbox_x2=round(d["x2"], 2),
                        bbox_y2=round(d["y2"], 2),
                        bbox_area_px=round(area, 1),
                        img_width=cfg.CAM_W,
                        img_height=cfg.CAM_H,
                    ))
                    per_class.setdefault(d["class_name"], []).append(d["confidence"])
                    n_dets += 1

                # ── show OpenCV window (sampled frames only = matches what is logged) ──
                if viz is not None:
                    overlay = [
                        f"pass={pass_type}  frame={frame_index}  y={y:.1f}m  dets={len(dets)}",
                        f"conf>={detector.conf}  res={cfg.CAM_W}x{cfg.CAM_H}  (press 'q' to stop)",
                    ]
                    if viz.show(frame, dets, overlay):
                        print("[PERCEP] user pressed 'q' — stopping this pass")
                        n_frames += 1
                        frame_index += 1
                        break

                n_frames += 1
                frame_index += 1
                if n_frames % 10 == 0:
                    print(f"[PERCEP]   y={y:6.1f} frames={n_frames} dets={n_dets}")

            y -= step
            tick += 1

        print(f"[PERCEP] pass='{pass_type}' done: {n_frames} frames, {n_dets} detections")
        stats = dict(pass_type=pass_type, n_frames_sampled=n_frames,
                     n_detections=n_dets, per_class=per_class)
        return rows, stats

    finally:
        for a in actor_list:
            try:
                if hasattr(a, "stop"):
                    a.stop()
            except Exception:
                pass
            try:
                a.destroy()
            except Exception:
                pass
