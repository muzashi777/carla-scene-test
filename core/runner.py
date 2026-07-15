# -*- coding: utf-8 -*-
"""
Single-case runner — wires all modules together and returns a RunRecord.
Shared by run_single.py (viz passed in = display enabled) and run_matrix.py (viz=None = headless).
Range/relative-speed values fed to the controller are CARLA ground-truth (as configured).
Frame delay applies only to the 'detected' flag to simulate perception latency.
"""
import queue
import math
from collections import deque

import carla

from core import actors
from core.scenario_cutin import CutInScenario
from core.types import Perception, EgoState
from core.metrics import RunRecord, MfddTracker
from core.conflict import cutin_is_conflict
from control.base_controller import make_controller, compensation_latency
from perception.degrade import PerceptionDegrader
from perception.predict import PerceptionPredictor
# imported to trigger register() calls (registers controller names in the registry)
import control.baseline_static_ttc   # noqa: F401
import control.proposed_dynamic_ttc  # noqa: F401
import control.proposed_enhanced     # noqa: F401
import control.enhanced_predictive   # noqa: F401
import control.enhanced_inflation    # noqa: F401


def run_case(sess, cfg, case, controller_name, delay_frames, detector,
             run_spec=None, case_idx=0, test_mode="original", viz=None):
    world = sess.world
    actor_list = []
    front_q = queue.Queue()
    top_q = queue.Queue()
    collision = {"hit": False, "with": None}
    spec = run_spec or {}
    _seed = (sum(ord(c) for c in spec.get("label", controller_name)) * 10000 + case_idx) % (2**32)

    label = f"{controller_name}"
    # ── check conflict case (kinematic, no sim needed) ──
    # a case is a conflict if an ego that never brakes would hit the dart within MAX_TICKS×FIXED_DT seconds
    # all cases in the cut-in matrix are conflicts (see proof in core/conflict.py)
    is_conflict = cutin_is_conflict(case, cfg)
    rec = RunRecord(
        label=label, controller=controller_name, delay_frames=delay_frames,
        ego_speed_kmh=case["ego_speed_kmh"], mu=case["mu"],
        trigger_d=case["trigger_d"], dart_speed_kmh=case["dart_speed_kmh"],
        is_conflict=is_conflict,
    )
    rec.noise_sigma_m  = spec.get("noise_sigma_m", 0.0)
    rec.noise_sigma_vr = spec.get("noise_sigma_vr", 0.0)
    rec.dropout_p      = spec.get("dropout_p", 0.0)
    rec.dropout_mode   = spec.get("dropout_mode", "freeze")
    rec.test_mode      = test_mode
    rec.seed           = _seed
    # latency compensation parameters (used by comp controllers; ignored by others)
    rec.comp_source    = spec.get("comp_source", "")
    rec.comp_L_frames  = spec.get("comp_L_frames",
                                  delay_frames if spec.get("comp_source") == "oracle" else 0)

    try:
        # ── EGO ──
        ego = actors.spawn_vehicle(world, **cfg.EGO_SPAWN)
        if not ego:
            rec.result_txt = "EGO spawn failed"; return rec, None
        actor_list.append(ego)
        ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        actors.set_friction(ego, case["mu"])

        # ── DART ──
        dart = actors.spawn_vehicle(world, **cfg.DART_SPAWN)
        if not dart:
            rec.result_txt = "DART spawn failed"; return rec, None
        actor_list.append(dart)
        dart.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

        # ── front camera (for YOLO) ──
        cam = actors.attach_rgb_camera(world, ego, cfg.CAM_FRONT_TF,
                                       cfg.CAM_W, cfg.CAM_H, lambda i: front_q.put(i),
                                       fov=cfg.CAM_FOV_DEG)
        actor_list.append(cam)
        # ── top-view camera (only when display is active) ──
        if viz is not None:
            cam_top = actors.attach_rgb_camera(world, ego, cfg.CAM_TOP_TF,
                                               cfg.CAM_W, cfg.CAM_H, lambda i: top_q.put(i))
            actor_list.append(cam_top)

        # ── collision ──
        def on_col(e):
            if not collision["hit"]:
                collision["hit"] = True
                collision["with"] = e.other_actor.type_id
        col = actors.attach_collision_sensor(world, ego, on_col)
        actor_list.append(col)

        # ── allow the scene to settle ──
        for _ in range(cfg.SETTLE_TICKS):
            wf = world.tick()
            try:
                actors.grab_synced(front_q, wf)
                if viz is not None:
                    actors.grab_synced(top_q, wf)
            except queue.Empty:
                pass

        # ── initialise controller + scenario ──
        # run_spec is passed to the controller (comp controllers read L from it; others ignore it)
        controller = make_controller(controller_name, cfg, run_spec=spec)
        controller.reset()
        degrader = PerceptionDegrader(
            delay_frames=delay_frames,
            noise_sigma_m=spec.get("noise_sigma_m", 0.0),
            noise_sigma_vr=spec.get("noise_sigma_vr", 0.0),
            dropout_p=spec.get("dropout_p", 0.0),
            dropout_mode=spec.get("dropout_mode", "freeze"),
            seed=_seed,
        )
        degrader.reset()
        # ── predictive-oracle compensation layer (TEST_MODE=latency_comp_all only) ──
        # the predictor sits "in front of" the base controller: degrade makes perception stale,
        # predict forecasts it back to the present (symmetric). Other modes have no 'predict' key → predictor=None (no-op)
        predictor = None
        if spec.get("predict"):
            L_sec, _cf, _cs = compensation_latency(spec, cfg)
            predictor = PerceptionPredictor(l_seconds=L_sec)
            predictor.reset()
        scen = CutInScenario(ego, dart, cfg, case)
        scen.start()
        ego_ms = actors.kmh_to_ms(case["ego_speed_kmh"])

        # Latency is handled entirely by PerceptionDegrader — this buffer is a no-op (maxlen=1).
        det_buffer = deque(maxlen=1)
        mfdd = MfddTracker(case["ego_speed_kmh"])
        ego_y0 = ego.get_location().y

        # ── surface-to-surface gap = centre distance − vehicle sizes ──
        # dist2d is the centre-to-centre distance; physical contact occurs when the bumpers touch (gap≈0)
        # dart crosses perpendicular → the face presented to ego is the side (extent.y)
        if cfg.AUTO_GAP_OFFSET:
            try:
                gap_offset = ego.bounding_box.extent.x + dart.bounding_box.extent.y
            except Exception:
                gap_offset = cfg.GAP_OFFSET
        else:
            gap_offset = cfg.GAP_OFFSET
        print(f"[GAP] gap_offset = {gap_offset:.2f} m (vehicle sizes subtracted from centre-to-centre distance)")

        def surface_gap(dc):
            return max(0.0, dc - gap_offset)

        prev_gap = surface_gap(actors.dist2d(ego, dart))

        brake_engaged = False
        brake_info = None        # (tick, gap, v_kmh, ego_y, ttc)
        stopped = False
        min_dist = 1e9
        min_gap = 1e9
        peak_decel = 0.0
        brake_v_model = None     # velocity in the kinematic model (initialised on first brake application) — see apply_kinematic_brake
        prev_v_ms = actors.speed_ms(ego)
        result_txt = "TIMEOUT"
        last_frame = None
        quit_flag = False

        # ── estimate dart speed / deceleration from its actual motion (reset each case) ──
        # fed to the proposed_enhanced controller (required-decel) — shared across all controllers
        # uses 'longitudinal' speed along the ego's heading: dart cuts laterally → this component ≈ 0
        #   → required_decel treats dart as a stationary obstacle (a_req=v_e²/2·gap) rather than one moving ahead
        prev_lead_ms = actors.long_speed_along(ego, dart)
        lead_decel_ema = 0.0
        rec.a_max = case["mu"] * cfg.GRAVITY     # deceleration ceiling μ·g (constant for the entire case)

        for tick in range(cfg.MAX_TICKS):
            wf = world.tick()
            try:
                if cfg.FRAME_SYNC:
                    img = actors.grab_synced(front_q, wf)
                    img_top = actors.grab_synced(top_q, wf) if viz is not None else None
                else:
                    img = front_q.get(timeout=2.0)
                    img_top = top_q.get(timeout=2.0) if viz is not None else None
            except queue.Empty:
                continue

            ego_y = ego.get_location().y
            v_kmh = actors.speed_kmh(ego)
            d = actors.dist2d(ego, dart)
            gap = surface_gap(d)
            min_dist = min(min_dist, d)
            min_gap = min(min_gap, gap)

            # ── check for collision immediately after reading state (before YOLO / frame drawing) ──
            # ensures fast break-out and that the retained frame matches the actual collision frame
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh

            scen.update()   # control dart (trigger / block)

            # ── PERCEPTION ──
            # YOLO: always drawn on display (debug / realism)
            frame = detector.carla_image_to_bgr(img)
            yolo_now, in_band, box_h = detector.detect(frame)
            # ground-truth: whether dart is in / entering the ego's forward path (predictive)
            gt_now, lon, lat = actors.inpath_hazard(
                ego, dart, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH,
                cfg.INPATH_LOOKAHEAD if cfg.INPATH_PREDICT else 0.0)

            # select the detection gate used for braking decisions, per DETECTION_SOURCE
            src = cfg.DETECTION_SOURCE
            if src == "yolo":
                detected_now = yolo_now
            elif src == "both_or":
                detected_now = gt_now or yolo_now
            else:  # "groundtruth"
                detected_now = gt_now

            # ── ground-truth range / relative speed (surface-to-surface) → TTC ──
            rel_speed = max(0.0, (prev_gap - gap) / cfg.FIXED_DT)
            prev_gap = gap
            ttc = (gap / rel_speed) if rel_speed > 1e-3 else math.inf

            # ── dart speed / deceleration (ground-truth, along ego's longitudinal axis) ──
            # longitudinal component ≈ 0 for a laterally cutting dart → treated as a stationary obstacle in required_decel
            lead_ms = actors.long_speed_along(ego, dart)
            raw_lead_decel = max(0.0, (prev_lead_ms - lead_ms) / cfg.FIXED_DT)
            prev_lead_ms = lead_ms
            lead_decel_ema = 0.3 * raw_lead_decel + 0.7 * lead_decel_ema   # EMA to reduce noise

            # ── frame-delay on detected flag only (perception latency) ──
            det_buffer.append(detected_now)
            perceived = det_buffer[0]

            perc = Perception(detected=detected_now, distance=gap,
                              rel_speed=rel_speed, ttc=ttc, box_h=box_h,
                              lead_speed=lead_ms, lead_decel=lead_decel_ema)
            ego_state = EgoState(speed_ms=actors.speed_ms(ego),
                                 speed_kmh=v_kmh, mu=case["mu"])
            perc, ego_state = degrader.apply(perc, ego_state)
            # compensate latency via predictor layer (if enabled) — forecasts perception L seconds forward
            if predictor is not None:
                perc, ego_state = predictor.apply(perc, ego_state)

            # ── controller decision ──
            ctrl = controller.decide(perc, ego_state)
            is_braking = ctrl.brake > 0.0

            if is_braking:
                if not brake_engaged:
                    brake_engaged = True
                    brake_info = (tick, gap, v_kmh, ego_y, ttc)
                    rec.a_req_at_brake = actors.required_decel(
                        ego_state.speed_ms, lead_ms, lead_decel_ema, gap)
                if cfg.BRAKE_MODEL == "kinematic":
                    # achievable deceleration is clamped at the friction ceiling a_max = μ·g (see actors.apply_kinematic_brake)
                    if brake_v_model is None:
                        brake_v_model = actors.speed_ms(ego)   # initialised from the actual speed at the moment braking begins
                    brake_v_model, a_applied = actors.apply_kinematic_brake(
                        ego, brake_v_model, ctrl.brake, case["mu"], cfg.FIXED_DT, cfg.GRAVITY)
                    if brake_engaged and a_applied > peak_decel:
                        peak_decel = a_applied   # peak_decel = actual deceleration after clamping (≤ a_max)
                else:
                    ego.apply_control(ctrl)
            elif not brake_engaged:
                scen.cruise_ego()   # maintain ego speed (only while braking has not started)
            # if already engaged but controller issues no brake (should not happen due to latch) → coast, do not reset speed

            # ── track MFDD + instantaneous / peak deceleration ──
            v_ms = actors.speed_ms(ego)
            inst_decel = (prev_v_ms - v_ms) / cfg.FIXED_DT if tick > 0 else 0.0
            prev_v_ms = v_ms
            # kinematic: peak_decel comes from the actual clamped deceleration (≤ a_max) in the brake block above
            # other models (apply_control): peak measured from the actual speed readback
            if brake_engaged and cfg.BRAKE_MODEL != "kinematic" and inst_decel > peak_decel:
                peak_decel = inst_decel
            mfdd.update(v_kmh, abs(ego_y - ego_y0))

            if tick % cfg.LOG_EVERY == 0 or brake_engaged:
                print(f"t={tick*cfg.FIXED_DT:5.2f}s | ego_y={ego_y:6.1f} "
                      f"v={v_kmh:5.1f} gap={gap:5.1f}m (d={d:4.1f}) ttc={ttc:5.2f} "
                      f"bk={ctrl.brake:.2f} decel={inst_decel:5.2f} "
                      f"{'BRAKE' if brake_engaged else 'cruise'} det={perceived}")

            # ── render display (if viz is active) ──
            if viz is not None:
                ov = [f"{controller_name} delay={delay_frames}f | v {v_kmh:.0f} | gap {gap:.1f}m ttc {ttc:.2f}",
                      f"{'BRAKE!' if brake_engaged else 'cruising'} | det[{src}]: {perceived} (lon {lon:.1f} lat {lat:.1f})"]
                hazard = {"camera": cam, "actor": dart, "in_path": gt_now,
                          "engaged": brake_engaged, "lon": lon, "lat": lat, "ttc": ttc}
                last_frame, q = viz.frame(frame, detector.last_results, ov, img_top, hazard)
                if q:
                    result_txt = "QUIT"; quit_flag = True; break

            # ── termination conditions ──
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh
                break
            if brake_engaged and v_kmh < cfg.STOP_KMH and not stopped:
                stopped = True
                result_txt = "AVOIDED"
                rec.avoided = True
                rec.s_clearance = gap
                if brake_info:
                    rec.brake_distance = abs(brake_info[3] - ego_y)
                break
            if ego_y < cfg.END_Y:
                result_txt = "NO BRAKE / passed"
                break

        # ── summarise indices ──
        rec.result_txt = result_txt
        rec.min_dist = min_dist
        rec.a_b_mfdd = mfdd.mfdd()
        rec.peak_decel = peak_decel
        if brake_info:
            rec.t_c_warn = 0.0 if math.isinf(brake_info[4]) else brake_info[4]
            v_at_brake = brake_info[2]
        else:
            v_at_brake = case["ego_speed_kmh"]
        rec.dv_speed_var = case["ego_speed_kmh"] - rec.collision_speed_kmh

        print("=" * 60)
        print(f"RESULT [{controller_name} delay={delay_frames}f "
              f"v={case['ego_speed_kmh']:.0f} mu={case['mu']} Δd={case['trigger_d']:.0f}] : {result_txt}")
        if brake_info:
            print(f"  brake engaged tick={brake_info[0]} gap={brake_info[1]:.1f}m "
                  f"v={brake_info[2]:.1f} ttc={rec.t_c_warn:.2f}s")
        print(f"  s={rec.s_clearance:.2f}m a_b(MFDD)={rec.a_b_mfdd:.2f} "
              f"peak_decel={rec.peak_decel:.2f} brake_dist={rec.brake_distance:.2f}m")
        print(f"  μ={case['mu']} → theoretical maximum deceleration ≈ μ·g = {case['mu']*cfg.GRAVITY:.2f} m/s²")
        print(f"  Δv={rec.dv_speed_var:.1f} min_gap={min_gap:.1f}m (min_dist={min_dist:.1f}m)")
        print("=" * 60)

        return rec, (last_frame, result_txt, quit_flag)

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
