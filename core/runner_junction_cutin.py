# -*- coding: utf-8 -*-
"""
Single-case runner for the Junction Cut-in scenario.
Wires all modules together and returns a JunctionCutInRecord.
Shared by run_single_junction_cutin.py (viz supplied) and run_matrix_junction_cutin.py (viz=None).

Two actors:
  ego      — AEB-controlled; cruises toward the intruder once it enters the lane.
  intruder — stationary at junction until trigger fires; turns right into ego lane;
             brakes to a full stop (the actual collision object the AEB must avoid).

Conflict: computed before the try: block using junction_cutin_is_conflict()
          (kinematic check against stationary INTRUDER_STOP position — no CARLA needed).

End-of-run condition: direction-agnostic distance-from-start guard (per Rev 2026-07-22):
  dist_from_start > initial_ego_intruder_dist + 15 m → NO BRAKE / passed
"""
import queue
import math
from collections import deque
from dataclasses import dataclass   # noqa: F401

import carla

from core import actors
from core.scenario_junction_cutin import JunctionCutInScenario
from core.types import Perception, EgoState
from core.metrics import MfddTracker
from core.conflict import junction_cutin_is_conflict
from control.base_controller import make_controller, compensation_latency
from perception.degrade import PerceptionDegrader
from perception.predict import PerceptionPredictor
import control.baseline_static_ttc   # noqa: F401
import control.proposed_dynamic_ttc  # noqa: F401
import control.proposed_enhanced     # noqa: F401
import control.enhanced_predictive   # noqa: F401
import control.enhanced_inflation    # noqa: F401


@dataclass
class JunctionCutInRecord:
    """Per-run record for the Junction Cut-in scenario."""
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    mu: float
    trigger_d: float = 0.0      # ego↔intruder distance at which turn fires (matrix axis, m)
    avoided: bool = False
    collision_with: str = ""
    s_clearance: float = 0.0
    a_b_mfdd: float = 0.0
    t_c_warn: float = 0.0
    dv_speed_var: float = 0.0
    collision_speed_kmh: float = 0.0
    peak_decel: float = 0.0
    brake_distance: float = 0.0
    min_dist: float = 0.0
    a_req_at_brake: float = 0.0
    a_max: float = 0.0
    is_conflict: bool = True
    result_txt: str = ""
    noise_sigma_m: float = 0.0
    noise_sigma_vr: float = 0.0
    dropout_p: float = 0.0
    dropout_mode: str = "freeze"
    test_mode: str = "original"
    seed: int = 0
    comp_source: str = ""
    comp_L_frames: int = 0
    # ── Diagnostic ───────────────────────────────────────────────────────────
    turn_tick: int = -1      # simulation tick when intruder turn triggered (-1 = never)
    turn_dist: float = -1.0  # ego↔intruder distance at turn trigger (m)


def run_case(sess, cfg, case, controller_name, delay_frames, detector,
             run_spec=None, case_idx=0, test_mode="original", viz=None):
    world = sess.world
    actor_list = []
    front_q = queue.Queue()
    top_q = queue.Queue()
    collision = {"hit": False, "with": None}
    spec = run_spec or {}
    _seed = (sum(ord(c) for c in spec.get("label", controller_name)) * 10000 + case_idx) % (2**32)

    trigger_d = case.get("trigger_d", cfg.TURN_TRIGGER_D)

    # ── Conflict check (kinematic, no sim needed) — before try: block ──
    # Conflict = unbraked ego would collide with stationary intruder at INTRUDER_STOP
    # within MAX_TICKS × FIXED_DT seconds.  Independent of trigger_d and controller.
    is_conflict = junction_cutin_is_conflict(case, cfg)

    rec = JunctionCutInRecord(
        label=controller_name, controller=controller_name, delay_frames=delay_frames,
        ego_speed_kmh=case["ego_speed_kmh"], mu=case["mu"],
        trigger_d=trigger_d, is_conflict=is_conflict,
    )
    rec.noise_sigma_m  = spec.get("noise_sigma_m", 0.0)
    rec.noise_sigma_vr = spec.get("noise_sigma_vr", 0.0)
    rec.dropout_p      = spec.get("dropout_p", 0.0)
    rec.dropout_mode   = spec.get("dropout_mode", "freeze")
    rec.test_mode      = test_mode
    rec.seed           = _seed
    rec.comp_source    = spec.get("comp_source", "")
    rec.comp_L_frames  = spec.get("comp_L_frames",
                                  delay_frames if spec.get("comp_source") == "oracle" else 0)

    try:
        # ── EGO ──
        ego = actors.spawn_vehicle(world, **cfg.EGO_SPAWN)
        if not ego:
            print(f"[SPAWN] EGO blocked at "
                  f"({cfg.EGO_SPAWN['x']:.3f},{cfg.EGO_SPAWN['y']:.3f},{cfg.EGO_SPAWN['z']:.3f})")
            rec.result_txt = "SPAWN_BLOCKED"; return rec, None
        actor_list.append(ego)
        ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        actors.set_friction(ego, case["mu"])

        # ── INTRUDER ──
        intruder = actors.spawn_vehicle(world, **cfg.INTRUDER_SPAWN)
        if not intruder:
            print(f"[SPAWN] INTRUDER blocked at "
                  f"({cfg.INTRUDER_SPAWN['x']:.3f},{cfg.INTRUDER_SPAWN['y']:.3f},"
                  f"{cfg.INTRUDER_SPAWN['z']:.3f})")
            rec.result_txt = "SPAWN_BLOCKED"; return rec, None
        actor_list.append(intruder)
        intruder.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

        # ── Front camera ──
        cam = actors.attach_rgb_camera(world, ego, cfg.CAM_FRONT_TF,
                                       cfg.CAM_W, cfg.CAM_H, lambda i: front_q.put(i),
                                       fov=cfg.CAM_FOV_DEG)
        actor_list.append(cam)
        if viz is not None:
            cam_top = actors.attach_rgb_camera(world, ego, cfg.CAM_TOP_TF,
                                               cfg.CAM_W, cfg.CAM_H, lambda i: top_q.put(i))
            actor_list.append(cam_top)

        # ── Collision sensor ──
        def on_col(e):
            if not collision["hit"]:
                collision["hit"] = True
                collision["with"] = e.other_actor.type_id
        col = actors.attach_collision_sensor(world, ego, on_col)
        actor_list.append(col)

        # ── Settle ──
        for _ in range(cfg.SETTLE_TICKS):
            wf = world.tick()
            try:
                actors.grab_synced(front_q, wf)
                if viz is not None:
                    actors.grab_synced(top_q, wf)
            except queue.Empty:
                pass

        # ── Direction-agnostic end-of-run guard ──────────────────────────────
        # Replaces the axis-aligned ego_y < END_Y guard used in scene03_2.
        # Computed from actual spawn positions — no manual tuning when coords change.
        initial_ego_intruder_dist = actors.dist2d(ego, intruder)
        max_travel_m = initial_ego_intruder_dist + 15.0
        ego_x0, ego_y0 = ego.get_location().x, ego.get_location().y

        # ── Controller + degrader + optional predictor ──
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
        predictor = None
        if spec.get("predict"):
            L_sec, _cf, _cs = compensation_latency(spec, cfg)
            predictor = PerceptionPredictor(l_seconds=L_sec)
            predictor.reset()

        scen = JunctionCutInScenario(ego, intruder, cfg, case)
        scen.start()

        det_buffer = deque(maxlen=1)
        mfdd = MfddTracker(case["ego_speed_kmh"])

        # ── Surface-to-surface gap ──
        # When intruder is in lane (same heading as ego): ego front ↔ intruder rear
        # = ego.extent.x + intruder.extent.x (both facing same direction).
        if cfg.AUTO_GAP_OFFSET:
            try:
                gap_offset = ego.bounding_box.extent.x + intruder.bounding_box.extent.x
            except Exception:
                gap_offset = cfg.GAP_OFFSET
        else:
            gap_offset = cfg.GAP_OFFSET
        print(f"[GAP] gap_offset = {gap_offset:.2f} m (ego+intruder extents)")

        def surface_gap(dc):
            return max(0.0, dc - gap_offset)

        prev_gap = surface_gap(actors.dist2d(ego, intruder))

        brake_engaged = False
        brake_info    = None   # (tick, gap, v_kmh, ego_y, ttc)
        stopped       = False
        min_dist      = 1e9
        min_gap       = 1e9
        peak_decel    = 0.0
        brake_v_model = None
        prev_v_ms     = actors.speed_ms(ego)
        result_txt    = "TIMEOUT"
        last_frame    = None
        quit_flag     = False

        prev_lead_ms    = actors.long_speed_along(ego, intruder)
        lead_decel_ema  = 0.0
        rec.a_max = case["mu"] * cfg.GRAVITY

        for tick in range(cfg.MAX_TICKS):
            wf = world.tick()
            try:
                if cfg.FRAME_SYNC:
                    img     = actors.grab_synced(front_q, wf)
                    img_top = actors.grab_synced(top_q, wf) if viz is not None else None
                else:
                    img     = front_q.get(timeout=2.0)
                    img_top = top_q.get(timeout=2.0) if viz is not None else None
            except queue.Empty:
                continue

            v_kmh = actors.speed_kmh(ego)
            d     = actors.dist2d(ego, intruder)
            gap   = surface_gap(d)
            min_dist = min(min_dist, d)
            min_gap  = min(min_gap, gap)

            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with       = collision["with"]
                rec.collision_speed_kmh  = v_kmh

            just_triggered = scen.update(tick=tick)
            if just_triggered:
                rec.turn_tick = scen.turn_tick
                rec.turn_dist = scen.turn_dist
                print(f"[JUNCTION] trigger tick={tick} t={tick*cfg.FIXED_DT:.2f}s "
                      f"ego↔intruder={scen.turn_dist:.1f}m")

            # ── PERCEPTION — against the intruder ──
            frame = detector.carla_image_to_bgr(img)
            yolo_now, in_band, box_h = detector.detect(frame)
            gt_now, lon, lat = actors.inpath_hazard(
                ego, intruder, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH,
                cfg.INPATH_LOOKAHEAD if cfg.INPATH_PREDICT else 0.0)

            src = cfg.DETECTION_SOURCE
            if src == "yolo":
                detected_now = yolo_now
            elif src == "both_or":
                detected_now = gt_now or yolo_now
            else:   # "groundtruth"
                detected_now = gt_now

            # ── Ground-truth range / relative speed (surface-to-surface) → TTC ──
            rel_speed = max(0.0, (prev_gap - gap) / cfg.FIXED_DT)
            prev_gap  = gap
            ttc = (gap / rel_speed) if rel_speed > 1e-3 else math.inf

            # ── Intruder speed / decel (longitudinal component along ego heading) ──
            lead_ms = actors.long_speed_along(ego, intruder)
            raw_lead_decel = max(0.0, (prev_lead_ms - lead_ms) / cfg.FIXED_DT)
            prev_lead_ms   = lead_ms
            lead_decel_ema = 0.3 * raw_lead_decel + 0.7 * lead_decel_ema

            det_buffer.append(detected_now)
            perceived = det_buffer[0]

            perc = Perception(detected=detected_now, distance=gap,
                              rel_speed=rel_speed, ttc=ttc, box_h=box_h,
                              lead_speed=lead_ms, lead_decel=lead_decel_ema)
            ego_state = EgoState(speed_ms=actors.speed_ms(ego),
                                 speed_kmh=v_kmh, mu=case["mu"])
            perc, ego_state = degrader.apply(perc, ego_state)
            if predictor is not None:
                perc, ego_state = predictor.apply(perc, ego_state)

            # ── Controller decision ──
            ctrl = controller.decide(perc, ego_state)
            is_braking = ctrl.brake > 0.0

            if is_braking:
                if not brake_engaged:
                    brake_engaged = True
                    brake_info = (tick, gap, v_kmh, ego.get_location().y, ttc)
                    rec.a_req_at_brake = actors.required_decel(
                        ego_state.speed_ms, lead_ms, lead_decel_ema, gap)
                if cfg.BRAKE_MODEL == "kinematic":
                    if brake_v_model is None:
                        brake_v_model = actors.speed_ms(ego)
                    brake_v_model, a_applied = actors.apply_kinematic_brake(
                        ego, brake_v_model, ctrl.brake, case["mu"], cfg.FIXED_DT, cfg.GRAVITY)
                    if brake_engaged and a_applied > peak_decel:
                        peak_decel = a_applied
                else:
                    ego.apply_control(ctrl)
            elif not brake_engaged:
                scen.cruise_ego()

            v_ms       = actors.speed_ms(ego)
            inst_decel = (prev_v_ms - v_ms) / cfg.FIXED_DT if tick > 0 else 0.0
            prev_v_ms  = v_ms
            if brake_engaged and cfg.BRAKE_MODEL != "kinematic" and inst_decel > peak_decel:
                peak_decel = inst_decel
            ego_loc = ego.get_location()
            mfdd.update(v_kmh, math.hypot(ego_loc.x - ego_x0, ego_loc.y - ego_y0))

            if tick % cfg.LOG_EVERY == 0 or brake_engaged or just_triggered:
                print(f"t={tick*cfg.FIXED_DT:5.2f}s | v={v_kmh:5.1f} "
                      f"gap={gap:5.1f}m (d={d:4.1f}) ttc={ttc:5.2f} "
                      f"bk={ctrl.brake:.2f} decel={inst_decel:5.2f} "
                      f"{'BRAKE' if brake_engaged else 'cruise'} det={perceived}"
                      f"{' [TURN!]' if just_triggered else ''}")

            if viz is not None:
                ov = [f"{controller_name} delay={delay_frames}f | v {v_kmh:.0f} "
                      f"| gap {gap:.1f}m ttc {ttc:.2f}",
                      f"{'BRAKE!' if brake_engaged else 'cruising'} "
                      f"| det[{src}]: {perceived} (lon {lon:.1f} lat {lat:.1f})"]
                hazard = {"camera": cam, "actor": intruder, "in_path": gt_now,
                          "engaged": brake_engaged, "lon": lon, "lat": lat, "ttc": ttc}
                last_frame, q = viz.frame(frame, detector.last_results, ov, img_top, hazard)
                if q:
                    result_txt = "QUIT"; quit_flag = True; break

            # ── Termination ──
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with      = collision["with"]
                rec.collision_speed_kmh = v_kmh
                break
            if brake_engaged and v_kmh < cfg.STOP_KMH and not stopped:
                stopped    = True
                result_txt = "AVOIDED"
                rec.avoided     = True
                rec.s_clearance = gap
                if brake_info:
                    rec.brake_distance = abs(ego.get_location().y - brake_info[3])
                break
            dist_from_start = math.hypot(ego_loc.x - ego_x0, ego_loc.y - ego_y0)
            if dist_from_start > max_travel_m:
                result_txt = "NO BRAKE / passed"
                break

        # ── Summarise ──
        rec.result_txt = result_txt
        rec.min_dist   = min_dist
        rec.a_b_mfdd   = mfdd.mfdd()
        rec.peak_decel = peak_decel
        if brake_info:
            rec.t_c_warn = 0.0 if math.isinf(brake_info[4]) else brake_info[4]
        rec.dv_speed_var = case["ego_speed_kmh"] - rec.collision_speed_kmh

        print("=" * 60)
        print(f"RESULT [{controller_name} delay={delay_frames}f "
              f"v={case['ego_speed_kmh']:.0f} mu={case['mu']} "
              f"trigger_d={trigger_d:.0f}m] : {result_txt}")
        if brake_info:
            print(f"  brake onset tick={brake_info[0]} gap={brake_info[1]:.1f}m "
                  f"v={brake_info[2]:.1f} ttc={rec.t_c_warn:.2f}s")
        print(f"  s={rec.s_clearance:.2f}m a_b(MFDD)={rec.a_b_mfdd:.2f} "
              f"peak_decel={rec.peak_decel:.2f} brake_dist={rec.brake_distance:.2f}m")
        print(f"  μ={case['mu']} → a_max = μ·g = {case['mu']*cfg.GRAVITY:.2f} m/s²")
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
