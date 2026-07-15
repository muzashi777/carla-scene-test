# -*- coding: utf-8 -*-
"""
Single-case runner for the lead-brake scenario — ties all modules together, returns LeadBrakeRecord
Shared by both run_single_lead.py (viz supplied = with display) and run_matrix_lead.py (viz=None = headless)
Kept separate from core/runner.py (cut-in scenario) for ease of per-scenario testing/debugging
Differs from runner.py in that: the lead vehicle travels ahead in the same lane + surface-to-surface gap is rear-to-front
"""
import queue
import math
from collections import deque
from dataclasses import dataclass, asdict  # noqa: F401  (asdict used via metrics.write_csv)

import carla

from core import actors
from core.scenario_lead_brake import LeadBrakeScenario
from core.types import Perception, EgoState
from core.metrics import MfddTracker
from core.conflict import lead_brake_is_conflict
from control.base_controller import make_controller, compensation_latency
from perception.degrade import PerceptionDegrader
from perception.predict import PerceptionPredictor
# import to trigger register() (registers controller names)
import control.baseline_static_ttc   # noqa: F401
import control.proposed_dynamic_ttc  # noqa: F401
import control.proposed_enhanced     # noqa: F401
import control.enhanced_predictive   # noqa: F401
import control.enhanced_inflation    # noqa: F401


@dataclass
class LeadBrakeRecord:
    """Run record specific to the lead-brake scenario — metric field names match those used by metrics.summarize/write_csv"""
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    lead_speed_kmh: float          # lead vehicle speed before braking (km/h)
    mu: float
    headway_thw: float             # time headway THW (s) — 0 if using fixed-distance mode
    headway_d: float               # lead-to-ego gap at start in "actual metres" (computed from THW×speed or fixed value)
    avoided: bool = False
    collision_with: str = ""
    s_clearance: float = 0.0       # m (>0 = surface-to-surface clearance when fully stopped; 0 = collision)
    a_b_mfdd: float = 0.0          # m/s²
    t_c_warn: float = 0.0          # s (TTC at brake onset)
    dv_speed_var: float = 0.0      # km/h
    collision_speed_kmh: float = 0.0
    peak_decel: float = 0.0        # m/s²
    brake_distance: float = 0.0    # m
    min_dist: float = 0.0
    a_req_at_brake: float = 0.0    # m/s² deceleration "required" at brake onset (indicates how close to the μ·g limit)
    a_max: float = 0.0             # m/s² deceleration cap = μ·g for this case
    is_conflict: bool = True       # True = ego collides (kinematic, no-brake) — see core/conflict.py
    result_txt: str = ""
    noise_sigma_m: float = 0.0
    noise_sigma_vr: float = 0.0
    dropout_p: float = 0.0
    dropout_mode: str = "freeze"
    test_mode: str = "original"
    seed: int = 0
    comp_source: str = ""          # source of L used for compensation (''=no compensation, 'oracle', 'mismatched')
    comp_L_frames: int = 0         # L actually used for compensation (frames) — may ≠ delay_frames in mismatched mode


def run_case(sess, cfg, case, controller_name, delay_frames, detector,
             run_spec=None, case_idx=0, test_mode="original", viz=None):
    world = sess.world
    actor_list = []
    front_q = queue.Queue()
    top_q = queue.Queue()
    collision = {"hit": False, "with": None}
    spec = run_spec or {}
    _seed = (sum(ord(c) for c in spec.get("label", controller_name)) * 10000 + case_idx) % (2**32)

    # ── Lead vehicle headway: THW (seconds) → actual metres  or  fixed distance (original mode) ──
    # THW is always based on "ego" speed (we are the follower) even when lead_speed differs from ego
    ego_ms = case["ego_speed_kmh"] / 3.6
    if getattr(cfg, "USE_THW", False):
        headway_thw = case["headway_thw"]
        headway_d = ego_ms * headway_thw          # THW → actual metres
    else:
        headway_thw = 0.0
        headway_d = case["headway_d"]             # fixed distance (original behaviour)
    if headway_d > cfg.INPATH_MAX_RANGE:
        print(f"[WARN] headway_d={headway_d:.1f}m > INPATH_MAX_RANGE={cfg.INPATH_MAX_RANGE}m "
              f"— ground-truth detection gate may not see the lead vehicle at start")

    # ── Check conflict case (kinematic, no simulator needed) ──
    # This case is a conflict if an ego that never brakes would collide with the lead vehicle within MAX_TICKS×FIXED_DT seconds
    # Computed before creating rec so it can be written to CSV even if spawn fails (see core/conflict.py)
    _case_for_conflict = {**case, "headway_d": headway_d}
    is_conflict = lead_brake_is_conflict(_case_for_conflict, cfg)

    rec = LeadBrakeRecord(
        label=controller_name, controller=controller_name, delay_frames=delay_frames,
        ego_speed_kmh=case["ego_speed_kmh"], lead_speed_kmh=case["lead_speed_kmh"],
        mu=case["mu"], headway_thw=headway_thw, headway_d=headway_d,
        is_conflict=is_conflict,
    )
    rec.noise_sigma_m  = spec.get("noise_sigma_m", 0.0)
    rec.noise_sigma_vr = spec.get("noise_sigma_vr", 0.0)
    rec.dropout_p      = spec.get("dropout_p", 0.0)
    rec.dropout_mode   = spec.get("dropout_mode", "freeze")
    rec.test_mode      = test_mode
    rec.seed           = _seed
    # Latency compensation parameters (used by comp controllers; other controllers ignore these)
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

        # ── LEAD (parked ahead of ego in the same lane; y = ego.y − headway_d because ego travels in -Y direction) ──
        lead_y = cfg.EGO_SPAWN["y"] - headway_d
        lead = actors.spawn_vehicle(
            world, x=cfg.LEAD_SPAWN["x"], y=lead_y, z=cfg.LEAD_SPAWN["z"],
            yaw=cfg.LEAD_SPAWN["yaw"], model=cfg.LEAD_SPAWN["model"])
        if not lead:
            rec.result_txt = "LEAD spawn failed"; return rec, None
        actor_list.append(lead)
        lead.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

        # ── Front camera (for YOLO) ──
        cam = actors.attach_rgb_camera(world, ego, cfg.CAM_FRONT_TF,
                                       cfg.CAM_W, cfg.CAM_H, lambda i: front_q.put(i),
                                       fov=cfg.CAM_FOV_DEG)
        actor_list.append(cam)
        # ── Top-view camera (only when displaying visuals) ──
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

        # ── Allow the scene to settle ──
        for _ in range(cfg.SETTLE_TICKS):
            wf = world.tick()
            try:
                actors.grab_synced(front_q, wf)
                if viz is not None:
                    actors.grab_synced(top_q, wf)
            except queue.Empty:
                pass

        # ── Initialise controller + scenario ──
        # Pass run_spec to controller (comp controllers read L from here; others leave it untouched)
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
        # ── Predictive-oracle compensation layer (TEST_MODE=latency_comp_all only) ──
        # Predictor is placed "in front of" the base controller: degrader makes perception stale,
        # predictor forecasts it back to fresh (symmetric pair). Other modes have no 'predict' key → predictor=None (no-op)
        predictor = None
        if spec.get("predict"):
            L_sec, _cf, _cs = compensation_latency(spec, cfg)
            predictor = PerceptionPredictor(l_seconds=L_sec)
            predictor.reset()
        scen = LeadBrakeScenario(ego, lead, cfg, case)
        scen.start()

        # Latency is handled entirely by PerceptionDegrader — this buffer is a no-op (maxlen=1).
        det_buffer = deque(maxlen=1)
        mfdd = MfddTracker(case["ego_speed_kmh"])
        ego_y0 = ego.get_location().y

        # ── "Surface-to-surface" gap = centre-to-centre distance − vehicle extents ──
        # This scenario is rear-to-front; both vehicles face the same direction → subtract half the length of each (extent.x)
        if cfg.AUTO_GAP_OFFSET:
            try:
                gap_offset = ego.bounding_box.extent.x + lead.bounding_box.extent.x
            except Exception:
                gap_offset = cfg.GAP_OFFSET
        else:
            gap_offset = cfg.GAP_OFFSET
        print(f"[GAP] gap_offset = {gap_offset:.2f} m (subtracting half-length of ego + lead from centre-to-centre distance)")

        def surface_gap(dc):
            return max(0.0, dc - gap_offset)

        prev_gap = surface_gap(actors.dist2d(ego, lead))

        brake_engaged = False
        brake_info = None        # (tick, gap, v_kmh, ego_y, ttc)
        stopped = False
        min_dist = 1e9
        min_gap = 1e9
        peak_decel = 0.0
        brake_v_model = None     # speed in the kinematic model (captured at first brake application) — see apply_kinematic_brake
        prev_v_ms = actors.speed_ms(ego)
        result_txt = "TIMEOUT"
        last_frame = None
        quit_flag = False

        # ── Estimate lead vehicle speed/deceleration from actual motion (reset each case) ──
        # Do not read LEAD_DECEL directly from config — estimate via finite-difference + EMA for realism/reproducibility
        prev_lead_ms = actors.speed_ms(lead)
        lead_decel_ema = 0.0
        rec.a_max = case["mu"] * cfg.GRAVITY     # deceleration ceiling μ·g (constant throughout the case)

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
            d = actors.dist2d(ego, lead)
            gap = surface_gap(d)
            min_dist = min(min_dist, d)
            min_gap = min(min_gap, gap)

            # ── Check collision immediately after reading state (before YOLO / frame rendering) ──
            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh

            scen.update()   # keep lead stationary

            # ── PERCEPTION ──
            frame = detector.carla_image_to_bgr(img)
            yolo_now, in_band, box_h = detector.detect(frame)
            # ground-truth: is lead in the path ahead of ego (lead is stationary → no prediction needed)
            gt_now, lon, lat = actors.inpath_hazard(
                ego, lead, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH,
                cfg.INPATH_LOOKAHEAD if cfg.INPATH_PREDICT else 0.0)

            src = cfg.DETECTION_SOURCE
            if src == "yolo":
                detected_now = yolo_now
            elif src == "both_or":
                detected_now = gt_now or yolo_now
            else:  # "groundtruth"
                detected_now = gt_now

            # ── Ground-truth relative distance/speed (surface-to-surface) → TTC ──
            rel_speed = max(0.0, (prev_gap - gap) / cfg.FIXED_DT)
            prev_gap = gap
            ttc = (gap / rel_speed) if rel_speed > 1e-3 else math.inf

            # ── Lead vehicle speed/deceleration (ground-truth, estimated from actual motion) ──
            lead_ms = actors.speed_ms(lead)
            raw_lead_decel = max(0.0, (prev_lead_ms - lead_ms) / cfg.FIXED_DT)
            prev_lead_ms = lead_ms
            lead_decel_ema = 0.3 * raw_lead_decel + 0.7 * lead_decel_ema   # EMA to reduce noise

            # ── Delay detected signal by frames (perception latency) ──
            det_buffer.append(detected_now)
            perceived = det_buffer[0]

            perc = Perception(detected=detected_now, distance=gap,
                              rel_speed=rel_speed, ttc=ttc, box_h=box_h,
                              lead_speed=lead_ms, lead_decel=lead_decel_ema)
            ego_state = EgoState(speed_ms=actors.speed_ms(ego),
                                 speed_kmh=v_kmh, mu=case["mu"])
            perc, ego_state = degrader.apply(perc, ego_state)
            # Compensate latency with predictor layer (if enabled) — forecasts perception forward by L seconds
            if predictor is not None:
                perc, ego_state = predictor.apply(perc, ego_state)

            # ── Controller decision ──
            ctrl = controller.decide(perc, ego_state)
            is_braking = ctrl.brake > 0.0

            if is_braking:
                if not brake_engaged:
                    brake_engaged = True
                    brake_info = (tick, gap, v_kmh, ego_y, ttc)
                    rec.a_req_at_brake = actors.required_decel(
                        ego_state.speed_ms, lead_ms, lead_decel_ema, gap)
                if cfg.BRAKE_MODEL == "kinematic":
                    # Actual achievable deceleration is clamped at the friction ceiling a_max = μ·g (see actors.apply_kinematic_brake)
                    if brake_v_model is None:
                        brake_v_model = actors.speed_ms(ego)   # initialise from actual speed at brake onset
                    brake_v_model, a_applied = actors.apply_kinematic_brake(
                        ego, brake_v_model, ctrl.brake, case["mu"], cfg.FIXED_DT, cfg.GRAVITY)
                    if brake_engaged and a_applied > peak_decel:
                        peak_decel = a_applied   # peak_decel = actual deceleration after clamping (≤ a_max)
                else:
                    ego.apply_control(ctrl)
            elif not brake_engaged:
                scen.cruise_ego()   # maintain speed (only before braking begins)

            # ── Track MFDD + instantaneous/peak deceleration ──
            v_ms = actors.speed_ms(ego)
            inst_decel = (prev_v_ms - v_ms) / cfg.FIXED_DT if tick > 0 else 0.0
            prev_v_ms = v_ms
            # kinematic: peak_decel comes from actual deceleration after clamping (≤ a_max) in the brake step above
            # Other models (apply_control): measure peak from actual speed readback
            if brake_engaged and cfg.BRAKE_MODEL != "kinematic" and inst_decel > peak_decel:
                peak_decel = inst_decel
            mfdd.update(v_kmh, abs(ego_y - ego_y0))

            if tick % cfg.LOG_EVERY == 0 or brake_engaged:
                print(f"t={tick*cfg.FIXED_DT:5.2f}s | ego_y={ego_y:6.1f} "
                      f"v={v_kmh:5.1f} gap={gap:5.1f}m (d={d:4.1f}) ttc={ttc:5.2f} "
                      f"bk={ctrl.brake:.2f} decel={inst_decel:5.2f} "
                      f"{'BRAKE' if brake_engaged else 'cruise'} det={perceived}")

            # ── Display frame (if viz is available) ──
            if viz is not None:
                ov = [f"{controller_name} delay={delay_frames}f | v {v_kmh:.0f} | gap {gap:.1f}m ttc {ttc:.2f}",
                      f"{'BRAKE!' if brake_engaged else 'cruising'} | det[{src}]: {perceived} (lon {lon:.1f} lat {lat:.1f})"]
                hazard = {"camera": cam, "actor": lead, "in_path": gt_now,
                          "engaged": brake_engaged, "lon": lon, "lat": lat, "ttc": ttc}
                last_frame, q = viz.frame(frame, detector.last_results, ov, img_top, hazard)
                if q:
                    result_txt = "QUIT"; quit_flag = True; break

            # ── Termination conditions ──
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

        # ── Summarise metrics ──
        rec.result_txt = result_txt
        rec.min_dist = min_dist
        rec.a_b_mfdd = mfdd.mfdd()
        rec.peak_decel = peak_decel
        if brake_info:
            rec.t_c_warn = 0.0 if math.isinf(brake_info[4]) else brake_info[4]
        rec.dv_speed_var = case["ego_speed_kmh"] - rec.collision_speed_kmh

        print("=" * 60)
        hw_txt = (f"THW={headway_thw:.1f}s→{headway_d:.1f}m" if getattr(cfg, "USE_THW", False)
                  else f"headway={headway_d:.0f}m")
        print(f"RESULT [{controller_name} delay={delay_frames}f "
              f"v={case['ego_speed_kmh']:.0f} vlead={case['lead_speed_kmh']:.0f} "
              f"mu={case['mu']} {hw_txt}] : {result_txt}")
        if brake_info:
            print(f"  Brake onset tick={brake_info[0]} gap={brake_info[1]:.1f}m "
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
