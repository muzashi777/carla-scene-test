# -*- coding: utf-8 -*-
"""
Single-case runner for the Cut-out scenario.
Wires all modules together and returns a CutOutRecord.
Shared by run_single_cutout.py (viz supplied) and run_matrix_cutout.py (viz=None).

Three actors:
  ego    — AEB-controlled; cruises toward the revealed stationary target.
  lead   — starts ahead of ego at headway_d; matches ego speed until it cuts right.
  target — stationary from t=0; the actual collision object the AEB must avoid.

Geometry (train000, yaw = -146.54°):
  lead spawn = EGO_SPAWN + headway_d × forward_vector(yaw)
    forward_vector = (cos(yaw_rad), sin(yaw_rad))
  This is the general form of the existing axis-aligned formula in runner_lead_brake.py:
    yaw=-90° → cos=-0, sin=-1 → lead_y = EGO_SPAWN.y - headway_d  (unchanged behaviour)

Perception is tracked against the STATIONARY TARGET throughout:
  gap = surface_gap(dist2d(ego, target))
  lead_speed = 0, lead_decel = 0 → proposed_enhanced uses a_req = v_e² / (2·gap).

End-of-run condition replaces the axis-aligned ego_y < END_Y guard with:
  dist_from_start > initial_ego_target_dist + 15 m  (direction-agnostic).
"""
import queue
import math
from collections import deque
from dataclasses import dataclass, asdict   # noqa: F401

import carla

from core import actors
from core.scenario_cutout import CutOutScenario
from core.types import Perception, EgoState
from core.metrics import MfddTracker
from core.conflict import cutout_is_conflict
from control.base_controller import make_controller, compensation_latency
from perception.degrade import PerceptionDegrader
from perception.predict import PerceptionPredictor
import control.baseline_static_ttc   # noqa: F401
import control.proposed_dynamic_ttc  # noqa: F401
import control.proposed_enhanced     # noqa: F401
import control.enhanced_predictive   # noqa: F401
import control.enhanced_inflation    # noqa: F401


@dataclass
class CutOutRecord:
    """Per-run record for the Cut-out scenario."""
    label: str
    controller: str
    delay_frames: int
    ego_speed_kmh: float
    mu: float
    reveal_ttc: float = 0.0         # target TTC (s) at the cut-out trigger tick (matrix axis)
    headway_d: float = 0.0          # ego↔lead headway in metres (= FIXED_HEADWAY_THW × ego_ms)
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
    # ── Reveal / reaction diagnostics ────────────────────────────────────────
    range_at_reveal: float = -1.0   # surface gap (m) at first reveal tick (-1 = never revealed)
    ttc_at_reveal: float = -1.0     # TTC (s) at first reveal tick
    time_reveal_to_brake: float = -1.0  # s from reveal to brake onset (-1 = no brake after reveal)


def run_case(sess, cfg, case, controller_name, delay_frames, detector,
             run_spec=None, case_idx=0, test_mode="original", viz=None):
    world = sess.world
    actor_list = []
    front_q = queue.Queue()
    top_q = queue.Queue()
    collision = {"hit": False, "with": None}
    spec = run_spec or {}
    _seed = (sum(ord(c) for c in spec.get("label", controller_name)) * 10000 + case_idx) % (2**32)

    # ── Lead headway: max(FIXED_HEADWAY_THW × ego_ms, MIN_HEADWAY_M) ──
    ego_ms       = case["ego_speed_kmh"] / 3.6
    fixed_thw    = getattr(cfg, "FIXED_HEADWAY_THW", 0.8)
    min_hw_m     = getattr(cfg, "MIN_HEADWAY_M", 5.0)
    reveal_ttc   = case.get("reveal_ttc", 2.0)
    trigger_ttc  = getattr(cfg, "CUTOUT_TRIGGER_TTC", 1.5)

    headway_d = case.get(
        "headway_d",
        max(fixed_thw * ego_ms, min_hw_m),
    )

    # TTC-based trigger distance floor: lead gets at least CUTOUT_TRIGGER_TTC s to clear.
    # Formula branch: ego-to-target surface gap at trigger = reveal_ttc × ego_ms.
    # Floor branch:   lead TTC to target = CUTOUT_TRIGGER_TTC s (speed-scaled trigger).
    _formula_d = reveal_ttc * ego_ms + cfg.GAP_OFFSET - headway_d
    _floor_d   = trigger_ttc * ego_ms
    cutout_trigger_d = case.get(
        "cutout_trigger_d",
        max(_formula_d, _floor_d),
    )

    is_conflict = cutout_is_conflict(case, cfg)

    rec = CutOutRecord(
        label=controller_name, controller=controller_name, delay_frames=delay_frames,
        ego_speed_kmh=case["ego_speed_kmh"], mu=case["mu"],
        reveal_ttc=reveal_ttc, headway_d=headway_d,
        is_conflict=is_conflict,
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
            print(f"[SPAWN] EGO blocked at ({cfg.EGO_SPAWN['x']:.3f},{cfg.EGO_SPAWN['y']:.3f},{cfg.EGO_SPAWN['z']:.3f})")
            rec.result_txt = "EGO spawn failed"; return rec, None
        actor_list.append(ego)
        ego.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        actors.set_friction(ego, case["mu"])

        # Enforce minimum centre-to-centre headway so the lead's bounding box does not
        # overlap the ego at spawn (CARLA's try_spawn_actor rejects overlapping actors).
        # Proxy: treat lead half-length ≈ ego half-length (both vehicle.ue4.audi.tt class).
        _ego_half    = ego.bounding_box.extent.x
        _min_headway = max(2.0 * _ego_half + getattr(cfg, "SPAWN_CLEARANCE_M", 0.5),
                           min_hw_m)
        if headway_d < _min_headway:
            print(f"[SPAWN] headway_d={headway_d:.3f}m < min {_min_headway:.3f}m "
                  f"(bbox_min={2*_ego_half + getattr(cfg,'SPAWN_CLEARANCE_M',0.5):.3f}m, "
                  f"MIN_HEADWAY_M={min_hw_m:.1f}m); clamping to {_min_headway:.3f}m")
            headway_d        = _min_headway
            rec.headway_d    = headway_d
            _formula_d2      = reveal_ttc * ego_ms + cfg.GAP_OFFSET - headway_d
            cutout_trigger_d = max(_formula_d2, trigger_ttc * ego_ms)

        # Infeasibility guard: if the post-clamp lead-to-target surface gap at trigger is
        # below MIN_TRIGGER_SURF_GAP_M, the lead cannot complete the lane-change before
        # hitting the target.  Mark the cell as SCENARIO_INFEASIBLE (kept in matrix/CSV).
        _trig_surf_gap = cutout_trigger_d - 2.0 * _ego_half
        _min_trig_gap  = getattr(cfg, "MIN_TRIGGER_SURF_GAP_M", 1.0)
        if _trig_surf_gap < _min_trig_gap:
            print(f"[SPAWN] SCENARIO_INFEASIBLE: trigger surf gap {_trig_surf_gap:.3f}m "
                  f"< {_min_trig_gap:.1f}m at reveal_ttc={reveal_ttc:.1f}s, "
                  f"{case['ego_speed_kmh']:.0f} km/h — lead cannot manoeuvre out before "
                  f"hitting stationary target; marking cell and skipping run")
            rec.result_txt = "SCENARIO_INFEASIBLE"
            return rec, None

        # ── LEAD — spawn ahead of ego along the ego forward vector ──
        yaw_rad = math.radians(cfg.EGO_SPAWN["yaw"])
        lead_x = cfg.EGO_SPAWN["x"] + headway_d * math.cos(yaw_rad)
        lead_y = cfg.EGO_SPAWN["y"] + headway_d * math.sin(yaw_rad)
        lead = actors.spawn_vehicle(
            world, x=lead_x, y=lead_y,
            z=cfg.LEAD_SPAWN["z"], yaw=cfg.LEAD_SPAWN["yaw"],
            model=cfg.LEAD_SPAWN["model"])
        if not lead:
            print(f"[SPAWN] LEAD blocked at ({lead_x:.3f},{lead_y:.3f},{cfg.LEAD_SPAWN['z']:.3f}) "
                  f"— headway_d={headway_d:.1f}m")
            rec.result_txt = "SPAWN_BLOCKED"
            return rec, None
        actor_list.append(lead)
        lead.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        print(f"[LEAD] headway_d={headway_d:.1f}m → spawn ({lead_x:.2f}, {lead_y:.2f})")

        # ── TARGET (stationary) ──
        target = actors.spawn_vehicle(world, **cfg.TARGET_SPAWN)
        if not target:
            print(f"[SPAWN] TARGET blocked at ({cfg.TARGET_SPAWN['x']:.3f},{cfg.TARGET_SPAWN['y']:.3f},{cfg.TARGET_SPAWN['z']:.3f})")
            rec.result_txt = "TARGET spawn failed"; return rec, None
        actor_list.append(target)
        target.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))

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

        # ── Initial ego→target distance (direction-agnostic end-of-run guard) ──
        initial_ego_target_dist = actors.dist2d(ego, target)
        max_travel_m = initial_ego_target_dist + 15.0
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
        # Inject derived cutout_trigger_d into case so CutOutScenario picks it up.
        _case_with_trigger = dict(case, cutout_trigger_d=cutout_trigger_d)
        scen = CutOutScenario(ego, lead, target, cfg, _case_with_trigger)
        scen.start()

        det_buffer = deque(maxlen=1)
        mfdd = MfddTracker(case["ego_speed_kmh"])

        # ── Surface-to-surface gap: ego front ↔ target rear ──
        if cfg.AUTO_GAP_OFFSET:
            try:
                gap_offset = ego.bounding_box.extent.x + target.bounding_box.extent.x
            except Exception:
                gap_offset = cfg.GAP_OFFSET
        else:
            gap_offset = cfg.GAP_OFFSET
        print(f"[GAP] gap_offset = {gap_offset:.2f} m (ego+target extents)")

        def surface_gap(dc):
            return max(0.0, dc - gap_offset)

        prev_gap = surface_gap(actors.dist2d(ego, target))

        # ── Ego-perceives-lead: track lead surface gap separately ──
        # While the target is occluded by the lead, the ego treats the lead as an
        # in-path obstacle (car-following). Once the lead cuts out and the target is
        # revealed, the ego re-targets the stationary target.  Controller logic and
        # thresholds are unchanged; only the perceived obstacle switches.
        prev_lead_gap = max(0.0, actors.dist2d(ego, lead) - gap_offset)
        lead_decel_ema_lead = 0.0   # lead cruises at constant speed → decel ≈ 0

        brake_engaged = False
        brake_info = None
        stopped = False
        min_dist = 1e9
        min_gap = 1e9
        peak_decel = 0.0
        brake_v_model = None
        prev_v_ms = actors.speed_ms(ego)
        result_txt = "TIMEOUT"
        last_frame = None
        quit_flag = False
        reveal_tick = -1      # first tick where detected_now=True (occlusion gate open)
        reveal_gap  = -1.0   # surface gap at that tick
        reveal_ttc_val = -1.0  # TTC at that tick

        # Perception tracks target (stationary); lead_speed/lead_decel are always 0.
        lead_decel_ema = 0.0
        rec.a_max = case["mu"] * cfg.GRAVITY

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

            v_kmh = actors.speed_kmh(ego)
            # Gap and detection always measured to the TARGET (stationary obstacle)
            d = actors.dist2d(ego, target)
            gap = surface_gap(d)
            min_dist = min(min_dist, d)
            min_gap = min(min_gap, gap)

            if collision["hit"]:
                result_txt = f"COLLISION with {collision['with']}"
                rec.collision_with = collision["with"]
                rec.collision_speed_kmh = v_kmh

            # update() drives the lead cut-out logic; returns True on trigger tick
            just_triggered = scen.update()
            if just_triggered:
                print(f"[CUT-OUT] Lead cut-out triggered at t={tick*cfg.FIXED_DT:.2f}s "
                      f"lead-to-target dist = {actors.dist2d(lead, target):.1f}m")

            # ── PERCEPTION — against the stationary target ──
            frame = detector.carla_image_to_bgr(img)
            yolo_now, in_band, box_h = detector.detect(frame)
            gt_now, lon, lat = actors.inpath_hazard(
                ego, target, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH,
                cfg.INPATH_LOOKAHEAD if cfg.INPATH_PREDICT else 0.0)

            src = cfg.DETECTION_SOURCE
            if src == "yolo":
                detected_now = yolo_now
            elif src == "both_or":
                detected_now = gt_now or yolo_now
            else:
                detected_now = gt_now

            # ── Occlusion gate: suppress detection while lead blocks sight line ──
            if getattr(cfg, "OCCLUSION_GATE", False) and detected_now:
                _, lead_lon, lead_lat = actors.inpath_hazard(
                    ego, lead, cfg.INPATH_MAX_RANGE + 50.0, 999.0, 0.0)
                # Shift eye-point from ego actor origin to camera mount (x-axis only;
                # the gate is 2D lon/lat so z and y offsets have no effect).
                _cam_x = cfg.CAM_FRONT_TF.get("x", 0.0)
                if actors.sight_line_occluded(
                        lead_lon - _cam_x, lead_lat, lon - _cam_x, lat,
                        getattr(cfg, "OCCLUSION_LAT_CLEAR", 1.5),
                        getattr(cfg, "OCCLUSION_LON_MARGIN", 2.0)):
                    detected_now = False

            # ── Record first reveal tick (pre-degrader, geometric moment) ──
            if detected_now and reveal_tick < 0:
                reveal_tick = tick
                reveal_gap  = gap

            rel_speed = max(0.0, (prev_gap - gap) / cfg.FIXED_DT)
            prev_gap = gap
            ttc = (gap / rel_speed) if rel_speed > 1e-3 else math.inf

            # Capture TTC now that rel_speed is available
            if reveal_tick == tick and reveal_ttc_val < 0:
                reveal_ttc_val = ttc if not math.isinf(ttc) else -1.0

            # ── Ego-perceives-lead: car-follow the lead while target is occluded ──
            # While the lead is in the ego lane and the target is still occluded,
            # the ego perceives the lead as an in-path obstacle.  The controller
            # receives the lead's gap/rel_speed/ttc and treats it as a moving lead.
            # When the lead cuts out and the target becomes detectable, the ego
            # automatically re-targets the stationary target (occlusion gate opens).
            lead_d   = actors.dist2d(ego, lead)
            lead_gap = max(0.0, lead_d - gap_offset)
            lead_rel_speed = max(0.0, (prev_lead_gap - lead_gap) / cfg.FIXED_DT)
            prev_lead_gap  = lead_gap
            lead_ttc_ego   = (lead_gap / lead_rel_speed) if lead_rel_speed > 1e-3 else math.inf
            lead_ms_actual = actors.speed_ms(lead)

            lead_gt_now, _ll, _llt = actors.inpath_hazard(
                ego, lead, cfg.INPATH_MAX_RANGE, cfg.INPATH_HALF_WIDTH, 0.0)

            # Use lead as obstacle while: target occluded AND lead in path AND lead not yet settled
            _lead_settled = (scen.phase >= 3)  # _SETTLED
            use_lead_as_obs = (not detected_now and lead_gt_now and not _lead_settled)

            if use_lead_as_obs:
                # Report lead as the perceived obstacle; lead is moving so AEB car-follows
                obs_gap       = lead_gap
                obs_rel_speed = lead_rel_speed
                obs_ttc       = lead_ttc_ego
                obs_lead_ms   = lead_ms_actual
                obs_decel     = lead_decel_ema_lead
            else:
                # Target is visible (or lead settled); report target as obstacle
                obs_gap       = gap
                obs_rel_speed = rel_speed
                obs_ttc       = ttc
                obs_lead_ms   = actors.speed_ms(target)   # ≈ 0
                obs_decel     = lead_decel_ema

            det_buffer.append(detected_now or use_lead_as_obs)
            perceived = det_buffer[0]

            perc = Perception(detected=(detected_now or use_lead_as_obs),
                              distance=obs_gap, rel_speed=obs_rel_speed,
                              ttc=obs_ttc, box_h=box_h,
                              lead_speed=obs_lead_ms, lead_decel=obs_decel)
            ego_state = EgoState(speed_ms=actors.speed_ms(ego), speed_kmh=v_kmh, mu=case["mu"])
            perc, ego_state = degrader.apply(perc, ego_state)
            if predictor is not None:
                perc, ego_state = predictor.apply(perc, ego_state)

            ctrl = controller.decide(perc, ego_state)
            is_braking = ctrl.brake > 0.0

            if is_braking:
                if not brake_engaged:
                    brake_engaged = True
                    brake_info = (tick, gap, v_kmh, ego.get_location().y, ttc)
                    rec.a_req_at_brake = actors.required_decel(
                        ego_state.speed_ms, obs_lead_ms, obs_decel, obs_gap)
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

            v_ms = actors.speed_ms(ego)
            inst_decel = (prev_v_ms - v_ms) / cfg.FIXED_DT if tick > 0 else 0.0
            prev_v_ms = v_ms
            if brake_engaged and cfg.BRAKE_MODEL != "kinematic" and inst_decel > peak_decel:
                peak_decel = inst_decel
            ego_loc = ego.get_location()
            mfdd.update(v_kmh, math.hypot(ego_loc.x - ego_x0, ego_loc.y - ego_y0))

            if tick % cfg.LOG_EVERY == 0 or brake_engaged or just_triggered:
                print(f"t={tick*cfg.FIXED_DT:5.2f}s | v={v_kmh:5.1f} gap={gap:5.1f}m "
                      f"(d={d:4.1f}) ttc={ttc:5.2f} "
                      f"bk={ctrl.brake:.2f} decel={inst_decel:5.2f} "
                      f"{'BRAKE' if brake_engaged else 'cruise'} det={perceived}"
                      f"{' [CUT-OUT!]' if just_triggered else ''}")

            if viz is not None:
                ov = [f"{controller_name} delay={delay_frames}f | v {v_kmh:.0f} | gap {gap:.1f}m ttc {ttc:.2f}",
                      f"{'BRAKE!' if brake_engaged else 'cruising'} | det[{src}]: {perceived} (lon {lon:.1f} lat {lat:.1f})"]
                hazard = {"camera": cam, "actor": target, "in_path": gt_now,
                          "engaged": brake_engaged, "lon": lon, "lat": lat, "ttc": ttc}
                last_frame, q = viz.frame(frame, detector.last_results, ov, img_top, hazard)
                if q:
                    result_txt = "QUIT"; quit_flag = True; break

            # ── Termination ──
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
                    rec.brake_distance = abs(ego.get_location().y - brake_info[3])
                break
            dist_from_start = math.hypot(ego.get_location().x - ego_x0,
                                         ego.get_location().y - ego_y0)
            if dist_from_start > max_travel_m:
                result_txt = "NO BRAKE / passed"
                break

        # ── Summarise ──
        rec.result_txt = result_txt
        rec.min_dist = min_dist
        rec.a_b_mfdd = mfdd.mfdd()
        rec.peak_decel = peak_decel
        if brake_info:
            rec.t_c_warn = 0.0 if math.isinf(brake_info[4]) else brake_info[4]
        rec.dv_speed_var = case["ego_speed_kmh"] - rec.collision_speed_kmh

        if reveal_tick >= 0:
            rec.range_at_reveal = reveal_gap
            rec.ttc_at_reveal   = reveal_ttc_val
            if brake_info is not None and brake_info[0] >= reveal_tick:
                rec.time_reveal_to_brake = (brake_info[0] - reveal_tick) * cfg.FIXED_DT

        print("=" * 60)
        hw_txt = f"headway={headway_d:.1f}m(thw={headway_d/ego_ms:.2f}s)"
        print(f"RESULT [{controller_name} delay={delay_frames}f "
              f"v={case['ego_speed_kmh']:.0f} mu={case['mu']} {hw_txt}] : {result_txt}")
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
