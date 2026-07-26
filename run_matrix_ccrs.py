# -*- coding: utf-8 -*-
"""
Iterate through all TEST MATRIX cases for the CCRs scenario (headless, no display by default).
Writes results/ccrs_matrix_<TEST_MODE>_<stamp>.csv + prints CPEIM summary.

Before running: load the train000 scene in CARLA.
Usage:
    python run_matrix_ccrs.py                   # original controllers, no viz
    TEST_MODE=latency python run_matrix_ccrs.py
    MATRIX_VIZ=1 python run_matrix_ccrs.py      # opt-in display (all runs)
"""
import sys, os, itertools, datetime, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_ccrs as cfg
from core import actors
from core.carla_session import CarlaSession
from core.runner_ccrs import run_case
from core.metrics import write_csv, summarize
from core.viz import Viz
from perception.yolo_detector import YoloDetector


def build_cases():
    """5 speeds × 5 approach_d × 2 μ = 50 cases/controller.

    Fixed-target / ego-sweep design:
      target_pos = EGO_SPAWN + min(approach_d) × forward_vector   (FIXED for all cases)
      ego_pos    = EGO_SPAWN − (approach_d − min_d) × forward_vector  (swept backward)

    The target occupies a single validated spot (min_d ahead of the original ego spawn).
    Larger approach_d values move the ego further back; the target never moves.
    The ego's heading and scenario logic are unchanged.

    Ego positions (all 5) are in the BACKWARD direction from the original spawn (away from
    scene obstacles). They are on clear road by geometry; verify with try_spawn_actor output.
    """
    yaw_rad = math.radians(cfg.EGO_SPAWN["yaw"])
    fwd_x   = math.cos(yaw_rad)
    fwd_y   = math.sin(yaw_rad)
    min_d   = min(cfg.MATRIX["approach_d"])

    # Target: fixed at min_d ahead of original ego spawn
    target_x = cfg.EGO_SPAWN["x"] + min_d * fwd_x
    target_y = cfg.EGO_SPAWN["y"] + min_d * fwd_y

    cases = []
    for v, d, m in itertools.product(
            cfg.MATRIX["ego_speed_kmh"], cfg.MATRIX["approach_d"], cfg.MATRIX["mu"]):
        # Ego swept backward so distance from ego to (fixed) target equals d
        offset = d - min_d
        ego_x  = cfg.EGO_SPAWN["x"] - offset * fwd_x
        ego_y  = cfg.EGO_SPAWN["y"] - offset * fwd_y
        cases.append({
            "ego_speed_kmh": v,
            "approach_d":    d,
            "mu":            m,
            "ego_x":         round(ego_x, 3),
            "ego_y":         round(ego_y, 3),
            "target_x":      round(target_x, 3),
            "target_y":      round(target_y, 3),
        })
    return cases


def main():
    test_mode = os.environ.get("TEST_MODE", "original")
    matrix_runs = cfg.build_matrix_runs(test_mode)

    viz_enabled = os.environ.get("MATRIX_VIZ", "0") == "1"

    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    cases = build_cases()
    print(f"[MATRIX/ccrs] {len(cases)} cases/controller × {len(matrix_runs)} runs "
          f"= {len(cases)*len(matrix_runs)} runs  TEST_MODE={test_mode}"
          f"  MATRIX_VIZ={'on' if viz_enabled else 'off'}")

    # Spectator offset from original EGO_SPAWN (cosmetic; tracks per-case ego position).
    # Original: SPECTATOR_TF ≈ EGO_SPAWN + (-1.15, -0.52) — slightly behind ego looking fwd.
    _spec_dx = cfg.SPECTATOR_TF["x"] - cfg.EGO_SPAWN["x"]   # ≈ -1.15 m
    _spec_dy = cfg.SPECTATOR_TF["y"] - cfg.EGO_SPAWN["y"]   # ≈ -0.52 m

    records = []
    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        actors.check_scene(sess.world, cfg.EXPECTED_SCENE)
        actors.set_spectator(sess.world, cfg.SPECTATOR_TF)
        viz = Viz(cfg) if viz_enabled else None
        for run in matrix_runs:
            for i, case in enumerate(cases):
                print(f"\n--- [{run['label']}] case {i+1}/{len(cases)} {case} ---")
                # Move spectator to follow the per-case ego spawn (ego sweeps backward).
                # Ego can be up to ~40 m behind original spawn at approach_d=70 m —
                # the original fixed spectator would not frame the action at large distances.
                if "ego_x" in case:
                    _spec_tf = dict(cfg.SPECTATOR_TF,
                                    x=case["ego_x"] + _spec_dx,
                                    y=case["ego_y"] + _spec_dy)
                    actors.set_spectator(sess.world, _spec_tf)
                rec, _ = run_case(sess, cfg, case, run["controller"],
                                  run.get("delay_frames", 0), detector,
                                  run_spec=run, case_idx=i, test_mode=test_mode, viz=viz)
                rec.label = run["label"]
                records.append(rec)
        if viz is not None:
            viz.close()

    # ── Write CSV ──
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = getattr(cfg, "RESULTS_PREFIX", "ccrs_matrix")
    csv_path = os.path.join(cfg.RESULTS_DIR, f"{prefix}_{test_mode}_{stamp}.csv")
    write_csv(records, csv_path)
    print(f"\n[CSV] wrote results to {csv_path}")

    # ── Summarize ──
    summary = summarize(records)
    print("\n" + "=" * 72)
    print("Summary per controller (CCRs scenario):")
    labels = [r["label"] for r in matrix_runs]
    for label in labels:
        if label not in summary:
            continue
        s = summary[label]
        print(f"  {label:20s} | Rc_all={s['rc_all']*100:5.1f}% ({s['avoided']}/{s['n']}) "
              f"| Rc_conflict={s['rc_conflict']*100:5.1f}% ({s['n_conflict']} conflict)")
        print(f"  {'':20s}   MFDD={s['mean_ab']:5.2f} m/s² (n={s['n_ab']}) "
              f"| s_c={s['mean_sc']:.2f}m (n={s['n_sc']}) "
              f"| Tc={s['mean_tc']:.2f}s (n={s['n_tc']}) "
              f"| Δv={s['mean_dv']:.1f} km/h")

    if labels and labels[0] in summary:
        base = labels[0]
        print("-" * 72)
        for prop in labels[1:]:
            if prop not in summary:
                continue
            delta = (summary[prop]["rc"] - summary[base]["rc"]) * 100
            status = "✓ Pass" if delta >= 20 else "✗ Not yet — tune DYN_K_* / REQ_*_FRAC"
            print(f"Δ Rc ({prop} − {base}) = {delta:+.1f}%  | target +20%: {status}")
    print("=" * 72)
    print(f"[TEST_MODE={test_mode}]  CSV: {csv_path}")


if __name__ == "__main__":
    main()
