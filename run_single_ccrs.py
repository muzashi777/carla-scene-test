# -*- coding: utf-8 -*-
"""
Run a single CCRs (Car-to-Car Rear Stationary) case with display.
Useful for debugging / tuning / presentation.

Before running: load the train000 scene in CARLA.
Usage:  python run_single_ccrs.py
Adjust: edit SINGLE_* in config/scenario_ccrs.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config.scenario_ccrs as cfg
from core import actors
from core.carla_session import CarlaSession
from core.runner_ccrs import run_case
from core.viz import Viz
from perception.yolo_detector import YoloDetector


def main():
    detector = YoloDetector(
        cfg.YOLO_MODEL, cfg.YOLO_DEVICE, cfg.CONF_THRESH, cfg.IOU_THRESH,
        cfg.TARGET_CLASSES, cfg.VEHICLE_CLS, cfg.LANE_LEFT, cfg.LANE_RIGHT, cfg.MIN_BOX_H,
    )
    viz = Viz(cfg) if cfg.SHOW_WINDOW else None

    with CarlaSession(cfg.HOST, cfg.PORT, cfg.TIMEOUT, cfg.FIXED_DT) as sess:
        actors.check_scene(sess.world, cfg.EXPECTED_SCENE)
        actors.set_spectator(sess.world, cfg.SPECTATOR_TF)
        print(f"[SIM] sync ON dt={cfg.FIXED_DT}s | controller={cfg.SINGLE_CONTROLLER} "
              f"delay={cfg.SINGLE_DELAY_FRAMES}f | scene=CCRs (train000)")
        rec, viz_out = run_case(
            sess, cfg, cfg.SINGLE_CASE,
            cfg.SINGLE_CONTROLLER, cfg.SINGLE_DELAY_FRAMES, detector, viz=viz,
        )
        if viz is not None and viz_out is not None:
            last_frame, result_txt, quit_flag = viz_out
            if not quit_flag:
                sess.unlock()
                viz.finish(last_frame, result_txt)
            viz.close()


if __name__ == "__main__":
    main()
