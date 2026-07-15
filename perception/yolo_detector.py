# -*- coding: utf-8 -*-
"""
YOLO-based detector — single responsibility: report whether 'a vehicle is in the ego lane band and close enough'
Distance/speed used to compute TTC comes from CARLA ground-truth (see run loop)
This module therefore acts as a perception trigger (simulates real detection, including misses/latency)
"""
import numpy as np
from ultralytics import YOLO


class YoloDetector:
    def __init__(self, model_path, device, conf, iou, target_classes,
                 vehicle_cls, lane_left, lane_right, min_box_h):
        print(f"[YOLO] loading {model_path} ...")
        self.model = YOLO(model_path)
        print("[YOLO] loaded.")
        self.device = device
        self.conf = conf
        self.iou = iou
        self.target_classes = target_classes
        self.vehicle_cls = vehicle_cls
        self.lane_left = lane_left
        self.lane_right = lane_right
        self.min_box_h = min_box_h
        self.last_results = None   # kept for the drawing module to use

    @staticmethod
    def carla_image_to_bgr(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
        return arr[:, :, :3].copy()

    def detect(self, frame_bgr):
        """
        Returns (detected, in_band, best_box_h)
          detected   = a vehicle is in the lane band and tall enough (≥ min_box_h)
          in_band    = list of (cx, h) for every vehicle in the lane band
          best_box_h = height of the tallest box in the band (0 if none)
        """
        results = self.model.predict(
            source=frame_bgr, conf=self.conf, iou=self.iou,
            classes=self.target_classes, verbose=False, device=self.device,
        )
        self.last_results = results
        in_band = []
        for r in results:
            for b in r.boxes:
                if int(b.cls[0]) in self.vehicle_cls:
                    x1, y1, x2, y2 = b.xyxy[0]
                    cx = float((x1 + x2) / 2)
                    h = float(y2 - y1)
                    if self.lane_left < cx < self.lane_right:
                        in_band.append((cx, h))
        best_h = max((h for _, h in in_band), default=0.0)
        detected = best_h >= self.min_box_h
        return detected, in_band, best_h

    def _class_name(self, cid):
        """COCO class name from id (supports both dict and list for model.names)"""
        names = self.model.names
        try:
            return names[cid]
        except (KeyError, IndexError, TypeError):
            return str(cid)

    def detect_all(self, frame_bgr):
        """
        Run YOLO on a single frame and return 'all detections' from the model (no lane/height filtering).
        Used only for perception-quality logging (see perception/scene_logger.py) —
        not related to the AEB brake gate (detect()); uses the exact same conf/iou/classes/device settings
        and therefore reflects the same detector that AEB uses.
        Returns a list of dicts: {class_id, class_name, confidence, x1, y1, x2, y2}
        """
        results = self.model.predict(
            source=frame_bgr, conf=self.conf, iou=self.iou,
            classes=self.target_classes, verbose=False, device=self.device,
        )
        self.last_results = results
        dets = []
        for r in results:
            for b in r.boxes:
                cid = int(b.cls[0])
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                dets.append(dict(
                    class_id=cid, class_name=self._class_name(cid),
                    confidence=float(b.conf[0]),
                    x1=x1, y1=y1, x2=x2, y2=y2,
                ))
        return dets
