# -*- coding: utf-8 -*-
"""
ตัวตรวจจับด้วย YOLO — หน้าที่เดียว: บอกว่า 'มีรถในแถบเลน ego และใกล้พอไหม'
ระยะ/ความเร็วที่ใช้คำนวณ TTC จะมาจาก ground-truth ของ CARLA (ดูใน run loop)
ตัวนี้จึงทำหน้าที่เป็น perception trigger (เลียนการตรวจจับจริง รวมถึงพลาด/หน่วงได้)
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
        self.last_results = None   # เก็บไว้ให้ตัววาดใช้

    @staticmethod
    def carla_image_to_bgr(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
        return arr[:, :, :3].copy()

    def detect(self, frame_bgr):
        """
        คืน (detected, in_band, best_box_h)
          detected   = มีรถในเลนและสูงพอ (≥ min_box_h)
          in_band    = list ของ (cx, h) ของรถทุกคันในแถบเลน
          best_box_h = ความสูงกล่องที่ใหญ่สุดในแถบ (0 ถ้าไม่มี)
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
