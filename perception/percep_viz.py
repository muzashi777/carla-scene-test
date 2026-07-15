# -*- coding: utf-8 -*-
"""
OpenCV visualizer for the perception-logging pass only (run_perception_log.py)
──────────────────────────────────────────────────────────────────────
Completely separate from core/viz.py used in the AEB scenario — not imported by any
runner/scenario/controller, so it does not affect other driving tests.
Draws all detections returned by YOLO (bounding box + class name + raw confidence).
cv2 is imported lazily (only when a window is opened) to allow headless runs without cv2.
"""
import cv2


class PercepViz:
    def __init__(self, cfg, win="3DGS perception - YOLO raw detections"):
        self.cfg = cfg
        self.win = win

    def _text(self, frame, txt, org, color=(255, 255, 255), scale=0.5):
        # Draw black outline first to ensure readability on both light and dark backgrounds
        cv2.putText(frame, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 1, cv2.LINE_AA)

    def show(self, frame_bgr, dets, overlay_lines):
        """Draw all detection boxes + overlay text then display; returns True if user presses 'q'"""
        for d in dets:
            color = self.cfg.CLASS_COLORS.get(d["class_id"], (0, 200, 0))
            x1, y1, x2, y2 = int(d["x1"]), int(d["y1"]), int(d["x2"]), int(d["y2"])
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, 2)
            label = f'{d["class_name"]} {d["confidence"] * 100:.0f}%'
            self._text(frame_bgr, label, (x1 + 2, max(y1 - 5, 12)), color)
        y = 22
        for line in overlay_lines:
            self._text(frame_bgr, line, (10, y), (255, 255, 255), 0.6)
            y += 26
        cv2.imshow(self.win, frame_bgr)
        return (cv2.waitKey(1) & 0xFF) == ord('q')

    def close(self):
        cv2.destroyAllWindows()
