# -*- coding: utf-8 -*-
"""ตัวโชว์ภาพด้วย OpenCV (ใช้เฉพาะ run_single.py) — แยกออกมาให้ headless ไม่ต้อง import cv2"""
import cv2
import numpy as np


class Viz:
    def __init__(self, cfg):
        self.cfg = cfg
        self.win = "AEB cut-in - YOLO"
        self.win_top = "Top View"

    def frame(self, frame_bgr, results, overlay_lines, img_top):
        c = self.cfg
        cv2.line(frame_bgr, (c.LANE_LEFT, 0), (c.LANE_LEFT, c.CAM_H), (0, 255, 255), 1)
        cv2.line(frame_bgr, (c.LANE_RIGHT, 0), (c.LANE_RIGHT, c.CAM_H), (0, 255, 255), 1)
        if results:
            for r in results:
                for b in r.boxes:
                    cls_id = int(b.cls[0]); conf = float(b.conf[0])
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    cx = (x1 + x2) // 2; h = y2 - y1
                    in_band = (cls_id in c.VEHICLE_CLS) and (c.LANE_LEFT < cx < c.LANE_RIGHT)
                    if in_band and h >= c.MIN_BOX_H:
                        color, label, thick = (0, 0, 255), f"BRAKE h={h}", 3
                    elif in_band:
                        color, label, thick = (0, 165, 255), f"far h={h}", 1
                    else:
                        color, label, thick = (0, 200, 0), f"{conf:.2f}", 1
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, thick)
                    cv2.putText(frame_bgr, label, (x1 + 1, max(y1 - 4, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, thick, cv2.LINE_AA)
        for i, t in enumerate(overlay_lines):
            cv2.putText(frame_bgr, t, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(self.win, frame_bgr)

        if img_top is not None:
            arr = np.frombuffer(img_top.raw_data, dtype=np.uint8).reshape(
                (img_top.height, img_top.width, 4))
            cv2.imshow(self.win_top, arr[:, :, :3].copy())

        q = (cv2.waitKey(1) & 0xFF == ord('q'))
        return frame_bgr, q

    def finish(self, last_frame, result_txt):
        if last_frame is None:
            return
        hold = last_frame.copy()
        cv2.putText(hold, f"{result_txt}  -  PRESS q TO CLOSE",
                    (10, self.cfg.CAM_H - 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 255, 255), 2)
        print(">>> กด 'q' ที่หน้าต่างภาพเพื่อปิด")
        while True:
            cv2.imshow(self.win, hold)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    def close(self):
        cv2.destroyAllWindows()
