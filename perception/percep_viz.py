# -*- coding: utf-8 -*-
"""
ตัวโชว์ภาพ OpenCV เฉพาะ perception-logging pass (run_perception_log.py)
──────────────────────────────────────────────────────────────────────
แยกออกจาก core/viz.py ของฉาก AEB อย่างสิ้นเชิง — ไม่ถูก import โดย runner/
scenario/controller ใด ๆ จึงไม่กระทบการทดสอบขับขี่อื่น
วาด 'ทุก detection' ที่ YOLO คืนมา (กล่อง + ชื่อคลาส + confidence ดิบ)
import cv2 แบบ lazy (เฉพาะตอนเปิดหน้าต่าง) เพื่อให้รัน headless ได้โดยไม่ต้องมี cv2
"""
import cv2


class PercepViz:
    def __init__(self, cfg, win="3DGS perception - YOLO raw detections"):
        self.cfg = cfg
        self.win = win

    def _text(self, frame, txt, org, color=(255, 255, 255), scale=0.5):
        # วาดเส้นขอบดำก่อนเพื่อให้อ่านออกบนพื้นหลังสว่าง/มืด
        cv2.putText(frame, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 1, cv2.LINE_AA)

    def show(self, frame_bgr, dets, overlay_lines):
        """วาดกล่อง detection ทั้งหมด + ข้อความหัวภาพ แล้วแสดง; คืน True ถ้าผู้ใช้กด 'q'"""
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
