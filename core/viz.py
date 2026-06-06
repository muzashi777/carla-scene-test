# -*- coding: utf-8 -*-
"""
ตัวโชว์ภาพด้วย OpenCV (ใช้เฉพาะ run_single.py) — แยกออกมาให้ headless ไม่ต้อง import cv2
วาด 2 ชั้น:
  1) กล่อง YOLO (เขียว) = ทุกวัตถุที่โมเดลเห็น
  2) กล่องสถานะ (ส้ม/แดง) = เป้าที่อยู่ในเงื่อนไข corridor (ground-truth) ที่ระบบใช้ตัดสินเบรก
     ฉายตำแหน่งจริงของ dart ลงบนภาพด้วย camera projection
"""
import cv2
import numpy as np


class Viz:
    def __init__(self, cfg):
        self.cfg = cfg
        self.win = "AEB cut-in - YOLO + hazard"
        self.win_top = "Top View"
        # intrinsic matrix ของกล้องหน้า (ใช้ฉาย 3D→2D)
        w, h, fov = cfg.CAM_W, cfg.CAM_H, cfg.CAM_FOV_DEG
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        self.K = np.array([[focal, 0, w / 2.0],
                           [0, focal, h / 2.0],
                           [0, 0, 1.0]])

    # ── projection 3D โลก → พิกเซลภาพ ─────────────────────────────
    def _project(self, camera, loc):
        try:
            w2c = np.array(camera.get_transform().get_inverse_matrix())
        except Exception:
            return None
        p = np.array([loc.x, loc.y, loc.z, 1.0])
        pc = w2c @ p
        pc = np.array([pc[1], -pc[2], pc[0]])  # UE → standard camera axes
        if pc[2] <= 0.05:                      # อยู่หลังกล้อง
            return None
        pi = self.K @ pc
        return pi[0] / pi[2], pi[1] / pi[2]

    def _project_actor_box(self, camera, actor):
        """ฉาย 8 มุมของ bounding box → คืนกรอบ 2D (x1,y1,x2,y2) หรือ None"""
        try:
            verts = actor.bounding_box.get_world_vertices(actor.get_transform())
        except Exception:
            verts = None
        pts = []
        if verts:
            for v in verts:
                pp = self._project(camera, v)
                if pp:
                    pts.append(pp)
        else:
            pp = self._project(camera, actor.get_location())
            if pp:
                pts.append(pp)
        if not pts:
            return None
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        x1 = int(max(0, min(xs))); y1 = int(max(0, min(ys)))
        x2 = int(min(self.cfg.CAM_W, max(xs))); y2 = int(min(self.cfg.CAM_H, max(ys)))
        if x2 - x1 < 3 or y2 - y1 < 3:
            return None
        return x1, y1, x2, y2

    def _draw_hazard(self, frame, hazard):
        """วาดกล่องสถานะของเป้าที่อยู่ในเงื่อนไข corridor"""
        if not hazard or not hazard.get("in_path"):
            return
        box = self._project_actor_box(hazard["camera"], hazard["actor"])
        if box is None:
            return
        x1, y1, x2, y2 = box
        if hazard.get("engaged"):
            color, tag = (0, 0, 255), "BRAKING"          # แดง = กำลังเบรก
        else:
            color, tag = (0, 165, 255), "IN-PATH"        # ส้ม = อยู่ในเส้นทาง
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        label = f"{tag}  lon {hazard['lon']:.1f}m lat {hazard['lat']:.1f}m ttc {hazard['ttc']:.2f}"
        ytxt = max(y1 - 10, 18)
        cv2.rectangle(frame, (x1, ytxt - 16), (x1 + 8 * len(label), ytxt + 4), color, -1)
        cv2.putText(frame, label, (x1 + 2, ytxt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def frame(self, frame_bgr, results, overlay_lines, img_top, hazard=None):
        c = self.cfg
        # แถบเลนพิกเซล (อ้างอิงโหมด yolo เดิม)
        cv2.line(frame_bgr, (c.LANE_LEFT, 0), (c.LANE_LEFT, c.CAM_H), (0, 255, 255), 1)
        cv2.line(frame_bgr, (c.LANE_RIGHT, 0), (c.LANE_RIGHT, c.CAM_H), (0, 255, 255), 1)
        # ชั้น 1: กล่อง YOLO (เขียว = เห็นเฉยๆ)
        if results:
            for r in results:
                for b in r.boxes:
                    cls_id = int(b.cls[0]); conf = float(b.conf[0])
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 200, 0), 1)
                    cv2.putText(frame_bgr, f"{conf:.2f}", (x1 + 1, max(y1 - 4, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        # ชั้น 2: กล่องสถานะ hazard (ส้ม/แดง = เป้าที่ระบบใช้ตัดสินเบรก)
        self._draw_hazard(frame_bgr, hazard)
        # ข้อความสรุปมุมบน
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
