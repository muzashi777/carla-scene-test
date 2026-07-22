# -*- coding: utf-8 -*-
"""
OpenCV display module (used only by run_single.py) — separated so headless mode does not need to import cv2
Draws detection boxes only:
  1) YOLO box (green) = every object the model sees
  2) hazard box (orange/red) = target that meets the corridor condition and is at risk of collision
"""
import cv2
import numpy as np


class Viz:
    def __init__(self, cfg):
        self.cfg = cfg
        self.win = "AEB cut-in - YOLO + hazard"
        # intrinsic matrix of the front camera (used to project 3D→2D)
        w, h, fov = cfg.CAM_W, cfg.CAM_H, cfg.CAM_FOV_DEG
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        self.K = np.array([[focal, 0, w / 2.0],
                           [0, focal, h / 2.0],
                           [0, 0, 1.0]])

    # ── 3D world → image pixel projection ─────────────────────────────
    def _project(self, camera, loc):
        try:
            w2c = np.array(camera.get_transform().get_inverse_matrix())
        except Exception:
            return None
        p = np.array([loc.x, loc.y, loc.z, 1.0])
        pc = w2c @ p
        pc = np.array([pc[1], -pc[2], pc[0]])  # UE → standard camera axes
        if pc[2] <= 0.05:                      # behind the camera
            return None
        pi = self.K @ pc
        return pi[0] / pi[2], pi[1] / pi[2]

    def _project_actor_box(self, camera, actor):
        """Project 8 corners of the bounding box → return 2D box (x1,y1,x2,y2) or None"""
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
        """Draw only the box for targets that meet the corridor condition"""
        if not hazard or not hazard.get("in_path"):
            return
        box = self._project_actor_box(hazard["camera"], hazard["actor"])
        if box is None:
            return
        x1, y1, x2, y2 = box
        if hazard.get("engaged"):
            color = (0, 0, 255)          # red  = in-path AND actively braking
        else:
            color = (0, 200, 0)          # green = detected / tracked, not yet braking
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    def frame(self, frame_bgr, results, overlay_lines, img_top, hazard=None):
        _ = overlay_lines, img_top
        # Layer 1: YOLO boxes (green = detected only)
        if results:
            for r in results:
                for b in r.boxes:
                    conf = float(b.conf[0])
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 200, 0), 1)
                    label = f"{conf * 100:.0f}%"
                    cv2.putText(frame_bgr, label, (x1 + 2, max(y1 - 4, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        # Layer 2: hazard box (orange/red = target the system uses to decide braking)
        self._draw_hazard(frame_bgr, hazard)
        cv2.imshow(self.win, frame_bgr)

        q = (cv2.waitKey(1) & 0xFF == ord('q'))
        return frame_bgr, q

    def finish(self, last_frame, result_txt):
        if last_frame is None:
            return
        hold = last_frame.copy()
        _ = result_txt
        print(">>> Press 'q' on the image window to close")
        while True:
            cv2.imshow(self.win, hold)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    def close(self):
        cv2.destroyAllWindows()
