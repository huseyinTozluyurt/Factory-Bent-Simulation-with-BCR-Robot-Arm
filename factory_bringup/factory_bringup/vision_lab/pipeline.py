"""
DetectionPipeline (no ROS imports): one detector + one world -> world-frame detections per bent.

    pipe = DetectionPipeline("opencv_contours", world="dual")
    dets = pipe.process(bgr)                         # List[ObjectDetection], x / y / bent filled
    view = pipe.draw(bgr, dets)                      # annotated image

The same object is used live by the ROS lab node and offline on image files / synthetic scenes,
so a detector that works offline works in the simulation unchanged.
"""

import math
import time
from typing import Dict, List, Optional, Union

import cv2
import numpy as np

from factory_bringup.vision_lab.detectors.base import Detector, DetectorContext, ObjectDetection, create_detector
from factory_bringup.vision_lab.worlds import BELT_Z, OBJECT_TOP_Z, WorldProfile, get_world

BENT_COLORS = {1: (0, 255, 0), 2: (0, 200, 255), None: (160, 160, 160)}


def build_lane_masks(camera, lanes, belt_z: float) -> Dict[str, np.ndarray]:
    uu, vv = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
    wx, wy = camera.pixel_to_world(uu, vv, belt_z)
    masks = {}
    for lane in lanes:
        if getattr(lane, "seg", None):                   # a straight piece: stops at the belt ends
            x0, y0, x1, y1 = lane.seg
            L = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            along = (wx - x0) * ux + (wy - y0) * uy
            across = -(wx - x0) * uy + (wy - y0) * ux
            m = (np.abs(across) <= lane.half_width) & (along >= 0) & (along <= L)
        else:
            r = np.hypot(wx - lane.center_x, wy - lane.center_y)
            m = (r >= lane.inner_radius) & (r <= lane.outer_radius)
        masks[lane.name] = m.astype(np.uint8) * 255
    return masks


class DetectionPipeline:
    def __init__(self, detector: Union[str, Detector, type] = "opencv_contours", world=None,
                 detector_options: Optional[Dict] = None, belt_z: float = BELT_Z, object_top_z: float = OBJECT_TOP_Z,
                 keep_off_lane: bool = False, min_score: float = 0.0):
        self.world: WorldProfile = world if isinstance(world, WorldProfile) else get_world(world)
        self.detector = create_detector(detector, **(detector_options or {}))
        self.belt_z, self.object_top_z = belt_z, object_top_z
        self.keep_off_lane = keep_off_lane      # also report objects that are on no belt (bent None)
        self.min_score = min_score
        self.ctx: Optional[DetectorContext] = None
        self._segments = any(getattr(l, "seg", None) for l in self.world.lanes)
        self.size = None
        self.last_ms = 0.0

    def _ensure(self, w: int, h: int):
        if self.size == (w, h):
            return
        cam = self.world.camera(w, h)
        masks = build_lane_masks(cam, self.world.lanes, self.belt_z)
        self.ctx = DetectorContext(camera=cam, lanes=self.world.lanes, lane_masks=masks, belt_z=self.belt_z,
                                   object_top_z=self.object_top_z, world=self.world)
        self.detector.setup(self.ctx)
        self.size = (w, h)

    @property
    def camera(self):
        return self.ctx.camera if self.ctx else self.world.camera()

    def bent_at_pixel(self, u: float, v: float) -> Optional[int]:
        ui, vi = int(round(u)), int(round(v))
        for i, lane in enumerate(self.world.lanes):
            m = self.ctx.lane_masks[lane.name]
            if 0 <= vi < m.shape[0] and 0 <= ui < m.shape[1] and m[vi, ui]:
                return i + 1
        return None

    def to_world(self, d: ObjectDetection) -> ObjectDetection:
        cam, z = self.ctx.camera, self.object_top_z
        if d.x is None or d.y is None:
            wx, wy = cam.pixel_to_world(d.u, d.v, z)
            d.x, d.y = float(wx), float(wy)
        if d.box_px is not None and (d.size_x is None or d.yaw is None):
            bx, by = cam.pixel_to_world(d.box_px[:, 0], d.box_px[:, 1], z)
            a = math.hypot(bx[1] - bx[0], by[1] - by[0])
            b = math.hypot(bx[2] - bx[1], by[2] - by[1]) if len(bx) > 2 else a
            if a >= b:
                yaw, sx, sy = math.atan2(by[1] - by[0], bx[1] - bx[0]), a, b
            else:
                yaw, sx, sy = math.atan2(by[2] - by[1], bx[2] - bx[1]), b, a
            if d.size_x is None:
                d.size_x, d.size_y = float(sx), float(sy)
            if d.yaw is None and not d.extra.get("axis_aligned"):
                d.yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
        if d.bent is None or self._segments:
            d.bent = self.bent_at_pixel(d.u, d.v)
        return d

    def process(self, bgr: np.ndarray, exclude_mask: Optional[np.ndarray] = None) -> List[ObjectDetection]:
        h, w = bgr.shape[:2]
        self._ensure(w, h)
        self.ctx.exclude_mask = exclude_mask
        t0 = time.perf_counter()
        raw = self.detector.detect(bgr) or []
        out = []
        for d in raw:
            if d.score < self.min_score:
                continue
            if exclude_mask is not None:
                ui, vi = int(round(d.u)), int(round(d.v))
                if 0 <= vi < h and 0 <= ui < w and exclude_mask[vi, ui]:
                    continue
            self.to_world(d)
            if d.bent is None and not self.keep_off_lane:
                continue
            out.append(d)
        self.last_ms = (time.perf_counter() - t0) * 1000.0
        return out

    # ------------------------------------------------------------------------------- drawing
    def draw(self, bgr: np.ndarray, dets: List[ObjectDetection], roi: Optional[Dict] = None, status: str = "",
             arm_mask: Optional[np.ndarray] = None, thumbnails: bool = True) -> np.ndarray:
        h, w = bgr.shape[:2]
        self._ensure(w, h)
        out = bgr.copy()
        if arm_mask is not None:
            tint = out.copy()
            tint[arm_mask > 0] = (255, 120, 0)
            out = cv2.addWeighted(tint, 0.35, out, 0.65, 0)
        for mask in self.ctx.lane_masks.values():
            cnts, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(out, cnts, -1, (255, 200, 0), 1)
        for name, (bx, by) in self.world.bins.items():
            c = self._px(bx, by, self.world.bin_size[2])
            cv2.drawMarker(out, c, (0, 140, 255) if name == "trash" else (80, 220, 80), cv2.MARKER_SQUARE, 30, 1)
            cv2.putText(out, name, (c[0] - 25, c[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        for d in dets:
            color = BENT_COLORS.get(d.bent, BENT_COLORS[None])
            if d.box_px is not None:
                cv2.polylines(out, [d.box_px.astype(np.int32)], True, color, 2)
            cv2.circle(out, (int(d.u), int(d.v)), 3, (0, 0, 255), -1)
            cv2.putText(out, f"{d.label} {d.score:.2f}", (int(d.u) + 8, int(d.v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        if roi and roi.get("station"):
            sx, sy = roi["station"]
            c = self._px(sx, sy)
            r_px = max(2, int(round(math.hypot(*np.subtract(self._px(sx + float(roi.get("roi_radius", 0.3)), sy), c)))))
            col = (0, 0, 255) if roi.get("trigger") else (0, 220, 255)
            cv2.circle(out, c, r_px, col, 2)
            if roi.get("nearest") and roi.get("pred"):
                cv2.arrowedLine(out, self._px(*roi["nearest"]), self._px(*roi["pred"]), (255, 0, 255), 2, tipLength=0.4)
        if thumbnails:
            tw, th = 256, 144
            for i, (name, img) in enumerate(list(self.detector.debug_layers().items())[:2]):
                if img is None or img.size == 0:
                    continue
                img = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                y = h - (th + 10) * (i + 1)
                if y < 30:
                    break
                out[y:y + th, 10:10 + tw] = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
                cv2.rectangle(out, (9, y - 1), (11 + tw, y + th), (255, 255, 255), 1)
                cv2.putText(out, name, (14, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        counts = "  ".join(f"bent {b}: {sum(1 for d in dets if d.bent == b)}" for b in range(1, len(self.world.lanes) + 1))
        head = f"[{self.detector.name}] {self.world.key}  {counts}  {self.last_ms:.0f} ms   {status[:80]}"
        cv2.rectangle(out, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(out, head, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    def _px(self, x, y, z=None):
        u, v = self.ctx.camera.world_to_pixel(x, y, self.object_top_z if z is None else z)
        return int(round(u)), int(round(v))
