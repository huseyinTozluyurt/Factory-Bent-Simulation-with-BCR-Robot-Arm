#!/usr/bin/env python3
"""
Side camera geometry + OpenCV object check at the catch station (no ROS imports).

The side camera looks across the bent at belt height, straight at the catch station
(Gazebo convention: camera looks along its +x, image right = its -y, image up = its +z).
The overhead camera tells where an object is on the belt (x, y); the side camera adds
the horizontal view: is something really standing on the belt at the station, where
along the belt is it, and how tall is it. The controller only grabs when both agree.

Pipeline per frame:
  grayscale -> Gaussian blur -> MOG2 background subtraction (the side view is static
  except for the products) -> Canny edges of the moving region -> morphology ->
  contours inside the station band -> keep blobs whose bottom sits on the belt at the
  station plane (far-side products of the same ring project much higher and are dropped)
  -> y position along the belt and top height on the plane x = station_x.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from factory_bringup.vision_geometry import OverheadCamera


def pixel_to_plane_x(camera: OverheadCamera, u, v, plane_x: float) -> Tuple[np.ndarray, np.ndarray]:
    """Intersect the pixel ray with the vertical plane x = plane_x; returns world (y, z)."""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    ray_c = np.stack([np.ones_like(u), -(u - camera.cx) / camera.fx, -(v - camera.cy) / camera.fy], axis=-1)
    ray_w = ray_c @ camera.R.T
    t = (plane_x - camera.C[0]) / ray_w[..., 0]
    return camera.C[1] + t * ray_w[..., 1], camera.C[2] + t * ray_w[..., 2]


@dataclass
class SideCheckParams:
    plane_x: float = -2.58          # station plane (products cross it at the catch point)
    band_y_min: float = -0.70       # part of the belt watched, along y on that plane
    band_y_max: float = 0.70
    belt_z: float = 1.42            # bottom of the products (they float 4.5 cm over the belt top 1.375)
    max_top_z: float = 1.95         # tallest product top is 1.84
    bottom_tol: float = 0.08        # blob bottom must be this close to belt_z on the station plane
    min_height: float = 0.06
    min_width: float = 0.05
    blur_kernel: int = 5
    mog_history: int = 300
    mog_var_threshold: float = 25.0
    canny_low: int = 30
    canny_high: int = 90
    morph_close: int = 9
    morph_open: int = 3
    min_area_px: int = 250
    warmup_frames: int = 15          # background model needs a few frames before blobs count


@dataclass
class SideDetection:
    y: float                 # centre along the belt on the station plane (world y)
    z_top: float             # top of the object (world z)
    z_bottom: float
    width: float             # extent along y (m)
    bbox: Tuple[int, int, int, int]
    area_px: int

    def as_dict(self) -> Dict:
        return {"y": round(self.y, 4), "z_top": round(self.z_top, 4), "z_bottom": round(self.z_bottom, 4),
                "width": round(self.width, 4), "height": round(self.z_top - self.z_bottom, 4),
                "bbox": list(self.bbox), "area_px": self.area_px}


class SideObjectChecker:
    def __init__(self, camera: OverheadCamera, params: SideCheckParams):
        self.camera = camera
        self.params = params
        self.mog = cv2.createBackgroundSubtractorMOG2(history=params.mog_history,
                                                      varThreshold=params.mog_var_threshold, detectShadows=False)
        self.frames = 0
        self.band_mask = self._band_mask()
        self.last_fg = None
        self.last_edges = None

    def _band_mask(self) -> np.ndarray:
        """Image region of the station band: y in [band_y_min, band_y_max], z from belt to max_top_z."""
        p, cam = self.params, self.camera
        corners = [(p.band_y_min, p.belt_z - 0.06), (p.band_y_max, p.belt_z - 0.06),
                   (p.band_y_max, p.max_top_z), (p.band_y_min, p.max_top_z)]
        pts = np.array([cam.world_to_pixel(p.plane_x, y, z) for y, z in corners], dtype=np.float32)
        mask = np.zeros((cam.height, cam.width), np.uint8)
        cv2.fillConvexPoly(mask, np.round(pts).astype(np.int32), 255)
        return mask

    def preprocess(self, bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        k = max(1, self.params.blur_kernel | 1)
        blur = cv2.GaussianBlur(gray, (k, k), 0)
        return blur, cv2.Canny(blur, self.params.canny_low, self.params.canny_high)

    def check(self, bgr: np.ndarray, learn: bool = True) -> List[SideDetection]:
        """Objects standing on the belt at the station, nearest the band centre first."""
        p = self.params
        blur, edges = self.preprocess(bgr)
        fg = self.mog.apply(blur, learningRate=-1 if learn else 0)
        self.frames += 1
        fg = cv2.bitwise_and(fg, self.band_mask)
        # edges of the moving region sharpen its outline; the closed/filled union is the object
        near = cv2.dilate(fg, np.ones((7, 7), np.uint8))
        fg = cv2.bitwise_or(fg, cv2.bitwise_and(edges, near))
        if p.morph_close > 1:
            fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((p.morph_close, p.morph_close), np.uint8))
        if p.morph_open > 1:
            fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((p.morph_open, p.morph_open), np.uint8))
        self.last_fg, self.last_edges = fg, edges
        if self.frames < p.warmup_frames:
            return []

        found: List[SideDetection] = []
        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = int(cv2.contourArea(c))
            if area < p.min_area_px:
                continue
            x, y, w, h = cv2.boundingRect(c)
            uc = x + w / 2.0
            (yb, zb) = pixel_to_plane_x(self.camera, uc, y + h, p.plane_x)
            (_, zt) = pixel_to_plane_x(self.camera, uc, y, p.plane_x)
            (yl, _) = pixel_to_plane_x(self.camera, x, y + h, p.plane_x)
            (yr, _) = pixel_to_plane_x(self.camera, x + w, y + h, p.plane_x)
            yb, zb, zt = float(yb), float(zb), float(zt)
            width = abs(float(yr) - float(yl))
            if abs(zb - p.belt_z) > p.bottom_tol:
                continue                     # not standing on the belt at the station (far side, arm, ...)
            if zt - zb < p.min_height or width < p.min_width:
                continue
            found.append(SideDetection(yb, zt, zb, width, (x, y, w, h), area))
        found.sort(key=lambda d: abs(d.y - 0.5 * (p.band_y_min + p.band_y_max)))
        return found


def compose_side_view(checker: SideObjectChecker, bgr: np.ndarray, detections: List[SideDetection],
                      status: str = "", target_y: Optional[float] = None, confirm: Optional[Dict] = None) -> np.ndarray:
    """The "Side Camera" window: band, blobs with y / height, station line and the cross-check verdict."""
    p, cam = checker.params, checker.camera
    out = bgr.copy()
    h, w = out.shape[:2]
    contours, _ = cv2.findContours(checker.band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, contours, -1, (0, 220, 255), 1)

    def px(y, z):
        u, v = cam.world_to_pixel(p.plane_x, y, z)
        return int(round(u)), int(round(v))

    mid_y = 0.5 * (p.band_y_min + p.band_y_max)
    cv2.line(out, px(mid_y, p.belt_z - 0.05), px(mid_y, p.max_top_z), (0, 220, 255), 1, cv2.LINE_AA)
    cv2.line(out, px(p.band_y_min, p.belt_z), px(p.band_y_max, p.belt_z), (0, 160, 255), 1, cv2.LINE_AA)
    if target_y is not None:
        cv2.drawMarker(out, px(target_y, p.belt_z), (255, 0, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)

    for i, d in enumerate(detections):
        x, y, bw, bh = d.bbox
        color = (0, 255, 0) if i == 0 else (0, 160, 0)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        cv2.line(out, px(d.y - d.width / 2, d.z_top), px(d.y + d.width / 2, d.z_top), (0, 0, 255), 2)
        cv2.putText(out, f"y={d.y:+.2f} top={d.z_top:.2f} h={d.z_top - d.z_bottom:.2f}",
                    (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # pipeline thumbnails: grayscale + foreground, Canny edges
    if checker.last_fg is not None:
        tw, th = 192, 108
        blur, _ = checker.preprocess(bgr)
        fg_view = cv2.cvtColor(cv2.resize(blur, (tw, th), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        fg_small = cv2.resize(checker.last_fg, (tw, th), interpolation=cv2.INTER_NEAREST)
        fg_view[fg_small > 0] = (0, 255, 0)
        edge_view = cv2.cvtColor(cv2.resize(checker.last_edges, (tw, th), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        for i, (img, name) in enumerate(((fg_view, "gray + moving"), (edge_view, "canny edges"))):
            y0 = h - (th + 8) * (i + 1)
            out[y0:y0 + th, w - tw - 8:w - 8] = img
            cv2.rectangle(out, (w - tw - 9, y0 - 1), (w - 7, y0 + th), (255, 255, 255), 1)
            cv2.putText(out, name, (w - tw - 4, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    verdict = ""
    if confirm:
        verdict = "CROSS-CHECK OK" if confirm.get("ok") else f"cross-check: {confirm.get('reason', '')}"
    cv2.rectangle(out, (0, 0), (w, 22), (0, 0, 0), -1)
    cv2.putText(out, f"side: {len(detections)} on belt   {verdict}   {status[:60]}", (6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if confirm and confirm.get("ok") else (255, 255, 255),
                1, cv2.LINE_AA)
    return out
