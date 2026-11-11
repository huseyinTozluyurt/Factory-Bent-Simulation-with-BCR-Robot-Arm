#!/usr/bin/env python3
"""
Release zone check on the second bent (no ROS imports).

A camera looks straight down at the release zone. The empty belt there is learned at
start-up (per-pixel median of the first frames, so objects riding past are not part of
it). Each frame:
  grayscale -> blur -> difference to the empty belt -> threshold (+ Canny edges inside
  the changed area) -> morphology -> contours inside the zone circle, with the arm masked
  out -> the zone is EMPTY when no blob is big enough.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from factory_bringup.vision_geometry import OverheadCamera


@dataclass
class ReleaseZoneParams:
    zone_x: float = 2.2171
    zone_y: float = -0.3736
    zone_radius: float = 0.40       # m on the belt; objects are up to 0.48 m long
    belt_z: float = 1.375
    blur_kernel: int = 5
    diff_threshold: int = 30
    canny_low: int = 30
    canny_high: int = 90
    morph_open: int = 3
    morph_close: int = 7
    min_blob_m2: float = 0.004      # smallest product top is 0.06 x 0.18 m
    reference_frames: int = 30      # frames for the empty-belt median


@dataclass
class ZoneBlob:
    x: float
    y: float
    area_m2: float
    bbox: Tuple[int, int, int, int]


class ReleaseZoneChecker:
    def __init__(self, camera: OverheadCamera, params: ReleaseZoneParams):
        self.camera = camera
        self.params = params
        self.frames: List[np.ndarray] = []
        self.reference: Optional[np.ndarray] = None
        p = params
        cu, cv = camera.world_to_pixel(p.zone_x, p.zone_y, p.belt_z)
        eu, ev = camera.world_to_pixel(p.zone_x + p.zone_radius, p.zone_y, p.belt_z)
        self.center_px = (int(round(cu)), int(round(cv)))
        self.radius_px = max(3, int(round(math.hypot(eu - cu, ev - cv))))
        self.zone_mask = np.zeros((camera.height, camera.width), np.uint8)
        cv2.circle(self.zone_mask, self.center_px, self.radius_px, 255, -1)
        depth = max(0.5, camera.z - p.belt_z)
        self.m2_per_px = (depth / camera.fx) ** 2
        self.last_changed = None
        self.last_edges = None

    @property
    def ready(self) -> bool:
        return self.reference is not None

    def preprocess(self, bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        k = max(1, self.params.blur_kernel | 1)
        return cv2.GaussianBlur(gray, (k, k), 0)

    def relearn(self):
        """Forget the empty-belt image; it is learned again from the next frames."""
        self.frames, self.reference = [], None

    def check(self, bgr: np.ndarray, exclude_mask: Optional[np.ndarray] = None) -> Tuple[Optional[bool], List[ZoneBlob]]:
        """(empty?, blobs in the zone). empty is None until the empty belt has been learned."""
        p = self.params
        blur = self.preprocess(bgr)
        if self.reference is None:
            self.frames.append(blur)
            if len(self.frames) >= p.reference_frames:
                self.reference = np.median(np.stack(self.frames), axis=0).astype(np.uint8)
                self.frames = []
            return None, []

        diff = cv2.absdiff(blur, self.reference)
        _, changed = cv2.threshold(diff, p.diff_threshold, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(blur, p.canny_low, p.canny_high)
        near = cv2.dilate(changed, np.ones((5, 5), np.uint8))
        changed = cv2.bitwise_or(changed, cv2.bitwise_and(edges, near))
        if p.morph_open > 1:
            changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, np.ones((p.morph_open, p.morph_open), np.uint8))
        if p.morph_close > 1:
            changed = cv2.morphologyEx(changed, cv2.MORPH_CLOSE, np.ones((p.morph_close, p.morph_close), np.uint8))
        changed = cv2.bitwise_and(changed, self.zone_mask)
        if exclude_mask is not None:
            changed[exclude_mask > 0] = 0
        self.last_changed, self.last_edges = changed, edges

        blobs: List[ZoneBlob] = []
        contours, _ = cv2.findContours(changed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area_m2 = cv2.contourArea(c) * self.m2_per_px
            if area_m2 < p.min_blob_m2:
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            wx, wy = self.camera.pixel_to_world(u, v, p.belt_z)
            blobs.append(ZoneBlob(float(wx), float(wy), float(area_m2), cv2.boundingRect(c)))
        return len(blobs) == 0, blobs


def compose_zone_view(checker: ReleaseZoneChecker, bgr: np.ndarray, empty: Optional[bool], blobs: List[ZoneBlob],
                      status: str = "", arm_mask: Optional[np.ndarray] = None) -> np.ndarray:
    """The "Release Zone" window: zone circle (green empty / red occupied), blobs, arm, thumbnails."""
    out = bgr.copy()
    h, w = out.shape[:2]
    if arm_mask is not None:
        tint = out.copy()
        tint[arm_mask > 0] = (255, 120, 0)
        out = cv2.addWeighted(tint, 0.35, out, 0.65, 0)
    color = (0, 200, 255) if empty is None else ((0, 220, 0) if empty else (0, 0, 255))
    cv2.circle(out, checker.center_px, checker.radius_px, color, 2)
    cv2.drawMarker(out, checker.center_px, color, cv2.MARKER_TILTED_CROSS, 12, 2)
    for b in blobs:
        x, y, bw, bh = b.bbox
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(out, f"({b.x:.2f}, {b.y:.2f})", (x, max(14, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    if checker.last_changed is not None:
        tw, th = 160, 120
        gray = cv2.cvtColor(cv2.resize(checker.preprocess(bgr), (tw, th), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        ch = cv2.resize(checker.last_changed, (tw, th), interpolation=cv2.INTER_NEAREST)
        gray[ch > 0] = (0, 0, 255)
        edges = cv2.cvtColor(cv2.resize(checker.last_edges, (tw, th), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        for i, (img, name) in enumerate(((gray, "gray + changed"), (edges, "canny edges"))):
            y0 = h - (th + 8) * (i + 1)
            out[y0:y0 + th, w - tw - 8:w - 8] = img
            cv2.rectangle(out, (w - tw - 9, y0 - 1), (w - 7, y0 + th), (255, 255, 255), 1)
            cv2.putText(out, name, (w - tw - 4, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    verdict = "learning empty belt..." if empty is None else ("EMPTY" if empty else f"OCCUPIED ({len(blobs)})")
    cv2.rectangle(out, (0, 0), (w, 22), (0, 0, 0), -1)
    cv2.putText(out, f"release zone: {verdict}   {status[:60]}", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                color, 1, cv2.LINE_AA)
    return out
