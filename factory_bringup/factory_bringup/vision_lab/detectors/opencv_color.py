"""
opencv_color: colour segmentation. Finds objects of chosen colours (product types or your own BGR
colours) on the belts, so a developer can say "only the paint cans" or "everything red".

Pipeline: Lab colour space -> pixels on a lane that differ from the belt colour -> each such pixel
gets the nearest target colour (or none, if no target is within `tolerance`) -> mask of all target
pixels -> opening / closing -> contours -> label = the most common target colour in the blob ->
minAreaRect -> world.

Options:
  targets (None)        list of product types (product_catalog.PRODUCT_TYPES) and/or {"name": [b, g, r]};
                        None = every product type
  tolerance (28.0)      max Lab distance (lightness weighted by lightness_weight) to count as a target colour
  lightness_weight (0.35)
  belt_margin (18.0)    Lab distance from the belt's median colour to count as "not belt"
  min_area_m2 (0.006), max_area_m2 (0.40), kernel_px (3)
"""

import math
from typing import Dict, List, Optional

import cv2
import numpy as np

from factory_bringup.product_catalog import PRODUCT_TYPES
from factory_bringup.vision_lab.detectors.base import Detector, DetectorContext, ObjectDetection, register_detector


@register_detector("opencv_color")
class OpenCVColorDetector(Detector):
    """Lab colour segmentation for chosen product types / colours."""

    def __init__(self, targets: Optional[List] = None, tolerance: float = 28.0, lightness_weight: float = 0.35,
                 belt_margin: float = 18.0, min_area_m2: float = 0.006, max_area_m2: float = 0.40,
                 kernel_px: int = 3):
        self.tolerance, self.lw, self.belt_margin = tolerance, lightness_weight, belt_margin
        self.min_area, self.max_area, self.kernel = min_area_m2, max_area_m2, max(1, int(kernel_px))
        colors: Dict[str, tuple] = {}
        for t in (targets or list(PRODUCT_TYPES)):
            if isinstance(t, dict):
                colors.update({k: tuple(v) for k, v in t.items()})
            elif t in PRODUCT_TYPES:
                r, g, b = PRODUCT_TYPES[t]["rgb"]
                colors[t] = (b * 255, g * 255, r * 255)
            else:
                raise ValueError(f"opencv_color: '{t}' is neither a product type nor a {{name: [b, g, r]}} colour")
        self.names = list(colors)
        px = np.uint8([[[int(round(c)) for c in colors[n]] for n in self.names]])
        self.ref_lab = cv2.cvtColor(px, cv2.COLOR_BGR2LAB)[0].astype(np.float32)
        self._layers: Dict[str, np.ndarray] = {}

    def setup(self, ctx: DetectorContext) -> None:
        super().setup(ctx)
        self.lanes = ctx.any_lane_mask
        self.inner = cv2.erode(self.lanes, np.ones((7, 7), np.uint8))

    def _weighted(self, lab: np.ndarray) -> np.ndarray:
        out = lab.astype(np.float32).copy()
        out[..., 0] *= self.lw
        return out

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        ctx = self.ctx
        lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2LAB)
        area = self.lanes.copy()
        if ctx.exclude_mask is not None:
            area[ctx.exclude_mask > 0] = 0
        sel = area > 0
        if not sel.any():
            return []
        belt = np.median(lab[(self.inner > 0)], axis=0) if (self.inner > 0).any() else np.zeros(3)
        pix = self._weighted(lab[sel])
        not_belt = np.linalg.norm(pix - self._weighted(belt[None, :]), axis=1) > self.belt_margin
        ref = self._weighted(self.ref_lab)
        d = np.linalg.norm(pix[:, None, :] - ref[None, :, :], axis=2)        # N x targets
        best = np.argmin(d, axis=1)
        ok = not_belt & (d[np.arange(len(best)), best] <= self.tolerance)
        label_img = np.full(bgr.shape[:2], -1, np.int32)
        label_img[sel] = np.where(ok, best, -1)
        self._layers = {"colour match": ((label_img >= 0) * 255).astype(np.uint8)}

        # One mask for "any target colour", then one label per blob by majority vote: a product whose
        # anti-aliased edges or shading fall nearer to a neighbouring catalogue colour stays one blob
        # instead of being split into small pieces (matters more the more product types there are).
        out: List[ObjectDetection] = []
        k = np.ones((self.kernel, self.kernel), np.uint8)
        m = ((label_img >= 0) * 255).astype(np.uint8)
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sel_idx = np.flatnonzero(sel.ravel())
        for c in cnts:
            if len(c) < 5:
                continue
            box = cv2.boxPoints(cv2.minAreaRect(c))
            wx, wy = ctx.camera.pixel_to_world(box[:, 0], box[:, 1], ctx.object_top_z)
            a = math.hypot(wx[1] - wx[0], wy[1] - wy[0]) * math.hypot(wx[2] - wx[1], wy[2] - wy[1])
            if not (self.min_area <= a <= self.max_area):
                continue
            mom = cv2.moments(c)
            if mom["m00"] <= 0:
                continue
            blob = np.zeros_like(m)
            cv2.drawContours(blob, [c], -1, 255, -1)
            core = cv2.erode(blob, np.ones((5, 5), np.uint8))       # the middle: no edge or belt tint
            core = core if (label_img[core > 0] >= 0).any() else blob
            labels = label_img[core > 0]
            labels = labels[labels >= 0]
            if labels.size == 0:
                continue
            i = int(np.bincount(labels).argmax())
            in_blob = (blob.ravel()[sel_idx] > 0) & ok & (best == i)
            dist = float(np.mean(d[in_blob, i])) if in_blob.any() else self.tolerance
            out.append(ObjectDetection(u=mom["m10"] / mom["m00"], v=mom["m01"] / mom["m00"], box_px=box,
                                       label=self.names[i], class_id=i,
                                       score=math.exp(-dist / max(1e-6, self.tolerance))))
        return out

    def debug_layers(self) -> Dict[str, np.ndarray]:
        return dict(self._layers)
