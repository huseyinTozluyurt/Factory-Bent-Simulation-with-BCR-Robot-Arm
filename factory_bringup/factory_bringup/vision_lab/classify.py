"""
Product type from colour (no ROS imports): the classic OpenCV way to name what a contour detector
found, and the auto-labeller of the YOLO dataset recorder.

Every product type in product_catalog.PRODUCT_TYPES has its own colour. The classifier compares the
mean colour of a detection (CIE Lab, lightness weighted down so the light in the hall matters less)
with each type's colour and gives the nearest type and a 0..1 score.

Several types share a colour family (the browns, the whites), so this is a baseline to beat, not
ground truth. Measured colours from the simulation can replace the catalogue ones:

    clf = ProductColorClassifier()
    clf.calibrate("paint_can", mean_bgr)           # or clf.load("~/.ros/vision_lab_colors.json")
"""

import json
import math
import os
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from factory_bringup.product_catalog import PRODUCT_TYPES


def _bgr_to_lab(bgr) -> np.ndarray:
    px = np.uint8([[[int(round(c)) for c in bgr]]])
    return cv2.cvtColor(px, cv2.COLOR_BGR2LAB)[0, 0].astype(np.float32)


class ProductColorClassifier:
    def __init__(self, types: Optional[Iterable[str]] = None, lightness_weight: float = 0.35,
                 scale: float = 25.0, min_score: float = 0.0):
        self.lightness_weight = lightness_weight
        self.scale = scale                      # Lab distance at which the score falls to 1/e
        self.min_score = min_score              # below this: label "unknown"
        self.reference_bgr: Dict[str, Tuple[float, float, float]] = {}
        for t in (types or PRODUCT_TYPES):
            r, g, b = PRODUCT_TYPES[t]["rgb"]
            self.reference_bgr[t] = (b * 255.0, g * 255.0, r * 255.0)
        self._refresh()

    def _refresh(self):
        self.names = list(self.reference_bgr)
        self.lab = np.stack([_bgr_to_lab(c) for c in self.reference_bgr.values()]) if self.names else np.zeros((0, 3))

    def calibrate(self, label: str, mean_bgr) -> None:
        """Use a colour measured in the simulation for this type from now on."""
        self.reference_bgr[label] = tuple(float(c) for c in mean_bgr)
        self._refresh()

    def save(self, path: str) -> None:
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump({k: list(v) for k, v in self.reference_bgr.items()}, f, indent=1)

    def load(self, path: str) -> "ProductColorClassifier":
        with open(os.path.expanduser(path)) as f:
            for k, v in json.load(f).items():
                self.reference_bgr[k] = tuple(v)
        self._refresh()
        return self

    def distances(self, mean_bgr) -> np.ndarray:
        lab = _bgr_to_lab(mean_bgr)
        d = self.lab - lab
        d[:, 0] *= self.lightness_weight
        return np.sqrt((d ** 2).sum(axis=1))

    def classify_color(self, mean_bgr) -> Tuple[str, float]:
        if not self.names:
            return "object", 0.0
        d = self.distances(mean_bgr)
        i = int(np.argmin(d))
        score = math.exp(-float(d[i]) / self.scale)
        return (self.names[i], score) if score >= self.min_score else ("unknown", score)

    def top_k(self, mean_bgr, k: int = 3) -> List[Tuple[str, float]]:
        d = self.distances(mean_bgr)
        return [(self.names[i], math.exp(-float(d[i]) / self.scale)) for i in np.argsort(d)[:k]]

    def classify_region(self, bgr: np.ndarray, polygon_px: np.ndarray, shrink: float = 0.6) -> Tuple[str, float, Tuple]:
        """Label of the object inside this polygon. Only the middle `shrink` part is averaged, so the
        object's edges and the belt around it do not tint the colour."""
        poly = np.asarray(polygon_px, np.float32).reshape(-1, 2)
        c = poly.mean(axis=0)
        inner = (c + (poly - c) * shrink).astype(np.int32)
        mask = np.zeros(bgr.shape[:2], np.uint8)
        cv2.fillPoly(mask, [inner], 255)
        if not mask.any():
            cv2.circle(mask, (int(c[0]), int(c[1])), 2, 255, -1)
        mean = cv2.mean(bgr, mask=mask)[:3]
        label, score = self.classify_color(mean)
        return label, score, mean
