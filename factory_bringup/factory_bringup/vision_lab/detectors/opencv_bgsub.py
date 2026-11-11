"""
opencv_bgsub: background subtraction (MOG2). Learns what the empty belts look like and reports what
is new on them: no colour or brightness assumptions about the products.

The belts are near black and evenly coloured, so their movement hardly shows. A product that stays
still for longer than `history` frames fades into the background (a product held at the station);
use learning_rate=0 after a warm-up to freeze the background.

Options: history (300), var_threshold (32), learning_rate (-1 = automatic), warmup_frames (30),
         min_area_m2 (0.006), max_area_m2 (0.40), classify (True)
"""

import math
from typing import Dict, List

import cv2
import numpy as np

from factory_bringup.vision_lab.classify import ProductColorClassifier
from factory_bringup.vision_lab.detectors.base import Detector, DetectorContext, ObjectDetection, register_detector


@register_detector("opencv_bgsub")
class OpenCVBackgroundSubtractionDetector(Detector):
    """MOG2 background subtraction on the lanes (+ colour label)."""

    def __init__(self, history: int = 300, var_threshold: float = 32.0, learning_rate: float = -1.0,
                 warmup_frames: int = 30, min_area_m2: float = 0.006, max_area_m2: float = 0.40, classify: bool = True):
        self.history, self.var_threshold, self.learning_rate = int(history), float(var_threshold), float(learning_rate)
        self.warmup = int(warmup_frames)
        self.min_area, self.max_area = min_area_m2, max_area_m2
        self.classifier = ProductColorClassifier() if classify else None
        self._layers: Dict[str, np.ndarray] = {}

    def setup(self, ctx: DetectorContext) -> None:
        super().setup(ctx)
        self.mog = cv2.createBackgroundSubtractorMOG2(history=self.history, varThreshold=self.var_threshold,
                                                      detectShadows=True)
        self.frames = 0
        self.lanes = ctx.any_lane_mask

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        ctx = self.ctx
        fg = self.mog.apply(bgr, learningRate=self.learning_rate)
        self.frames += 1
        fg = np.where(fg == 255, 255, 0).astype(np.uint8)          # 127 = shadow
        fg &= self.lanes
        if ctx.exclude_mask is not None:
            fg[ctx.exclude_mask > 0] = 0
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        self._layers = {"foreground": fg}
        if self.frames < self.warmup:
            return []
        out = []
        cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if len(c) < 5:
                continue
            box = cv2.boxPoints(cv2.minAreaRect(c))
            wx, wy = ctx.camera.pixel_to_world(box[:, 0], box[:, 1], ctx.object_top_z)
            a = math.hypot(wx[1] - wx[0], wy[1] - wy[0]) * math.hypot(wx[2] - wx[1], wy[2] - wy[1])
            if not (self.min_area <= a <= self.max_area):
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            label, score = "object", 1.0
            if self.classifier is not None:
                label, score, _ = self.classifier.classify_region(bgr, box)
            out.append(ObjectDetection(u=m["m10"] / m["m00"], v=m["m01"] / m["m00"], box_px=box, label=label, score=score))
        return out

    def debug_layers(self) -> Dict[str, np.ndarray]:
        return dict(self._layers)
