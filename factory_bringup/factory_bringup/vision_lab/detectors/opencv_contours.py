"""
opencv_contours: the project's tuned OpenCV detector (vision_geometry.RingObjectDetector), the same
one ring_object_detector runs, as a vision-lab detector. Optionally names each object by colour.

Pipeline: grayscale -> Gaussian blur -> (brighter than the belt OR filled Canny edges) -> morphology
-> contours inside each lane -> minAreaRect -> world on the product-top plane -> colour label.

Options (all of vision_geometry.DetectorParams, plus):
  classify (True)      name each object with ProductColorClassifier; False = label "object"
  color_file (None)    JSON of measured colours for the classifier (ProductColorClassifier.save)
"""

from dataclasses import fields
from typing import Dict, List

import cv2
import numpy as np

from factory_bringup.vision_geometry import DetectorParams, RingObjectDetector
from factory_bringup.vision_lab.classify import ProductColorClassifier
from factory_bringup.vision_lab.detectors.base import Detector, DetectorContext, ObjectDetection, register_detector


@register_detector("opencv_contours")
class OpenCVContourDetector(Detector):
    """Tuned grayscale + Canny contour detector of the project (+ colour label)."""

    def __init__(self, classify: bool = True, color_file: str = None, **params):
        names = {f.name for f in fields(DetectorParams)}
        unknown = set(params) - names
        if unknown:
            raise ValueError(f"opencv_contours: unknown options {sorted(unknown)}; known: {sorted(names)}")
        self.params = DetectorParams(**params)
        self.classifier = ProductColorClassifier() if classify else None
        if self.classifier and color_file:
            self.classifier.load(color_file)
        self.core = None
        self._layers: Dict[str, np.ndarray] = {}

    def setup(self, ctx: DetectorContext) -> None:
        super().setup(ctx)
        self.params.belt_z, self.params.object_top_z = ctx.belt_z, ctx.object_top_z
        self.core = RingObjectDetector(ctx.camera, ctx.lanes, self.params)
        # use the lab's lane masks (straight pieces stop at the belt ends: pipeline world)
        self.core.ring_masks = dict(ctx.lane_masks)
        self.core.ring_masks_eroded = {k: cv2.erode(m, np.ones((7, 7), np.uint8)) for k, m in ctx.lane_masks.items()}

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        dets, fg = self.core.detect(bgr, exclude_mask=self.ctx.exclude_mask)
        self._layers = {"foreground": fg}
        out = []
        for d in dets:
            label, score = "object", 1.0
            if self.classifier is not None:
                label, score, _ = self.classifier.classify_region(bgr, d.box_px)
            out.append(ObjectDetection(
                u=d.u, v=d.v, box_px=d.box_px, label=label, score=score,
                x=d.x, y=d.y, yaw=d.yaw, size_x=d.size_x, size_y=d.size_y,
                bent=int(d.ring[-1]) if d.ring and d.ring[-1].isdigit() else None,
            ))
        return out

    def debug_layers(self) -> Dict[str, np.ndarray]:
        if self.core is None:
            return {}
        return dict(self._layers)
