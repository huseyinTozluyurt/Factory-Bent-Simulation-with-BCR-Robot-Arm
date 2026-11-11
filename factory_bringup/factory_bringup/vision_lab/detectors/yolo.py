"""
yolo: Ultralytics YOLO (v8 / 11 / later) as a vision-lab detector. Optional: needs
    pip install ultralytics          (pulls in PyTorch; a GPU is used if PyTorch sees one)

Plain detection models give axis-aligned boxes; oriented-box models (*-obb.pt) give the box angle,
and segmentation models (*-seg.pt) give an outline whose minAreaRect is used. Either way the
pipeline turns the box centre into world x, y and decides the bent.

A COCO model (yolov8n.pt) does not know the factory products. Train one on the simulation itself:
record a dataset that the OpenCV detector labels for you,
    ros2 run factory_bringup vision_lab run --detector opencv_contours --record ~/vision_lab_dataset
then  yolo detect train data=~/vision_lab_dataset/data.yaml model=yolov8n.pt imgsz=1280 epochs=50
and   --detector yolo --opt model=runs/detect/train/weights/best.pt

Options:
  model ("yolov8n.pt")   weights file or Ultralytics model name
  conf (0.25), iou (0.5), imgsz (1280: products are only ~20 px wide in the overhead image)
  device (None = auto, e.g. "cpu", "cuda:0")
  classes (None)         keep only these class ids or names
  label_map (None)       {"model class name": "label you want"}
  crop_to_lanes (True)   run on the part of the image around the belts only (faster, fewer false hits)
"""

from typing import Dict, List, Optional

import cv2
import numpy as np

from factory_bringup.vision_lab.detectors.base import Detector, DetectorContext, ObjectDetection, register_detector


@register_detector("yolo")
class YoloDetector(Detector):
    """Ultralytics YOLO detection / OBB / segmentation model (pip install ultralytics)."""

    def __init__(self, model: str = "yolov8n.pt", conf: float = 0.25, iou: float = 0.5, imgsz: int = 1280,
                 device: Optional[str] = None, classes: Optional[List] = None, label_map: Optional[Dict] = None,
                 crop_to_lanes: bool = True, verbose: bool = False):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError("The 'yolo' detector needs Ultralytics:  pip install ultralytics  "
                              "(in the Python that runs ROS 2, e.g. pip install --user ultralytics "
                              "or a venv created with --system-site-packages).") from exc
        import os
        path = os.path.expanduser(str(model))
        self.model = YOLO(path if os.path.exists(path) else str(model))
        self.conf, self.iou, self.imgsz, self.device = float(conf), float(iou), int(imgsz), device
        self.label_map = dict(label_map or {})
        self.crop_to_lanes = bool(crop_to_lanes)
        self.verbose = verbose
        names = getattr(self.model, "names", {}) or {}
        self.class_names = dict(names) if isinstance(names, dict) else dict(enumerate(names))
        self.class_ids = None
        if classes is not None:
            by_name = {v: k for k, v in self.class_names.items()}
            self.class_ids = [int(by_name.get(c, c)) for c in classes]

    def setup(self, ctx: DetectorContext) -> None:
        super().setup(ctx)
        self.offset = (0, 0)
        self.crop = None
        if self.crop_to_lanes:
            ys, xs = np.nonzero(ctx.any_lane_mask)
            if len(xs):
                pad = 24
                x1, y1 = max(0, xs.min() - pad), max(0, ys.min() - pad)
                x2, y2 = min(ctx.camera.width, xs.max() + pad), min(ctx.camera.height, ys.max() + pad)
                self.crop = (int(x1), int(y1), int(x2), int(y2))
                self.offset = (int(x1), int(y1))

    def _label(self, cls_id: int) -> str:
        name = self.class_names.get(cls_id, str(cls_id))
        return self.label_map.get(name, name)

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        img = bgr if self.crop is None else bgr[self.crop[1]:self.crop[3], self.crop[0]:self.crop[2]]
        res = self.model.predict(img, conf=self.conf, iou=self.iou, imgsz=self.imgsz, device=self.device,
                                 classes=self.class_ids, verbose=self.verbose)[0]
        ox, oy = self.offset
        out: List[ObjectDetection] = []

        obb = getattr(res, "obb", None)
        if obb is not None and len(obb):
            polys = obb.xyxyxyxy.cpu().numpy()
            for poly, c, k in zip(polys, obb.conf.cpu().numpy(), obb.cls.cpu().numpy().astype(int)):
                poly = poly.reshape(4, 2) + (ox, oy)
                u, v = poly.mean(axis=0)
                out.append(ObjectDetection(u=float(u), v=float(v), box_px=poly, label=self._label(k), score=float(c), class_id=int(k)))
            return out

        boxes = getattr(res, "boxes", None)
        if boxes is None or not len(boxes):
            return out
        masks = getattr(res, "masks", None)
        outlines = masks.xy if masks is not None else [None] * len(boxes)
        for xyxy, c, k, outline in zip(boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(),
                                       boxes.cls.cpu().numpy().astype(int), outlines):
            x1, y1, x2, y2 = (float(t) for t in xyxy)
            box_px = None
            if outline is not None and len(outline) >= 5:
                box_px = cv2.boxPoints(cv2.minAreaRect(np.asarray(outline, np.float32))) + (ox, oy)
            det = ObjectDetection(u=(x1 + x2) / 2 + ox, v=(y1 + y2) / 2 + oy, label=self._label(k), score=float(c),
                                  class_id=int(k), box_px=box_px, bbox=(x1 + ox, y1 + oy, x2 + ox, y2 + oy))
            if box_px is None:
                det.extra["axis_aligned"] = True        # yaw from an axis-aligned box means nothing
            out.append(det)
        return out
