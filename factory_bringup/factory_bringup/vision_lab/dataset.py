"""
YOLO dataset recorder (no ROS imports): saves overhead images with labels made by the running
detector, in the Ultralytics folder layout, so a YOLO model can be trained on the simulation.

    rec = YoloDatasetRecorder("~/vision_lab_dataset", every_n=10)
    rec.add(bgr, detections)          # called by the lab for every processed frame
    rec.close()                       # writes data.yaml

    yolo detect train data=~/vision_lab_dataset/data.yaml model=yolov8n.pt imgsz=1280 epochs=50
    yolo obb train data=~/vision_lab_dataset/data.yaml model=yolov8n-obb.pt ...   (with fmt="obb")

The labels are only as good as the detector that made them: look through a sample of the images
(the *_check.png files written every `preview_every` saved frames) before training on them.
"""

import os
import random
from typing import List, Optional

import cv2
import numpy as np

from factory_bringup.product_catalog import PRODUCT_TYPES


class YoloDatasetRecorder:
    def __init__(self, out_dir: str, every_n: int = 10, val_fraction: float = 0.2, fmt: str = "detect",
                 single_class: bool = False, class_names: Optional[List[str]] = None, min_score: float = 0.0,
                 preview_every: int = 25, seed: int = 0):
        if fmt not in ("detect", "obb"):
            raise ValueError("fmt must be 'detect' or 'obb'")
        self.dir = os.path.abspath(os.path.expanduser(out_dir))
        self.every_n, self.val_fraction, self.fmt = max(1, int(every_n)), val_fraction, fmt
        self.single_class = single_class
        self.names = ["product"] if single_class else list(class_names or PRODUCT_TYPES)
        self.min_score, self.preview_every = min_score, preview_every
        self.rng = random.Random(seed)
        self.frames = self.saved = self.skipped = 0
        for split in ("train", "val"):
            os.makedirs(os.path.join(self.dir, "images", split), exist_ok=True)
            os.makedirs(os.path.join(self.dir, "labels", split), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "check"), exist_ok=True)
        self.start = len(os.listdir(os.path.join(self.dir, "images", "train"))) + \
            len(os.listdir(os.path.join(self.dir, "images", "val")))
        self.write_yaml()

    def add(self, bgr: np.ndarray, dets) -> bool:
        """Save this frame if it is the every_n-th one; returns True when saved."""
        self.frames += 1
        if self.frames % self.every_n:
            return False
        h, w = bgr.shape[:2]
        lines = []
        for d in dets:
            if d.score < self.min_score:
                continue
            cls = 0 if self.single_class else (self.names.index(d.label) if d.label in self.names else -1)
            if cls < 0:
                # an object we cannot name would teach the model that it is background: skip the frame
                self.skipped += 1
                return False
            if self.fmt == "obb" and d.box_px is not None:
                pts = np.clip(d.box_px / (w, h), 0, 1).reshape(-1)
                lines.append(f"{cls} " + " ".join(f"{p:.6f}" for p in pts))
            else:
                x1, y1, x2, y2 = d.bbox
                lines.append(f"{cls} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
        split = "val" if self.rng.random() < self.val_fraction else "train"
        stem = f"frame_{self.start + self.saved:06d}"
        cv2.imwrite(os.path.join(self.dir, "images", split, stem + ".png"), bgr)
        with open(os.path.join(self.dir, "labels", split, stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        if self.preview_every and self.saved % self.preview_every == 0:
            chk = bgr.copy()
            for d in dets:
                if d.box_px is not None:
                    cv2.polylines(chk, [d.box_px.astype(np.int32)], True, (0, 255, 0), 1)
                cv2.putText(chk, d.label, (int(d.u) + 6, int(d.v) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.imwrite(os.path.join(self.dir, "check", stem + "_check.png"), chk)
        self.saved += 1
        return True

    def write_yaml(self):
        with open(os.path.join(self.dir, "data.yaml"), "w") as f:
            f.write(f"# vision lab dataset ({self.fmt}); labels made by the lab's detector\n")
            f.write(f"path: {self.dir}\ntrain: images/train\nval: images/val\nnames:\n")
            for i, n in enumerate(self.names):
                f.write(f"  {i}: {n}\n")

    def close(self) -> str:
        self.write_yaml()
        return (f"dataset {self.dir}: {self.saved} frames saved this run, {self.skipped} skipped "
                f"(an object the class list does not name)")
