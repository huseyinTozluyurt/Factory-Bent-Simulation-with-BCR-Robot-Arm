"""Example: a detector of your own, plugged in from this file.
   run_vision_lab.sh run --script custom_detector
   run_vision_lab.sh offline --synthetic 10 --world all --detector custom_detector.py:BrightTopDetector"""
import cv2

from factory_bringup.vision_lab import Detector, ObjectDetection
from factory_bringup.vision_lab.classify import ProductColorClassifier


class BrightTopDetector(Detector):
    """Otsu threshold on the lanes, then a colour label."""

    def __init__(self, min_area_px=25):
        self.min_area_px = min_area_px
        self.clf = ProductColorClassifier()

    def setup(self, ctx):
        super().setup(ctx)
        self.lanes = ctx.any_lane_mask

    def detect(self, bgr):
        gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary &= self.lanes
        if self.ctx.exclude_mask is not None:
            binary[self.ctx.exclude_mask > 0] = 0
        self.binary = binary
        out = []
        for c in cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < self.min_area_px:
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            label, score, _ = self.clf.classify_region(bgr, box)
            out.append(ObjectDetection(u=rect[0][0], v=rect[0][1], box_px=box, label=label, score=score))
        return out

    def debug_layers(self):
        return {"otsu": self.binary} if hasattr(self, "binary") else {}


DETECTOR = BrightTopDetector


def main(lab):
    lab.wait_for_frame()
    print("Bright-top detector sees:", [(d.label, d.bent) for d in lab.detections()])
    lab.put_in_trash()
    print(lab.report())
