"""
LAB 05_my_opencv_detector: My own OpenCV detector picking products
Created 2026-10-31 from the 'custom_opencv' template.

Own-detector experiment: MyDetector below is the detector of this lab. Return ObjectDetection
objects in pixels; the lab works out world position, size, angle and bent. Try it without the
simulation first:  ./run_vision_lab.sh offline --lab 05_my_opencv_detector --synthetic 10 --world all

Run:  ./run_vision_lab.sh run 05_my_opencv_detector
"""
import cv2

from factory_bringup.vision_lab import Detector, ObjectDetection
from factory_bringup.vision_lab.classify import ProductColorClassifier


class MyDetector(Detector):
    """Otsu threshold + contours on the belts, colour label."""

    def __init__(self, min_area_px=25, blur=5):
        self.min_area_px, self.blur = min_area_px, blur
        self.clf = ProductColorClassifier()

    def setup(self, ctx):
        super().setup(ctx)                       # ctx.camera, ctx.lane_masks, ctx.world, ctx.object_top_z
        self.lanes = ctx.any_lane_mask

    def detect(self, bgr):
        gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (self.blur, self.blur), 0)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mask &= self.lanes
        if self.ctx.exclude_mask is not None:    # the arm, seen from above
            mask[self.ctx.exclude_mask > 0] = 0
        self.mask = mask
        out = []
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < self.min_area_px:
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            label, score, _ = self.clf.classify_region(bgr, box)
            out.append(ObjectDetection(u=rect[0][0], v=rect[0][1], box_px=box, label=label, score=score))
        return out

    def debug_layers(self):                      # thumbnails in the Vision Lab window
        return {"my mask": self.mask} if hasattr(self, "mask") else {}


LAB_TITLE = 'My own OpenCV detector picking products'
WORLD = None
DETECTOR = MyDetector
DETECTOR_OPTIONS = {"min_area_px": 25}
ARM_PARAMS = {}


def main(lab):
    lab.wait_for_frame()
    print("MyDetector sees:", [(d.label, d.bent, round(d.x, 2), round(d.y, 2)) for d in lab.detect_objects()])
    lab.snapshot()
    lab.move_to_recycle_bin()
    print(lab.report())
