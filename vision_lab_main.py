#!/usr/bin/env python3
"""
VISION LAB: main function. Edit this file, then run it against the simulation:

    ./run_vision_lab.sh sim --world dual          # terminal 1: Gazebo, bents, arm, cameras (once)
    ./run_vision_lab.sh run                       # terminal 2: runs main(lab) below; edit, run again

or offline (no Gazebo, no ROS):  ./run_vision_lab.sh offline --synthetic 20 --world all

Works in every world: dual, circular, straight, straight-circular, circular-straight,
straight-straight. lab.world tells you which one runs and what it has (lab.world.has_bent2,
lab.world.destinations). Worlds without bent 2 (circular, straight) can only use the bins.
"""

# ---------------------------------------------------------------------------------------------
# 1. DETECTOR: pick one (the command line --detector / --opt overrides these)
#    "opencv_contours"  the project's tuned grayscale + Canny contour detector, colour-labelled
#    "opencv_color"     Lab colour segmentation, DETECTOR_OPTIONS = {"targets": ["paint_can", ...]}
#    "opencv_bgsub"     MOG2 background subtraction
#    "yolo"             Ultralytics YOLO, DETECTOR_OPTIONS = {"model": "best.pt", "conf": 0.4}
#    "my_detectors.py:MyDetector" or the class itself (see class RedObjectDetector below)
# ---------------------------------------------------------------------------------------------
WORLD = None                       # None = whatever the simulation runs
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {"motion_type": "auto"}   # any vision_pick_place parameter, e.g. "swing_duration": 1.5

import cv2                         # noqa: E402
import numpy as np                 # noqa: E402

from factory_bringup.vision_lab import Detector, ObjectDetection, register_detector  # noqa: E402


# ---------------------------------------------------------------------------------------------
# 2. YOUR OWN DETECTOR (optional): register it and set DETECTOR = "red_objects"
# ---------------------------------------------------------------------------------------------
@register_detector("red_objects")
class RedObjectDetector(Detector):
    """Example: HSV threshold for red products (paint can grey is ignored, soda six-pack is found)."""

    def __init__(self, min_area_px: int = 30):
        self.min_area_px = min_area_px

    def setup(self, ctx):
        super().setup(ctx)              # ctx.camera, ctx.lane_masks, ctx.world, ctx.object_top_z
        self.lanes = ctx.any_lane_mask

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        red &= self.lanes
        out = []
        for c in cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < self.min_area_px:
                continue
            (u, v), _, _ = rect = cv2.minAreaRect(c)
            out.append(ObjectDetection(u=u, v=v, box_px=cv2.boxPoints(rect), label="red", score=1.0))
        return out          # pixels only: the lab works out world x / y, size, yaw and the bent


# ---------------------------------------------------------------------------------------------
# 3. MAIN FUNCTION: the arm and the bins
# ---------------------------------------------------------------------------------------------
def main(lab):
    print(f"World: {lab.world.key}, the arm can deliver to: {lab.world.destinations}")

    # --- vision ---------------------------------------------------------------------------
    lab.wait_for_frame()
    for d in lab.detections(bent=1):
        print(f"bent 1: {d.label} ({d.score:.2f}) at x={d.x:.2f} y={d.y:.2f}, {d.size_x:.2f} x {d.size_y:.2f} m")

    # optional: put your own products on bent 1 (product_catalog.PRODUCT_TYPES)
    # lab.spawn_products(["paint_can", "cereal_box", "pizza_box", "water_jug"], spacing=1.2)

    # --- one product at a time ------------------------------------------------------------
    if lab.world.has_bent2:
        r = lab.move_to_bent2()                 # same as lab.transfer("bent2"): bent 1 -> bent 2
        print("bent to bent:", r)
    print("trash:", lab.put_in_trash())         # next product into the trash bin
    print("recycling:", lab.drop_in_recycle_bin())   # next product into the recycling bin

    # --- a sorting policy decided by YOUR detector ----------------------------------------
    def policy(ctx):
        """ctx.label / ctx.detection: what the detector saw; ctx.product_type: simulation truth
        (use it only to score your detector, a real factory does not have it)."""
        if ctx.label in ("paint_can", "glass_jar", "coffee_can"):
            return "recycling"
        if ctx.label in ("sneaker_pair", "packet_pouch", "unknown"):
            return "trash"
        return "bent2" if lab.world.has_bent2 else "skip"

    lab.run(policy, count=4, timeout=600)
    # the same as a table:  lab.run({"paint_can": "recycling", "sneaker_pair": "trash", "*": "bent2"}, count=4)

    print(lab.report())                         # where they went + how often the label was right


if __name__ == "__main__":                      # python3 vision_lab_main.py  ==  vision_lab run --script this file
    import sys
    from factory_bringup.vision_lab.main import main as lab_cli
    sys.exit(lab_cli(["run", "--script", __file__] + sys.argv[1:]))
