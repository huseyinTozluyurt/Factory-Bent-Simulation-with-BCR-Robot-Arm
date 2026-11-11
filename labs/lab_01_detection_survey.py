"""
LAB 01_detection_survey: Detection survey: what the camera sees on each bent
Created 2026-10-31 from the 'detect' template.

Detection experiment: the arm does not move. The detector runs on the overhead camera for
DURATION seconds; the lab counts what it sees per bent and label, and saves annotated snapshots.

Run:      ./run_vision_lab.sh run 01_detection_survey
Offline:  ./run_vision_lab.sh offline --lab 01_detection_survey --synthetic 10 --world all
"""
import time
from collections import Counter

LAB_TITLE = 'Detection survey: what the camera sees on each bent'
WORLD = None                        # None = whatever world the simulation runs
DETECTOR = "opencv_contours"        # opencv_contours, opencv_color, opencv_bgsub, yolo, file.py:Class
DETECTOR_OPTIONS = {}
ARM_PARAMS = {}

DURATION = 20.0                     # seconds of watching
SNAPSHOT_EVERY = 5.0                # seconds between saved snapshots


def main(lab):
    lab.wait_for_frame()
    counts, frames, last_snap = Counter(), 0, 0.0
    end = time.time() + DURATION
    while time.time() < end:
        if not lab.wait_for_frame(2.0):
            continue
        frames += 1
        for d in lab.detect_objects():
            counts[(d.bent, d.label)] += 1
        if time.time() - last_snap >= SNAPSHOT_EVERY:
            print("snapshot:", lab.snapshot())
            last_snap = time.time()
    print(f"{frames} frames in {DURATION:.0f} s")
    for (bent, label), n in sorted(counts.items(), key=lambda kv: (kv[0][0] or 0, -kv[1])):
        print(f"  bent {bent}: {label:20s} seen in {n} frames ({n / max(1, frames):.1f} per frame)")
