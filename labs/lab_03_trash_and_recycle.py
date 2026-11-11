"""
LAB 03_trash_and_recycle: Drop products into the trash can and the recycling bin
Created 2026-10-31 from the 'bins' template.

Bin experiment: the arm takes the next products from bent 1 and drops them, in the order of
SEQUENCE, into the trash can (left of the arm) or the recycling bin (right of the arm). Works in
every world.

Run:  ./run_vision_lab.sh run 03_trash_and_recycle
"""
LAB_TITLE = 'Drop products into the trash can and the recycling bin'
WORLD = None
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {}
ARM_PARAMS = {}

SEQUENCE = ["trash", "recycling", "trash", "recycling"]


def main(lab):
    for i, where in enumerate(SEQUENCE, 1):
        if where == "trash":
            result = lab.move_to_trash()
        else:
            result = lab.move_to_recycle_bin()
        print(f"{i}/{len(SEQUENCE)} -> {where}: {result}")
    print(lab.report())
