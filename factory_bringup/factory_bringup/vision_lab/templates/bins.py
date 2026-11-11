# TEMPLATE: drop products into the trash can and the recycling bin
"""
LAB {{NAME}}: {{TITLE}}
Created {{DATE}} from the '{{TEMPLATE}}' template.

Bin experiment: the arm takes the next products from bent 1 and drops them, in the order of
SEQUENCE, into the trash can (left of the arm) or the recycling bin (right of the arm). Works in
every world.

Run:  ./run_vision_lab.sh run {{NAME}}
"""
LAB_TITLE = "{{TITLE}}"
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
