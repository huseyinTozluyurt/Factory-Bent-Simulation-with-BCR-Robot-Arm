"""
LAB 02_bent_to_bent: Move products from bent 1 to bent 2
Created 2026-10-31 from the 'transfer' template.

Bent-to-bent transfer: the arm catches COUNT products on bent 1 and puts them on bent 2. In worlds
without bent 2 (circular, straight) they go to the recycling bin instead.

Run:  ./run_vision_lab.sh run 02_bent_to_bent
"""
LAB_TITLE = 'Move products from bent 1 to bent 2'
WORLD = None
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {}
ARM_PARAMS = {"motion_type": "auto"}     # auto, cycle, random, standard, mirror, high_carry, low_fast ...

COUNT = 3


def main(lab):
    print(f"World {lab.world.key}: destinations {lab.world.destinations}")
    for i in range(COUNT):
        if lab.world.has_bent2:
            result = lab.move_to_bent(2)            # bent 1 -> bent 2
        else:
            result = lab.move_to_recycle_bin()      # no bent 2 in this world
        print(f"{i + 1}/{COUNT}: {result}")
    print(lab.report())
