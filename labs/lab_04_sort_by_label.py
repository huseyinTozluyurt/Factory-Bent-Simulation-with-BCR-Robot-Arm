"""
LAB 04_sort_by_label: Sort by detector label into bent 2, trash and recycling
Created 2026-10-31 from the 'sort' template.

Sorting experiment: for every product the arm catches, decide() gets what the detector saw at the
catch station and returns where it goes. The report shows how often the detector's label matched
the simulation's truth.

Run:  ./run_vision_lab.sh run 04_sort_by_label
"""
from factory_bringup.product_catalog import RECYCLABLE

LAB_TITLE = 'Sort by detector label into bent 2, trash and recycling'
WORLD = None
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

COUNT = 6
TRASH = {"sneaker_pair", "packet_pouch", "unknown"}


def decide(ctx):
    """ctx.label / ctx.detection: what the detector saw. Return bent2, trash, recycling or skip."""
    if ctx.label is None:
        return "bent2" if ctx.world.has_bent2 else "skip"
    if ctx.label in TRASH:
        return "trash"
    if ctx.label in RECYCLABLE:
        return "recycling"
    return "bent2" if ctx.world.has_bent2 else "trash"


def main(lab):
    lab.on_result(lambda r: print(f"  #{r.number}: {r.label} -> {r.destination}  (truth {r.product_type})"))
    lab.run(decide, count=COUNT, timeout=900)
    print(lab.report())
