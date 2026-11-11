"""Example: sort by the detector's colour label: recyclable types into recycling, the rest into trash
(any world; on worlds with bent 2 the unknown ones go on to bent 2).
   run_vision_lab.sh run --script sort_into_bins"""
from factory_bringup.product_catalog import RECYCLABLE

DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}


def main(lab):
    def policy(ctx):
        if ctx.label is None:                      # the detector did not see it at the station
            return "bent2" if lab.world.has_bent2 else "skip"
        return "recycling" if ctx.label in RECYCLABLE else "trash"

    lab.on_result(lambda r: print(f"  -> #{r.number} {r.label} went to {r.destination} (truth {r.product_type})"))
    lab.run(policy, count=6, timeout=900)
    print(lab.report())
