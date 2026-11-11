# TEMPLATE: the whole pipeline factory: every arm station sorts by vision (pipeline world)
"""
LAB {{NAME}}: {{TITLE}}
Created {{DATE}} from the '{{TEMPLATE}}' template.

Pipeline world (pipeline_factory_lab.sdf): products enter on S1 and travel through 12 arm stations.
Each arm has its own camera, its own trash and recycling bin, and a rule from pipeline_layout.py
(all / recyclable / cardboard / not_large / large / small) that it judges on what YOUR detector sees.
decide(ctx) runs for every product that reaches an arm: ctx.station says which arm asks.

Run:  ./run_vision_lab.sh sim --world pipeline      (terminal 1)
      ./run_vision_lab.sh run {{NAME}}             (terminal 2)
or without Gazebo:  ./run_vision_lab.sh run {{NAME}} --fake
In the other worlds the same file runs at the single arm.
"""

LAB_TITLE = "{{TITLE}}"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

STATIONS = "all"                     # or e.g. ["A1", "A2", "A3"]
COUNT = 20                           # stop after this many deliveries (all stations together)
TIMEOUT = 900
TRASH = {"sneaker_pair", "packet_pouch", "unknown"}


def decide(ctx):
    """Return bent2 (the station's place conveyor), trash, recycling or skip (leave it for the next arm)."""
    if ctx.label in TRASH:
        return "trash"                                   # every station throws away what it cannot use
    rule = getattr(ctx.lab, "layout_rule", None)
    if rule is not None:                                 # pipeline world: the station's own rule
        return rule(ctx)
    return "bent2" if ctx.world.has_bent2 else "recycling"


def main(lab):
    lab.on_result(lambda r: print(f"  {r.station or ''} #{r.number}: {r.label} -> {r.destination} "
                                  f"({r.status}, truth {r.product_type})"))
    if hasattr(lab, "all_stations"):                     # pipeline world
        print("Stations:", ", ".join(lab.all_stations))
        lab.run(decide, count=COUNT, timeout=TIMEOUT, stations=STATIONS)
        for station in ("A1", "A2"):
            lab.snapshot(f"{station}.png", station=station)
    else:
        lab.run(decide, count=min(COUNT, 6), timeout=TIMEOUT)
        lab.snapshot()
    print(lab.report())
