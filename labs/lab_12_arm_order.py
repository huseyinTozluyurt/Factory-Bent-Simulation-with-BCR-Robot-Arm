"""
LAB 12_arm_order: Every arm sorts in its own order: trash, recycling, next bent (and mixed orders)
Created 2026-11-17.

Products enter on S1 and travel through the pipeline. Whenever a product comes near an arm, the arm
picks it and puts it where its ORDER says, in turn: with ["trash", "recycling", "bent2"] the first
product it catches goes into its trash bin, the second into its recycling bin, the third onto the
next bent (towards the next arm), then trash again, and so on. Every arm has its own order (ORDERS
below); change them, or mix them, as you like. An arm without an entry uses DEFAULT_ORDER.

The order only advances when a pick really succeeded, so a missed grasp is retried for the same step.
Arms find products with the camera; with locate "assist" (default) an arm also picks a product its
camera missed, at the simulation's position, and the report counts those "assists".

A "skip" step lets one product pass (the step advances when the arm lets it go).
SHARE: arms that share the products of one conveyor. In the tree-of-three world (--world pipeline-tree)
A1, A2 and A3 all pick from S1; an arm of SHARE takes a third of the products that reach it (it catches
up after a busy spell), otherwise it lets them pass. A3 at the end of S1
is not in SHARE and takes all the others, so the three branches get about a third each (ORDERS_TREE);
the ring arms share their ring the same way.

Tree world:  ./run_vision_lab.sh sim --world pipeline-tree --drawn-cameras   (terminal 1)
             ./run_vision_lab.sh run 12_arm_order --world pipeline-tree      (terminal 2)

Run:  ./run_vision_lab.sh sim --world pipeline                (terminal 1)
      ./run_vision_lab.sh run 12_arm_order                    (terminal 2)
Without Gazebo:      ./run_vision_lab.sh run 12_arm_order --fake
Camera only:         ./run_vision_lab.sh run 12_arm_order --arm locate=vision
Products / time:     ARM_ORDER_COUNT=40 ARM_ORDER_TIMEOUT=1800 ./run_vision_lab.sh run 12_arm_order
The log (lab.log) and report.txt go to labs/results/12_arm_order/<time>/.
"""
import os
import threading
import time

from factory_bringup.vision_lab.pipeline_testing import S1Feeder, product_mix

LAB_TITLE = "Each arm in its own order: trash, recycling, next bent"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

DEFAULT_ORDER = ["trash", "recycling", "bent2"]
# arm -> its order (destinations: "trash", "recycling", "bent2" = the next bent, "skip" = let it pass)
ORDERS = {
    "A1": ["trash", "recycling", "bent2", "bent2", "bent2"],   # A1 feeds the whole factory: mostly bent2
    "A2": ["recycling", "bent2", "trash", "bent2"],
    "A3": ["trash", "bent2", "recycling", "bent2"],
    "A4": ["bent2", "trash", "bent2", "recycling"],
    "A5": ["recycling", "trash", "bent2", "bent2"],
    "A6": ["trash", "recycling", "bent2"],
    "A7": ["recycling", "trash", "bent2"],
    "A8": ["bent2", "recycling", "trash", "bent2"],
    "A9": ["trash", "bent2", "recycling"],
    "A10": ["bent2", "trash", "recycling", "bent2"],
    "A11": ["recycling", "bent2", "trash"],
    "A12": ["trash", "recycling", "bent2"],
}


# the tree-of-three world (pipeline_tree_layout.py): S1 -A1/A2/A3-> C1/C2/C3, each ring -> 3 output lines
ORDERS_TREE = {
    "A1": ["bent2", "bent2", "trash", "bent2", "recycling"],   # S1 -> C1 (shares S1, see SHARE_TREE)
    "A2": ["bent2", "recycling", "bent2", "trash"],            # S1 -> C2 (shares S1)
    "A3": ["bent2", "trash", "bent2", "recycling", "bent2"],   # end of S1: all the products A1, A2 let pass
    "A4": ["trash", "recycling", "bent2"],
    "A5": ["recycling", "bent2", "trash"],
    "A6": ["bent2", "trash", "recycling", "bent2"],
    "A7": ["recycling", "trash", "bent2"],
    "A8": ["bent2", "recycling", "trash"],
    "A9": ["trash", "bent2", "recycling", "bent2"],
    "A10": ["trash", "bent2", "recycling"],
    "A11": ["bent2", "trash", "recycling"],
    "A12": ["recycling", "trash", "bent2", "bent2"],
}
# tree world: these arms take turns with the other arms on their conveyor (S1, C1, C2, C3); the last arm of
# each conveyor (A3, A6, A9, A12) takes the rest, so every arm gets products
SHARE_TREE = ["A1", "A2", "A4", "A5", "A7", "A8", "A10", "A11"]
SHARE = []                            # pipeline world: no sharing
STATIONS = "all"                      # or e.g. ["A1", "A2", "A3"]
COUNT = int(os.environ.get("ARM_ORDER_COUNT", "40"))         # products fed on S1, one after another
TIMEOUT = float(os.environ.get("ARM_ORDER_TIMEOUT", "1800"))
ITEMS = ["cereal_box", "glass_jar", "sneaker_pair", "coffee_can", "shoe_box", "basketball", "pizza_box",
         "packet_pouch", "tool_case", "toy_box", "water_jug", "milk_carton", "paint_can"]


class OrderPolicy:
    """decide(ctx): the arm's current step; the step advances when that arm's pick is delivered."""

    def __init__(self, orders, default, share=()):
        self.orders = {a.upper(): list(o) for a, o in orders.items()}
        self.default = list(default)
        self.step = {}
        self.picks = {}
        self.share = {a.upper() for a in share}
        self.lock = threading.Lock()

    def order(self, arm):
        return self.orders.get(arm, self.default)

    def my_turn(self, lab, arm):
        """SHARE: on S1 the arm takes 1/3 of the products that reach it (A3 at the end takes the rest),
        on a ring no more than its fair share of what the ring's arms picked (PipelineLab.share_turn)."""
        if arm not in self.share:
            return True
        return lab.share_turn(lab, arm, self.picks)

    def __call__(self, ctx):
        with self.lock:
            if not self.my_turn(ctx.lab, ctx.station):
                return "skip"
            o = self.order(ctx.station)
            return o[self.step.get(ctx.station, 0) % len(o)]

    def on_result(self, r):
        if not r.station:
            return
        with self.lock:
            o = self.order(r.station)
            now = o[self.step.get(r.station, 0) % len(o)]
            if r.status == "delivered":
                self.picks[r.station] = self.picks.get(r.station, 0) + 1
            if r.status == "delivered" or (r.status == "skipped" and now == "skip"):
                self.step[r.station] = self.step.get(r.station, 0) + 1


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("This lab is for the pipeline worlds: add --world pipeline or --world pipeline-tree (or --fake)")
    from factory_bringup import pipeline_layout as L
    tree = L.LAYOUT == "tree"
    policy = OrderPolicy(ORDERS_TREE if tree else ORDERS, DEFAULT_ORDER, SHARE_TREE if tree else SHARE)
    print(f"Arm orders in {L.WORLD_NAME}", flush=True)
    lab.on_result(policy.on_result)
    lab.on_result(lambda r: r.status != "skipped" and print(
        f"    {r.station:3s} #{r.number}: {r.label or '?':18s} -> {r.destination:9s} {r.status:9s} "
        f"(truth {r.product_type})", flush=True))
    stations = lab.all_stations if STATIONS == "all" else [s.upper() for s in STATIONS]
    for a in stations:
        print(f"  {a:3s} order: {' -> '.join(policy.order(a))}", flush=True)

    lab.feed = False
    feeder = S1Feeder(lab, product_mix(COUNT, ITEMS)).start()
    sim = lab.sim
    t0 = time.time()

    def finished():
        with sim.lock:
            names = list(feeder.names)
            moving = [n for n in names if n in sim.products and sim.products[n].state in ("carried", "falling")]
            waiting = [n for n in names if n in sim.products and sim.products[n].state == "belt"]
        # done: everything fed, nothing in a gripper, and nothing left that a working arm could still take
        return feeder.done and not moving and not waiting
    try:
        lab.run(policy, stations=stations, timeout=TIMEOUT, until=finished)
    finally:
        feeder.stop()

    report = lab.report()
    counts = sim.summary()
    lines = [f"{len(feeder.names)} products fed in {time.time() - t0:.0f} s", "where they are: "
             + ", ".join(f"{k} {v}" for k, v in counts.items())]
    for a in stations:
        got = {k: v for k, v in counts.items() if k.endswith(f"_bin_{a}")}
        lines.append(f"  {a:3s} {' -> '.join(policy.order(a)):40s} steps {policy.step.get(a, 0):3d}  "
                     + ", ".join(f"{k} {v}" for k, v in got.items()))
    text = "\n".join(lines) + "\n\n" + report
    print("\n" + "=" * 78 + "\n" + text, flush=True)
    os.makedirs(lab.output_dir, exist_ok=True)
    with open(os.path.join(lab.output_dir, "report.txt"), "w") as f:
        f.write(text + "\n")
    print(f"report: {os.path.join(lab.output_dir, 'report.txt')}")
