"""
LAB 13_rendezvous: Rendezvous pick-and-place: the belts never stop, the arms meet products on them
Created 2026-11-17.

In the other pipeline labs an arm HOLDS the product it picks and its belt stands still while the arm
comes down (schedule "classic"). Here every belt keeps moving (schedule "rendezvous", rendezvous.py):

  - each arm predicts where a product on its pick belt will be (camera: position + speed along the belt;
    simulation position as a counted "assist" when the camera misses it),
  - it finds the rendezvous: the earliest moment the product is inside the arm's reach window on the
    belt and the arm can be there (approach time, IK time, when its current delivery ends),
  - it starts just in time, stretches its approach to arrive over the product together with it, comes
    down along with the moving product and grasps it at the planned contact point;
  - a scheduler spreads the products of a shared conveyor over its arms by their rendezvous windows and
    when each arm is free (assign=balanced: fewest picks first; assign=earliest: whoever meets it first),
    and one arm commits to a product at a time, so arms never go for the same one;
  - an arm places onto its next bent only into a gap of the moving belt (the hand-off to the next arms).

POLICY
  rules   every arm sorts by its rule in pipeline_layout.py; the arms with rule all / share of a conveyor
          share its products (tree world: A1, A2, A3 on S1)
  spread  every arm passes every product on to its next bent and all arms of a conveyor share it
          (pipeline world: C1 A2/A3, C2 A6/A7, C3 A8/A9, C4 A11/A12)
Default: rules in the tree-of-three world, spread in the pipeline world.

Compare with the classic schedule (same products, same policy): RENDEZVOUS_MODE=classic. The report
gives the time, the products per minute into the docks, the picks of every arm, how long the belts stood
for arms (0 s with rendezvous) and the rendezvous statistics (contact error, lost / missed rendezvous).

Tree world:  ./run_vision_lab.sh sim --world pipeline-tree --drawn-cameras   (terminal 1)
             ./run_vision_lab.sh run 13_rendezvous --world pipeline-tree     (terminal 2)
Pipeline:    ./run_vision_lab.sh sim --world pipeline --drawn-cameras        (terminal 1)
             ./run_vision_lab.sh run 13_rendezvous                           (terminal 2)
Without Gazebo:      ./run_vision_lab.sh run 13_rendezvous --fake [--world pipeline-tree]
Classic, to compare: RENDEZVOUS_MODE=classic ./run_vision_lab.sh run 13_rendezvous ...
Settings:            RENDEZVOUS_COUNT=30 RENDEZVOUS_TIMEOUT=1800 RENDEZVOUS_ASSIGN=earliest RENDEZVOUS_POLICY=rules
Camera only:         ./run_vision_lab.sh run 13_rendezvous --arm locate=vision
The log (lab.log) and report.txt go to labs/results/13_rendezvous/<time>/.
"""
import os
import time

from factory_bringup.vision_lab.pipeline_testing import S1Feeder, product_mix

LAB_TITLE = "Rendezvous scheduling: arms meet products on moving belts, shared conveyors spread over their arms"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {"schedule": "rendezvous"}

MODE = os.environ.get("RENDEZVOUS_MODE", "rendezvous").strip().lower()      # rendezvous or classic
ASSIGN = os.environ.get("RENDEZVOUS_ASSIGN", "balanced").strip().lower()    # balanced or earliest
POLICY = os.environ.get("RENDEZVOUS_POLICY", "").strip().lower()            # rules, spread ("" = by world)
COUNT = int(os.environ.get("RENDEZVOUS_COUNT", "30"))                       # products fed on S1
TIMEOUT = float(os.environ.get("RENDEZVOUS_TIMEOUT", "1800"))
ITEMS = ["cereal_box", "glass_jar", "sneaker_pair", "coffee_can", "shoe_box", "basketball", "pizza_box",
         "packet_pouch", "tool_case", "toy_box", "water_jug", "milk_carton", "paint_can", "shipping_box_large"]


def main(lab):
    if not hasattr(lab, "set_scheduler"):
        raise SystemExit("This lab is for the pipeline worlds: add --world pipeline or --world pipeline-tree (or --fake)")
    from factory_bringup import pipeline_layout as L
    tree = L.LAYOUT == "tree"
    policy = POLICY or ("rules" if tree else "spread")
    if policy not in ("rules", "spread"):
        raise SystemExit(f"RENDEZVOUS_POLICY={policy}: use rules or spread")
    if MODE == "rendezvous":
        lab.set_scheduler("rendezvous", assign=ASSIGN, pool="conveyor" if policy == "spread" else "rules")
    elif MODE == "classic":
        lab.set_scheduler("classic")
    else:
        raise SystemExit(f"RENDEZVOUS_MODE={MODE}: use rendezvous or classic")
    decide = None if policy == "rules" else "bent2"
    print(f"Rendezvous lab in {L.WORLD_NAME}: schedule {lab.schedule}, policy {policy}"
          + (f", assign {ASSIGN}" if MODE == "rendezvous" else "") + f", {COUNT} products on S1", flush=True)
    lab.on_result(lambda r: r.status != "skipped" and print(
        f"    {r.station:3s} #{r.number}: {r.label or '?':18s} -> {r.destination:9s} {r.status:9s} "
        f"(truth {r.product_type})", flush=True))

    lab.feed = False
    feeder = S1Feeder(lab, product_mix(COUNT, ITEMS)).start()
    sim = lab.sim
    t0 = time.time()

    def finished():
        with sim.lock:
            names = [n for n in feeder.names if n in sim.products]
            busy = [n for n in names if sim.products[n].state in ("belt", "carried", "falling")]
        return feeder.done and not busy
    try:
        lab.run(decide, stations="all", timeout=TIMEOUT, until=finished)
    finally:
        feeder.stop()

    took = time.time() - t0
    counts = sim.summary()
    docked = sum(v for k, v in counts.items() if k.startswith("dock_"))
    picks = {}
    for r in lab.results:
        if r.status == "delivered":
            picks[r.station] = picks.get(r.station, 0) + 1
    lines = [f"schedule {lab.schedule}, policy {policy}" + (f", assign {ASSIGN}" if MODE == "rendezvous" else ""),
             f"{len(feeder.names)} products fed, all done in {took:.0f} s; {docked} in the docks "
             f"({60.0 * docked / max(took, 1.0):.1f} per minute)",
             "where they are: " + ", ".join(f"{k} {v}" for k, v in counts.items()),
             "picks per arm: " + ", ".join(f"{a} {picks.get(a, 0)}" for a in lab.all_stations)]
    text = "\n".join(lines) + "\n\n" + lab.report()
    print("\n" + "=" * 78 + "\n" + text, flush=True)
    os.makedirs(lab.output_dir, exist_ok=True)
    with open(os.path.join(lab.output_dir, "report.txt"), "w") as f:
        f.write(text + "\n")
    print(f"report: {os.path.join(lab.output_dir, 'report.txt')}")
