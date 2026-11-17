"""
LAB 11_all_boxes: Whole pipeline test, 70 products, every arm and bent, every box at least once
Created 2026-11-17.

70 products enter on conveyor S1 one after another. All 13 bents (S1 ... S9, C1 ... C4) run and all
12 BCR arms (A1 ... A12) work at the same time. The test PASSES when
  - every one of the 29 boxes has received at least one product:
        the trash bin and the recycling bin of every station   24 bins  (trash_bin_A1 ... recycling_bin_A12)
        the dock box past every output line                     5 docks (Dock 1 ... Dock 5)
  - and every product has arrived in a box: once all boxes are filled, the remaining products are
    transferred to Dock 1 ... Dock 5 (none left on a belt, in a gripper or on the floor).

    S1 -A1-> C1 -A2-> S2 -A4-> C2 -A6-> S4 -> Dock 1 cardboard     each arm also drops into
                                  -A7-> S5 -> Dock 2 containers    its own trash bin and
                -A3-> S3 -A5-> C3 -A9-> S7 -> Dock 3 large         its own recycling bin
                                  -A8-> S6 -A10-> C4 -A11-> S8 -> Dock 4 express
                                                     -A12-> S9 -> Dock 5 standard

How the arms decide (pipeline_testing.AllBoxesPolicy):
  while a box is empty: a product whose LABEL suits an empty bin of the station (recyclable ->
  recycling bin, a TRASH label -> trash bin) goes into it; otherwise to bent 2 while the boxes further
  down still need products; otherwise into an empty bin of the station; otherwise the arm leaves it.
  once all boxes have one: every product goes on to the docks, spread over the 5 docks.

Tree-of-three world (S1 feeds A1, A2, A3; 33 boxes: 24 bins + Dock 1 ... Dock 9):
      ./run_vision_lab.sh sim --world pipeline-tree --drawn-cameras   (terminal 1)
      ./run_vision_lab.sh run 11_all_boxes --world pipeline-tree      (terminal 2)

Run:  ./run_vision_lab.sh sim --world pipeline                (terminal 1)
      ./run_vision_lab.sh check                               (once: can the lab spawn / move / see in Gazebo?)
      ./run_vision_lab.sh run 11_all_boxes                    (terminal 2)
Without Gazebo:      ./run_vision_lab.sh run 11_all_boxes --fake --belt-speed 0.8
Other settings:      ALL_BOXES_COUNT=40 ALL_BOXES_TIMEOUT=5400 PIPELINE_DECIDE_BY=truth ./run_vision_lab.sh run 11_all_boxes
The report goes to labs/results/11_all_boxes/<time>/test_report.txt with snapshots of every station;
the lab exits with status 1 when the test FAILS (for scripts and CI).
"""
import os

from factory_bringup.vision_lab.pipeline_testing import format_all_boxes, run_all_boxes

LAB_TITLE = "Whole pipeline test: 70 products, all bents and arms, every box, the rest to the docks"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

COUNT = int(os.environ.get("ALL_BOXES_COUNT", "70"))   # products fed on S1, one after another
# the types they are drawn from (shuffled, the same order every run): recyclable, trash, every size
ITEMS = ["cereal_box", "glass_jar", "sneaker_pair", "coffee_can", "shoe_box", "basketball", "pizza_box",
         "packet_pouch", "tool_case", "toy_box", "water_jug", "milk_carton", "paint_can", "shipping_box_large"]
TRASH = ("sneaker_pair", "packet_pouch", "unknown")   # labels that go into a station's trash bin
TIMEOUT = float(os.environ.get("ALL_BOXES_TIMEOUT", "3600"))
BELT_SPEED = None                     # m/s for this test (None = the lab's, i.e. --belt-speed or the default)


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("This test is for the pipeline worlds: add --world pipeline or --world pipeline-tree (or --fake)")
    from factory_bringup import pipeline_layout as L
    from factory_bringup.vision_lab.pipeline_testing import all_boxes
    decide_by = os.environ.get("PIPELINE_DECIDE_BY", "label")
    print(f"All-boxes test ({L.WORLD_NAME}): {COUNT} products on S1, {len(L.STRAIGHTS) + len(L.CIRCLES)} bents, "
          f"{len(L.ARMS)} arms, {len(all_boxes())} boxes; decisions by {decide_by}; limit {TIMEOUT:.0f} s", flush=True)
    lab.on_result(lambda r: r.status != "skipped" and print(
        f"    {r.station:3s} #{r.number}: {r.label or '?':18s} -> {r.destination:9s} {r.status:9s} "
        f"(truth {r.product_type})", flush=True))

    res = run_all_boxes(lab, count=COUNT, items=ITEMS, timeout=TIMEOUT, decide_by=decide_by, belt_speed=BELT_SPEED,
                        trash_labels=TRASH)
    text = format_all_boxes(res)
    print("\n" + "=" * 78 + "\n" + text, flush=True)

    os.makedirs(lab.output_dir, exist_ok=True)
    for station in lab.all_stations:
        lab.snapshot(f"{station}.png", station=station)
    with open(os.path.join(lab.output_dir, "test_report.txt"), "w") as f:
        f.write(text + "\n\n" + lab.report() + "\n")
    print(f"report: {os.path.join(lab.output_dir, 'test_report.txt')}")
    if not res.passed:
        raise SystemExit(1)
