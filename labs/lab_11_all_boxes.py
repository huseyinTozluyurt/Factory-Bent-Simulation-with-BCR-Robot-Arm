"""
LAB 11_all_boxes: Whole pipeline test, every bent and every arm running, every box gets a product
Created 2026-11-17.

All 13 bents (S1 ... S9, C1 ... C4) run and all 12 BCR arms (A1 ... A12) work at the same time. The
feeder keeps putting products on S1, and the test PASSES when every one of the 29 boxes of the
factory has received at least one product:

    the trash bin and the recycling bin of every station   24 bins  (trash_bin_A1 ... recycling_bin_A12)
    the dock box past every output line                     5 docks (dock_S4, S5, S7, S8, S9)

and so every arm has delivered and every bent has carried products.

    S1 -A1-> C1 -A2-> S2 -A4-> C2 -A6-> S4 -> dock 1        each arm also drops into
                                  -A7-> S5 -> dock 2        its own trash bin and
                -A3-> S3 -A5-> C3 -A9-> S7 -> dock 3        its own recycling bin
                                  -A8-> S6 -A10-> C4 -A11-> S8 -> dock 4
                                                     -A12-> S9 -> dock 5

How the arms decide (pipeline_testing.AllBoxesPolicy): a product whose LABEL suits an empty bin of
the station (recyclable -> recycling bin, a TRASH label -> trash bin) goes into it; otherwise it goes
on to bent 2 while the boxes further down still need products; otherwise into an empty bin of the
station; otherwise the arm leaves it (on a ring it rides on to the next arm). So vision sorts into
the bins, and the factory still fills every box whatever the random product mix is.

Run:  ./run_vision_lab.sh sim --world pipeline                (terminal 1)
      ./run_vision_lab.sh run 11_all_boxes                    (terminal 2)
or both in one:      ./run_vision_lab.sh --world pipeline 11_all_boxes
Without Gazebo:      ./run_vision_lab.sh run 11_all_boxes --fake --belt-speed 0.8
Arms and belts only (bins by the true type, not the detector):  PIPELINE_DECIDE_BY=truth ...
Longer / shorter limit:  ALL_BOXES_TIMEOUT=2400 ...
The report goes to labs/results/11_all_boxes/<time>/test_report.txt with snapshots of every station;
the lab exits with status 1 when the test FAILS (for scripts and CI).
"""
import os

from factory_bringup.vision_lab.pipeline_testing import format_all_boxes, run_all_boxes

LAB_TITLE = "Whole pipeline test: all bents, all arms, every box at least once"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

# what the feeder puts on S1: a mix of recyclable, trash and other products of every size
ITEMS = ["cereal_box", "glass_jar", "sneaker_pair", "coffee_can", "shoe_box", "basketball", "pizza_box",
         "packet_pouch", "tool_case", "toy_box", "water_jug", "milk_carton", "paint_can", "shipping_box_large"]
TRASH = ("sneaker_pair", "packet_pouch", "unknown")   # labels that go into a station's trash bin
TIMEOUT = float(os.environ.get("ALL_BOXES_TIMEOUT", "1800"))
BELT_SPEED = None                     # m/s for this test (None = the lab's, i.e. --belt-speed or the default)


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("This test is for the pipeline world: add --world pipeline (or --fake)")
    decide_by = os.environ.get("PIPELINE_DECIDE_BY", "label")
    print(f"All-boxes test: 13 bents, 12 arms, 29 boxes; decisions by {decide_by}; limit {TIMEOUT:.0f} s",
          flush=True)
    lab.on_result(lambda r: r.status != "skipped" and print(
        f"    {r.station:3s} #{r.number}: {r.label or '?':18s} -> {r.destination:9s} {r.status:9s} "
        f"(truth {r.product_type})", flush=True))

    res = run_all_boxes(lab, items=ITEMS, timeout=TIMEOUT, decide_by=decide_by, belt_speed=BELT_SPEED,
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
