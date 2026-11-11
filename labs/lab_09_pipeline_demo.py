"""
LAB 09_pipeline_demo: Pipeline factory demo, detection to sorting in three acts
Created 2026-10-31.

A scripted demo of the vision lab in the pipeline world (pipeline_factory_lab.sdf):

  Act 1  Detection   a known mix of products arrives at station A1; the camera's detections are
                     printed next to the simulation's truth and saved as an annotated snapshot.
  Act 2  One arm     arm A1 moves one product to its bent 2 (ring C1), one into its trash bin and
                     one into its recycling bin, each chosen by vision.
  Act 3  The factory all 12 arms work at once; each throws away what the detector calls trash and
                     otherwise follows its station rule (recyclable, cardboard, large, small ...)
                     judged on the detector's label and size.

Run:  ./run_vision_lab.sh sim --world pipeline     (terminal 1)
      ./run_vision_lab.sh run 09_pipeline_demo     (terminal 2)
or without Gazebo:  ./run_vision_lab.sh run 09_pipeline_demo --fake --belt-speed 0.8
Results (snapshots, a video of station A1, report.txt) go to labs/results/09_pipeline_demo/<time>/.
"""
import os
import time

import cv2

from factory_bringup.vision_lab.pipeline_world import station

LAB_TITLE = "Pipeline factory demo: detection, one arm, the whole factory"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

ACT1_ITEMS = ["cereal_box", "sneaker_pair", "glass_jar", "shoe_box", "coffee_can", "basketball"]
ACT3_ITEMS = ["pizza_box", "tool_case", "shipping_box_large", "water_jug", "packet_pouch", "toy_box"]
ACT3_COUNT = 8                       # deliveries in act 3 (all stations together)
ACT3_TIMEOUT = 600
TRASH = {"sneaker_pair", "packet_pouch", "unknown"}
SNAPSHOT_STATIONS = ["A1", "A2", "A3", "A4", "A5", "A6"]


def banner(text):
    print("\n" + "=" * 78 + f"\n  {text}\n" + "=" * 78, flush=True)


def decide(ctx):
    """Act 3: trash what the detector calls trash, otherwise the station's own rule."""
    if ctx.label in TRASH:
        return "trash"
    return ctx.lab.layout_rule(ctx)


class StationVideo:
    """Writes the annotated view of one station camera to an mp4 while the demo runs."""

    def __init__(self, lab, station, path, fps=5):
        self.lab, self.station, self.path, self.fps = lab, station, path, fps
        self.writer = None
        lab.on_frame(self.frame)

    def frame(self, bgr, dets, station=None):
        if station != self.station:
            return
        view = self.lab.stations[station].pipe.draw(bgr, dets, status=f"station {station}")
        view = cv2.resize(view, (960, 540))
        if self.writer is None:
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (960, 540))
        self.writer.write(view)

    def close(self):
        if self.writer is not None:
            self.writer.release()


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("This demo is for the pipeline world: ./run_vision_lab.sh run 09_pipeline_demo --world pipeline")
    os.makedirs(lab.output_dir, exist_ok=True)
    video = StationVideo(lab, "A1", os.path.join(lab.output_dir, "station_A1.mp4"))
    lab.on_result(lambda r: print(f"    {r.station} #{r.number}: {r.label or '?':18s} -> {r.destination:9s} "
                                  f"{r.status:9s} (truth {r.product_type})", flush=True))
    t0 = time.time()

    # ------------------------------------------------------------------ act 1
    banner("Act 1: detection at station A1 (S1 inbound line, picked by arm A1)")
    print(lab.spawn_products(ACT1_ITEMS, bent=1, station="A1", spacing=0.9, start_offset=1.2))
    time.sleep(4.0)                                  # let them queue at the end of S1
    dets = lab.detect_objects(bent=1, station="A1")
    px, py = station("A1")["pick"]
    in_view = [p for p in lab.sim.on_conveyor("S1") if abs(p.x - px) < 2.3]     # simulation truth
    print(f"  the camera sees {len(dets)} products on S1; {len(in_view)} are really in view: "
          + ", ".join(p.ptype for p in sorted(in_view, key=lambda p: -p.x)))
    for d in sorted(dets, key=lambda d: -d.x):
        near = lab.sim.nearest("S1", d.x, d.y, 0.3)
        real = near.ptype if near else "-"
        mark = "ok" if real == d.label else "WRONG"
        print(f"    {d.label:18s} score {d.score:.2f}  at ({d.x:6.2f}, {d.y:5.2f})  "
              f"size {d.size_x or 0:.2f} x {d.size_y or 0:.2f} m   truth {real:18s} [{mark}]")
    print("  snapshot:", lab.snapshot("act1_detection_A1.png", station="A1"))

    # ------------------------------------------------------------------ act 2
    banner("Act 2: arm A1 moves three products, chosen by vision")
    for what, call in (("bent 2 (ring C1)", lab.move_to_bent), ("trash bin", lab.move_to_trash),
                       ("recycling bin", lab.move_to_recycle_bin)):
        print(f"  next product -> {what}")
        r = call(2, timeout=120, station="A1") if call == lab.move_to_bent else call(timeout=120, station="A1")
        if r is None:
            print("    (no product reached A1 in time)")
    print("  snapshot:", lab.snapshot("act2_after_A1.png", station="A1"))
    print("  where products are:", lab.summary())

    # ------------------------------------------------------------------ act 3
    banner(f"Act 3: the whole factory, all 12 stations, until {ACT3_COUNT} deliveries")
    print(lab.spawn_products(ACT3_ITEMS, conveyor="S1", spacing=1.2))
    print("  station rules: " + ", ".join(f"{a}={station(a)['rule']}" for a in lab.all_stations))
    lab.run(decide, stations="all", count=ACT3_COUNT, timeout=ACT3_TIMEOUT)
    for s in SNAPSHOT_STATIONS:
        lab.snapshot(f"act3_{s}.png", station=s)

    # ------------------------------------------------------------------ report
    banner(f"Report ({time.time() - t0:.0f} s)")
    report = lab.report()
    print(report)
    video.close()
    with open(os.path.join(lab.output_dir, "report.txt"), "w") as f:
        f.write(report + "\n")
    print(f"\nResults in {lab.output_dir}: snapshots, station_A1.mp4, report.txt")
