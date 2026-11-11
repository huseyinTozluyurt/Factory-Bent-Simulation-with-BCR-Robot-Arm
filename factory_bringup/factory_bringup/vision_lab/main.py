#!/usr/bin/env python3
"""
vision_lab: the main entry point of the vision lab.

  ros2 run factory_bringup vision_lab run     [LAB | --script my_lab.py] [--world W] [--detector D] [--opt k=v ...]
  ros2 run factory_bringup vision_lab new     NAME [--template T] [--world W]   (creates labs/lab_NAME.py)
  ros2 run factory_bringup vision_lab list | templates
  ros2 run factory_bringup vision_lab offline (--image FILE_OR_DIR ... | --synthetic N) [--detector D[,D2]]
  ros2 run factory_bringup vision_lab worlds
  ros2 run factory_bringup vision_lab detectors

run      live, on the running simulation (start it with run_vision_lab.sh sim). Calls main(lab) of
         your script; without --script it uses ./vision_lab_main.py if there is one, else it only shows
         the detections until Ctrl+C.
         In the pipeline world (--world pipeline[:A2], or found running) lab is a PipelineLab: the same
         calls, per arm station (--station A1 ... A12); --fake runs it without Gazebo.
offline  no ROS needed: runs detectors on saved images or on synthetic scenes of any world and prints
         precision / recall / label accuracy (synthetic) or the detections (images); writes annotated
         images to --out.

Your script (see vision_lab_main.py in the project folder):
    WORLD = None                  # optional defaults, the command line wins
    DETECTOR = "opencv_contours"
    DETECTOR_OPTIONS = {}
    ARM_PARAMS = {}

    def main(lab):                # lab: factory_bringup.vision_lab.VisionLab
        lab.transfer("bent2")
"""

import argparse
import glob
import importlib.util
import os
import sys
import time
from typing import Dict, List, Optional


def load_script(path: str):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise SystemExit(f"Script not found: {path}")
    spec = importlib.util.spec_from_file_location("vision_lab_script", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["vision_lab_script"] = module
    sys.path.insert(0, os.path.dirname(path))           # so the script can import its neighbours
    spec.loader.exec_module(module)
    if not callable(getattr(module, "main", None)):
        raise SystemExit(f"{path} has no main(lab) function")
    return module


def find_script(arg: Optional[str]) -> Optional[str]:
    if arg:
        if os.path.isfile(os.path.expanduser(arg)):
            return arg
        from factory_bringup.vision_lab.labfiles import find_lab
        lab_file = find_lab(arg)                         # a lab name: labs/lab_<name>.py
        if lab_file:
            return lab_file
        try:                                             # a bare name: one of the installed examples
            from ament_index_python.packages import get_package_share_directory
            share = os.path.join(get_package_share_directory("factory_bringup"), "vision_lab_examples")
            for cand in (arg, arg + ".py"):
                if os.path.isfile(os.path.join(share, cand)):
                    return os.path.join(share, cand)
        except Exception:
            pass
        raise SystemExit(f"Script not found: {arg}")
    return "vision_lab_main.py" if os.path.isfile("vision_lab_main.py") else None


def results_dir(script: Optional[str]) -> str:
    """<labs>/results/<lab name>/<date_time> for a lab file, else ./vision_lab_results/<script>/<date_time>."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if not script:
        return os.path.abspath(os.path.join("vision_lab_results", stamp))
    stem = os.path.splitext(os.path.basename(script))[0]
    base = os.path.dirname(os.path.abspath(script))
    if stem.startswith("lab_"):
        return os.path.join(base, "results", stem[4:], stamp)
    return os.path.abspath(os.path.join("vision_lab_results", stem, stamp))


def pipeline_running() -> bool:
    """Is the pipeline lab world running in Gazebo? (False when the gz Python bindings are missing.)"""
    try:
        from factory_bringup.vision_lab.pipeline_sim import GzBackend
        return GzBackend().running()
    except Exception:
        return False


def make_lab(a, world, detector, options, record_opts, arm_params, output_dir):
    """VisionLab (ROS, single-arm worlds) or PipelineLab (gz-transport, pipeline world)."""
    from factory_bringup.vision_lab.worlds import is_pipeline
    pipeline = a.fake or a.station or is_pipeline(world) or (world is None and pipeline_running())
    if pipeline:
        from factory_bringup.vision_lab.pipeline_lab import PipelineLab
        station = a.station or (world.split(":", 1)[1] if world and ":" in world else "A1")
        kw = {}
        if a.belt_speed:
            kw["belt_speed"] = a.belt_speed
        if a.dry_run:
            print("[vision_lab] --dry-run is not used in the pipeline world (its arms are moved by the lab itself)")
        return PipelineLab(station=station, detector=detector, detector_options=options,
                           backend="fake" if a.fake else "gz", show_window=not a.no_window,
                           record_dataset=a.record, record_options=record_opts, arm_params=arm_params,
                           output_dir=output_dir, feed=not a.no_feed, **kw)
    from factory_bringup.vision_lab.ros_lab import VisionLab
    return VisionLab(world=world, detector=detector, detector_options=options, show_window=not a.no_window,
                     dry_run=a.dry_run, record_dataset=a.record, record_options=record_opts, arm_params=arm_params,
                     output_dir=output_dir)


def offline_worlds(world_arg):
    from factory_bringup.vision_lab.pipeline_world import STATIONS, station_profile
    from factory_bringup.vision_lab.worlds import get_world, is_pipeline, list_worlds
    if world_arg == "all":
        return list_worlds(include_pipeline=True)
    if is_pipeline(world_arg) and ":" not in world_arg:
        return [station_profile(s) for s in STATIONS]
    return [get_world(world_arg)]


def cmd_run(a) -> int:
    from factory_bringup.vision_lab.detectors.base import parse_options
    script = find_script(a.lab or a.script)
    module = load_script(script) if script else None
    world = a.world or getattr(module, "WORLD", None)
    detector = a.detector or getattr(module, "DETECTOR", None) or "opencv_contours"
    options = dict(getattr(module, "DETECTOR_OPTIONS", {}) or {})
    if not a.detector or a.detector == getattr(module, "DETECTOR", None):
        options.update(parse_options(a.opt))
    else:
        options = parse_options(a.opt)
    arm_params = dict(getattr(module, "ARM_PARAMS", {}) or {})
    arm_params.update(parse_options(a.arm))
    record_opts = {"every_n": a.record_every}
    if a.record_format:
        record_opts["fmt"] = a.record_format

    try:
        lab = make_lab(a, world, detector, options, record_opts, arm_params, results_dir(script))
    except RuntimeError as exc:
        raise SystemExit(str(exc))
    if script:
        lab.log(f"Lab {os.path.basename(script)}: {getattr(module, 'LAB_TITLE', '')}  (results: {lab.output_dir})")
    try:
        if not lab.wait_for_frame(15.0):
            lab.log("No overhead camera image yet (is the simulation running? run_vision_lab.sh sim).")
        if module is not None:
            lab.log(f"Running main(lab) of {script}")
            module.main(lab)
            lab.log("main(lab) finished.")
            if lab.results:
                lab.log("\n" + lab.report())
            if a.stay:
                lab.log("--stay: keeping the detector running, Ctrl+C to quit.")
                while True:
                    time.sleep(1.0)
        else:
            lab.log("No script: showing the detections only (Ctrl+C to quit). Give one with --script.")
            while True:
                time.sleep(2.0)
                d = lab.detections()
                lab.log(f"{len(d)} objects: " + ", ".join(f"{x.label}@bent{x.bent}({x.x:.2f},{x.y:.2f})" for x in d[:8]))
    except KeyboardInterrupt:
        lab.log("Ctrl+C")
    finally:
        lab.close()
    return 0


def _images(paths: List[str]) -> List[str]:
    out = []
    for p in paths:
        p = os.path.expanduser(p)
        if os.path.isdir(p):
            for ext in ("png", "jpg", "jpeg", "bmp"):
                out += sorted(glob.glob(os.path.join(p, f"*.{ext}")))
        else:
            out += sorted(glob.glob(p)) or [p]
    return out


def cmd_offline(a) -> int:
    import cv2
    from factory_bringup.vision_lab.dataset import YoloDatasetRecorder
    from factory_bringup.vision_lab.detectors.base import parse_options
    from factory_bringup.vision_lab.pipeline import DetectionPipeline
    from factory_bringup.vision_lab.synthetic import match_truth, random_scene

    module = None
    if a.lab or a.script:                                # registers the lab's detectors; its DETECTOR is the default
        module = load_script(find_script(a.lab or a.script))
    if a.detector:
        options = parse_options(a.opt)
        detectors = [d.strip() for d in a.detector.split(",") if d.strip()]
    elif module is not None and getattr(module, "DETECTOR", None) is not None:
        options = dict(getattr(module, "DETECTOR_OPTIONS", {}) or {}, **parse_options(a.opt))
        detectors = [module.DETECTOR]
    else:
        options = parse_options(a.opt)
        detectors = ["opencv_contours"]
    world_arg = a.world or getattr(module, "WORLD", None)
    worlds = offline_worlds(world_arg)
    out_dir = os.path.expanduser(a.out) if a.out else None
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    rec = YoloDatasetRecorder(a.record, every_n=1) if a.record else None

    for det_name in detectors:
        for w in worlds:
            pipe = DetectionPipeline(det_name, w, options if len(detectors) == 1 else {})
            shown = det_name if isinstance(det_name, str) else pipe.detector.name
            shown = shown if shown != "detector" else type(pipe.detector).__name__
            if a.synthetic:
                totals: Dict[str, float] = {}
                for i in range(a.synthetic):
                    bgr, truth = random_scene(w, n_per_bent=a.per_bent, seed=a.seed + i)
                    dets = pipe.process(bgr)
                    m = match_truth(dets, truth)
                    for k in ("precision", "recall", "label_accuracy", "mean_xy_error_m"):
                        totals[k] = totals.get(k, 0.0) + (m[k] or 0.0) / a.synthetic
                    totals["ms"] = totals.get("ms", 0.0) + pipe.last_ms / a.synthetic
                    if rec:
                        rec.add(bgr, dets)
                    if out_dir and i < 3:
                        cv2.imwrite(os.path.join(out_dir, f"{shown}_{w.key}_{i}.png"), pipe.draw(bgr, dets))
                print(f"{shown:16s} {w.key:18s} precision {totals['precision']:.2f}  recall {totals['recall']:.2f}  "
                      f"label acc {totals['label_accuracy']:.2f}  xy err {totals['mean_xy_error_m'] * 100:.1f} cm  "
                      f"{totals['ms']:.0f} ms/frame  ({a.synthetic} synthetic scenes)")
            else:
                files = _images(a.image or [])
                if not files:
                    raise SystemExit("offline: give --image FILE/DIR or --synthetic N")
                for f in files:
                    bgr = cv2.imread(f)
                    if bgr is None:
                        print(f"cannot read {f}")
                        continue
                    dets = pipe.process(bgr)
                    print(f"{os.path.basename(f)} [{shown}, {w.key}]: {len(dets)} objects, {pipe.last_ms:.0f} ms")
                    for d in dets:
                        print(f"   bent {d.bent}  {d.label:20s} {d.score:.2f}  x {d.x:7.3f}  y {d.y:7.3f}  "
                              f"size {d.size_x or 0:.2f} x {d.size_y or 0:.2f}")
                    if rec:
                        rec.add(bgr, dets)
                    view = pipe.draw(bgr, dets)
                    if out_dir:
                        cv2.imwrite(os.path.join(out_dir, f"{shown}_{os.path.basename(f)}"), view)
                    if a.show:
                        cv2.imshow("Vision Lab (offline)", view)
                        cv2.waitKey(0)
    if rec:
        print(rec.close())
    return 0


def cmd_new(a) -> int:
    from factory_bringup.vision_lab.labfiles import create_lab, lab_dir
    path = create_lab(a.name, a.template, a.dir, world=a.world, detector=a.detector, title=a.title, force=a.force)
    print(f"Created {path}")
    print(f"Edit it, then run:  run_vision_lab.sh run {os.path.basename(path)[4:-3]}   "
          f"(labs folder: {lab_dir(a.dir)})")
    return 0


def cmd_list(a) -> int:
    from factory_bringup.vision_lab.labfiles import lab_dir, list_labs
    labs = list_labs(a.dir)
    print(f"Labs in {lab_dir(a.dir)}:")
    if not labs:
        print("  (none yet: vision_lab new my_first_lab --template transfer)")
    for name, _f, meta in labs:
        det = meta["detector"] if isinstance(meta["detector"], str) else meta["detector"] or "-"
        print(f"  {name:28s} {meta['title'][:50]:50s} world={meta['world'] or 'any'}  detector={det}")
    return 0


def cmd_templates(_a) -> int:
    from factory_bringup.vision_lab.labfiles import templates
    for name, doc in templates().items():
        print(f"  {name:15s} {doc}")
    return 0


def cmd_worlds(_a) -> int:
    from factory_bringup.vision_lab.worlds import UNSUPPORTED, list_worlds
    for w in list_worlds():
        print(w.describe())
    for k, why in UNSUPPORTED.items():
        print(f"{k:18s} (not supported) {why}")
    from factory_bringup.vision_lab.pipeline_world import describe
    print("\npipeline (pipeline_factory_lab.sdf): one profile per arm station, --world pipeline:A1 ... pipeline:A12\n"
          "  bent 1 = the conveyor the arm picks from, bent 2 = the one it places on, own trash and recycling bins")
    print("  " + describe().replace("\n", "\n  "))
    return 0


def cmd_detectors(_a) -> int:
    from factory_bringup.vision_lab.detectors.base import available_detectors
    for name, doc in available_detectors().items():
        print(f"{name:16s} {doc}")
    print("\nYour own: --detector path/to/file.py:ClassName or package.module:ClassName (a Detector subclass).")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="vision_lab", description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    def common(p):
        p.add_argument("--world", help="dual, circular, straight, straight-circular, circular-straight, "
                                       "straight-straight, pipeline, pipeline:A1 ... pipeline:A12 "
                                       "(run: default = the running simulation's; offline: also all)")
        p.add_argument("--detector", help="opencv_contours (default), opencv_color, opencv_bgsub, yolo, "
                                          "module:Class or file.py:Class")
        p.add_argument("--opt", action="append", default=[], metavar="KEY=VALUE", help="detector option (repeatable)")
        p.add_argument("--script", help="Python file with main(lab) (offline: only to register its detectors)")
        p.add_argument("--record", metavar="DIR", help="save auto-labelled YOLO training frames into DIR")

    r = sub.add_parser("run", help="live on the running simulation")
    r.add_argument("lab", nargs="?", help="lab name (labs/lab_<name>.py) or a Python file with main(lab)")
    common(r)
    r.add_argument("--arm", action="append", default=[], metavar="KEY=VALUE", help="vision_pick_place parameter")
    r.add_argument("--no-window", action="store_true")
    r.add_argument("--dry-run", action="store_true", help="plan and log arm goals, do not move the arm")
    r.add_argument("--record-every", type=int, default=10, help="save every N-th frame (default 10)")
    r.add_argument("--record-format", choices=["detect", "obb"], help="YOLO label format (default detect)")
    r.add_argument("--stay", action="store_true", help="keep running after main(lab) returns")
    r.add_argument("--station", help="pipeline world: the arm station the lab works at (A1 ... A12, default A1)")
    r.add_argument("--fake", action="store_true",
                   help="pipeline world without Gazebo: the same lab engine with drawn camera images")
    r.add_argument("--no-feed", action="store_true",
                   help="pipeline world: do not keep products coming (use lab.spawn_products)")
    r.add_argument("--belt-speed", type=float, help="pipeline world: belt speed in m/s")
    r.set_defaults(func=cmd_run)

    o = sub.add_parser("offline", help="images or synthetic scenes, no ROS needed")
    common(o)
    o.add_argument("--lab", help="lab name or file: use its DETECTOR / DETECTOR_OPTIONS / WORLD")
    o.set_defaults(world=None)
    o.add_argument("--image", nargs="+", help="image files, folders or globs")
    o.add_argument("--synthetic", type=int, default=0, metavar="N", help="N synthetic scenes per world")
    o.add_argument("--per-bent", type=int, default=5, help="products per bent in a synthetic scene")
    o.add_argument("--seed", type=int, default=0)
    o.add_argument("--out", help="folder for annotated images")
    o.add_argument("--show", action="store_true", help="show each image (any key = next)")
    o.set_defaults(func=cmd_offline)

    n = sub.add_parser("new", help="create a lab file from a template")
    n.add_argument("name", help="lab name, e.g. color_sorting -> labs/lab_color_sorting.py")
    n.add_argument("--template", "-t", default="transfer", help="see 'vision_lab templates' (default transfer)")
    n.add_argument("--world", help="world the lab is meant for (default: any)")
    n.add_argument("--detector", help="detector of the lab (default: the template's)")
    n.add_argument("--title", help="one-line title")
    n.add_argument("--dir", help="labs folder (default $VISION_LAB_DIR or ./labs)")
    n.add_argument("--force", action="store_true", help="overwrite an existing lab file")
    n.set_defaults(func=cmd_new)
    li = sub.add_parser("list", help="the lab files")
    li.add_argument("--dir", help="labs folder (default $VISION_LAB_DIR or ./labs)")
    li.set_defaults(func=cmd_list)
    sub.add_parser("templates", help="templates for new lab files").set_defaults(func=cmd_templates)
    sub.add_parser("worlds", help="the worlds and what the arm can do in each").set_defaults(func=cmd_worlds)
    sub.add_parser("detectors", help="the available detectors").set_defaults(func=cmd_detectors)
    return ap


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # ros2 run adds --ros-args ...; the lab's own options come before it
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    ap = build_parser()
    a = ap.parse_args(argv)
    if not getattr(a, "cmd", None):
        ap.print_help()
        return 0
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
