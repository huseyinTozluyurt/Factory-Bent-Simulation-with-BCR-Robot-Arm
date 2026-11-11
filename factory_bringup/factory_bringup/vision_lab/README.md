# Vision lab

A computer-vision development environment on top of the factory simulation. You write detectors
(OpenCV or YOLO) and one `main(lab)` function; the lab runs your detector on the overhead camera,
feeds it to the project's pick-and-place controller, and lets you move products with the BCR arm
from bent 1 to bent 2 or into the trash / recycling bins, in every world.

## Quick start

```bash
cd ~/ros2_ws && colcon build --packages-select factory_bringup --symlink-install && source install/setup.bash
chmod +x ~/ros2_ws/src/factory_bent_project/run_vision_lab.sh
cd ~/ros2_ws/src/factory_bent_project

./run_vision_lab.sh sim --world dual          # terminal 1: the simulation (order window as in run_factory.sh)
./run_vision_lab.sh run                       # terminal 2: main(lab) of vision_lab_main.py
```

Edit `vision_lab_main.py` and run terminal 2 again; the simulation keeps running.
`./run_vision_lab.sh --world straight` does both in one terminal (world's own products, no order window).

No Gazebo at hand? `./run_vision_lab.sh offline --synthetic 20 --world all` scores the detectors on
drawn scenes of every world (precision, recall, label accuracy, position error).

## Lab files

Experiments live as separate files in `labs/` (`labs/lab_<name>.py`, each with its own detector and
`main(lab)`). `run_vision_lab.sh new NAME --template T` creates one (templates: detect, transfer, bins,
sort, custom_opencv, yolo, dataset), `run_vision_lab.sh list` lists them, `run_vision_lab.sh run NAME`
runs one. See `labs/README.md`.

## Worlds

| `--world` | bent 1 | bent 2 | the arm can deliver to |
|---|---|---|---|
| dual (= circular-circular) | circular | circular | bent2, trash, recycling |
| circular | circular | none | trash, recycling (others stay on the belt) |
| straight | straight | none | trash, recycling (others ride into the collection box) |
| straight-circular | straight | circular | bent2, trash, recycling |
| circular-straight | circular | straight | bent2, trash, recycling |
| straight-straight | straight | straight | bent2, trash, recycling |
| pipeline, pipeline:A1 ... pipeline:A12 | the station's pick conveyor | its place conveyor | bent2, trash, recycling (each station has its own bins) |

`run` asks the running simulation which world it is, so `--world` is only needed for `sim`, `offline`
and the one-terminal mode. All worlds share the overhead camera, catch station, place point and bins;
the lanes come from `product_catalog.WORLD_LAYOUTS`, the table every node already uses.
The pipeline world has its own section below.

## The API (`lab` in `main(lab)`)

| call | does |
|---|---|
| `lab.detections(bent=1, label=None)` | latest detections in world coordinates: `label score x y yaw size_x size_y bent` |
| `lab.wait_for_frame()`, `lab.wait_for_object(bent=1, label=...)` | wait for the camera / a detection |
| `lab.latest_image()` | the last overhead BGR image |
| `lab.on_frame(fn)` | `fn(bgr, detections)` for every frame (your own logging / windows) |
| `lab.move_to_bent2()` | catch the next product at the station, put it on bent 2 |
| `lab.put_in_trash()`, `lab.drop_in_recycle_bin()` | next product into that bin |
| `lab.move_to_bent(2)`, `lab.move_to_trash()`, `lab.move_to_recycle_bin()` | the same three moves, lab-file names |
| `lab.detect_objects(bent=1)` | wait for a fresh frame, then its detections |
| `lab.snapshot()`, `lab.start_recording(dir)` / `lab.stop_recording()` | annotated image into the lab's results folder / YOLO dataset |
| `lab.transfer(to)` | the same with `to` = `"bent2"`, `"trash"`, `"recycling"` |
| `lab.run(policy, count=N, timeout=s)` | keep catching; `policy(ctx)` returns `"bent2"`, `"trash"`, `"recycling"` or `"skip"` |
| `lab.run({"paint_can": "recycling", "*": "bent2"})` | the same as a table keyed by the detector's label |
| `lab.run("gui")` | the order GUI's destinations, exactly as `run_factory.sh` |
| `lab.report()`, `lab.results`, `lab.on_result(fn)` | where each product went and whether the label was right |
| `lab.spawn_products([...], bent=1, spacing=1.0)` | put products on a bent (as the order GUI) |
| `lab.stop_bent(n)`, `lab.start_bent(n)`, `lab.home()`, `lab.stop()` | belts and arm |
| `lab.set_motion("cycle")`, `lab.set_arm_param(name, value)` | arm movement / any controller parameter |

`ctx` (`PickContext`) holds `ctx.detection` / `ctx.label` (what *your* detector saw at the station),
and `ctx.model` / `ctx.product_type` (the simulation's truth, for scoring only).

Standalone, without the CLI:

```python
from factory_bringup.vision_lab import VisionLab
with VisionLab(detector="opencv_color", detector_options={"targets": ["paint_can"]}) as lab:
    lab.drop_in_recycle_bin()
```

## Detectors

| name | what |
|---|---|
| `opencv_contours` | the project's tuned grayscale + Canny contour detector (`vision_geometry.py`), colour-labelled |
| `opencv_color` | Lab colour segmentation, `targets=[product types or {"name": [b,g,r]}]` |
| `opencv_bgsub` | MOG2 background subtraction |
| `yolo` | Ultralytics YOLO (detect, OBB or segmentation weights); `pip install ultralytics` |

Your own: subclass `Detector`, return `ObjectDetection(u, v, box_px=..., label=..., score=...)` in
pixels. The pipeline does pixel to world, bent assignment, arm masking, topics, window and dataset.
Use it with `@register_detector("name")` in your script, or `--detector file.py:Class`.
See `lab_examples/custom_detector.py` and the `RedObjectDetector` in `vision_lab_main.py`.

## YOLO on the simulation

COCO weights do not know the factory products. Let the OpenCV detector label the simulation for you:

```bash
./run_vision_lab.sh run --script bent_to_bent --record ~/vision_lab_dataset --record-every 10
# look at ~/vision_lab_dataset/check/*.png, then
yolo detect train data=~/vision_lab_dataset/data.yaml model=yolov8n.pt imgsz=1280 epochs=50
./run_vision_lab.sh run --script yolo_sort --opt model=$HOME/runs/detect/train/weights/best.pt
```

`--record-format obb` writes oriented boxes for `yolo obb train`. Products are about 20 px wide in the
1280 x 720 overhead image, so keep `imgsz=1280`.

## Pipeline world (pipeline_factory.sdf)

`pipeline_factory.sdf` is a layout only (static arms, no cameras, nothing moves), so the lab uses a
generated copy, `my_factory_worlds/worlds/pipeline_factory_lab.sdf` (built by
`my_factory_worlds/tools/make_pipeline_lab_world.py`; the original is not changed). It adds:

- one camera per arm station, straight down over the arm (`/pipeline/A1/camera` ... `/pipeline/A12/camera`),
  image right = from the pick point towards the place point;
- a recycling bin (left of the arm) and a trash bin (right) per station;
- each BCR arm as 8 movable link models, so the lab can pose it joint by joint.

The lab moves belts, products and arms itself (`pipeline_sim.py`, kinematic, 20 Hz) over gz-transport,
so no ROS controllers are involved. It needs the Python bindings: `sudo apt install python3-gz-transport13`.
Products queue at the end of a straight belt and drop into the dock boxes at the output lines.
The rings run so that products meet the selective arms before the arm that takes "all": C1 and C2
clockwise, C3 and C4 counter-clockwise (`pipeline_layout.py` says counter-clockwise for all, which
would hand everything on C1 and C2 to their "all" arm first).

```bash
./run_vision_lab.sh sim --world pipeline              # terminal 1: Gazebo with pipeline_factory_lab.sdf
./run_vision_lab.sh run 02_bent_to_bent --station A2  # terminal 2: any lab, at arm A2 (C1 -> S2)
./run_vision_lab.sh run 08_pipeline_sorting           # all 12 stations sort by vision
./run_vision_lab.sh run 08_pipeline_sorting --fake    # the same without Gazebo (drawn camera images)
./run_vision_lab.sh offline --world pipeline --synthetic 5   # detector metrics on all 12 station cameras
./run_vision_lab.sh worlds                            # stations, rules, bins
```

In the pipeline, `lab` is a `PipelineLab` with the same calls as above, plus `station=`:

| call | does |
|---|---|
| `lab.station`, `lab.all_stations` | the default station (`--station`, default A1) / A1 ... A12 |
| `lab.detect_objects(bent=1, station="A2")` | detections of A2's camera; bent 1 = its pick conveyor, bent 2 = its place conveyor |
| `lab.move_to_bent(2, station="A2")`, `lab.move_to_trash(station=...)`, `lab.move_to_recycle_bin(station=...)` | the next product reaching that arm |
| `lab.run(decide, stations="all")` | every station in its own thread; `ctx.station` says which arm asks |
| `lab.layout_rule(ctx)` | the station's rule from `pipeline_layout.py` (all, recyclable, cardboard, not_large, large, small), judged on the detector's label and size |
| `lab.spawn_products([...], conveyor="S1")` | products on any conveyor (switches the automatic feeder off) |
| `lab.summary()` | products per conveyor, bin and dock |

Without `spawn_products`, the lab keeps a few random products coming to the conveyors the working
arms pick from (S1 for the whole factory); `--no-feed` turns that off, `--belt-speed` changes the speed.
`--dry-run` and `set_motion` have no effect there. Not tested in Gazebo yet: the rendering of the
station cameras and posing the arm links with `set_pose_vector` (tested with the `--fake` backend).

## How it fits the existing nodes

`run_vision_lab.sh sim` is `run_factory.sh ... run_detector:=false run_controller:=false`. The lab then
runs, in one process, `vision_lab_detector` (publishes `/vision/ring1_objects`, `/vision/ring2_objects`,
`/vision/detections`, `/vision/debug_image`) and the unchanged `vision_pick_place` controller, subclassed
so the lab decides each destination. The attacher, side camera, release camera, order GUI and
`run_factory.sh` work as before; without those two launch arguments nothing changes.
