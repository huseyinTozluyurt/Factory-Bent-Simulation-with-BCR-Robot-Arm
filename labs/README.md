# Factory automation vision labs

Each `lab_<name>.py` here is one experiment: its own detector (OpenCV or YOLO) and its own
`main(lab)` that calls the robot arm. Run any of them by name against the running simulation.

```bash
./run_vision_lab.sh sim --world dual                 # terminal 1, once (any world)
./run_vision_lab.sh list                             # the labs in this folder
./run_vision_lab.sh run 02_bent_to_bent              # terminal 2: run one lab (a unique prefix like 02 works)
./run_vision_lab.sh new my_lab --template sort       # new lab file from a template
./run_vision_lab.sh templates                        # detect, transfer, bins, sort, custom_opencv, yolo, dataset, pipeline
./run_vision_lab.sh offline --lab 05 --synthetic 10 --world all   # try a lab's detector without Gazebo
```

| lab | experiment |
|---|---|
| 01_detection_survey | detection only: counts per bent and label, annotated snapshots |
| 02_bent_to_bent | bent 1 -> bent 2 (recycling bin in worlds without bent 2) |
| 03_trash_and_recycle | products into the trash can and the recycling bin |
| 04_sort_by_label | a `decide(ctx)` rule on the detector's label: bent 2 / trash / recycling |
| 05_my_opencv_detector | your own OpenCV detector class inside the lab file |
| 06_yolo_sorting | YOLO labels decide the destination (needs `pip install ultralytics` + trained weights) |
| 07_record_dataset | records an auto-labelled YOLO dataset while the arm keeps products moving |
| 08_pipeline_sorting | pipeline world: all 12 arm stations sort by vision, each with its own rule and bins |
| 09_pipeline_demo | pipeline demo in three acts: detection at A1, arm A1 to bent 2 / trash / recycling, then the whole factory; saves snapshots, a video of station A1 and report.txt |
| 10_pipeline_tests | pipeline test cases: products through chains of arms into bins, onto further bents and into the docks; PASS / FAIL per product |
| 11_all_boxes | whole pipeline test: all 13 bents and all 12 arms run until every one of the 29 boxes (24 station bins + 5 docks) has a product; PASS / FAIL |

## Pipeline world

Every lab also runs in the pipeline world (`pipeline_factory_lab.sdf`, the lab version of
`pipeline_factory.sdf`), at one arm station: bent 1 is the conveyor that arm picks from, bent 2 the
one it places on, and the station has its own trash and recycling bin.

```bash
./run_vision_lab.sh sim --world pipeline                  # terminal 1
./run_vision_lab.sh run 03_trash_and_recycle --station A6 # terminal 2: lab 03 at arm A6
./run_vision_lab.sh run 08_pipeline_sorting               # the whole factory
./run_vision_lab.sh run 09_pipeline_demo                  # the demo (add --fake --belt-speed 0.8 without Gazebo)
./run_vision_lab.sh run 10_pipeline_tests                 # the test cases (PIPELINE_TESTS=name,... for some)
./run_vision_lab.sh run 11_all_boxes                      # every bent, every arm, every box at least once
./run_vision_lab.sh run 02 --fake --station A2            # without Gazebo (drawn camera images)
./run_vision_lab.sh new my_line --template pipeline       # new whole-factory lab
```

## What a lab file contains

```python
LAB_TITLE = "..."                    # shown by `list`
WORLD = None                         # None = any world; or dual, circular, straight, ..., pipeline
DETECTOR = "opencv_contours"         # opencv_contours, opencv_color, opencv_bgsub, yolo, a Detector class
DETECTOR_OPTIONS = {}
ARM_PARAMS = {}                      # vision_pick_place parameters, e.g. {"motion_type": "cycle"}

def main(lab):
    lab.detect_objects(bent=1)       # detections: label, score, x, y, yaw, size_x, size_y, bent
    lab.move_to_bent(2)              # next product at the catch station -> bent 2
    lab.move_to_trash()              # -> trash can
    lab.move_to_recycle_bin()        # -> recycling bin
    lab.run(decide, count=5)         # decide(ctx) -> "bent2" | "trash" | "recycling" | "skip"
    lab.snapshot()                   # annotated image into results/<lab>/<time>/
    print(lab.report())              # destinations + detector label accuracy vs the simulation's truth
```

Results (snapshots) go to `labs/results/<lab name>/<date_time>/`. Command-line `--world`, `--detector`,
`--opt` and `--arm` override the values in the file. Full API: `factory_bringup/factory_bringup/vision_lab/README.md`.

## Pipeline test cases (lab 10)

`lab_10_pipeline_tests.py` holds 8 cases. Each puts known products on S1 and lets some arm stations
work with a routing table (by the detector's label): products go into a station's trash or recycling
bin, or onto the next bent, where the next arm picks them up again, until they reach a dock. The
expected route of every product is worked out from the same table applied to its true type, and the
product PASSES when it really took that route and ended there.

| case | what it checks |
|---|---|
| one_hop_A1 | A1: S1 -> C1 |
| bins_at_A1 | A1 sorts into its trash bin, its recycling bin and C1 |
| two_hops_to_S3 | A1 then A3: S1 -> C1 -> S3 |
| chain_to_large_dock | 4 arms: S1 -> C1 -> S3 -> C3 -> S7 -> dock 3 |
| branch_on_C1 | A2 takes some products to S2 -> C2 -> dock 2, A3 the rest to S3 -> C3 -> dock 3 |
| bins_along_the_chain | bins in the middle of the chain (A3 recycling, A5 trash), the rest to dock 3 |
| longest_route_to_S9 | 6 arms: S1 -> C1 -> S3 -> C3 -> S6 -> C4 -> S9 -> dock 5 |
| whole_factory_rules | stress test: all 12 stations with their pipeline_layout.py rules; products reach all 5 docks (needs every camera to keep up; with `--fake` on a small PC 1-2 products can go to the wrong dock) |

```bash
./run_vision_lab.sh run 10_pipeline_tests --fake --belt-speed 1.0          # all cases, no Gazebo
PIPELINE_TESTS=branch_on_C1 ./run_vision_lab.sh run 10_pipeline_tests     # one case
PIPELINE_DECIDE_BY=truth ./run_vision_lab.sh run 10_pipeline_tests        # arms and belts only, no vision
```

A failing product says why: the detector's label was wrong, a grasp missed, it fell, or it did not
arrive in time. Add a case by copying a `PipelineCase(...)` in the file (framework:
`factory_bringup/vision_lab/pipeline_testing.py`).

## Whole pipeline test: every box at least once (lab 11)

`lab_11_all_boxes.py` runs everything at once: all 13 bents (S1 ... S9, C1 ... C4) and all 12 arms
(A1 ... A12). The feeder keeps putting products on S1, and the test PASSES when each of the 29 boxes
has received at least one product: the trash bin and the recycling bin of every station (24) and the
5 dock boxes. That also means every arm delivered and every bent carried products.

Every station decides with `AllBoxesPolicy`: a product whose detector label suits one of the
station's empty bins (recyclable -> recycling, a trash label -> trash) goes into it; otherwise it
goes on to bent 2 while the boxes further down still need products (empty boxes minus products
already on their way); otherwise into an empty bin of the station; otherwise the arm leaves it. So
the result does not depend on the random product mix, and a missed or dropped product is replaced.

```bash
./run_vision_lab.sh sim --world pipeline                                   # terminal 1
./run_vision_lab.sh run 11_all_boxes                                       # terminal 2
./run_vision_lab.sh --world pipeline 11_all_boxes                          # both in one terminal
./run_vision_lab.sh run 11_all_boxes --fake --belt-speed 0.8               # no Gazebo
PIPELINE_DECIDE_BY=truth ./run_vision_lab.sh run 11_all_boxes --fake       # bins by the true type
ALL_BOXES_TIMEOUT=2400 ./run_vision_lab.sh run 11_all_boxes                # longer time limit (default 1800 s)
```

The report (`labs/results/11_all_boxes/<time>/test_report.txt`) lists, per station, the products in
its trash and recycling bins, how many it put on bent 2 and its missed grasps, then the 5 docks and
how many products rode each bent. Snapshots of all 12 station cameras are saved next to it. The lab
exits with status 1 when the test fails, so it can run in a script.
