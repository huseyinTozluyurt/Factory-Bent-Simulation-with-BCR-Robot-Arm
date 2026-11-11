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

## Pipeline world

Every lab also runs in the pipeline world (`pipeline_factory_lab.sdf`, the lab version of
`pipeline_factory.sdf`), at one arm station: bent 1 is the conveyor that arm picks from, bent 2 the
one it places on, and the station has its own trash and recycling bin.

```bash
./run_vision_lab.sh sim --world pipeline                  # terminal 1
./run_vision_lab.sh run 03_trash_and_recycle --station A6 # terminal 2: lab 03 at arm A6
./run_vision_lab.sh run 08_pipeline_sorting               # the whole factory
./run_vision_lab.sh run 09_pipeline_demo                  # the demo (add --fake --belt-speed 0.8 without Gazebo)
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
