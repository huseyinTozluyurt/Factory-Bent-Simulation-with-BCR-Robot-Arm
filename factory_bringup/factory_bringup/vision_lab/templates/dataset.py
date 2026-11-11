# TEMPLATE: record an auto-labelled YOLO training dataset while the arm keeps products moving
"""
LAB {{NAME}}: {{TITLE}}
Created {{DATE}} from the '{{TEMPLATE}}' template.

Dataset experiment: the OpenCV detector labels the overhead images, every RECORD_EVERY-th frame is
saved in YOLO format to DATASET_DIR, and the arm moves products to bent 2 / the bins so the scene
keeps changing. Check DATASET_DIR/check/*.png before training:
    yolo detect train data=DATASET_DIR/data.yaml model=yolov8n.pt imgsz=1280 epochs=50

Run:  ./run_vision_lab.sh run {{NAME}}
"""
LAB_TITLE = "{{TITLE}}"
WORLD = None
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

DATASET_DIR = "~/vision_lab_dataset/{{NAME}}"
RECORD_EVERY = 10
PICKS = 6


def main(lab):
    rec = lab.start_recording(DATASET_DIR, every_n=RECORD_EVERY)
    cycle = (["bent2"] if lab.world.has_bent2 else []) + ["trash", "recycling"]
    for i in range(PICKS):
        lab.transfer(cycle[i % len(cycle)])
    print(lab.stop_recording())
    print(f"dataset in {rec.dir}")
