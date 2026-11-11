"""
LAB 06_yolo_sorting: YOLO labels decide where products go
Created 2026-10-31 from the 'yolo' template.

YOLO experiment. Needs  pip install ultralytics  and weights that know the products: record a
dataset with the 'dataset' template, train  yolo detect train data=.../data.yaml model=yolov8n.pt
imgsz=1280 , then set MODEL to runs/detect/train/weights/best.pt.

Run:  ./run_vision_lab.sh run 06_yolo_sorting      (or --opt model=path/to/best.pt)
"""
LAB_TITLE = 'YOLO labels decide where products go'
WORLD = None
MODEL = "~/runs/detect/train/weights/best.pt"
DETECTOR = "yolo"
DETECTOR_OPTIONS = {"model": MODEL, "conf": 0.35, "imgsz": 1280}
ARM_PARAMS = {}

RULES = {"paint_can": "recycling", "glass_jar": "recycling", "sneaker_pair": "trash", "*": "bent2"}
COUNT = 5


def main(lab):
    lab.wait_for_frame()
    print("YOLO sees:", [(d.label, round(d.score, 2), d.bent) for d in lab.detect_objects()])
    rules = dict(RULES)
    if not lab.world.has_bent2:
        rules["*"] = "skip"
    lab.run(rules, count=COUNT, timeout=900)
    print(lab.report())
