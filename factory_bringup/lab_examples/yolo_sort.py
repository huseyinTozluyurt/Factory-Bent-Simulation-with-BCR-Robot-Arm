"""Example: YOLO picks the labels. Train the model on the simulation first (README: "YOLO"):
   run_vision_lab.sh run --script yolo_sort --opt model=~/runs/detect/train/weights/best.pt"""
DETECTOR = "yolo"
DETECTOR_OPTIONS = {"model": "yolov8n.pt", "conf": 0.35, "imgsz": 1280}


def main(lab):
    lab.wait_for_frame()
    print("YOLO sees:", [(d.label, round(d.score, 2), d.bent) for d in lab.detections()])
    table = {"paint_can": "recycling", "glass_jar": "recycling", "sneaker_pair": "trash",
             "*": "bent2" if lab.world.has_bent2 else "trash"}
    lab.run(table, count=5, timeout=900)
    print(lab.report())
