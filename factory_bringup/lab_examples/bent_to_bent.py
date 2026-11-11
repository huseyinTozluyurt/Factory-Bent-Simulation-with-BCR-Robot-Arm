"""Example: move N products from bent 1 to bent 2 with the tuned OpenCV detector.
   run_vision_lab.sh run --script bent_to_bent      (dual, straight-circular, circular-straight, straight-straight)"""
DETECTOR = "opencv_contours"


def main(lab):
    if not lab.world.has_bent2:
        print(f"{lab.world.key} has no bent 2: use sort_into_bins instead.")
        return
    for i in range(3):
        r = lab.move_to_bent2(timeout=180)
        print(f"{i + 1}: {r}")
    print(lab.report())
