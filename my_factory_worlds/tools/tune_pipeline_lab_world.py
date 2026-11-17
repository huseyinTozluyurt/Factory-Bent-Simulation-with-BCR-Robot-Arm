#!/usr/bin/env python3
"""
Tune pipeline_factory_lab.sdf for speed, in place (run_vision_lab.sh does this before it starts Gazebo):

  - station cameras: size and rate from PIPELINE_CAMERA (full 1280x720@5, balanced 960x540@4 = default,
    lite 640x360@3, or WIDTHxHEIGHT@RATE). The lab reads the same setting, so both always agree.
  - physics step: PIPELINE_PHYSICS_STEP seconds (default 0.004 instead of Gazebo's 0.001). The lab moves
    products and arm links itself (set_pose_vector), so physics has little to do; a 4x larger step costs
    4x less CPU. 0.001 restores the old behaviour.

Nothing changes when the file already has these values.
Usage:  python3 tune_pipeline_lab_world.py [world.sdf]
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "factory_bringup"))

from factory_bringup.vision_lab import pipeline_world as P  # noqa: E402


def tune(path: str) -> str:
    with open(path) as f:
        text = f.read()
    old = text
    (w, h), rate = P.CAM_SIZE, P.CAM_RATE
    step = float(os.environ.get("PIPELINE_PHYSICS_STEP", "0.004"))

    def camera(m):
        block = m.group(0)
        block = re.sub(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{rate}</update_rate>", block, count=1)
        block = re.sub(r"<width>\d+</width>", f"<width>{w}</width>", block, count=1)
        block = re.sub(r"<height>\d+</height>", f"<height>{h}</height>", block, count=1)
        return block
    text = re.sub(r'<sensor name="rgb_camera" type="camera">.*?</sensor>', camera, text, flags=re.S)

    physics = (f'<physics name="lab_physics" type="ignored"><max_step_size>{step:g}</max_step_size>'
               f'<real_time_factor>1.0</real_time_factor><real_time_update_rate>{1.0 / step:g}</real_time_update_rate>'
               f'</physics>')
    if re.search(r'<physics name="lab_physics".*?</physics>', text, flags=re.S):
        text = re.sub(r'<physics name="lab_physics".*?</physics>', physics, text, count=1, flags=re.S)
    elif "<physics" not in text:
        text = re.sub(r"(<world name=\"[^\"]*\">)", r"\1\n    " + physics, text, count=1)

    if text != old:
        with open(path, "w") as f:
            f.write(text)
        return f"{os.path.basename(path)}: cameras {w}x{h} at {rate} Hz, physics step {step:g} s (updated)"
    return f"{os.path.basename(path)}: cameras {w}x{h} at {rate} Hz, physics step {step:g} s"


if __name__ == "__main__":
    print(tune(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "worlds", P.SDF)))
