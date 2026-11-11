#!/usr/bin/env python3
"""
Builds pipeline_factory_lab.sdf, the vision-lab version of the pipeline world, from
pipeline_factory.sdf (left as it is). What it changes:

  - each of the 12 BCR arms (one static model per arm) becomes 8 movable link models
    arm_<A>__<link> (no gravity, no collision), so the lab can pose the arm joint by joint;
  - a camera over every arm station (topic /pipeline/<A>/camera), looking straight down;
  - a recycling bin and a trash bin at every station (copies of the dual world's bins);
  - the Sensors system, so the cameras render.

Positions come from factory_bringup/vision_lab/pipeline_world.py (the lab uses the same module).

    cd my_factory_worlds/tools && python3 make_pipeline_lab_world.py
"""

import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "factory_bringup"))

from factory_bringup import pipeline_layout as L                       # noqa: E402
from factory_bringup.vision_lab import pipeline_world as P             # noqa: E402
from factory_bringup.vision_lab.arm_model import ArmModel, euler      # noqa: E402

WORLDS = os.path.join(HERE, "..", "worlds")
SRC = os.path.join(WORLDS, "pipeline_factory.sdf")
DUAL = os.path.join(WORLDS, "dual_circular_factory_conveyor.sdf")
OUT = os.path.join(WORLDS, P.SDF)
ARM = ArmModel()


def wait_posture(arm: str):
    """Joint values with the gripper tip 2.0 m high over the arm's pick point (as in the layout world)."""
    st = P.station(arm)
    bx, by = st["base"]
    t = (st["pick"][0] - bx, st["pick"][1] - by, 2.0 - L.ARM_BASE_Z)
    q, err = ARM.solve(t, ARM.seeds_towards(t))
    assert err < 0.01, (arm, err)
    return q


def mat(c):
    r, g, b = c
    return (f"<material><ambient>{r * 0.8:.3f} {g * 0.8:.3f} {b * 0.8:.3f} 1</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse><specular>0.15 0.15 0.15 1</specular></material>")


def pose_str(T):
    r, p, y = euler(T[:3, :3])
    return f"{T[0, 3]:.4f} {T[1, 3]:.4f} {T[2, 3]:.4f} {r:.4f} {p:.4f} {y:.4f}"


def arm_models(arm: str) -> str:
    bx, by = P.station(arm)["base"]
    base = np.eye(4)
    base[:3, 3] = (bx, by, L.ARM_BASE_Z)
    frames = ARM.link_frames(wait_posture(arm))
    out = []
    for link in ARM.links:
        vis = "".join(
            f'<visual name="{v["name"]}"><pose>{pose_str(v["T"])}</pose><geometry><mesh><uri>{v["uri"]}</uri>'
            f'<scale>{v["scale"]}</scale></mesh></geometry>{mat(v["color"])}</visual>'
            for v in ARM.visuals(link))
        out.append(f"""    <model name="arm_{arm}__{link}">
      <pose>{pose_str(base @ frames[link])}</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        {vis}
      </link>
    </model>
""")
    return "".join(out)


def camera_model(arm: str) -> str:
    c = P.camera_pose(arm)
    w, h = P.CAM_SIZE
    return f"""    <model name="station_camera_{arm}">
      <static>true</static>
      <pose>{c['x']:.4f} {c['y']:.4f} {c['z']:.3f} {c['roll']} {c['pitch']} {c['yaw']:.6f}</pose>
      <link name="camera_link">
        <sensor name="rgb_camera" type="camera">
          <always_on>true</always_on>
          <update_rate>{P.CAM_RATE}</update_rate>
          <visualize>false</visualize>
          <topic>{P.camera_topic(arm)}</topic>
          <camera>
            <horizontal_fov>{c['hfov']}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>30.0</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def bin_models(dual: str) -> str:
    tmpl = {}
    for kind in ("recycling", "trash"):
        m = re.search(rf'\n    <model name="{kind}_bin">.*?\n    </model>', dual, re.S)
        if not m:
            raise SystemExit(f"{kind}_bin not found in {DUAL}")
        tmpl[kind] = m.group(0)
    out = []
    for arm in P.STATIONS:
        ux, uy = P.station_axis(arm)
        yaw = math.atan2(uy, ux)
        for kind, (x, y) in P.bins(arm).items():
            body = tmpl[kind].replace(f'<model name="{kind}_bin">', f'<model name="{P.bin_name(arm, kind)}">', 1)
            body = re.sub(r"<pose>[^<]*</pose>", f"<pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>", body, count=1)
            out.append(body)
    return "\n".join(out) + "\n"


def main():
    src = open(SRC).read()
    dual = open(DUAL).read()
    world = re.sub(r'\n    <model name="bcr_arm_A\d+">.*?\n    </model>', "", src, flags=re.S)
    removed = len(re.findall(r'<model name="bcr_arm_A\d+">', src))
    if "gz-sim-sensors-system" not in world:
        world = world.replace(
            '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
            '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>\n'
            '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
            '      <render_engine>ogre</render_engine>\n    </plugin>', 1)
    world = world.replace("Layout only: every conveyor and arm is static. Built by make_pipeline_world.py.",
                          "Vision-lab version (make_pipeline_lab_world.py): movable arm links, a camera and two bins\n"
                          "      per station. Belts, products and arms are moved by the lab (pipeline_sim.py).", 1)
    added = "\n    <!-- ===== vision lab: arms (one model per link), station cameras, bins ===== -->\n"
    for arm in P.STATIONS:
        added += arm_models(arm) + camera_model(arm)
    added += bin_models(dual)
    world = world.replace("\n  </world>", added + "  </world>", 1)
    with open(OUT, "w") as f:
        f.write(world)
    print(f"{OUT}: {removed} static arms replaced by {len(ARM.links)} link models each, "
          f"{len(P.STATIONS)} cameras, {2 * len(P.STATIONS)} bins")


if __name__ == "__main__":
    main()
