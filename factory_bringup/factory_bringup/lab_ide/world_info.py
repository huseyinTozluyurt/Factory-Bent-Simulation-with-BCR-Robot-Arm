"""
The worlds the vision lab knows, and what is in each of them (no Qt, no ROS imports).

    from factory_bringup.lab_ide.world_info import WORLDS, describe
    info = describe("pipeline-tree")     # dict: kind, gazebo world, sdf, bents, arms, bins, docks, flow
    python3 -m factory_bringup.lab_ide.world_info --world pipeline-tree        (JSON)

The single-arm worlds come from product_catalog.WORLD_FILES / WORLD_LAYOUTS, the pipeline worlds from
pipeline_layout.py (PIPELINE_LAYOUT=pipeline) and pipeline_tree_layout.py (PIPELINE_LAYOUT=tree), the
same tables the simulation and the lab read. pipeline_layout picks its tables once, at import, so
describe() reads every world in a separate Python process (cached).
"""

import argparse
import json
import os
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass(frozen=True)
class WorldEntry:
    key: str          # run_vision_lab.sh --world KEY
    title: str
    sdf: str          # file in my_factory_worlds/worlds
    kind: str         # "single" (one arm, bent 1 -> bent 2 / bins) or "pipeline" (12 arm stations)


WORLDS: List[WorldEntry] = [
    WorldEntry("pipeline", "Pipeline factory: S1 -> 4 sorting rings, 12 arms, 5 docks",
               "pipeline_factory_lab.sdf", "pipeline"),
    WorldEntry("pipeline-tree", "Tree-of-three pipeline: S1 feeds 3 arms -> 3 rings -> 9 docks",
               "pipeline_tree_factory_lab.sdf", "pipeline"),
    WorldEntry("dual", "Dual circular factory: ring bent 1 -> ring bent 2",
               "dual_circular_factory_conveyor.sdf", "single"),
    WorldEntry("circular", "Single circular factory bent (no bent 2)", "circular_factory_conveyor.sdf", "single"),
    WorldEntry("straight", "Straight conveyor factory bent (no bent 2)", "factory_conveyor.sdf", "single"),
    WorldEntry("straight-circular", "Straight bent 1 -> circular bent 2", "exp_straight_circular.sdf", "single"),
    WorldEntry("circular-straight", "Circular bent 1 -> straight bent 2", "exp_circular_straight.sdf", "single"),
    WorldEntry("straight-straight", "Straight bent 1 -> straight bent 2", "exp_straight_straight.sdf", "single"),
]
BY_KEY: Dict[str, WorldEntry] = {w.key: w for w in WORLDS}


def entry(key: str) -> WorldEntry:
    if key not in BY_KEY:
        raise ValueError(f"unknown world '{key}'; worlds: {', '.join(BY_KEY)}")
    return BY_KEY[key]


def package_root() -> str:
    """The folder that holds the factory_bringup package (src/.../factory_bringup or site-packages)."""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ------------------------------------------------------------------ in-process readers (one world per process)
def _single(key: str) -> Dict:
    from factory_bringup import product_catalog as C
    sdf, gz_name = C.WORLD_FILES[key]
    lay = dict(C.WORLD_LAYOUTS.get(gz_name, {}))
    has2 = bool(lay.get("has_bent2", True))
    shape = lambda r: "straight" if r >= C.LINE_RADIUS / 2 else "circular"  # noqa: E731
    bent1 = shape(lay.get("ring1_radius", 3.42))
    bent2 = shape(lay.get("ring2_product_radius", 3.42)) if has2 else None
    bins = {k: list(v) for k, v in C.BINS.items()}
    return {
        "key": key, "kind": "single", "title": entry(key).title, "sdf": sdf, "gazebo_world": gz_name,
        "bents": [{"name": "bent 1", "shape": bent1, "role": "pick (the arm catches products here)"}]
        + ([{"name": "bent 2", "shape": bent2, "role": "place (lab.move_to_bent(2))"}] if has2 else []),
        "has_bent2": has2,
        "arms": [{"name": "arm", "pick": "bent 1", "place": "bent 2" if has2 else None,
                  "catch_station": [C.STATION_X, 0.0], "bins": bins}],
        "bins": bins,
        "docks": [],
        "destinations": (["bent2"] if has2 else []) + list(bins),
        "flow": ("bent 1 -> arm -> " + ("bent 2 / " if has2 else "") + "recycling bin / trash bin"),
    }


def _pipeline(key: str) -> Dict:
    from factory_bringup import pipeline_layout as L
    bins: Dict[str, Dict[str, List[float]]] = {}
    try:                                   # needs the vision_lab package (OpenCV, numpy)
        from factory_bringup.vision_lab import pipeline_world as P
        bins = {a: {k: list(v) for k, v in P.bins(a).items()} for a in L.ARMS}
    except Exception:                      # noqa: BLE001 - names only, without positions
        bins = {a: {"recycling": None, "trash": None} for a in L.ARMS}
    conveyors = []
    for c, (cx, cy, sign) in L.CIRCLES.items():
        conveyors.append({"name": c, "shape": "ring", "sign": sign, "centre": [cx, cy], "dock": None})
    for c, (start, end, sign, dock) in L.STRAIGHTS.items():
        conveyors.append({"name": c, "shape": "straight", "sign": sign, "start": list(start), "end": list(end),
                          "dock": dock})
    arms = []
    for a, (base, (pick_conv, pick), (place_conv, place), rule) in L.ARMS.items():
        arms.append({"name": a, "base": list(base), "pick_conv": pick_conv, "pick": list(pick),
                     "place_conv": place_conv, "place": list(place), "rule": rule,
                     "bins": {f"{k}_bin_{a}": v for k, v in bins[a].items()}})
    docks = [{"name": f"dock_{c}", "conveyor": c, "title": dock.title()}
             for c, (_s, _e, _l, dock) in L.STRAIGHTS.items() if dock]
    doc = L.__doc__ or ""
    flow = doc[doc.index("Flow:"):] if "Flow:" in doc else doc.strip()
    flow = textwrap.dedent("     " + flow[len("Flow:"):].split("\n\n")[0]).strip("\n")
    return {
        "key": key, "kind": "pipeline", "layout": L.LAYOUT, "title": entry(key).title, "sdf": L.LAB_SDF,
        "gazebo_world": L.WORLD_NAME, "belt_speed": L.BELT_SPEED,
        "bents": conveyors, "arms": arms, "docks": docks, "has_bent2": True,
        "destinations": ["bent2", "trash", "recycling", "skip"], "flow": flow,
        "inbound": "S1",
    }


def _describe_here(key: str) -> Dict:
    return _pipeline(key) if entry(key).kind == "pipeline" else _single(key)


# ------------------------------------------------------------------ public
_CACHE: Dict[str, Dict] = {}


def describe(key: str, python: Optional[str] = None, timeout: float = 30.0) -> Dict:
    """What is in world `key` (see the module doc), read in a separate Python process."""
    if key in _CACHE:
        return _CACHE[key]
    e = entry(key)
    env = dict(os.environ)
    env["PYTHONPATH"] = package_root() + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    if e.kind == "pipeline":
        env["PIPELINE_LAYOUT"] = "tree" if "tree" in key else "pipeline"
    out = subprocess.run([python or sys.executable, "-m", "factory_bringup.lab_ide.world_info", "--world", key],
                         capture_output=True, text=True, env=env, timeout=timeout)
    if out.returncode != 0:
        raise RuntimeError(f"cannot read world {key}:\n{out.stderr.strip()[-2000:]}")
    _CACHE[key] = json.loads(out.stdout)
    return _CACHE[key]


def summary_text(info: Dict) -> str:
    """A few lines about the world, for the IDE's world panel."""
    lines = [f"{info['title']}", f"Gazebo world {info['gazebo_world']}  ({info['sdf']})", ""]
    if info["kind"] == "pipeline":
        rings = [b["name"] for b in info["bents"] if b["shape"] == "ring"]
        lines.append(f"{len(info['bents'])} bents: rings {', '.join(rings)}; straights "
                     + ", ".join(b["name"] for b in info["bents"] if b["shape"] == "straight"))
        lines.append(f"{len(info['arms'])} arms, {2 * len(info['arms'])} bins, {len(info['docks'])} docks; "
                     f"products enter on {info['inbound']}")
        lines += ["", "Flow:", info["flow"]]
    else:
        lines.append(", ".join(f"{b['name']} {b['shape']}" for b in info["bents"]))
        lines.append("destinations: " + ", ".join(info["destinations"]))
        lines.append("flow: " + info["flow"])
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", help="world key (default: list the worlds)")
    a = ap.parse_args(argv)
    if not a.world:
        for w in WORLDS:
            print(f"{w.key:18s} {w.kind:8s} {w.title}")
        return
    if entry(a.world).kind == "pipeline":       # before pipeline_layout is imported
        os.environ["PIPELINE_LAYOUT"] = "tree" if "tree" in a.world else "pipeline"
    print(json.dumps(_describe_here(a.world), indent=1))


if __name__ == "__main__":
    main()
