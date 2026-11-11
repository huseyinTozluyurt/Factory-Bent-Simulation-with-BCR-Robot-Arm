"""
Geometry of the pipeline world for the vision lab (no ROS, no Gazebo imports).

pipeline_layout.py defines the conveyors and the 12 arm stations. This module adds what the lab
needs on top, the same way for the world generator (make_pipeline_lab_world.py) and the running
lab (pipeline_sim.py / pipeline_lab.py):
  - the product path of every conveyor (path_xy / path_s / length),
  - one station camera per arm, looking straight down over the arm,
  - a recycling bin and a trash bin per arm, placed clear of every conveyor, dock and arm,
  - a WorldProfile per station: bent 1 = the arm's pick conveyor, bent 2 = its place conveyor.
"""

import math
from functools import lru_cache
from typing import Dict, List, Optional, Tuple

from factory_bringup import pipeline_layout as L
from factory_bringup.product_catalog import BIN_SIZE
from factory_bringup.vision_lab.worlds import Lane, WorldProfile

WORLD_NAME = L.WORLD_NAME
SDF = "pipeline_factory_lab.sdf"
STATIONS: List[str] = list(L.ARMS)
R = L.RING_PATH_RADIUS                  # products ride the rings at r = 3.42
RING_LANE = (3.10, 3.75)                # camera search lane of a ring (as in the other worlds)
LINE_HALF = 0.325                       # camera search half width of a straight belt
CAM_Z, CAM_HFOV, CAM_SIZE = 7.6, 1.40, (1280, 720)
CAM_RATE = 5
BIN_OFFSETS = (2.2, 1.95, 2.45)         # tried distances of a bin from the arm base, across the station axis
BIN_SHIFTS = (0.0, 0.4, -0.4, 0.8, -0.8)


def _ring_dir(conv: str) -> int:
    """+1 = counter-clockwise, -1 = clockwise: the way that brings a product from the arm placing onto
    the ring to the selective arms (rule != "all") before the arm that takes all the others.
    (pipeline_layout says counter-clockwise for all; that would send everything on C1 and C2 to their
    "all" arm first, so the lab runs C1 and C2 clockwise.)"""
    cx, cy, _ = L.CIRCLES[conv]
    ang = lambda p: math.atan2(p[1] - cy, p[0] - cx) % (2 * math.pi)
    feed = [ang(pl) for _b, _pk, (c, pl), _r in L.ARMS.values() if c == conv]
    picks = [(ang(pk), r) for _b, (c, pk), _pl, r in L.ARMS.values() if c == conv]
    if not feed or len(picks) < 2:
        return 1

    def order_ok(sign):
        travel = sorted(picks, key=lambda p: ((p[0] - feed[0]) * sign) % (2 * math.pi))
        return travel[-1][1] == "all" and travel[0][1] != "all"
    return 1 if order_ok(1) or not order_ok(-1) else -1


def ring_dir(conv: str) -> int:
    return RING_DIRS.get(conv, 1)


def is_ring(conv: str) -> bool:
    return conv in L.CIRCLES


def length(conv: str) -> float:
    if is_ring(conv):
        return 2 * math.pi * R
    (x0, y0), (x1, y1) = L.STRAIGHTS[conv][:2]
    return math.hypot(x1 - x0, y1 - y0)


def path_xy(conv: str, s: float) -> Tuple[float, float, float]:
    """(x, y, heading) at path coordinate s: rings counter-clockwise from angle 0, straights from start."""
    if is_ring(conv):
        cx, cy, _ = L.CIRCLES[conv]
        a = s / R
        return cx + R * math.cos(a), cy + R * math.sin(a), a + ring_dir(conv) * math.pi / 2
    (x0, y0), (x1, y1) = L.STRAIGHTS[conv][:2]
    n = length(conv)
    ux, uy = (x1 - x0) / n, (y1 - y0) / n
    return x0 + ux * s, y0 + uy * s, math.atan2(uy, ux)


def path_s(conv: str, x: float, y: float) -> float:
    if is_ring(conv):
        cx, cy, _ = L.CIRCLES[conv]
        return (math.atan2(y - cy, x - cx) % (2 * math.pi)) * R
    (x0, y0), (x1, y1) = L.STRAIGHTS[conv][:2]
    n = length(conv)
    return (x - x0) * (x1 - x0) / n + (y - y0) * (y1 - y0) / n


def dock_point(conv: str) -> Optional[Tuple[float, float]]:
    """Centre of the dock box past the end of an output line (None if the line has no dock)."""
    start, end, _label, dock = L.STRAIGHTS[conv]
    if not dock:
        return None
    n = length(conv)
    ux, uy = (end[0] - start[0]) / n, (end[1] - start[1]) / n
    return end[0] + ux * 1.7, end[1] + uy * 1.7


def lane(conv: str, anchor: Tuple[float, float], name: str) -> Lane:
    if is_ring(conv):
        cx, cy, _ = L.CIRCLES[conv]
        return Lane(name, cx, cy, RING_LANE[0], RING_LANE[1], anchor=anchor)
    (x0, y0), (x1, y1) = L.STRAIGHTS[conv][:2]
    # also a (huge) ring through the belt, for detectors that only know rings
    n = length(conv)
    nx, ny = -(y1 - y0) / n, (x1 - x0) / n
    big = 10000.0
    return Lane(name, x0 + nx * big, y0 + ny * big, big - LINE_HALF, big + LINE_HALF,
                seg=(x0, y0, x1, y1), half_width=LINE_HALF, anchor=anchor)


def station(arm: str) -> Dict:
    base, (pick_conv, pick), (place_conv, place), rule = L.ARMS[arm]
    return {"name": arm, "base": base, "pick_conv": pick_conv, "pick": pick, "place_conv": place_conv,
            "place": place, "rule": rule}


def station_axis(arm: str) -> Tuple[float, float]:
    st = station(arm)
    dx, dy = st["place"][0] - st["pick"][0], st["place"][1] - st["pick"][1]
    n = math.hypot(dx, dy)
    return dx / n, dy / n


def camera_pose(arm: str) -> Dict:
    """Straight down over the arm base; image right = from the pick point towards the place point."""
    bx, by = station(arm)["base"]
    ux, uy = station_axis(arm)
    return dict(x=bx, y=by, z=CAM_Z, roll=0.0, pitch=1.5708, yaw=math.atan2(ux, -uy), hfov=CAM_HFOV)


def camera_topic(arm: str) -> str:
    return f"/pipeline/{arm}/camera"


def _clearance(x: float, y: float, own: str) -> float:
    """How far the bin centre (x, y) is from the nearest thing it must not overlap (minus that thing's size)."""
    gaps = []
    for conv, (start, end, _l, _d) in L.STRAIGHTS.items():
        (x0, y0), (x1, y1) = start, end
        n = length(conv)
        t = max(0.0, min(n, ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / n))
        px, py = x0 + (x1 - x0) * t / n, y0 + (y1 - y0) * t / n
        gaps.append(math.hypot(x - px, y - py) - 0.65)              # belt + frames half width
        d = dock_point(conv)
        if d:
            gaps.append(math.hypot(x - d[0], y - d[1]) - 1.95)
    for cx, cy, _ in L.CIRCLES.values():
        r = math.hypot(x - cx, y - cy)
        gaps.append(min(abs(r - 3.5) - 0.65, r) if r > 1.0 else 2.0)  # belt r 2.95 .. 4.05; centre platform
    for a, (base, *_rest) in L.ARMS.items():
        gaps.append(math.hypot(x - base[0] - 0.2, y - base[1] - 0.154) - (0.45 if a == own else 1.3))
    x0, x1, y0, y1 = L.HALL
    gaps += [x - x0 - 0.5, x1 - x - 0.5, y - y0 - 0.5, y1 - y - 0.5]
    return min(gaps) - 0.64                                          # bin half diagonal


@lru_cache(maxsize=None)
def bins(arm: str) -> Dict[str, Tuple[float, float]]:
    """Recycling bin on the left of the pick -> place direction, trash bin on the right (as in the
    other worlds), at the first distance / shift along the axis that keeps them clear of everything."""
    bx, by = station(arm)["base"]
    ux, uy = station_axis(arm)
    nx, ny = -uy, ux
    out = {}
    for name, side in (("recycling", 1.0), ("trash", -1.0)):
        best, best_gap = None, -1e9
        for d in BIN_OFFSETS:
            for sh in BIN_SHIFTS:
                x, y = bx + side * nx * d + ux * sh, by + side * ny * d + uy * sh
                g = _clearance(x, y, arm)
                if g > best_gap:
                    best, best_gap = (x, y), g
                if g >= 0:
                    break
            if best_gap >= 0:
                break
        out[name] = (round(best[0], 3), round(best[1], 3))
    return out


def bin_name(arm: str, kind: str) -> str:
    return f"{kind}_bin_{arm}"


@lru_cache(maxsize=None)
def station_profile(arm: str) -> WorldProfile:
    arm = arm.upper()
    if arm not in L.ARMS:
        raise ValueError(f"unknown pipeline station '{arm}'; stations: {', '.join(STATIONS)}")
    st = station(arm)
    return WorldProfile(
        key=f"pipeline:{arm}", name=WORLD_NAME, sdf=SDF, has_bent2=True,
        bent1_shape="circular" if is_ring(st["pick_conv"]) else "straight",
        bent2_shape="circular" if is_ring(st["place_conv"]) else "straight",
        lanes=[lane(st["pick_conv"], st["pick"], "ring1"), lane(st["place_conv"], st["place"], "ring2")],
        layout={"station": st},
        station=tuple(st["pick"]), place_point=tuple(st["place"]),
        bins=dict(bins(arm)), bin_size=BIN_SIZE,
        camera_pose=camera_pose(arm), camera_size=CAM_SIZE, station_name=arm, image_topic=camera_topic(arm),
    )


RING_DIRS: Dict[str, int] = {c: _ring_dir(c) for c in L.CIRCLES}


def describe() -> str:
    lines = ["rings: " + ", ".join(f"{c} {'counter-clockwise' if d > 0 else 'clockwise'}" for c, d in RING_DIRS.items())]
    for a in STATIONS:
        st = station(a)
        b = bins(a)
        lines.append(f"pipeline:{a:4s} {st['pick_conv']} -> {st['place_conv']:3s} rule {st['rule']:10s} "
                     f"bins recycling ({b['recycling'][0]:.1f}, {b['recycling'][1]:.1f}) "
                     f"trash ({b['trash'][0]:.1f}, {b['trash'][1]:.1f})  clearance "
                     f"{min(_clearance(*b['recycling'], a), _clearance(*b['trash'], a)):+.2f} m")
    return "\n".join(lines)
