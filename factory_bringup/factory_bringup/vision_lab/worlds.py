"""
World profiles for the vision lab (no ROS imports).

Every world the arm works in shares the overhead camera (0, 0, 13) looking straight down, the
side camera at the catch station, the catch station itself (-2.58, 0), the place point on bent 2
and the two bins (recycling at +y, trash at -y). What differs is the shape of each bent and
whether there is a bent 2 at all. That comes from product_catalog.WORLD_LAYOUTS, the same table
every node of the simulation reads, so the lab never disagrees with the running simulation.

    from factory_bringup.vision_lab.worlds import get_world
    w = get_world("straight-circular")      # run_factory.sh key, Gazebo world name, or None = $FACTORY_WORLD_NAME
    w.has_bent2, w.bent1_shape, w.lanes, w.bins
"""

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from factory_bringup.product_catalog import (
    BIN_SIZE, BINS, DUAL_WORLD, LINE_RADIUS, STATION_X, WORLD_ENV, WORLD_FILES, WORLD_LAYOUTS,
)
from factory_bringup.vision_geometry import OverheadCamera, Ring

# what the dual circular world uses where a layout says nothing (ring_object_detector defaults)
_BASE = dict(
    ring1_center_x=-6.0, ring1_center_y=0.0, ring2_center_x=6.0, ring2_center_y=0.0,
    ring_inner_radius=3.10, ring_outer_radius=3.75, ring1_radius=3.42, ring2_product_radius=3.42,
    has_bent2=True,
)
OVERHEAD_CAMERA = dict(x=0.0, y=0.0, z=13.0, roll=0.0, pitch=1.5708, yaw=0.0, hfov=1.55)
STATION = (STATION_X, 0.0)
PLACE_POINT = (2.2171, -0.3736)
BELT_Z = 1.375
OBJECT_TOP_Z = 1.60

UNSUPPORTED: Dict[str, str] = {}      # every world is supported (the pipeline through pipeline_lab)


@dataclass
class Lane(Ring):
    """A product lane. Circular lanes are plain Rings; a straight conveyor piece also has `seg`
    (x0, y0, x1, y1: start to end, products move that way) and `half_width`, so its mask stops at
    the belt ends (the pipeline world has several conveyors in one camera view). `anchor` is the
    point the lane's path coordinate s counts from (the pick / place point)."""
    seg: Optional[Tuple[float, float, float, float]] = None
    half_width: float = 0.325
    anchor: Optional[Tuple[float, float]] = None


@dataclass
class WorldProfile:
    key: str                                  # run_factory.sh --world key (dual, circular, straight, ...)
    name: str                                 # Gazebo <world name>
    sdf: str                                  # file in my_factory_worlds/worlds
    has_bent2: bool
    bent1_shape: str                          # "circular" or "straight"
    bent2_shape: Optional[str]                # "circular", "straight" or None
    lanes: List[Ring] = field(default_factory=list)   # product lanes the camera searches: bent 1, then bent 2
    layout: Dict = field(default_factory=dict)
    station: Tuple[float, float] = STATION
    place_point: Tuple[float, float] = PLACE_POINT
    bins: Dict[str, Tuple[float, float]] = field(default_factory=lambda: dict(BINS))
    bin_size: Tuple[float, float, float] = BIN_SIZE
    camera_pose: Dict = field(default_factory=lambda: dict(OVERHEAD_CAMERA))   # x y z roll pitch yaw hfov
    camera_size: Tuple[int, int] = (1280, 720)
    station_name: Optional[str] = None        # pipeline world: A1 ... A12
    image_topic: str = "/overhead_camera/image"

    @property
    def destinations(self) -> List[str]:
        """Where the arm can take a product in this world."""
        return (["bent2"] if self.has_bent2 else []) + list(self.bins)

    def camera(self, width: Optional[int] = None, height: Optional[int] = None) -> OverheadCamera:
        w, h = self.camera_size
        return OverheadCamera(width=width or w, height=height or h, **self.camera_pose)

    def lane_of_bent(self, bent: int) -> Optional[Ring]:
        return self.lanes[bent - 1] if 0 < bent <= len(self.lanes) else None

    def describe(self) -> str:
        b2 = f"bent 2 {self.bent2_shape}" if self.has_bent2 else "no bent 2 (products not for a bin ride on)"
        return f"{self.key:18s} {self.name:30s} bent 1 {self.bent1_shape}, {b2}; bins: {', '.join(self.bins)}"


def _shape(radius: float) -> str:
    return "straight" if radius >= LINE_RADIUS / 2 else "circular"


def _profile(key: str, sdf: str, name: str) -> WorldProfile:
    lay = dict(_BASE, **WORLD_LAYOUTS.get(name, {}))
    has2 = bool(lay.get("has_bent2", True))
    rin, rout = lay["ring_inner_radius"], lay["ring_outer_radius"]
    lanes = [Ring("ring1", lay["ring1_center_x"], lay["ring1_center_y"], rin, rout)]
    if has2:
        lanes.append(Ring("ring2", lay["ring2_center_x"], lay["ring2_center_y"],
                          lay.get("ring2_lane_inner_radius") or rin, lay.get("ring2_lane_outer_radius") or rout))
    return WorldProfile(
        key=key, name=name, sdf=sdf, has_bent2=has2,
        bent1_shape=_shape(lay.get("ring1_radius", 3.42)),
        bent2_shape=_shape(lay.get("ring2_product_radius", 3.42)) if has2 else None,
        lanes=lanes, layout=lay,
        place_point=(lay.get("place_x", PLACE_POINT[0]), lay.get("place_y", PLACE_POINT[1])),
    )


WORLDS: Dict[str, WorldProfile] = {key: _profile(key, sdf, name) for key, (sdf, name) in WORLD_FILES.items()}
ALIASES = {"circular-circular": "dual", "dual_circular": "dual", "single": "circular"}


PIPELINE_KEYS = ("pipeline", "pipeline_factory", "pipeline_factory_world", "pipeline_factory_lab",
                 "pipeline_factory.sdf", "pipeline_factory_lab.sdf", "pipeline_factory.sh")


def is_pipeline(key_or_name: Optional[str]) -> bool:
    k = os.path.basename(str(key_or_name or os.environ.get(WORLD_ENV) or "")).split(":")[0]
    return k in PIPELINE_KEYS


def list_worlds(include_pipeline: bool = False) -> List[WorldProfile]:
    """The single-arm worlds; with include_pipeline also one profile per pipeline arm station."""
    out = list(WORLDS.values())
    if include_pipeline:
        from factory_bringup.vision_lab.pipeline_world import STATIONS, station_profile
        out += [station_profile(s) for s in STATIONS]
    return out


def get_world(key_or_name: Optional[str] = None, station: Optional[str] = None) -> WorldProfile:
    """World by run_factory.sh key, alias, Gazebo world name or sdf file name; None = $FACTORY_WORLD_NAME
    (set by run_factory.sh and factory_full.launch.py), else the dual circular world.
    The pipeline world ("pipeline", or "pipeline:A2") gives the profile of one arm station: its camera,
    its pick conveyor as bent 1, its place conveyor as bent 2 and its own two bins."""
    k = (key_or_name or os.environ.get(WORLD_ENV) or DUAL_WORLD).strip()
    k = os.path.basename(k)
    if is_pipeline(k):
        from factory_bringup.vision_lab.pipeline_world import station_profile
        return station_profile(station or (k.split(":", 1)[1] if ":" in k else "A1"))
    k = ALIASES.get(k, k)
    if k in WORLDS:
        return WORLDS[k]
    for w in WORLDS.values():
        if k in (w.name, w.sdf, w.sdf[:-4]):
            return w
    for bad, why in UNSUPPORTED.items():
        if bad in k:
            raise ValueError(why)
    raise ValueError(f"Unknown world '{key_or_name}'. Known: {', '.join(WORLDS)} (or their Gazebo world names).")


def path_point(lane: Ring, s: float) -> Tuple[float, float, float]:
    """(x, y, heading) of the product path of a lane, s metres along it from the catch / place side.
    For circular lanes s is arc length counter-clockwise from the point nearest the arm; for straight
    lanes it is distance along +y from y = 0. Used by the synthetic scenes and the examples."""
    if getattr(lane, "seg", None):
        x0, y0, x1, y1 = lane.seg
        L = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        ax, ay = lane.anchor or (x0, y0)
        return ax + ux * s, ay + uy * s, math.atan2(uy, ux)
    r = (lane.inner_radius + lane.outer_radius) / 2.0
    if getattr(lane, "anchor", None):
        r = 3.42 if abs(r - 3.425) < 0.2 else r
        a = math.atan2(lane.anchor[1] - lane.center_y, lane.anchor[0] - lane.center_x) + s / r
        return lane.center_x + r * math.cos(a), lane.center_y + r * math.sin(a), a + math.pi / 2
    if r >= LINE_RADIUS / 2:                         # a straight belt modelled as a huge ring
        x = lane.center_x + (r if lane.center_x < 0 else -r)
        return x, lane.center_y + s, math.pi / 2
    r = 3.42 if abs(r - 3.425) < 0.2 else r          # products ride r = 3.42 in the circular lane
    a0 = math.atan2(-lane.center_y, -lane.center_x)  # direction from the ring centre towards the arm (origin)
    a = a0 + s / r
    return lane.center_x + r * math.cos(a), lane.center_y + r * math.sin(a), a + math.pi / 2
