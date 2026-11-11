"""
Synthetic overhead-camera scenes (no ROS, no Gazebo): products of known type and position drawn on
the belts of any world, as the overhead camera would see them. For developing and unit-testing a
detector before the simulation runs, and for the self-test:

    from factory_bringup.vision_lab.synthetic import render_scene, random_scene
    bgr, truth = random_scene("straight-circular", n_per_bent=5, seed=1)
    # truth: [{"label": "paint_can", "x": .., "y": .., "bent": 1, "yaw": ..}, ...]

It is a flat-shaded drawing, not a render: a detector that passes here can still fail on the real
lighting, so check it in the simulation (vision_lab run) too.
"""

import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from factory_bringup.product_catalog import LINE_RADIUS, PRODUCT_TYPES
from factory_bringup.vision_lab.worlds import BELT_Z, WorldProfile, get_world, path_point

FLOOR_BGR = (118, 120, 122)
PIPELINE_FLOOR_BGR = (97, 102, 84)    # epoxy hall floor of the pipeline world
BELT_BGR = (28, 27, 26)
CIRCULAR_BELT = (2.95, 4.05)      # belt radii of a circular bent
STRAIGHT_HALF_WIDTH = 0.45


def _belt_mask(cam, lane) -> np.ndarray:
    uu, vv = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    wx, wy = cam.pixel_to_world(uu, vv, BELT_Z)
    if getattr(lane, "seg", None):
        x0, y0, x1, y1 = lane.seg
        n = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / n, (y1 - y0) / n
        along = (wx - x0) * ux + (wy - y0) * uy
        return (np.abs(-(wx - x0) * uy + (wy - y0) * ux) <= 0.54) & (along >= 0) & (along <= n)
    r = np.hypot(wx - lane.center_x, wy - lane.center_y)
    mid = (lane.inner_radius + lane.outer_radius) / 2
    if mid >= LINE_RADIUS / 2:
        m = (np.abs(r - mid) <= STRAIGHT_HALF_WIDTH) & (wy > -15.4) & (wy < 15.4)
    else:
        m = (r >= CIRCULAR_BELT[0]) & (r <= CIRCULAR_BELT[1])
    return m


_BACKGROUNDS: Dict = {}


def background(w: WorldProfile, width: int, height: int) -> np.ndarray:
    """Floor + belts of a world as the camera sees them (cached)."""
    key = (w.key, width, height)
    if key not in _BACKGROUNDS:
        cam = w.camera(width, height)
        img = np.empty((height, width, 3), np.uint8)
        img[:] = PIPELINE_FLOOR_BGR if w.station_name else FLOOR_BGR
        for lane in w.lanes:
            img[_belt_mask(cam, lane)] = BELT_BGR
        _BACKGROUNDS[key] = img
    return _BACKGROUNDS[key].copy()


def draw_product(img: np.ndarray, cam, ptype: str, x: float, y: float, heading: float, z_bottom: float = BELT_Z,
                 light: float = 0.92) -> float:
    """Draw one product's top face; returns its yaw as the detectors report it."""
    t = PRODUCT_TYPES[ptype]
    sx, sy, sz = t["size"]
    top = z_bottom + sz
    r, g, b = t["rgb"]
    col = tuple(int(min(255, c * 255 * light + 10)) for c in (b, g, r))
    if t["shape"] in ("cylinder", "sphere"):
        u, v = cam.world_to_pixel(x, y, top)
        u2, v2 = cam.world_to_pixel(x, y + sx / 2, top)
        rad = max(2, int(round(math.hypot(u2 - u, v2 - v))))
        cv2.circle(img, (int(round(u)), int(round(v))), rad, col, -1, cv2.LINE_AA)
        return 0.0
    c, s_ = math.cos(heading), math.sin(heading)
    corners = [(x + c * dx - s_ * dy, y + s_ * dx + c * dy)
               for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]
    pts = np.array([cam.world_to_pixel(px, py, top) for px, py in corners], np.float32)
    cv2.fillPoly(img, [np.round(pts).astype(np.int32)], col, cv2.LINE_AA)
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def add_noise(img: np.ndarray, noise: float, seed: Optional[int] = None) -> np.ndarray:
    if noise <= 0:
        return img
    rng = np.random.default_rng(seed)
    return np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)


def render_view(world, items: Sequence[Tuple[str, float, float, float, float]], width: Optional[int] = None,
                height: Optional[int] = None, noise: float = 3.0, seed: Optional[int] = None) -> np.ndarray:
    """A camera image of products at world poses: items = (type, x, y, heading, z_bottom)."""
    w: WorldProfile = world if isinstance(world, WorldProfile) else get_world(world)
    width, height = width or w.camera_size[0], height or w.camera_size[1]
    cam = w.camera(width, height)
    img = background(w, width, height)
    for ptype, x, y, heading, zb in sorted(items, key=lambda it: it[4]):    # lower ones first
        draw_product(img, cam, ptype, x, y, heading, zb)
    return add_noise(img, noise, seed)


def render_scene(world=None, products: Sequence[Tuple[str, int, float]] = (), width: Optional[int] = None,
                 height: Optional[int] = None, noise: float = 3.0, seed: Optional[int] = None,
                 light: float = 0.92) -> Tuple[np.ndarray, List[Dict]]:
    """products: (type, bent 1 | 2, s metres along the bent's path from the arm side)."""
    w: WorldProfile = world if isinstance(world, WorldProfile) else get_world(world)
    width, height = width or w.camera_size[0], height or w.camera_size[1]
    cam = w.camera(width, height)
    img = background(w, width, height)
    truth = []
    for ptype, bent, s in products:
        lane = w.lane_of_bent(bent)
        if lane is None:
            raise ValueError(f"world {w.key} has no bent {bent}")
        x, y, heading = path_point(lane, s)
        yaw = draw_product(img, cam, ptype, x, y, heading, BELT_Z, light)
        truth.append({"label": ptype, "x": x, "y": y, "bent": bent, "yaw": yaw})
    return add_noise(img, noise, seed), truth


def random_scene(world=None, n_per_bent: int = 5, seed: Optional[int] = None, types: Optional[List[str]] = None,
                 spacing: float = 1.2, **kw) -> Tuple[np.ndarray, List[Dict]]:
    """Random products, `spacing` metres apart, on every bent of the world."""
    w = world if isinstance(world, WorldProfile) else get_world(world)
    rng = random.Random(seed)
    types = types or list(PRODUCT_TYPES)
    prods = []
    for bent in range(1, len(w.lanes) + 1):
        lane = w.lane_of_bent(bent)
        if getattr(lane, "anchor", None):            # pipeline station: around the pick / place point
            lo, hi = -2.4, 2.4
            if getattr(lane, "seg", None):
                x0, y0, x1, y1 = lane.seg
                n = math.hypot(x1 - x0, y1 - y0)
                sa = (lane.anchor[0] - x0) * (x1 - x0) / n + (lane.anchor[1] - y0) * (y1 - y0) / n
                lo, hi = max(lo, -sa + 0.3), min(hi, n - sa - 0.3)
            k = max(1, min(n_per_bent, int((hi - lo) // 0.7) + 1))
            step = (hi - lo) / max(1, k - 1) if k > 1 else 0.0
            for i in range(k):
                prods.append((rng.choice(types), bent, lo + i * step + rng.uniform(-0.1, 0.1) * (k > 1)))
            continue
        straight = (lane.inner_radius + lane.outer_radius) / 2 >= LINE_RADIUS / 2
        start = -5.0 if straight else 0.6
        for i in range(n_per_bent):
            s = start + i * spacing + rng.uniform(-0.15, 0.15)
            prods.append((rng.choice(types), bent, s))
    return render_scene(w, prods, seed=seed, **kw)


def match_truth(dets, truth: List[Dict], radius: float = 0.25) -> Dict:
    """Precision / recall / label accuracy of detections against the scene truth (greedy nearest match)."""
    used, tp, label_ok, errs = set(), 0, 0, []
    for t in truth:
        best, bd = None, radius
        for i, d in enumerate(dets):
            if i in used:
                continue
            dist = math.hypot(d.x - t["x"], d.y - t["y"])
            if dist <= bd and d.bent == t["bent"]:
                best, bd = i, dist
        if best is not None:
            used.add(best)
            tp += 1
            errs.append(bd)
            label_ok += dets[best].label == t["label"]
    return {"truth": len(truth), "detected": len(dets), "matched": tp,
            "precision": tp / max(1, len(dets)), "recall": tp / max(1, len(truth)),
            "label_accuracy": label_ok / max(1, tp), "mean_xy_error_m": float(np.mean(errs)) if errs else None}
