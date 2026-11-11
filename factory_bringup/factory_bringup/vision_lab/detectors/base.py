"""
The pluggable detector interface of the vision lab (no ROS imports).

A detector gets one BGR image of the overhead camera and returns ObjectDetection objects in
pixels. Everything after that is done for every detector in the same way by DetectionPipeline:
pixel -> world on the product-top plane, which bent (lane) the object is on, the arm mask, the
ROS topics the pick-and-place reads, the window, the dataset recorder.

Write your own:

    from factory_bringup.vision_lab import Detector, ObjectDetection, register_detector

    @register_detector("my_detector")
    class MyDetector(Detector):
        def __init__(self, threshold=40):          # options: --opt threshold=60 on the command line
            self.threshold = threshold

        def setup(self, ctx):                      # once per image size; ctx.camera, ctx.lane_masks, ctx.world ...
            super().setup(ctx)

        def detect(self, bgr):
            ...
            return [ObjectDetection(u=cx, v=cy, box_px=four_corners, label="paint_can", score=0.9)]

and pick it with  --detector my_detector  (when your file is imported), or without registering
--detector path/to/my_file.py:MyDetector  or  --detector my_package.my_module:MyDetector.
"""

import importlib
import importlib.util
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Union

import numpy as np

from factory_bringup.vision_geometry import OverheadCamera, Ring


@dataclass
class ObjectDetection:
    """One detected object. A detector fills the pixel part (u, v and box_px or bbox) and, if it
    knows them, label / score / class_id; the pipeline fills the world part (x, y, yaw, size, bent).
    A detector that already knows world coordinates may set x / y itself."""
    u: float                                    # centre, pixels
    v: float
    label: str = "object"
    score: float = 1.0
    class_id: int = -1
    box_px: Optional[np.ndarray] = field(default=None, repr=False)   # 4 x 2 corners (rotated box), pixels
    bbox: Optional[Tuple[float, float, float, float]] = None          # x1, y1, x2, y2, pixels
    # world part (metres, world frame), filled by the pipeline when None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    size_x: Optional[float] = None
    size_y: Optional[float] = None
    bent: Optional[int] = None                  # 1 = pick bent, 2 = place bent, None = off the belts
    extra: Dict = field(default_factory=dict)   # anything else a detector wants to pass on

    def __post_init__(self):
        if self.box_px is not None:
            self.box_px = np.asarray(self.box_px, dtype=np.float32).reshape(-1, 2)
        if self.bbox is None and self.box_px is not None:
            x1, y1 = self.box_px.min(axis=0)
            x2, y2 = self.box_px.max(axis=0)
            self.bbox = (float(x1), float(y1), float(x2), float(y2))
        if self.box_px is None and self.bbox is not None:
            x1, y1, x2, y2 = self.bbox
            self.box_px = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)

    @property
    def ring(self) -> Optional[str]:
        """ring1 / ring2: the names the older nodes use for bent 1 / bent 2."""
        return f"ring{self.bent}" if self.bent else None

    @property
    def xy(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def as_dict(self) -> Dict:
        r = lambda v, n=4: None if v is None else round(float(v), n)
        d = {"label": self.label, "score": r(self.score, 3), "class_id": self.class_id,
             "bent": self.bent, "ring": self.ring, "x": r(self.x), "y": r(self.y), "yaw": r(self.yaw),
             "size_x": r(self.size_x, 3), "size_y": r(self.size_y, 3), "u": r(self.u, 1), "v": r(self.v, 1)}
        if self.size_x is not None and self.size_y is not None:
            d["area_m2"] = r(self.size_x * self.size_y)
        if self.extra:
            d["extra"] = self.extra
        return d


@dataclass
class DetectorContext:
    """What a detector may use: built by the pipeline for each image size."""
    camera: OverheadCamera
    lanes: List[Ring]                           # bent 1 lane, then bent 2 lane (if the world has one)
    lane_masks: Dict[str, np.ndarray]           # ring name -> uint8 mask (255 = on that lane), image size
    belt_z: float
    object_top_z: float
    world: object                               # vision_lab.worlds.WorldProfile
    exclude_mask: Optional[np.ndarray] = None   # pixels to ignore this frame (the arm seen from above)

    @property
    def any_lane_mask(self) -> np.ndarray:
        out = np.zeros((self.camera.height, self.camera.width), np.uint8)
        for m in self.lane_masks.values():
            out |= m
        return out


class Detector:
    """Base class of every detector. Override detect(); setup() and debug_layers() are optional."""
    name = "detector"

    def setup(self, ctx: DetectorContext) -> None:
        self.ctx = ctx

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        raise NotImplementedError

    def debug_layers(self) -> Dict[str, np.ndarray]:
        """Intermediate images of the last detect() (gray / binary masks) for the window thumbnails."""
        return {}

    def __repr__(self):
        return f"<{type(self).__name__} '{self.name}'>"


# ------------------------------------------------------------------------------------- registry
_REGISTRY: Dict[str, Callable[..., Detector]] = {}
_BUILTIN_MODULES = {          # imported on first use, so a missing optional package only matters when chosen
    "opencv_contours": "factory_bringup.vision_lab.detectors.opencv_contours",
    "opencv_color": "factory_bringup.vision_lab.detectors.opencv_color",
    "opencv_bgsub": "factory_bringup.vision_lab.detectors.opencv_bgsub",
    "yolo": "factory_bringup.vision_lab.detectors.yolo",
}


def register_detector(name: str):
    """Class decorator: makes the detector available as --detector <name>."""
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def available_detectors() -> Dict[str, str]:
    """name -> first line of its docstring (built-ins and every registered one)."""
    out = {}
    for name, mod in _BUILTIN_MODULES.items():
        try:
            importlib.import_module(mod)
        except ImportError as exc:
            out[name] = f"(not importable here: {exc})"
    for name, cls in _REGISTRY.items():
        out[name] = ((cls.__doc__ or "").strip().splitlines() or [""])[0]
    return out


def _load_attr(spec: str):
    """'pkg.module:Name' or '/path/file.py:Name' -> the object."""
    target, _, attr = spec.rpartition(":")
    if target.endswith(".py") or os.sep in target:
        path = os.path.abspath(os.path.expanduser(target))
        mod_name = "vision_lab_user_" + os.path.splitext(os.path.basename(path))[0]
        mspec = importlib.util.spec_from_file_location(mod_name, path)
        if mspec is None:
            raise ImportError(f"cannot load {path}")
        module = importlib.util.module_from_spec(mspec)
        sys.modules[mod_name] = module
        mspec.loader.exec_module(module)
    else:
        module = importlib.import_module(target)
    return getattr(module, attr)


def create_detector(spec: Union[str, Detector, type, Callable[..., Detector]], **options) -> Detector:
    """
    A detector from:
      - an instance (returned as it is),
      - a Detector subclass or factory function (called with options),
      - a registered / built-in name: opencv_contours, opencv_color, opencv_bgsub, yolo, ...
      - 'package.module:ClassName' or 'path/to/file.py:ClassName'.
    """
    if isinstance(spec, Detector):
        return spec
    if callable(spec) and not isinstance(spec, str):
        return spec(**options)
    if spec in _BUILTIN_MODULES and spec not in _REGISTRY:
        importlib.import_module(_BUILTIN_MODULES[spec])
    if spec in _REGISTRY:
        return _REGISTRY[spec](**options)
    if ":" in spec:
        return create_detector(_load_attr(spec), **options)
    raise ValueError(f"Unknown detector '{spec}'. Built-in: {', '.join(_BUILTIN_MODULES)}; "
                     f"or give 'module:Class' / 'file.py:Class'.")


def parse_options(pairs: List[str]) -> Dict:
    """['conf=0.4', 'classes=[0,2]', 'model=best.pt'] -> {'conf': 0.4, 'classes': [0, 2], 'model': 'best.pt'}"""
    import ast
    out = {}
    for p in pairs or []:
        k, sep, v = p.partition("=")
        if not sep:
            raise ValueError(f"option '{p}' is not key=value")
        try:
            out[k.strip()] = ast.literal_eval(v)
        except (ValueError, SyntaxError):
            low = v.strip().lower()
            out[k.strip()] = {"true": True, "false": False, "none": None}.get(low, v)
    return out
