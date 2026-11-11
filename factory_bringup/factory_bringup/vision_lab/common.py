"""
Shared lab types (no ROS imports): destinations, PickContext (what a policy gets) and PickResult.
Used by the single-arm lab (ros_lab.VisionLab) and the pipeline lab (pipeline_lab.PipelineLab).
"""
import time
from dataclasses import dataclass, field
from typing import Optional

from factory_bringup.vision_lab.detectors.base import ObjectDetection
from factory_bringup.vision_lab.worlds import WorldProfile

DESTINATIONS = ("bent2", "trash", "recycling", "skip")
_DEST_ALIASES = {
    "bent2": "bent2", "bent_2": "bent2", "bent 2": "bent2", "ring2": "bent2", "place": "bent2", "conveyor": "bent2",
    "trash": "trash", "trash_bin": "trash", "garbage": "trash", "waste": "trash",
    "recycling": "recycling", "recycle": "recycling", "recycle_bin": "recycling", "recycling_bin": "recycling",
    "skip": "skip", "none": "skip", "leave": "skip", "stay": "skip",
}

def normalize_destination(dest) -> str:
    key = str(dest).strip().lower() if dest is not None else "skip"
    if key not in _DEST_ALIASES:
        raise ValueError(f"unknown destination '{dest}': use one of {', '.join(DESTINATIONS)}")
    return _DEST_ALIASES[key]


@dataclass
class PickContext:
    """What a policy gets for each product the arm is about to take."""
    number: Optional[int]                     # 1, 2, 3 ... in the order products reach the arm
    detection: Optional[ObjectDetection]      # what YOUR detector saw at the catch station (label, score, size ...)
    model: Optional[str]                      # Gazebo model name (simulation truth, e.g. order_paint_can_3)
    product_type: Optional[str]               # type from the model name (simulation truth)
    world: WorldProfile
    lab: object = field(repr=False, default=None)
    station: Optional[str] = None             # pipeline world: the arm station (A1 ... A12) deciding

    @property
    def label(self) -> Optional[str]:
        """The detector's label of this product (None if the detector did not see it at the station)."""
        return self.detection.label if self.detection else None


@dataclass
class PickResult:
    number: Optional[int]
    destination: str                          # bent2 / trash / recycling / skip
    status: str                               # delivered, skipped
    label: Optional[str]                      # detector label
    score: Optional[float]
    model: Optional[str]
    product_type: Optional[str]               # truth, to score the detector
    time: float = field(default_factory=time.time)
    station: Optional[str] = None

    @property
    def label_correct(self) -> Optional[bool]:
        if self.label is None or self.product_type is None:
            return None
        return self.label == self.product_type
