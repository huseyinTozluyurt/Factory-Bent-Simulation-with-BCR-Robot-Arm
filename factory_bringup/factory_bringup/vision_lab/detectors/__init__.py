"""Detectors of the vision lab. Built-ins: opencv_contours, opencv_color, opencv_bgsub, yolo (optional)."""
from factory_bringup.vision_lab.detectors.base import (  # noqa: F401
    Detector, DetectorContext, ObjectDetection, available_detectors, create_detector, parse_options, register_detector,
)
