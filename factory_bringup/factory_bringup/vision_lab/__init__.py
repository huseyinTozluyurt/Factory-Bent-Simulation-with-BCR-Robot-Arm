"""
Vision lab: a computer-vision development environment for the factory simulation.

    from factory_bringup.vision_lab import VisionLab, Detector, ObjectDetection, register_detector

Pure-Python parts (detectors, pipeline, worlds, synthetic scenes, dataset recorder) import without
ROS; VisionLab (the live simulation API) needs rclpy and is imported on first use. PipelineLab (the
pipeline world, over gz-transport or with --fake drawn cameras) needs no ROS.
See README.md next to this file, or vision_lab_main.py in the project folder.
"""
from factory_bringup.vision_lab.detectors import (  # noqa: F401
    Detector, DetectorContext, ObjectDetection, available_detectors, create_detector, parse_options, register_detector,
)
from factory_bringup.vision_lab.pipeline import DetectionPipeline  # noqa: F401
from factory_bringup.vision_lab.worlds import WorldProfile, get_world, list_worlds  # noqa: F401


def __getattr__(name):
    if name == "VisionLab":
        from factory_bringup.vision_lab import ros_lab
        return ros_lab.VisionLab
    if name in ("PickContext", "PickResult", "DESTINATIONS"):
        from factory_bringup.vision_lab import common
        return getattr(common, name)
    if name == "PipelineLab":
        from factory_bringup.vision_lab import pipeline_lab
        return pipeline_lab.PipelineLab
    raise AttributeError(name)
