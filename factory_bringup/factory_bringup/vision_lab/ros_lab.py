"""
VisionLab: the live API of the vision lab on the running simulation (needs ROS 2).

    from factory_bringup.vision_lab import VisionLab

    with VisionLab(detector="opencv_contours") as lab:          # world: found from the running simulation
        print(lab.detections(bent=1))                            # what the detector sees on bent 1
        lab.transfer("bent2")                                    # next product from bent 1 onto bent 2
        lab.put_in_trash()                                       # next one into the trash bin
        lab.drop_in_recycle_bin()                                # next one into the recycling bin
        lab.run({"paint_can": "recycling", "*": "bent2"}, count=5)   # sort 5 by the detector's label

What it starts in this process:
  - "vision_lab_detector": runs your detector on /overhead_camera/image and publishes what the
    existing nodes expect (/vision/ring1_objects, /vision/ring2_objects, /vision/detections,
    /vision/debug_image), plus the "Vision Lab" window. It replaces ring_object_detector.
  - "vision_pick_place": the project's pick-and-place controller (vision_pick_place.py, unchanged),
    subclassed so your code decides where each product goes. It replaces the standalone controller.
So the simulation must run without those two:  run_vision_lab.sh sim  (or run_factory.sh ...
run_detector:=false run_controller:=false). The attacher, the side and release cameras stay as they are.

How a pick works (the controller's station mode, as in run_factory.sh): the arm waits at home, the
detector's bent-1 detections are tracked towards the catch station, the side camera confirms the
product, the product is stopped, the arm comes down onto it, attaches, lifts, and then asks your
policy where it goes: "bent2", "trash", "recycling", or "skip" (leave it on the belt).
"""

import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Union

import cv2
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from factory_bringup.product_catalog import BINS, PRODUCT_TYPES, WORLD_ENV, WORLD_LAYOUTS, type_of
from factory_bringup.vision_geometry import arm_mask_from_points
from factory_bringup.vision_pick_place import ATTACHER_NODE, SkipObject, StopRequested, VisionPickPlace
from factory_bringup.vision_lab.dataset import YoloDatasetRecorder
from factory_bringup.vision_lab.detectors.base import ObjectDetection
from factory_bringup.vision_lab.pipeline import DetectionPipeline
from factory_bringup.vision_lab.worlds import WorldProfile, get_world
from factory_bringup.vision_lab.common import (  # noqa: F401  (re-exported)
    DESTINATIONS, PickContext, PickResult, normalize_destination,
)

# vision_pick_place.launch.py's values for the controller (the lab runs it without that launch file)
ARM_DEFAULTS = {
    "use_sim_time": True, "mode": "station", "auto_start": False, "loop": True, "dry_run": False,
    "sync_from_attacher": True, "station_x": -2.58, "station_y": 0.0, "hover_z": 2.0, "roi_radius": 0.30,
    "catch_latency": 0.35, "track_coast_time": 5.0, "use_side_camera": True, "catch_window": 0.25,
    "cross_check_tol": 0.20, "side_confirm_timeout": 2.0, "descend_gap": 0.01, "descend_duration": 0.8,
    "lift_height": 0.50, "lift_max_z": 2.35, "swing_direction": "left", "carry_swing_duration": 3.0,
    "release_settle_sec": 0.6, "pick_zone_x": -2.45, "pick_zone_y": 0.0, "pick_zone_radius": 0.9,
    "pick_surface_z": 1.375, "stable_frames": 5, "stable_tolerance": 0.03, "place_x": 2.2171, "place_y": -0.3736,
    "place_yaw": -1.2781, "place_exact": True, "place_slots": 1, "use_release_camera": True, "wait_at": "home",
    "motion_variety": True, "motion_type": "auto", "place_wait_offset": 0.8, "place_slot_spacing": 0.3,
    "place_surface_z": 1.375, "place_tip_height": 0.47, "place_drop_clearance": 0.03, "approach_height": 0.25,
    "ik_tolerance": 0.05, "transit_pose": [0.0, 0.45, 0.0, -0.65, 0.0, 0.35, 0.0],
    "pick_seed": [1.30, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0], "move_duration": 1.0, "swing_duration": 2.0,
    "settle_timeout_sec": 1.5,
}


class LabArm(VisionPickPlace):
    """vision_pick_place.VisionPickPlace with the destination of each product decided by the lab."""

    def __init__(self, lab: "VisionLab"):
        self.lab = lab
        self.policy: Optional[Callable[[PickContext], str]] = None
        self.forced: Optional[str] = None
        self.decisions: Dict = {}
        self.results: List[PickResult] = []
        self.target_count: Optional[int] = None
        self.session_done = 0
        self.products_ready = threading.Event()
        self.prepared = False
        super().__init__()

    # the attacher calls /vision_pick_place/start once a start-up order is on the belt
    def on_start(self, request, response):
        self.products_ready.set()
        response.success, response.message = True, "Vision lab: the lab script drives the arm (products noted as ready)."
        return response

    def status(self, text: str, warn: bool = False):
        super().status(text, warn)
        for cb in list(self.lab._status_callbacks):
            try:
                cb(text)
            except Exception:
                pass

    # ------------------------------------------------------------------ destination
    def _decide(self) -> str:
        key = (self.object_number, self.held_model)
        if key in self.decisions:
            return self.decisions[key]
        if self.forced:
            dest = self.forced
        else:
            ctx = self.lab._context(self.object_number, self.held_model)
            try:
                dest = normalize_destination(self.policy(ctx))
            except Exception as exc:
                dest = "bent2" if self.p("has_bent2") else "skip"
                self.status(f"LAB policy failed for object #{self.object_number} ({exc}); using '{dest}'.", warn=True)
            self.lab._last_context[key] = ctx
        if dest == "bent2" and not self.p("has_bent2"):
            self.status(f"LAB: this world has no bent 2; object #{self.object_number} stays on the belt.", warn=True)
            dest = "skip"
        self.decisions[key] = dest
        self.status(f"LAB DECISION: object #{self.object_number} ({self.held_model or 'unknown'}) -> {dest}")
        return dest

    def goes_to(self):
        if self.policy is None and self.forced is None:
            return super().goes_to()            # GUI destinations file and numbered_destinations, as before
        return self._decide(), True

    def grab_from_above(self, target, q_hover, come_from_home=False):
        lab_decides = self.policy is not None or self.forced is not None
        if lab_decides and self.p("has_bent2") and not self.p("dry_run"):
            # ask which product it is before stopping it, so "skip" leaves it running (the controller
            # already does this in worlds without bent 2)
            self.set_attacher_params({"auto_select_target": True, "select_near_point": True,
                                      "select_x": target["x"], "select_y": target["y"], "hold_peek": True})
            try:
                peek = json.loads(self.call_trigger(self.hold_client, "/hold_object_for_pick"))
            finally:
                self.set_attacher_params({"hold_peek": False})
            self.held_model = peek.get("model")
            self.note_object(self.held_model)
            if self._decide() == "skip":
                self._record("skip", "skipped")
                raise SkipObject(f"object #{self.object_number} ({self.held_model}): the lab says skip")
        try:
            return super().grab_from_above(target, q_hover, come_from_home)
        except SkipObject:
            if not self.results or self.results[-1].number != self.object_number:
                self._record("skip", "skipped")
            raise

    def deliver(self, from_xy, seeds, swing_duration=None):
        if self.object_number is None:
            self.note_object(self.held_model)
        dest = self.goes_to()[0]
        super().deliver(from_xy, seeds, swing_duration)
        self._record(dest if dest in DESTINATIONS else "bent2", "delivered")
        self.session_done += 1
        if self.target_count and self.session_done >= self.target_count:
            self.set_parameters([Parameter("loop", value=False)])

    def _record(self, dest: str, status: str):
        key = (self.object_number, self.held_model)
        ctx = self.lab._last_context.get(key) or self.lab._context(self.object_number, self.held_model)
        det = ctx.detection
        res = PickResult(number=self.object_number, destination=dest, status=status,
                         label=det.label if det else None, score=det.score if det else None,
                         model=self.held_model, product_type=type_of(self.held_model))
        self.results.append(res)
        for cb in list(self.lab._result_callbacks):
            try:
                cb(res)
            except Exception:
                pass

    # ------------------------------------------------------------------ sessions
    def prepare(self):
        if self.prepared:
            return
        self.sync_from_attacher()
        self.ensure_chain()
        end = time.time() + 5.0
        while not all(n in self.joint_positions for n in self.p("joint_names")) and time.time() < end:
            time.sleep(0.05)
        if self.start_pose is None and all(n in self.joint_positions for n in self.p("joint_names")):
            self.start_pose = self.current_vector()
        self.prepared = True

    def home_pose(self) -> List[float]:
        home = list(self.p("home_pose"))
        return home if len(home) == len(self.p("joint_names")) else list(self.start_pose or self.p("transit_pose"))

    def run_session(self, count: Optional[int]):
        """Blocking: catch and deliver `count` products (None = until stop())."""
        self.busy = True
        self.stop_flag = False
        self.target_count = count
        self.session_done = 0
        self.set_parameters([Parameter("loop", value=count != 1)])
        try:
            self.prepare()
            if self.p("mode") == "pick_zone":
                self.run_pick_zone()
            else:
                self.run_station()
        except StopRequested:
            self.abort_hold()
            self.status("Stopped on request.", warn=True)
        except Exception:
            self.abort_hold()
            raise
        finally:
            self.busy = False
            self.target_count = None


class LabDetectorNode(Node):
    """Runs the lab's DetectionPipeline on the overhead camera and publishes the vision topics."""

    WINDOW = "Vision Lab"

    def __init__(self, lab: "VisionLab", pipeline: DetectionPipeline, image_topic: str, show_window: bool,
                 mask_arm: bool = True, recorder: Optional[YoloDatasetRecorder] = None):
        super().__init__("vision_lab_detector", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.lab, self.pipe, self.recorder = lab, pipeline, recorder
        self.bridge = CvBridge()
        self.ring_pubs = [self.create_publisher(PoseArray, "/vision/ring1_objects", 10),
                          self.create_publisher(PoseArray, "/vision/ring2_objects", 10)]
        self.json_pub = self.create_publisher(String, "/vision/detections", 10)
        self.debug_pub = self.create_publisher(Image, "/vision/debug_image", 2)
        self.create_subscription(Image, image_topic, self.on_image, 2)
        self.create_subscription(String, "/vision_pick_place/roi", self.on_roi, 5)
        self.create_subscription(String, "/vision_pick_place/status", self.on_status, 5)
        self.roi, self.controller_status = None, ""
        self.mask_arm = mask_arm
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.show_window = show_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if show_window and not self.show_window:
            self.get_logger().warn("No DISPLAY: the Vision Lab window is off (the debug image is still published).")
        self.window_open = False
        self.frames = 0

    def on_roi(self, msg):
        try:
            self.roi = json.loads(msg.data)
        except ValueError:
            self.roi = None

    def on_status(self, msg):
        self.controller_status = msg.data

    def arm_mask(self):
        if not self.mask_arm or self.lab.arm is None or self.pipe.ctx is None:
            return None
        arm = self.lab.arm
        bx, by, bz, yaw = (float(arm.p(k)) for k in ("base_x", "base_y", "base_z", "base_yaw"))
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(bx, by, bz)]
        for frame in ["link1", "link2", "link3", "link4", "link5", "link6", "flange", "gripper_tip"]:
            try:
                tf = self.tf_buffer.lookup_transform(str(arm.p("base_frame")), frame, rclpy.time.Time(),
                                                     timeout=rclpy.duration.Duration(seconds=0.0))
            except Exception:
                continue
            t = tf.transform.translation
            pts.append((bx + c * t.x - s * t.y, by + s * t.x + c * t.y, bz + t.z))
        return arm_mask_from_points(self.pipe.ctx.camera, pts, 0.40) if len(pts) >= 3 else None

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return
        mask = self.arm_mask()
        try:
            dets = self.pipe.process(bgr, exclude_mask=mask)
        except Exception as exc:
            self.get_logger().error(f"Detector {self.pipe.detector.name} failed: {exc}", throttle_duration_sec=5.0)
            return
        for i, pub in enumerate(self.ring_pubs):
            arr = PoseArray()
            arr.header.stamp, arr.header.frame_id = msg.header.stamp, "world"
            for d in dets:
                if d.bent == i + 1:
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = d.x, d.y, self.pipe.object_top_z
                    yaw = d.yaw or 0.0
                    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                    arr.poses.append(p)
            pub.publish(arr)
        self.json_pub.publish(String(data=json.dumps([d.as_dict() for d in dets])))
        self.lab._on_frame(bgr, dets)
        rec = self.lab.recorder
        if rec is not None:
            rec.add(bgr, dets)
        if self.debug_pub.get_subscription_count() or self.show_window:
            view = self.pipe.draw(bgr, dets, self.roi, self.controller_status, mask)
            if self.debug_pub.get_subscription_count():
                out = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)
            if self.show_window:
                if not self.window_open:
                    cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.WINDOW, 1280, 720)
                    self.window_open = True
                cv2.imshow(self.WINDOW, view)
                cv2.waitKey(1)
        self.frames += 1


class VisionLab:
    """
    Parameters
      world             run_factory.sh key / Gazebo world name; None = ask the running simulation
      detector          name ("opencv_contours", "opencv_color", "opencv_bgsub", "yolo"), "module:Class",
                        "file.py:Class", a Detector class or instance
      detector_options  dict of options for the detector (e.g. {"model": "best.pt", "conf": 0.4})
      show_window       the "Vision Lab" window with the live detections
      dry_run           detect and plan, log the arm goals, but do not move the arm
      record_dataset    folder: save auto-labelled YOLO training frames there while running
      arm_params        extra / changed vision_pick_place parameters, e.g. {"motion_type": "cycle"}
    """

    def __init__(self, world: Optional[str] = None, detector="opencv_contours", detector_options: Optional[Dict] = None,
                 show_window: bool = True, dry_run: bool = False, record_dataset: Optional[str] = None,
                 record_options: Optional[Dict] = None, image_topic: str = "/overhead_camera/image",
                 arm_params: Optional[Dict] = None, mask_arm: bool = True, output_dir: Optional[str] = None,
                 start: bool = True):
        self._world_arg = world
        self._detector_spec, self._detector_options = detector, dict(detector_options or {})
        self._show_window, self._dry_run, self._mask_arm = show_window, dry_run, mask_arm
        self._record_dir, self._record_options = record_dataset, dict(record_options or {})
        self._image_topic = image_topic
        self._arm_params = dict(arm_params or {})
        self.world: Optional[WorldProfile] = None
        self.pipeline: Optional[DetectionPipeline] = None
        self.arm: Optional[LabArm] = None
        self.detector_node: Optional[LabDetectorNode] = None
        self.recorder: Optional[YoloDatasetRecorder] = None
        self._lock = threading.Condition()
        self._latest = (0.0, [], None)                     # wall time, detections, bgr
        self._history: deque = deque(maxlen=120)          # (wall time, detections)
        self._frame_callbacks: List[Callable] = []
        self._status_callbacks: List[Callable] = []
        self._result_callbacks: List[Callable] = []
        self._last_context: Dict = {}
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir or "vision_lab_results"))
        self._threads: List[threading.Thread] = []
        self._own_rclpy = False
        self._started = False
        if start:
            self.start()

    # ------------------------------------------------------------------ life cycle
    def start(self) -> "VisionLab":
        if self._started:
            return self
        if not rclpy.ok():
            rclpy.init()
            self._own_rclpy = True
        self.world = self._resolve_world()
        os.environ[WORLD_ENV] = self.world.name              # every layout lookup below uses this world
        self.pipeline = DetectionPipeline(self._detector_spec, self.world, self._detector_options)
        if self._record_dir:
            self.recorder = YoloDatasetRecorder(self._record_dir, **self._record_options)
        self.arm = LabArm(self)
        self._apply_arm_params()
        self._check_no_duplicates()
        self.detector_node = LabDetectorNode(self, self.pipeline, self._image_topic, self._show_window,
                                             self._mask_arm, self.recorder)
        det_exec = SingleThreadedExecutor()                  # one thread: OpenCV windows want that
        det_exec.add_node(self.detector_node)
        arm_exec = MultiThreadedExecutor(num_threads=4)
        arm_exec.add_node(self.arm)
        for ex, name in ((det_exec, "vision_lab_detector"), (arm_exec, "vision_lab_arm")):
            t = threading.Thread(target=self._spin, args=(ex,), name=name, daemon=True)
            t.start()
            self._threads.append(t)
        self._executors = (det_exec, arm_exec)
        self._started = True
        self.log(f"Vision lab ready: world {self.world.key} ({self.world.name}), detector {self.pipeline.detector.name}, "
                 f"destinations {', '.join(self.world.destinations)}" + (" [DRY RUN]" if self._dry_run else ""))
        return self

    def _apply_arm_params(self):
        """vision_pick_place.launch.py's values, except where this world's layout already set one (as when
        the launch file runs the controller), then the caller's arm_params."""
        layout = WORLD_LAYOUTS.get(self.world.name, {})
        values = {k: v for k, v in ARM_DEFAULTS.items() if k not in layout}
        values.update(dry_run=self._dry_run, auto_start=False)
        values.update(self._arm_params)
        unknown = [k for k in values if not self.arm.has_parameter(k)]
        if unknown:
            raise ValueError(f"unknown vision_pick_place parameters: {unknown}")
        self.arm.set_parameters([Parameter(k, value=v) for k, v in values.items()])

    @staticmethod
    def _spin(executor):
        try:
            executor.spin()
        except Exception:
            pass

    def close(self):
        if not self._started:
            return
        if self.arm is not None and self.arm.busy:
            self.stop()
        if self.recorder is not None:
            self.log(self.recorder.close())
        for ex in self._executors:
            ex.shutdown(timeout_sec=1.0)
        for node in (self.detector_node, self.arm):
            if node is not None:
                node.destroy_node()
        if self._show_window:
            cv2.destroyAllWindows()
        if self._own_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._started = False

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    def _resolve_world(self) -> WorldProfile:
        running = self._running_world_name()
        if self._world_arg:
            w = get_world(self._world_arg)
            if running and running != w.name:
                raise RuntimeError(f"You asked for world '{w.key}' ({w.name}) but the simulation runs '{running}'. "
                                   f"Start the lab with --world {get_world(running).key}, or restart the simulation.")
            return w
        if running:
            return get_world(running)
        w = get_world(None)
        self.log(f"Could not ask the simulation which world runs (attacher not reachable); using {w.key}.")
        return w

    def _running_world_name(self) -> Optional[str]:
        node = rclpy.create_node("vision_lab_probe")
        try:
            cli = node.create_client(GetParameters, f"{ATTACHER_NODE}/get_parameters")
            if not cli.wait_for_service(timeout_sec=3.0):
                return None
            req = GetParameters.Request()
            req.names = ["world_name"]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            res = fut.result()
            return res.values[0].string_value if res and res.values and res.values[0].string_value else None
        finally:
            node.destroy_node()

    def _check_no_duplicates(self):
        time.sleep(0.5)
        names = [n for n, _ in self.arm.get_node_names_and_namespaces()]
        if names.count("vision_pick_place") > 1:
            self.log("WARNING: another vision_pick_place controller is running. Start the simulation with "
                     "'run_vision_lab.sh sim' (or run_controller:=false) so only the lab moves the arm.")
        if "ring_object_detector" in names:
            self.log("WARNING: ring_object_detector is running too and also publishes /vision/ring1_objects. "
                     "Start the simulation with 'run_vision_lab.sh sim' (or run_detector:=false).")

    def log(self, text: str):
        print(f"[vision_lab] {text}", flush=True)

    # ------------------------------------------------------------------ vision
    def _on_frame(self, bgr, dets):
        now = time.time()
        with self._lock:
            self._latest = (now, dets, bgr)
            self._history.append((now, dets))
            self._lock.notify_all()
        for cb in list(self._frame_callbacks):
            try:
                cb(bgr, dets)
            except Exception as exc:
                self.log(f"on_frame callback failed: {exc}")

    def on_frame(self, callback: Callable[[np.ndarray, List[ObjectDetection]], None]):
        """Call callback(bgr, detections) for every processed overhead frame (in the detector thread)."""
        self._frame_callbacks.append(callback)
        return callback

    def on_result(self, callback: Callable[[PickResult], None]):
        """Call callback(PickResult) after every delivered or skipped product."""
        self._result_callbacks.append(callback)
        return callback

    def on_status(self, callback: Callable[[str], None]):
        """Call callback(text) for every step of the pick-and-place (the /vision_pick_place/status texts)."""
        self._status_callbacks.append(callback)
        return callback

    def wait_for_frame(self, timeout: float = 10.0) -> bool:
        t0 = self._latest[0]
        with self._lock:
            return self._lock.wait_for(lambda: self._latest[0] > t0, timeout=timeout)

    def detections(self, bent: Optional[int] = None, label: Optional[str] = None, fresh: float = 1.0) -> List[ObjectDetection]:
        """Latest detections (world frame), optionally only one bent / one label. [] if the last frame
        is older than `fresh` seconds."""
        t, dets, _ = self._latest
        if time.time() - t > fresh:
            return []
        return [d for d in dets if (bent is None or d.bent == bent) and (label is None or d.label == label)]

    def latest_image(self) -> Optional[np.ndarray]:
        return self._latest[2]

    def detect_objects(self, bent: Optional[int] = None, label: Optional[str] = None) -> List[ObjectDetection]:
        """What the detector sees now (waits for a fresh frame first); same fields as detections()."""
        self.wait_for_frame(2.0)
        return self.detections(bent, label)

    def snapshot(self, name: Optional[str] = None) -> Optional[str]:
        """Save the last overhead image with the detections drawn on it into lab.output_dir; returns the path."""
        t, dets, bgr = self._latest
        if bgr is None:
            return None
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, name or time.strftime("snapshot_%H%M%S.png", time.localtime(t)))
        cv2.imwrite(path, self.pipeline.draw(bgr, dets))
        return path

    def start_recording(self, out_dir: str, **options) -> YoloDatasetRecorder:
        """Save auto-labelled YOLO training frames (see dataset.YoloDatasetRecorder) from now on."""
        self.recorder = YoloDatasetRecorder(out_dir, **options)
        return self.recorder

    def stop_recording(self) -> str:
        rec, self.recorder = self.recorder, None
        return rec.close() if rec else "not recording"

    def wait_for_object(self, bent: int = 1, label: Optional[str] = None, near=None, radius: float = 1.0,
                        timeout: float = 30.0) -> Optional[ObjectDetection]:
        """First detection on that bent (with that label, within radius of near=(x, y)) within timeout."""
        end = time.time() + timeout
        while time.time() < end:
            for d in self.detections(bent, label):
                if near is None or math.hypot(d.x - near[0], d.y - near[1]) <= radius:
                    return d
            self.wait_for_frame(min(1.0, max(0.05, end - time.time())))
        return None

    def detection_at_station(self, max_age: float = 4.0, radius: float = 0.6) -> Optional[ObjectDetection]:
        """The detector's best view of the product at the catch station in the last max_age seconds (the arm
        may hide it from the overhead camera when it comes down)."""
        sx, sy = self.world.station
        now, best, bd = time.time(), None, radius
        with self._lock:
            hist = list(self._history)
        for t, dets in reversed(hist):
            if now - t > max_age:
                break
            for d in dets:
                if d.bent != 1:
                    continue
                dist = math.hypot(d.x - sx, d.y - sy)
                if dist < bd - 1e-9 or (best is not None and dist <= bd + 0.05 and d.score > best.score):
                    best, bd = d, min(bd, dist)
            if best is not None and bd < 0.2:
                break
        return best

    def _context(self, number, model) -> PickContext:
        return PickContext(number=number, detection=self.detection_at_station(), model=model,
                           product_type=type_of(model), world=self.world, lab=self)

    # ------------------------------------------------------------------ arm
    def _session(self, count: Optional[int], timeout: Optional[float]) -> List[PickResult]:
        if self.arm.busy:
            raise RuntimeError("the arm is already running a session (call lab.stop() first)")
        start = len(self.arm.results)
        err: List[BaseException] = []

        def work():
            try:
                self.arm.run_session(count)
            except BaseException as exc:          # noqa: B902 - handed to the caller
                err.append(exc)

        t = threading.Thread(target=work, name="vision_lab_session", daemon=True)
        t.start()
        try:
            t.join(timeout)
            if t.is_alive():
                self.log(f"Session timed out after {timeout:.0f} s; stopping the arm.")
                self.stop()
                t.join(30.0)
        except KeyboardInterrupt:
            self.log("Ctrl+C: stopping the arm after the current motion ...")
            self.stop()
            t.join(30.0)
            raise
        finally:
            self.arm.policy = None
            self.arm.forced = None
        if err:
            raise err[0]
        return self.arm.results[start:]

    def transfer(self, to: str = "bent2", timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Catch the next product that reaches the catch station on bent 1 and take it `to`:
        "bent2" (bent to bent), "trash" or "recycling". Returns its PickResult (None on timeout)."""
        dest = normalize_destination(to)
        if dest == "skip":
            raise ValueError("transfer(to='skip') would never pick anything")
        if dest == "bent2" and not self.world.has_bent2:
            raise ValueError(f"world {self.world.key} has no bent 2; use 'trash' or 'recycling'")
        self.arm.forced = dest
        done = [r for r in self._session(1, timeout) if r.status == "delivered"]
        return done[-1] if done else None

    def move_to_bent2(self, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 onto bent 2."""
        return self.transfer("bent2", timeout)

    def put_in_trash(self, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 into the trash bin (left of the arm)."""
        return self.transfer("trash", timeout)

    def drop_in_recycle_bin(self, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 into the recycling bin (right of the arm)."""
        return self.transfer("recycling", timeout)

    # the same three moves under the names the lab files use
    def move_to_bent(self, bent: int = 2, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 onto bent `bent` (only 2 exists: the arm picks from bent 1)."""
        if bent != 2:
            raise ValueError("the arm picks from bent 1 and can place only on bent 2")
        return self.transfer("bent2", timeout)

    def move_to_trash(self, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 into the trash can."""
        return self.transfer("trash", timeout)

    def move_to_recycle_bin(self, timeout: Optional[float] = 120.0) -> Optional[PickResult]:
        """Next product from bent 1 into the recycling bin."""
        return self.transfer("recycling", timeout)

    def run(self, policy: Union[Callable[[PickContext], str], Dict[str, str], str, None] = None,
            count: Optional[int] = None, timeout: Optional[float] = None, by: str = "label") -> List[PickResult]:
        """
        Keep catching products from bent 1 and let `policy` decide where each goes.
          policy: a function (PickContext) -> "bent2" | "trash" | "recycling" | "skip",
                  a dict {label: destination, "*": default} matched against the detector's label
                  (by="type": against the simulation's true product type instead),
                  one destination for every product, or
                  None / "gui": the order GUI's destinations, as run_factory.sh does.
          count:  stop after this many delivered products (None = until timeout / stop() / Ctrl+C)
        Returns the PickResults of this run (delivered and skipped).
        """
        if policy is None or policy == "gui":
            self.arm.policy = self.arm.forced = None
        elif isinstance(policy, str):
            self.arm.forced = normalize_destination(policy)
        elif isinstance(policy, dict):
            table = {k: normalize_destination(v) for k, v in policy.items()}
            default = table.get("*", "bent2" if self.world.has_bent2 else "skip")

            def table_policy(ctx: PickContext) -> str:
                key = ctx.product_type if by == "type" else ctx.label
                return table.get(key, default)
            self.arm.policy = table_policy
        elif callable(policy):
            self.arm.policy = policy
        else:
            raise TypeError("policy must be a function, a dict, a destination name or None")
        return self._session(count, timeout)

    def stop(self):
        """Stop the arm after its current motion (the product it holds is let go where it is safe)."""
        if self.arm is not None:
            self.arm.stop_flag = True

    def home(self, duration: float = 3.0):
        """Move the arm to its start position."""
        self.arm.prepare()
        self.arm.stop_flag = False
        self.arm.move(self.arm.home_pose(), duration, "vision lab: home")

    def set_motion(self, motion_type: str):
        """auto, cycle, random, standard, mirror, high_carry, low_fast, full_turn, reverse_return."""
        self.arm.set_parameters([Parameter("motion_type", value=str(motion_type))])

    def set_arm_param(self, name: str, value):
        """Change any vision_pick_place parameter (e.g. "swing_duration", 1.5)."""
        self.arm.set_parameters([Parameter(name, value=value)])

    @property
    def results(self) -> List[PickResult]:
        return list(self.arm.results)

    def report(self, results: Optional[List[PickResult]] = None) -> str:
        """Summary of the picks: where they went and how often the detector's label matched the truth."""
        rs = self.results if results is None else results
        by_dest: Dict[str, int] = {}
        for r in rs:
            by_dest[r.destination] = by_dest.get(r.destination, 0) + 1
        judged = [r.label_correct for r in rs if r.label_correct is not None]
        acc = f"{sum(judged)}/{len(judged)} = {100.0 * sum(judged) / len(judged):.0f}%" if judged else "n/a"
        lines = [f"{len(rs)} products: " + ", ".join(f"{k} {v}" for k, v in sorted(by_dest.items())),
                 f"detector label correct (vs simulation truth): {acc}"]
        for r in rs:
            mark = {True: "ok", False: "WRONG", None: "--"}[r.label_correct]
            lines.append(f"  #{r.number}: {r.status:9s} -> {r.destination:9s} label={r.label} "
                         f"({'' if r.score is None else f'{r.score:.2f}'}) truth={r.product_type} [{mark}]")
        return "\n".join(lines)

    # ------------------------------------------------------------------ belts and products
    def _trigger(self, service: str, timeout: float = 5.0) -> str:
        cli = self.arm.create_client(Trigger, service)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"{service} not available (is the attacher running?)")
            fut = cli.call_async(Trigger.Request())
            end = time.time() + timeout
            while not fut.done() and time.time() < end:
                time.sleep(0.05)
            res = fut.result() if fut.done() else None
            if res is None or not res.success:
                raise RuntimeError(f"{service}: {res.message if res else 'no answer'}")
            return res.message
        finally:
            self.arm.destroy_client(cli)

    def stop_bent(self, bent: int = 1) -> str:
        return self._trigger("/stop_bent2" if bent == 2 else "/stop_bent")

    def start_bent(self, bent: int = 1) -> str:
        return self._trigger("/start_bent2" if bent == 2 else "/start_bent")

    def spawn_products(self, items: List[str], bent: int = 1, spacing: float = 1.0, start_offset: float = 1.5,
                       clear: bool = True, timeout: float = 30.0) -> str:
        """Put a row of products on a bent (as the order GUI does): items are product types
        (product_catalog.PRODUCT_TYPES), the first one start_offset m before the catch station."""
        bad = [i for i in items if i not in PRODUCT_TYPES]
        if bad:
            raise ValueError(f"unknown product types {bad}; known: {', '.join(PRODUCT_TYPES)}")
        if bent == 2 and not self.world.has_bent2:
            raise ValueError(f"world {self.world.key} has no bent 2")
        pub = self.arm.create_publisher(String, "/spawn_order", 10)
        done = threading.Event()
        reply = {}

        def on_result(msg):
            reply.update(json.loads(msg.data))
            done.set()
        sub = self.arm.create_subscription(String, "/spawn_order/result", on_result, 10)
        try:
            end = time.time() + 5.0
            while pub.get_subscription_count() == 0 and time.time() < end:
                time.sleep(0.1)
            if pub.get_subscription_count() == 0:
                raise RuntimeError("nobody listens on /spawn_order (is bcr_endpoint_object_attacher running?)")
            pub.publish(String(data=json.dumps({"items": list(items), "bent": bent, "spacing": spacing,
                                                "start_offset": start_offset, "clear": clear})))
            if not done.wait(timeout):
                raise RuntimeError("no /spawn_order/result within the timeout")
            if not reply.get("ok"):
                raise RuntimeError(reply.get("message", "spawn failed"))
            return reply.get("message", "")
        finally:
            self.arm.destroy_subscription(sub)
            self.arm.destroy_publisher(pub)

    def wait_for_products(self, timeout: float = 120.0) -> bool:
        """True once the start-up order of run_factory.sh / run_vision_lab.sh is on the belt (the attacher
        then calls /vision_pick_place/start, which the lab notes instead of starting the old loop)."""
        return self.arm.products_ready.wait(timeout)
