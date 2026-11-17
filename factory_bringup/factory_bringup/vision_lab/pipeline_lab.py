"""
PipelineLab: the vision lab in the pipeline world (pipeline_factory_lab.sdf) and in the tree-of-three
world (pipeline_tree_factory_lab.sdf, --world pipeline-tree): 13 conveyors, 12 BCR arm stations
A1 ... A12, each with its own camera, recycling bin and trash bin.

It has the same calls as VisionLab, so a lab file runs in every world. In the pipeline they act on
one station: `station=` picks it, otherwise lab.station (the --station option, default A1).
At a station, bent 1 is the conveyor the arm picks from and bent 2 the conveyor it places on:

    lab.detect_objects(bent=1, station="A2")      # what A2's camera sees on its pick conveyor (C1)
    lab.move_to_bent(2, station="A2")             # next product reaching A2: C1 -> S2
    lab.move_to_trash(station="A6")               # next product reaching A6 into A6's trash bin
    lab.spawn_products(["paint_can", ...], conveyor="S1")
    lab.run(stations="all")                       # every arm sorts by its rule in pipeline_layout.py,
                                                  #   using the detector's labels and sizes
    lab.run(decide, stations=["A2", "A3"])        # your own decide(ctx); ctx.station says which arm asks
    lab.set_scheduler("rendezvous")               # belts never stop: arms meet products on the moving belt
                                                  #   (rendezvous.py; or --arm schedule=rendezvous)

Backends: "gz" drives the running Gazebo (run_vision_lab.sh sim --world pipeline), "fake" runs the
same engine without Gazebo and draws the camera images (vision_lab run LAB --world pipeline --fake).
No ROS is needed for either.
"""

import copy
import math
import os
import random
import threading
import time
from collections import deque
from typing import Callable, Dict, Iterable, List, Optional, Union

import cv2
import numpy as np

from factory_bringup import pipeline_layout as L
from factory_bringup.product_catalog import BIN_SIZE, PRODUCT_TYPES, RECYCLABLE, type_of
from factory_bringup.vision_geometry import arm_mask_from_points
from factory_bringup.vision_lab import pipeline_world as P
from factory_bringup.vision_lab.common import DESTINATIONS, PickContext, PickResult, normalize_destination
from factory_bringup.vision_lab.dataset import YoloDatasetRecorder
from factory_bringup.vision_lab.detectors.base import Detector, ObjectDetection
from factory_bringup.vision_lab.pipeline import DetectionPipeline
from factory_bringup.vision_lab.pipeline_sim import BELT_TOP, FakeBackend, GzBackend, PipelineSim
from factory_bringup.vision_lab.rendezvous import (ASSIGN_MODES, POOL_MODES, SCHEDULES, RendezvousScheduler,
                                                   offset as rv_offset, track_from_detection, track_from_product, window)
from factory_bringup.vision_lab.worlds import WorldProfile

# product types a "cardboard" rule takes (A6), and the size limits of "large" (A9) / "small" (A11)
CARDBOARD = {"shipping_box_large", "cereal_box", "shoe_box", "long_package", "pickup_box", "pizza_box", "toy_box",
             "gift_box", "tissue_box", "laptop_box", "kettle_box", "electronics_box", "book_parcel", "milk_carton"}
LARGE_M, SMALL_M = 0.33, 0.22


class MissedGrasp(RuntimeError):
    pass


class _SkipList:
    """Products an arm leaves alone: for good (missed / unreachable), or for a while after decide()
    said "skip" (it has passed the arm by then; on a ring it is judged again on its next lap, which
    also corrects a skip that went to the wrong product when two stood close together)."""

    def __init__(self, hold_sec: float = 8.0):
        self.until: Dict[str, float] = {}
        self.hold_sec = hold_sec
        self.recorded: set = set()

    def add(self, name: str, forever: bool = True):
        self.until[name] = math.inf if forever else time.time() + self.hold_sec

    def __contains__(self, name: str) -> bool:
        return self.until.get(name, 0.0) > time.time()


class _Station:
    def __init__(self, lab: "PipelineLab", arm: str, detector):
        self.arm = arm
        self.profile: WorldProfile = P.station_profile(arm)
        self.pipe = DetectionPipeline(detector, self.profile, lab._detector_options)
        self.raw = None                       # (bgr, t) newest camera frame
        self.latest = (0.0, [], None)         # (t, detections, bgr) newest processed frame
        self.history: deque = deque(maxlen=60)
        self.cond = threading.Condition()
        self.active = False
        self.number = 0
        self.view = None


class PipelineLab:
    def __init__(self, station: str = "A1", detector="opencv_contours", detector_options: Optional[Dict] = None,
                 backend: str = "gz", show_window: bool = True, record_dataset: Optional[str] = None,
                 record_options: Optional[Dict] = None, output_dir: Optional[str] = None,
                 belt_speed: float = L.BELT_SPEED, arm_params: Optional[Dict] = None, start: bool = True,
                 feed: bool = True, feed_items: Optional[List[str]] = None, seed: Optional[int] = None,
                 lazy_cameras: Optional[bool] = None, **_ignored):
        self.station = station.upper()
        if self.station not in P.STATIONS:
            raise ValueError(f"unknown station {station}; stations: {', '.join(P.STATIONS)}")
        self._detector_spec, self._detector_options = detector, dict(detector_options or {})
        self._backend_name = backend
        self._show_window = show_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir or "vision_lab_results"))
        self.recorder = YoloDatasetRecorder(record_dataset, **(record_options or {})) if record_dataset else None
        self.timing = {"move_duration": 0.9, "swing_duration": 2.5, "grasp_tolerance": 0.10, "roi_radius": 0.45,
                       "place_wait_sec": 30.0,
                       "pause_belt": "grasp",
                       "locate": os.environ.get("PIPELINE_LOCATE", "assist"),
                       # schedule: classic     an arm holds the product it picks and its belt stands still
                       #                       (pause_belt) while it grasps
                       #           rendezvous  belts never stop: every arm plans where and when it meets a
                       #                       product on the moving belt, a scheduler spreads the products
                       #                       of shared conveyors over their arms (rendezvous.py)
                       "schedule": os.environ.get("PIPELINE_SCHEDULE", "classic").strip().lower(),
                       "rendezvous_assign": os.environ.get("RENDEZVOUS_ASSIGN", "balanced").strip().lower(),
                       "rendezvous_pool": os.environ.get("RENDEZVOUS_POOL", "rules").strip().lower(),
                       "rendezvous_window": 0.8}
        # locate: how an arm finds the product it grasps
        #   vision  only the camera (a product the detector misses is not picked; a position more than
        #           grasp_tolerance off is a missed grasp)
        #   assist  the camera first; when it has not seen a product that has reached the arm for a
        #           second, or its position is too far off, the arm uses the simulation's position
        #           (counted and logged as "assist", so you see how good the vision is)
        #   truth   the simulation's position and type only (tests the arms and belts without vision)   # grasp: a belt stands still while an arm grasps from or places
                                                # on it; cycle: the pick belt stands for the whole pick+deliver; off
        self.timing.update({k: v for k, v in (arm_params or {}).items() if k in self.timing})
        self.rendezvous: Optional[RendezvousScheduler] = None
        self.set_scheduler(self.timing["schedule"], quiet=True)
        self.belt_speed = belt_speed
        self.sim: Optional[PipelineSim] = None
        self.stations: Dict[str, _Station] = {}
        self._results: List[PickResult] = []
        self._result_callbacks: List[Callable] = []
        self._frame_callbacks: List[Callable] = []
        self._status_callbacks: List[Callable] = []
        self._stop = threading.Event()
        self._arm_locks = {a: threading.Lock() for a in P.STATIONS}
        self._threads: List[threading.Thread] = []
        self._closed = threading.Event()
        self._started = False
        # the feeder keeps products coming to the conveyors the working arms pick from (the pipeline world
        # has no order feeder of its own); spawn_products() switches it off
        self.feed = feed
        self.feed_items = list(feed_items or PRODUCT_TYPES)
        self._feed_targets: set = set()
        self._rng = random.Random(seed)
        # lazy cameras: a station's detector skips frames while nothing is coming to its arm (judged from
        # the engine's product positions; it only decides WHEN to run the detector, never what it sees).
        # PIPELINE_LAZY_CAMERAS=0 turns it off.
        if lazy_cameras is None:
            lazy_cameras = os.environ.get("PIPELINE_LAZY_CAMERAS", "1").strip().lower() not in ("0", "false", "no", "off")
        self.lazy_cameras = lazy_cameras
        self._size_warned: set = set()
        self.assists: Dict[str, int] = {}                 # station -> picks the simulation's position helped
        self._near_since: Dict[str, float] = {}            # product -> since when it waits at an arm unseen
        self._frames: Dict[str, int] = {}                  # station -> camera frames received (health log)
        self._ndets: Dict[str, int] = {}                   # station -> detector runs that found something
        if start:
            self.start()

    # ------------------------------------------------------------------ life cycle
    def start(self) -> "PipelineLab":
        if self._started:
            return self
        backend = FakeBackend() if self._backend_name == "fake" else GzBackend()
        if not backend.running():
            raise RuntimeError(f"The {P.WORLD_KEY} lab world ({P.WORLD_NAME}) is not running in Gazebo. Start it with\n"
                               f"  ./run_vision_lab.sh sim --world {P.WORLD_KEY} --drawn-cameras\n"
                               "or try the lab without Gazebo: --fake")
        self.sim = PipelineSim(backend, self.belt_speed)
        self.sim.log = self.log
        for a in P.STATIONS:
            det = self._detector_spec
            if isinstance(det, Detector) and a != self.station:
                det = copy.deepcopy(det)
            self.stations[a] = _Station(self, a, det)
            if isinstance(backend, GzBackend):
                backend.subscribe_camera(P.camera_topic(a), self._camera_cb(a), wanted=self._frame_wanted(a))
            else:
                backend.subscribe_camera(P.camera_topic(a), self._camera_cb(a))
        self.sim.start()
        if hasattr(backend, "ensure_cameras"):            # Gazebo: images through the bindings or the ROS bridge
            try:
                backend.ensure_cameras(log=self.log)
            except RuntimeError as exc:
                self.log(str(exc))
        self.activate(self.station)
        self._spawn(self._feed_loop, "pipeline_lab_feeder")
        self._spawn(self._health_loop, "pipeline_lab_health")
        if self._show_window:
            self._spawn(self._window_loop, "pipeline_lab_window")
        self._started = True
        self.log(f"Pipeline lab ready ({'no Gazebo, drawn cameras' if self._backend_name == 'fake' else 'Gazebo'}): "
                 f"station {self.station} ({P.station(self.station)['pick_conv']} -> {P.station(self.station)['place_conv']}), "
                 f"detector {self.stations[self.station].pipe.detector.name}")
        self._log_scheduler()
        return self

    def close(self):
        if not self._started:
            return
        self.stop()
        self._closed.set()
        if self.recorder is not None:
            self.log(self.recorder.close())
        self.sim.stop()
        for t in self._threads:                       # detector / window / feeder threads end with _closed
            if t is not threading.current_thread():
                t.join(3.0)
        if self._show_window:
            cv2.destroyAllWindows()
        self._started = False

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    def _spawn(self, fn, name, *args):
        t = threading.Thread(target=fn, args=args, name=name, daemon=True)
        t.start()
        self._threads.append(t)
        return t

    def log(self, text: str):
        print(f"[vision_lab] {text}", flush=True)
        try:                                               # also into <results>/lab.log, for later reading
            os.makedirs(self.output_dir, exist_ok=True)
            with open(os.path.join(self.output_dir, "lab.log"), "a") as f:
                f.write(time.strftime("%H:%M:%S ") + text + "\n")
        except Exception:
            pass
        for cb in list(self._status_callbacks):
            try:
                cb(text)
            except Exception:
                pass

    # ------------------------------------------------------------------ worlds / stations
    @property
    def world(self) -> WorldProfile:
        """The default station's view of the pipeline (has_bent2, bins, lanes ...)."""
        return self.stations[self.station].profile if self.stations else P.station_profile(self.station)

    @property
    def all_stations(self) -> List[str]:
        return list(P.STATIONS)

    def _st(self, station: Optional[str]) -> _Station:
        name = (station or self.station).upper()
        if name not in self.stations:
            raise ValueError(f"unknown station {station}; stations: {', '.join(P.STATIONS)}")
        return self.stations[name]

    def activate(self, *stations: str):
        """Run the detector on these stations' cameras (only active stations cost CPU)."""
        for s in stations:
            st = self._st(s)
            if not st.active:
                st.active = True
                if hasattr(self.sim.backend, "active"):        # fake cameras: draw only what is watched
                    self.sim.backend.active = {a for a, x in self.stations.items() if x.active}
                self._spawn(self._detect_loop, f"detect_{st.arm}", st)

    # ------------------------------------------------------------------ vision
    def _camera_cb(self, arm: str):
        want_w, want_h = P.station_profile(arm).camera_size

        def cb(bgr, t):
            st = self.stations[arm]
            self._frames[arm] = self._frames.get(arm, 0) + 1
            h, w = bgr.shape[:2]
            if (w, h) != (want_w, want_h):                 # world built with another PIPELINE_CAMERA setting:
                if arm not in self._size_warned:           # scale to the size the geometry expects
                    self._size_warned.add(arm)
                    self.log(f"{arm}: camera sends {w}x{h}, the lab expects {want_w}x{want_h} (PIPELINE_CAMERA); "
                             "scaling the images. Use the same PIPELINE_CAMERA in both terminals.")
                bgr = cv2.resize(bgr, (want_w, want_h), interpolation=cv2.INTER_LINEAR)
            with st.cond:
                st.raw = (bgr, t)
                st.cond.notify_all()
        return cb

    def _arm_mask(self, st: _Station):
        cam = st.pipe.ctx.camera if st.pipe.ctx else None
        if cam is None:
            return None
        cx, cy = st.profile.camera_pose["x"], st.profile.camera_pose["y"]
        mask = None
        for a, arm in self.sim.arms.items():
            if math.hypot(arm.base[0] - cx, arm.base[1] - cy) > 8.0:
                continue
            m = arm_mask_from_points(cam, self.sim.arm_points(a), 0.40)
            mask = m if mask is None else (mask | m)
        return mask

    def _detect_loop(self, st: _Station):
        last = 0.0
        while not self._closed.is_set():
            with st.cond:
                st.cond.wait_for(lambda: (st.raw is not None and st.raw[1] > last) or self._closed.is_set(), timeout=1.0)
                if st.raw is None or st.raw[1] <= last:
                    continue
                bgr, t = st.raw
            last = t
            if self.lazy_cameras and st.pipe.ctx is not None and not self._needs_camera(st.arm):
                with st.cond:                                  # nothing near this arm: no detector run
                    st.latest = (t, [], bgr)
                    st.cond.notify_all()
                continue
            try:
                if st.pipe.ctx is None:
                    st.pipe.process(bgr)                       # builds the lane masks
                mask = self._arm_mask(st)
                dets = st.pipe.process(bgr, exclude_mask=mask)
            except Exception as exc:
                self.log(f"{st.arm}: detector failed: {exc}")
                time.sleep(1.0)
                continue
            now = t                                            # capture time of the frame
            if dets:
                self._ndets[st.arm] = self._ndets.get(st.arm, 0) + 1
            with st.cond:
                st.latest = (now, dets, bgr)
                st.history.append((now, dets))
                st.cond.notify_all()
            if self.recorder is not None:
                self.recorder.add(bgr, dets)
            for cb in list(self._frame_callbacks):
                try:
                    cb(bgr, dets, st.arm)
                except TypeError:
                    cb(bgr, dets)
                except Exception as exc:
                    self.log(f"on_frame callback failed: {exc}")
            if self._show_window:
                st.view = (bgr, dets, mask)

    def _health_loop(self, period: float = 30.0):
        """Every `period` s, one line per working station: camera frames, detector hits, products at the arm."""
        last: Dict[str, int] = {}
        while not self._closed.wait(period):
            parts = []
            for a, st in self.stations.items():
                if not st.active:
                    continue
                n = self._frames.get(a, 0)
                near = sum(1 for p in self.sim.on_conveyor(P.station(a)["pick_conv"])
                           if math.hypot(p.x - P.station(a)["pick"][0], p.y - P.station(a)["pick"][1]) < 1.0)
                parts.append(f"{a} frames+{n - last.get(a, 0)} hits {self._ndets.get(a, 0)} at-arm {near}"
                             + (f" assists {self.assists[a]}" if self.assists.get(a) else ""))
                last[a] = n
            if parts:
                self.log("health: " + "; ".join(parts))

    def _frame_wanted(self, arm: str, idle_period: float = 2.0):
        """For the Gazebo camera callback: take every frame of a busy station, one every idle_period s
        of an idle one (so its picture stays fresh), and none of a station whose detector is off."""
        last = [0.0]

        def wanted() -> bool:
            st = self.stations.get(arm)
            if st is None or not st.active:
                return st is not None and st.raw is None          # one frame, then only when activated
            now = time.time()
            if (not self.lazy_cameras or st.pipe.ctx is None or self._needs_camera(arm)
                    or now - last[0] > idle_period):
                last[0] = now
                return True
            return False
        return wanted

    def _needs_camera(self, arm: str, reach: float = 4.5) -> bool:
        """Is anything happening at this arm: a product on its pick belt within `reach` m of the pick point
        (about the camera's view), or the arm carrying one?"""
        if self.sim is None:
            return True
        a = self.sim.arms.get(arm)
        if a is not None and a.carrying:
            return True
        st = P.station(arm)
        px, py = st["pick"]
        return any(math.hypot(p.x - px, p.y - py) < reach for p in self.sim.on_conveyor(st["pick_conv"]))

    def _window_loop(self):
        win = "Vision Lab - pipeline"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 720)
        while not self._closed.is_set():
            act = [s for s in self.stations.values() if s.active and s.view is not None]
            if act:
                tiles = []
                for s in act:
                    bgr, dets, mask = s.view
                    v = s.pipe.draw(bgr, dets, status=f"station {s.arm}", arm_mask=mask, thumbnails=False)
                    tiles.append(cv2.resize(v, (480, 270)))
                cols = 1 if len(tiles) == 1 else (2 if len(tiles) <= 4 else 3)
                while len(tiles) % cols:
                    tiles.append(np.zeros_like(tiles[0]))
                grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
                cv2.imshow(win, grid)
            for _ in range(10):                            # keep the window answering: pump events often
                cv2.waitKey(50)
                if self._closed.is_set():
                    break
        cv2.destroyWindow(win)

    def wait_for_frame(self, timeout: float = 10.0, station: Optional[str] = None) -> bool:
        st = self._st(station)
        self.activate(st.arm)
        t0 = st.latest[0]
        with st.cond:
            return st.cond.wait_for(lambda: st.latest[0] > t0, timeout=timeout)

    def detections(self, bent: Optional[int] = None, label: Optional[str] = None, fresh: float = 2.0,
                   station: Optional[str] = None) -> List[ObjectDetection]:
        st = self._st(station)
        self.activate(st.arm)
        t, dets, _ = st.latest
        if time.time() - t > fresh:
            return []
        return [d for d in dets if (bent is None or d.bent == bent) and (label is None or d.label == label)]

    def detect_objects(self, bent: Optional[int] = None, label: Optional[str] = None,
                       station: Optional[str] = None) -> List[ObjectDetection]:
        self.wait_for_frame(3.0, station)
        return self.detections(bent, label, station=station)

    def latest_image(self, station: Optional[str] = None) -> Optional[np.ndarray]:
        return self._st(station).latest[2]

    def wait_for_object(self, bent: int = 1, label: Optional[str] = None, near=None, radius: float = 1.0,
                        timeout: float = 30.0, station: Optional[str] = None) -> Optional[ObjectDetection]:
        end = time.time() + timeout
        while time.time() < end:
            for d in self.detections(bent, label, station=station):
                if near is None or math.hypot(d.x - near[0], d.y - near[1]) <= radius:
                    return d
            self.wait_for_frame(min(1.0, max(0.05, end - time.time())), station)
        return None

    def detection_at_station(self, max_age: float = 4.0, radius: float = 0.6,
                             station: Optional[str] = None) -> Optional[ObjectDetection]:
        """The detector's best recent view of a product at the station's pick point."""
        st = self._st(station)
        sx, sy = P.station(st.arm)["pick"]
        now, best, bd = time.time(), None, radius
        with st.cond:
            hist = list(st.history)
        for t, dets in reversed(hist):
            if now - t > max_age:
                break
            for d in dets:
                dist = math.hypot(d.x - sx, d.y - sy)
                if d.bent == 1 and dist < bd:
                    best, bd = d, dist
        return best

    def snapshot(self, name: Optional[str] = None, station: Optional[str] = None) -> Optional[str]:
        st = self._st(station)
        t, dets, bgr = st.latest
        if bgr is None:
            return None
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, name or time.strftime(f"{st.arm}_%H%M%S.png", time.localtime(t)))
        cv2.imwrite(path, st.pipe.draw(bgr, dets, status=f"station {st.arm}"))
        return path

    def on_frame(self, callback):
        """callback(bgr, detections[, station]) for every processed frame of an active station."""
        self._frame_callbacks.append(callback)
        return callback

    def on_result(self, callback):
        self._result_callbacks.append(callback)
        return callback

    def on_status(self, callback):
        self._status_callbacks.append(callback)
        return callback

    def start_recording(self, out_dir: str, **options) -> YoloDatasetRecorder:
        self.recorder = YoloDatasetRecorder(out_dir, **options)
        return self.recorder

    def stop_recording(self) -> str:
        rec, self.recorder = self.recorder, None
        return rec.close() if rec else "not recording"

    # ------------------------------------------------------------------ belts and products
    def _conv(self, bent: int, station: Optional[str]) -> str:
        s = P.station(self._st(station).arm)
        return s["pick_conv"] if bent == 1 else s["place_conv"]

    def spawn_products(self, items: List[str], bent: int = 1, spacing: float = 1.0, start_offset: float = 1.5,
                       clear: bool = False, conveyor: Optional[str] = None, station: Optional[str] = None,
                       timeout: float = 30.0) -> str:
        """Put products on a conveyor: by name (S1 ... S9, C1 ... C4), or bent 1 / 2 of a station; the first
        one start_offset m before that station's pick point (or at the start of the conveyor)."""
        bad = [i for i in items if i not in PRODUCT_TYPES]
        if bad:
            raise ValueError(f"unknown product types {bad}")
        conv = (conveyor or self._conv(bent, station)).upper()
        if conv not in L.STRAIGHTS and conv not in L.CIRCLES:
            raise ValueError(f"unknown conveyor {conveyor}")
        if clear:
            for p in list(self.sim.on_conveyor(conv)):
                self.sim.remove(p.name)
        start_s = None
        if conveyor is None:
            st = P.station(self._st(station).arm)
            pt = st["pick"] if bent == 1 else st["place"]
            sign = P.ring_dir(conv) if P.is_ring(conv) else 1
            start_s = P.path_s(conv, *pt) - sign * start_offset
        if self.feed:
            self.feed = False
            self.log("spawn_products: you feed the conveyors yourself now (the automatic feeder is off)")
        names = self.sim.spawn_row(list(items), conv, spacing, start_s)
        return f"{len(names)} products on {conv}"

    def stop_bent(self, bent: int = 1, station: Optional[str] = None, conveyor: Optional[str] = None) -> str:
        conv = (conveyor or self._conv(bent, station)).upper()
        self.sim.running_belts[conv] = False
        return f"{conv} stopped"

    def start_bent(self, bent: int = 1, station: Optional[str] = None, conveyor: Optional[str] = None) -> str:
        conv = (conveyor or self._conv(bent, station)).upper()
        self.sim.running_belts[conv] = True
        return f"{conv} running"

    def wait_for_products(self, timeout: float = 0.0) -> bool:
        return True

    def summary(self) -> Dict[str, int]:
        """Products per conveyor, bin and dock."""
        return self.sim.summary()

    def _feeding(self, stations: Iterable[str]):
        """Feed the pick conveyors of these stations that no other of them places onto (all: just S1)."""
        names = [s.upper() for s in stations]
        placed = {P.station(a)["place_conv"] for a in names}
        self._feed_targets = {P.station(a)["pick_conv"] for a in names} - placed

    def _feed_loop(self, keep: int = 3, min_gap: float = 1.2):
        while not self._closed.is_set():
            time.sleep(1.0)
            if not self.feed:
                continue
            for conv in list(self._feed_targets):
                prods = self.sim.on_conveyor(conv)
                if len(prods) >= keep or not self.sim.belt_moving(conv):    # no spawning while an arm is busy
                    continue
                ring = P.is_ring(conv)
                if ring:
                    feed = [pl for _b, _pk, (c, pl), _r in L.ARMS.values() if c == conv]
                    s0 = (P.path_s(conv, *feed[0]) if feed else 0.0) + P.ring_dir(conv) * 0.6
                    circ = P.length(conv)
                    near = [abs((p.s - s0 + circ / 2) % circ - circ / 2) for p in prods]
                else:
                    s0 = 0.4
                    near = [abs(p.s - s0) for p in prods]
                if all(d > min_gap for d in near):
                    try:
                        self.sim.spawn_row([self._rng.choice(self.feed_items)], conv, 1.0, s0)
                    except Exception as exc:             # keep feeding: log, wait, try again
                        self.log(f"feeder: {exc}")
                        time.sleep(3.0)

    # ------------------------------------------------------------------ arm
    def _move(self, arm: str, xyz, duration: float, seeds=None):
        q = self.sim.ik(arm, xyz, [self.sim.arms[arm].q] + list(seeds or []))
        self.sim.move_arm(arm, q, duration, self._stop)
        return q

    def _bin_ik(self, arm: str, x: float, y: float, z: float):
        bx, by = self.sim.arms[arm].base
        d = math.hypot(x - bx, y - by)
        ux, uy = (bx - x) / max(d, 1e-6), (by - y) / max(d, 1e-6)
        last = None
        for dz in (0.0, 0.15, 0.3):
            for shift in (0.0, 0.1, 0.2, 0.3):
                try:
                    return self.sim.ik(arm, (x + ux * shift, y + uy * shift, z + dz), [self.sim.arms[arm].q])
                except RuntimeError as exc:
                    last = exc
        raise last

    def home(self, station: Optional[str] = None, duration: float = 2.0):
        arm = self._st(station).arm
        self.sim.move_arm(arm, self.sim.wait_posture(arm), duration, self._stop)

    def _approaching(self, st: _Station, skip: "_SkipList"):
        """(detection, product, frame time) of a product the camera sees close to the pick point."""
        px, py = P.station(st.arm)["pick"]
        best, bd = None, self.timing["roi_radius"]
        t = st.latest[0]
        for d in self.detections(1, station=st.arm, fresh=1.5):
            if math.hypot(d.x - px, d.y - py) > 2.0:
                continue
            x, y = self._predict(st, d, t)              # where it is now: frames can be far apart
            dist = math.hypot(x - px, y - py)
            if dist <= bd:
                prod = self.sim.nearest(P.station(st.arm)["pick_conv"], x, y, 0.4)
                if prod is not None and prod.name not in skip:
                    best, bd = (d, prod, t), dist
        locate = self.timing.get("locate", "assist")
        if locate == "vision" or (best is not None and locate != "truth"):
            if best is not None:
                self._near_since.pop(best[1].name, None)
            return best
        return self._unseen_at_arm(st, skip, wait=0.0 if locate == "truth" else 1.0)

    def _unseen_at_arm(self, st: _Station, skip: "_SkipList", wait: float):
        """locate=assist / truth: a product that has reached the arm's pick point (simulation truth) and
        that the camera has not reported for `wait` s, as a detection at its true position."""
        conv = P.station(st.arm)["pick_conv"]
        px, py = P.station(st.arm)["pick"]
        now = time.time()
        best, bd = None, self.timing["roi_radius"]
        for p in self.sim.on_conveyor(conv):
            if p.name in skip:
                continue
            dist = math.hypot(p.x - px, p.y - py)
            if dist > bd:
                continue
            first = self._near_since.setdefault(p.name, now)
            if now - first >= wait:
                best, bd = p, dist
        if best is None:
            return None
        sx, sy, _ = best.size
        label = best.ptype if self.timing.get("locate") == "truth" else "unknown"
        det = ObjectDetection(u=0.0, v=0.0, label=label, score=0.0, x=best.x, y=best.y, size_x=sx, size_y=sy, bent=1)
        det.assisted = True
        return det, best, now

    def _along(self, conv: str, x0: float, y0: float, x1: float, y1: float) -> float:
        """Distance along the conveyor's travel direction from (x0, y0) to (x1, y1)."""
        ds = P.path_s(conv, x1, y1) - P.path_s(conv, x0, y0)
        if P.is_ring(conv):
            n = P.length(conv)
            ds = ((ds + n / 2) % n - n / 2) * P.ring_dir(conv)
        return ds

    def _speed(self, st: _Station, d: ObjectDetection, t: float) -> Optional[float]:
        """The product's speed along its belt from the camera: the same product in an earlier frame
        (products never overtake, so it is the nearest detection a little upstream). None = unknown."""
        conv = P.station(st.arm)["pick_conv"]
        with st.cond:
            hist = list(st.history)
        for t0, dets in reversed(hist):
            dt = t - t0
            if dt < 0.15:
                continue
            if dt > 3.0:
                break
            best = None
            for e in dets:
                if e.bent != 1 or math.hypot(e.x - d.x, e.y - d.y) > self.belt_speed * dt * 1.4 + 0.15:
                    continue
                ds = self._along(conv, e.x, e.y, d.x, d.y)
                if -0.06 <= ds <= self.belt_speed * dt * 1.3 + 0.1 and (best is None or ds < best):
                    best = ds
            return None if best is None else max(0.0, best) / dt
        return None

    def _predict(self, st: _Station, d: ObjectDetection, t: float, until: Optional[float] = None):
        """Where a product seen at (d.x, d.y) at time t is now: at the speed the camera measured, or at
        the belt's speed when the camera has no earlier view of it."""
        conv = P.station(st.arm)["pick_conv"]
        v = self._speed(st, d, t)
        if v is None:
            v = self.belt_speed if self.sim.running_belts.get(conv, True) else 0.0   # it moved until the hold
        dt = min(2.0, max(0.0, (until or time.time()) - t))
        if v * dt < 1e-3:
            return d.x, d.y
        s0 = P.path_s(conv, d.x, d.y)
        sign = P.ring_dir(conv) if P.is_ring(conv) else 1
        s1 = s0 + sign * v * dt
        if not P.is_ring(conv):
            s1 = min(s1, max(s0, P.length(conv) - 0.15))           # products stop at the end of a line
        cx, cy, _ = P.path_xy(conv, s0)
        x, y, _ = P.path_xy(conv, s1)
        return x + d.x - cx, y + d.y - cy                         # keep the sideways offset

    def _remeasure(self, st: _Station, xy, radius: float = 0.4, frames: int = 3,
                   after: float = 0.0, timeout: float = 6.0, label: Optional[str] = None) -> Optional[ObjectDetection]:
        """The held (now standing) product's position from frames taken after `after` (the hold), near
        where it should be (a detector can miss a product in one frame, so any of `frames` frames counts)."""
        best, bd, used = None, radius, 0
        end = time.time() + timeout
        while used < frames and time.time() < end:
            if not self.wait_for_frame(1.5, st.arm):
                continue
            t = st.latest[0]
            if t < after + 0.05:                        # still a frame from before the hold
                continue
            used += 1
            for d in self.detections(1, station=st.arm, fresh=10.0):
                if label is not None and d.label != label:   # not the product we decided on
                    continue
                dist = math.hypot(d.x - xy[0], d.y - xy[1])
                if dist <= bd:
                    best, bd = d, dist
            if best is not None and used >= 2:
                break
        return best

    def _too_early(self, arm: str, target: ObjectDetection, prod) -> bool:
        """True when the product stands upstream of the pick point, out of the arm's reach: let it come on."""
        st = P.station(arm)
        conv = st["pick_conv"]
        if not self.sim.running_belts.get(conv, True):     # stopped for good: it will not come closer
            return False
        try:
            for dz in (0.25, 0.01):
                self.sim.ik(arm, (target.x, target.y, BELT_TOP + prod.height + dz), [self.sim.arms[arm].q])
            return False
        except RuntimeError:
            pass
        sign = P.ring_dir(conv) if P.is_ring(conv) else 1
        ds = (P.path_s(conv, target.x, target.y) - P.path_s(conv, *st["pick"])) * sign
        if P.is_ring(conv):
            n = P.length(conv)
            ds = (ds + n / 2) % n - n / 2
        return ds < 0

    def _grab(self, arm: str, target: ObjectDetection, prod) -> None:
        top = BELT_TOP + prod.height
        tx, ty = target.x, target.y
        self._move_highest(arm, tx, ty, [top + 0.25, top + 0.15], self.timing["move_duration"] * 1.3)
        self._move(arm, (tx, ty, top + 0.01), self.timing["move_duration"])
        off = math.hypot(prod.x - tx, prod.y - ty)
        if off > self.timing["grasp_tolerance"] and self.timing.get("locate", "assist") != "vision":
            self.assists[arm] = self.assists.get(arm, 0) + 1
            self.log(f"{arm}: assist #{self.assists[arm]}: vision put {prod.ptype} {off * 100:.0f} cm off; "
                     "grasping at the simulation's position")
            tx, ty = prod.x, prod.y
            self._move_highest(arm, tx, ty, [top + 0.25, top + 0.15], self.timing["move_duration"])
            self._move(arm, (tx, ty, top + 0.01), self.timing["move_duration"])
            off = math.hypot(prod.x - tx, prod.y - ty)
        if off > self.timing["grasp_tolerance"]:
            self._move(arm, (tx, ty, top + 0.3), self.timing["move_duration"])
            raise MissedGrasp(f"vision put the product {off * 100:.0f} cm from where it is "
                              f"(tolerance {self.timing['grasp_tolerance'] * 100:.0f} cm)")
        self.sim.attach(arm, prod.name)
        self._move_highest(arm, tx, ty, [min(top + 0.5, 2.35), top + 0.35, top + 0.2, top + 0.12], self.timing["move_duration"])

    def _move_highest(self, arm: str, x: float, y: float, heights, duration: float):
        """Lift to the first of these heights the arm can reach at (x, y)."""
        last = None
        for z in heights:
            try:
                return self._move(arm, (x, y, z), duration)
            except RuntimeError as exc:
                if isinstance(exc, InterruptedError):
                    raise
                last = exc
        raise last

    def _deliver(self, arm: str, dest: str, prod) -> None:
        st = P.station(arm)
        prof = P.station_profile(arm)
        if dest == "bent2":
            px, py = st["place"]
            self._move_highest(arm, px, py, [2.0, 1.8, 1.6], self.timing["swing_duration"])
            end = time.time() + self.timing["place_wait_sec"]
            rv = self.schedule == "rendezvous"           # moving belt: wait for a gap that stays free
            while not (self._place_gap(st["place_conv"], px, py, self.timing["move_duration"]) if rv
                       else self.sim.place_free(st["place_conv"], px, py)) and time.time() < end:
                if self._stop.is_set():
                    raise InterruptedError("stopped")
                time.sleep(0.1 if rv else 0.2)
            pause = not rv and self.timing.get("pause_belt", "grasp") != "off"
            if pause:                                      # placing: the place belt stands still
                self.sim.pause_belt(st["place_conv"], arm)
            try:
                self._move(arm, (px, py, BELT_TOP + prod.height + 0.03), self.timing["move_duration"])
                self.sim.release_on_conveyor(arm, st["place_conv"], px, py)
                self._move_highest(arm, px, py, [2.0, 1.8, 1.6], self.timing["move_duration"] * 0.7)
            finally:
                self.sim.resume_belt(st["place_conv"], arm)
        else:
            bx, by = prof.bins[dest]
            q_high = self._bin_ik(arm, bx, by, 2.1)
            self.sim.move_arm(arm, q_high, self.timing["swing_duration"], self._stop)
            q_drop = self._bin_ik(arm, bx, by, BIN_SIZE[2] + prod.height + 0.12)
            self.sim.move_arm(arm, q_drop, self.timing["move_duration"], self._stop)
            self.sim.release_into(arm, P.bin_name(arm, dest), bx, by)
        self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"], self._stop)

    def _cycle(self, arm: str, decide: Callable[[PickContext], str], deadline: Optional[float],
               skip: "_SkipList", halt: Optional[threading.Event] = None) -> Optional[PickResult]:
        """Wait for the next product at this station, decide, pick, deliver. None on timeout / stop."""
        if self.schedule == "rendezvous":
            return self._cycle_rendezvous(arm, decide, deadline, skip, halt)
        st = self._st(arm)
        self.activate(arm)
        failures = 0
        misses: Dict[str, int] = {}
        while (not self._stop.is_set() and not (halt is not None and halt.is_set())
               and (deadline is None or time.time() < deadline)):
            hit = self._approaching(st, skip)
            if hit is None:
                self.wait_for_frame(1.0, arm)
                continue
            det, prod, seen = hit
            st.number += 1
            ctx = PickContext(number=st.number, detection=det, model=prod.name, product_type=type_of(prod.name),
                              world=st.profile, lab=self, station=arm)
            try:
                dest = normalize_destination(decide(ctx))
            except Exception as exc:
                self.log(f"{arm}: decide() failed for #{st.number} ({exc}); leaving it on the belt")
                dest = "skip"
            if dest == "skip":
                skip.add(prod.name, forever=False)
                if prod.name not in skip.recorded:             # one record per product, not per lap
                    skip.recorded.add(prod.name)
                    self._record(ctx, "skip", "skipped")
                continue
            self.sim.hold(prod.name, True)
            held_at = time.time()
            pick_conv = P.station(arm)["pick_conv"]
            pause = self.timing.get("pause_belt", "grasp")
            try:
                if getattr(det, "assisted", False):        # the simulation's position: no remeasure
                    target = copy.copy(det)
                    target.x, target.y = prod.x, prod.y
                    if self.timing.get("locate") != "truth":
                        self.assists[arm] = self.assists.get(arm, 0) + 1
                        self.log(f"{arm}: assist #{self.assists[arm]}: the camera did not report {prod.ptype} at the arm; "
                                 "using the simulation's position")
                    guess = (prod.x, prod.y)
                else:
                    guess = self._predict(st, det, seen, held_at)
                    target = self._remeasure(st, guess, after=held_at, label=det.label)
                if target is None:                         # not seen again: grasp where vision predicted it
                    self.log(f"{arm}: #{st.number} not seen again after the hold; grasping at the predicted spot")
                    target = copy.copy(det)
                    target.x, target.y = guess
                if self._too_early(arm, target, prod):           # held before it reached the arm
                    self.sim.hold(prod.name, False)
                    st.number -= 1
                    self.wait_for_frame(1.0, arm)
                    continue
                if pause != "off":                         # the arm commits: its pick belt stands still
                    self.sim.pause_belt(pick_conv, arm)
                with self._arm_locks[arm]:
                    self.log(f"{arm}: #{st.number} {det.label} -> {dest}")
                    self._grab(arm, target, prod)
                    if pause == "grasp":                   # product lifted: the belt may move again
                        self.sim.resume_belt(pick_conv, arm)
                    self._deliver(arm, dest, prod)
                return self._record(ctx, dest, "delivered")
            except MissedGrasp as exc:
                failures += 1
                self.sim.hold(prod.name, False)
                misses[prod.name] = misses.get(prod.name, 0) + 1
                if misses[prod.name] >= 3:                         # try a product 3 times, then leave it
                    skip.add(prod.name)
                self._record(ctx, dest, "missed")
                self.log(f"{arm}: MISSED #{st.number}: {exc}")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"])
                if failures >= 5:
                    self.log(f"{arm}: 5 missed grasps in a row; check the detector / camera (pausing 10 s)")
                    failures = 0
                    self._stop.wait(10.0)
            except InterruptedError:
                self.sim.hold(prod.name, False)
                a = self.sim.arms[arm]
                if a.carrying:                                     # stopped while carrying: put it down
                    self.sim.release_into(arm, "floor", *self.sim.tip(arm)[:2], floor=0.0)
                return None
            except RuntimeError as exc:
                if self.sim.arms[arm].carrying:                    # put it back where it was
                    tx, ty, _ = self.sim.tip(arm)
                    self.sim.release_on_conveyor(arm, P.station(arm)["pick_conv"], tx, ty)
                self.sim.hold(prod.name, False)
                misses[prod.name] = misses.get(prod.name, 0) + 1
                if misses[prod.name] >= 3 or not P.is_ring(P.station(arm)["pick_conv"]):
                    skip.add(prod.name)                            # a ring brings it round again
                self.log(f"{arm}: #{st.number} not reachable: {exc}")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"])
            finally:
                self.sim.resume_belt(pick_conv, arm)
        return None

    def _record(self, ctx: PickContext, dest: str, status: str) -> PickResult:
        det = ctx.detection
        r = PickResult(number=ctx.number, destination=dest, status=status, label=det.label if det else None,
                       score=det.score if det else None, model=ctx.model, product_type=ctx.product_type,
                       station=ctx.station)
        self._results.append(r)
        for cb in list(self._result_callbacks):
            try:
                cb(r)
            except Exception:
                pass
        return r

    def transfer(self, to: str = "bent2", timeout: Optional[float] = 120.0, station: Optional[str] = None) -> Optional[PickResult]:
        """The next product reaching the station: to "bent2" (its place conveyor), "trash" or "recycling"."""
        dest = normalize_destination(to)
        if dest == "skip":
            raise ValueError("transfer(to='skip') would never pick anything")
        self._stop.clear()
        self._feeding([self._st(station).arm])
        return self._cycle(self._st(station).arm, lambda ctx: dest,
                           None if timeout is None else time.time() + timeout, _SkipList())

    def move_to_bent2(self, timeout: Optional[float] = 120.0, station: Optional[str] = None):
        return self.transfer("bent2", timeout, station)

    def move_to_bent(self, bent: int = 2, timeout: Optional[float] = 120.0, station: Optional[str] = None):
        if bent != 2:
            raise ValueError("a station picks from its bent 1 and places on its bent 2")
        return self.transfer("bent2", timeout, station)

    def put_in_trash(self, timeout: Optional[float] = 120.0, station: Optional[str] = None):
        return self.transfer("trash", timeout, station)

    def drop_in_recycle_bin(self, timeout: Optional[float] = 120.0, station: Optional[str] = None):
        return self.transfer("recycling", timeout, station)

    move_to_trash = put_in_trash
    move_to_recycle_bin = drop_in_recycle_bin

    @staticmethod
    def layout_rule(ctx: PickContext) -> str:
        """The station's rule from pipeline_layout.py, judged on what the detector saw."""
        rule = P.station(ctx.station)["rule"]
        d = ctx.detection
        if rule == "all":
            return "bent2"
        if rule == "share":                          # tree world: the S1 arms share the products evenly
            if getattr(ctx.lab, "schedule", "classic") == "rendezvous":
                return "bent2"                       # the rendezvous scheduler assigns the products
            return "bent2" if PipelineLab._share_turn(ctx.lab, ctx.station) else "skip"
        if d is None:
            return "skip"
        size = max(d.size_x or 0.0, d.size_y or 0.0)
        ok = {"recyclable": d.label in RECYCLABLE, "cardboard": d.label in CARDBOARD,
              "not_large": size < LARGE_M, "large": size >= LARGE_M, "small": size <= SMALL_M}.get(rule, True)
        return "bent2" if ok else "skip"

    @staticmethod
    def share_turn(lab, arm: str, picks: Dict[str, int]) -> bool:
        """Is it `arm`'s turn on a conveyor that n arms share? picks = products each arm took so far.
        On a straight (S1 of the tree world) an arm takes 1/n of the products that have reached it: it
        counts every product that came past its pick point, also while it was busy, and catches up when it
        is free again; the last arm of the straight takes all the others. On a ring (products come round
        again) an arm takes a product while it has taken no more than 1/n of what the ring's arms took
        (counting the ones in a gripper)."""
        conv = P.station(arm)["pick_conv"]
        group = [a for a in P.STATIONS if P.station(a)["pick_conv"] == conv]
        if len(group) < 2 or lab is None or getattr(lab, "sim", None) is None:
            return True
        sim = lab.sim
        if not P.is_ring(conv):
            s_pick = P.path_s(conv, *P.station(arm)["pick"])
            with sim.lock:
                upstream = sum(1 for p in sim.products.values()
                               if p.state == "belt" and p.conv == conv and p.s < s_pick - 0.6)
                reached = len(sim.products) - upstream   # this one included
            return picks.get(arm, 0) * len(group) < reached
        count = lambda a: picks.get(a, 0) + (1 if a != arm and sim.arms[a].carrying else 0)  # noqa: E731
        return count(arm) * len(group) <= sum(count(a) for a in group)

    @staticmethod
    def _share_turn(lab, arm: str) -> bool:
        """Rule "share" (see share_turn), counting the products each arm sent onto its bent 2."""
        picks: Dict[str, int] = {}
        for r in (lab.results if lab is not None else []):
            if r.status == "delivered" and r.destination == "bent2":
                picks[r.station] = picks.get(r.station, 0) + 1
        return PipelineLab.share_turn(lab, arm, picks)

    def run(self, policy: Union[Callable[[PickContext], str], Dict[str, str], str, None] = None,
            count: Optional[int] = None, timeout: Optional[float] = None, by: str = "label",
            stations: Union[str, Iterable[str], None] = None,
            until: Optional[Callable[[], bool]] = None) -> List[PickResult]:
        """
        Keep catching products at the given stations (default: lab.station; "all" = A1 ... A12, each in
        its own thread) and let `policy` decide: a function (ctx) -> destination, a dict {label: dest,
        "*": default}, one destination, or None / "rules" = every station's rule from pipeline_layout.py.
        count = stop after that many delivered products in total; until() = stop when it returns True
        (the arms finish their current delivery first).
        """
        names = P.STATIONS if stations == "all" else [s.upper() for s in ([stations] if isinstance(stations, str) else (stations or [self.station]))]
        if policy is None or policy in ("rules", "gui"):
            decide = self.layout_rule
        elif isinstance(policy, str):
            fixed = normalize_destination(policy)
            decide = lambda ctx: fixed  # noqa: E731
        elif isinstance(policy, dict):
            table = {k: normalize_destination(v) for k, v in policy.items()}
            default = table.get("*", "bent2")
            decide = lambda ctx: table.get(ctx.product_type if by == "type" else ctx.label, default)  # noqa: E731
        elif callable(policy):
            decide = policy
        else:
            raise TypeError("policy must be a function, a dict, a destination name or None")
        self._stop.clear()
        self.activate(*names)
        self._feeding(names)
        start = len(self._results)
        deadline = None if timeout is None else time.time() + timeout
        done = threading.Event()

        def worker(arm):
            skip = _SkipList()
            while not self._stop.is_set() and not done.is_set():
                r = self._cycle(arm, decide, deadline, skip, halt=done)
                if r is None and (deadline is not None and time.time() >= deadline):
                    break
                if count and sum(1 for x in self._results[start:] if x.status == "delivered") >= count:
                    done.set()                       # the other arms finish their current delivery
        threads = [self._spawn(worker, f"station_{a}", a) for a in names]
        if until is not None:
            def watch():
                while not done.is_set() and any(t.is_alive() for t in threads):
                    try:
                        if until():
                            done.set()
                    except Exception as exc:
                        self.log(f"run(until=...) failed: {exc}")
                        done.set()
                    time.sleep(0.5)
            self._spawn(watch, "run_until")
        try:
            for t in threads:
                while t.is_alive():
                    t.join(0.5)
        except KeyboardInterrupt:
            self.log("Ctrl+C: stopping the arms after their current motion ...")
            self.stop()
            for t in threads:
                t.join(30.0)
            raise
        self._stop.clear()
        return self._results[start:]

    def stop(self):
        self._stop.set()

    def set_motion(self, motion_type: str):
        self.log("set_motion: the pipeline arms always move the same way (no motion types here).")

    def set_arm_param(self, name: str, value):
        if name not in self.timing:
            raise ValueError(f"pipeline arm parameters: {', '.join(self.timing)}")
        self.timing[name] = value

    @property
    def results(self) -> List[PickResult]:
        return list(self._results)

    def report(self, results: Optional[List[PickResult]] = None) -> str:
        rs = self.results if results is None else results
        by: Dict[str, int] = {}
        for r in rs:
            key = f"{r.station}:{r.destination}" if r.status == "delivered" else f"{r.station}:{r.status}"
            by[key] = by.get(key, 0) + 1
        judged = [r.label_correct for r in rs if r.label_correct is not None]
        acc = f"{sum(judged)}/{len(judged)} = {100.0 * sum(judged) / len(judged):.0f}%" if judged else "n/a"
        lines = [f"{len(rs)} decisions: " + ", ".join(f"{k} {v}" for k, v in sorted(by.items())),
                 f"arm locate mode {self.timing.get('locate')}: "
                 + (", ".join(f"{a} {n}" for a, n in sorted(self.assists.items())) + " picks used the simulation's position"
                    if self.assists else "every pick found by the camera"),
                 f"detector label correct (vs simulation truth): {acc}",
                 "where products are now: " + ", ".join(f"{k} {v}" for k, v in self.summary().items())]
        if self.rendezvous is not None and self.schedule == "rendezvous":
            lines.append(self.rendezvous.report(self.sim.stopped_time))
        else:
            stopped = sum(self.sim.stopped_time.values())
            lines.append(f"schedule {self.schedule}: belts stood {stopped:.0f} s in total for arms"
                         + (" (" + ", ".join(f"{c} {v:.0f} s" for c, v in self.sim.stopped_time.items() if v > 0.5) + ")"
                            if stopped > 0.5 else ""))
        for r in rs[-30:]:
            mark = {True: "ok", False: "WRONG", None: "--"}[r.label_correct]
            lines.append(f"  {r.station} #{r.number}: {r.status:9s} -> {r.destination:9s} label={r.label} "
                         f"truth={r.product_type} [{mark}]")
        return "\n".join(lines)

    # ------------------------------------------------------------------ rendezvous scheduling
    @property
    def schedule(self) -> str:
        """classic (hold the product, pause its belt) or rendezvous (meet it on the moving belt)."""
        return self.timing.get("schedule", "classic")

    def set_scheduler(self, mode: str = "rendezvous", assign: Optional[str] = None, pool: Optional[str] = None,
                      quiet: bool = False) -> Optional[RendezvousScheduler]:
        """Choose how the arms pick: "classic" or "rendezvous" (see rendezvous.py). assign: "balanced"
        (default) or "earliest"; pool: "rules" (default: the arms with rule all / share of a conveyor
        share its products), "conveyor" (every arm of a conveyor; for policies where all of them take
        every product) or "none"."""
        mode = str(mode or "classic").strip().lower()
        if mode not in SCHEDULES:
            raise ValueError(f"schedule must be one of {', '.join(SCHEDULES)}")
        if assign is not None:
            self.timing["rendezvous_assign"] = str(assign).strip().lower()
        if pool is not None:
            self.timing["rendezvous_pool"] = str(pool).strip().lower()
        if self.timing["rendezvous_assign"] not in ASSIGN_MODES:
            raise ValueError(f"rendezvous_assign must be one of {', '.join(ASSIGN_MODES)}")
        if self.timing["rendezvous_pool"] not in POOL_MODES:
            raise ValueError(f"rendezvous_pool must be one of {', '.join(POOL_MODES)}")
        self.timing["schedule"] = mode
        self.rendezvous = None
        if mode == "rendezvous":
            self.rendezvous = RendezvousScheduler(self, self.timing["rendezvous_assign"], self.timing["rendezvous_pool"])
            if not quiet or getattr(self, "_started", False):
                self._log_scheduler()
        return self.rendezvous

    def _log_scheduler(self):
        r = self.rendezvous
        if r is None:
            return
        pools = ", ".join(f"{c}: {'/'.join(a)}" for c, a in r.pools.items() if len(a) >= 2) or "no shared conveyor"
        self.log(f"schedule rendezvous: belts keep moving, arms meet the products on them; assign={r.assign_mode}, "
                 f"shared conveyors {pools}")

    def _place_gap(self, conv: str, x: float, y: float, descend: float, clearance: float = 0.5) -> bool:
        """Moving belt: is (x, y) free when the arm has come down (`descend` s from now)? Once placed, the
        product moves with the others, so the gap at the moment of release is the gap it keeps."""
        now = time.time()
        for t in (now + descend, now + descend + 0.3):
            for p in self.sim.on_conveyor(conv):
                px, py = track_from_product(p, now).xy_at(t)
                if math.hypot(px - x, py - y) <= clearance:
                    return False
        return True

    def _rv_candidates(self, st: _Station, skip: "_SkipList", sched: RendezvousScheduler):
        """(plan, detection, product) for every product coming to this arm that it may take, earliest first."""
        arm = st.arm
        conv = P.station(arm)["pick_conv"]
        now = time.time()
        locate = self.timing.get("locate", "assist")
        tracks = {}
        if locate != "truth":                             # the camera's view: position, speed along the belt
            t = st.latest[0]
            for d in self.detections(1, station=arm, fresh=1.5):
                x, y = self._predict(st, d, t)
                prod = self.sim.nearest(conv, x, y, 0.4)
                if prod is None or prod.name in skip or prod.name in tracks:
                    continue
                v = self._speed(st, d, t)
                if v is None:                             # seen once only: wait for a second view (a product
                    o = rv_offset(arm, conv, P.path_s(conv, x, y))          # entering the picture is cut off)
                    if o < window(arm, prod.height, self.timing.get("rendezvous_window", 0.8))[0]:
                        continue
                    v = self.belt_speed if self.sim.belt_moving(conv) else 0.0
                tracks[prod.name] = (track_from_detection(conv, d.x, d.y, t, v, prod.size[0] / 2), d, prod)
                self._near_since.pop(prod.name, None)
        if locate != "vision":                            # assist / truth: products the camera has not reported
            for p in self.sim.on_conveyor(conv):
                if p.name in tracks or p.name in skip:
                    continue
                lo, hi = window(arm, p.height, self.timing.get("rendezvous_window", 0.8))
                o = track_from_product(p, now).offset_at(arm, now)
                if locate == "assist":
                    if not lo - 1.0 <= o <= hi:
                        self._near_since.pop(p.name, None)
                        continue
                    if now - self._near_since.setdefault(p.name, now) < 0.5:
                        continue
                elif not -6.0 <= o <= hi:
                    continue
                sx, sy, _ = p.size
                label = p.ptype if locate == "truth" else "unknown"
                det = ObjectDetection(u=0.0, v=0.0, label=label, score=0.0, x=p.x, y=p.y, size_x=sx, size_y=sy, bent=1)
                det.assisted = True
                tracks[p.name] = (track_from_product(p, now), det, p)
        out = []
        for name, (tr, det, prod) in tracks.items():
            if not sched.may_take(arm, name):
                continue
            plan = sched.plan(arm, name, tr, prod.height, now)
            if plan is not None:
                out.append((plan, det, prod))
        out.sort(key=lambda c: c[0].t_contact)
        return out

    def _cycle_rendezvous(self, arm: str, decide: Callable[[PickContext], str], deadline: Optional[float],
                          skip: "_SkipList", halt: Optional[threading.Event] = None) -> Optional[PickResult]:
        """The rendezvous version of _cycle: plan, start just in time, meet the product on the moving belt."""
        st = self._st(arm)
        self.activate(arm)
        sched = self.rendezvous or self.set_scheduler("rendezvous")
        stats = sched.stats[arm]
        decided: Dict[str, tuple] = {}
        misses: Dict[str, int] = {}
        failures = 0
        while (not self._stop.is_set() and not (halt is not None and halt.is_set())
               and (deadline is None or time.time() < deadline)):
            sched.tick()
            cands = self._rv_candidates(st, skip, sched)
            if not cands:
                self.wait_for_frame(0.5, arm)
                continue
            plan, det, prod = cands[0]
            wait = plan.t_start - time.time()
            if wait > 0.15:                               # not yet: plan again with a fresher view
                self._stop.wait(min(0.4, wait))
                continue
            if prod.name not in decided:
                st.number += 1
                ctx = PickContext(number=st.number, detection=det, model=prod.name, product_type=type_of(prod.name),
                                  world=st.profile, lab=self, station=arm)
                try:
                    dest = normalize_destination(decide(ctx))
                except Exception as exc:
                    self.log(f"{arm}: decide() failed for #{st.number} ({exc}); leaving it on the belt")
                    dest = "skip"
                decided[prod.name] = (ctx, dest)
            ctx, dest = decided[prod.name]
            if dest == "skip":
                decided.pop(prod.name, None)
                skip.add(prod.name, forever=False)
                if prod.name not in skip.recorded:
                    skip.recorded.add(prod.name)
                    self._record(ctx, "skip", "skipped")
                continue
            if not sched.commit(arm, plan):
                continue
            picked = False
            try:
                with self._arm_locks[arm]:
                    self.log(f"{arm}: #{ctx.number} {det.label} -> {dest}: rendezvous in {plan.t_contact - time.time():.1f} s, "
                             f"{plan.o_contact:+.2f} m from the pick point ({plan.track.source}, "
                             f"{plan.track.v:.2f} m/s)")
                    self._rv_grab(arm, plan, prod, sched)
                    picked = True
                    t_contact = time.time()
                    self._deliver(arm, dest, prod)
                    sched.tail[arm].add(time.time() - t_contact)
                stats.picks += 1
                decided.pop(prod.name, None)
                return self._record(ctx, dest, "delivered")
            except _LateRendezvous as exc:
                stats.late += 1
                self.log(f"{arm}: #{ctx.number} rendezvous lost ({exc}); planning again")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["move_duration"])
            except MissedGrasp as exc:
                stats.missed += 1
                failures += 1
                misses[prod.name] = misses.get(prod.name, 0) + 1
                if misses[prod.name] >= 3:
                    skip.add(prod.name)
                    decided.pop(prod.name, None)
                self._record(ctx, dest, "missed")
                self.log(f"{arm}: MISSED #{ctx.number}: {exc}")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"])
                if failures >= 5:
                    self.log(f"{arm}: 5 missed rendezvous in a row; check the detector / camera (pausing 10 s)")
                    failures = 0
                    self._stop.wait(10.0)
            except InterruptedError:
                if self.sim.arms[arm].carrying:
                    self.sim.release_into(arm, "floor", *self.sim.tip(arm)[:2], floor=0.0)
                return None
            except RuntimeError as exc:
                if self.sim.arms[arm].carrying:                    # put it back on its belt
                    tx, ty, _ = self.sim.tip(arm)
                    self.sim.release_on_conveyor(arm, P.station(arm)["pick_conv"], tx, ty)
                misses[prod.name] = misses.get(prod.name, 0) + 1
                if misses[prod.name] >= 3 or not P.is_ring(P.station(arm)["pick_conv"]):
                    skip.add(prod.name)
                    decided.pop(prod.name, None)
                self.log(f"{arm}: #{ctx.number} not reachable: {exc}")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"])
            finally:
                sched.release(arm, prod.name, picked)
        return None

    def _rv_grab(self, arm: str, plan, prod, sched: RendezvousScheduler) -> None:
        """Hover over the product as it arrives, come down with it, grasp at the planned contact point."""
        top = BELT_TOP + prod.height
        move = self.timing["move_duration"]
        approach = move * 1.3
        t0 = time.time()
        hx, hy = plan.hover_xy()
        cx, cy = plan.contact_xy()
        q_h, last = None, None
        for dz in (0.25, 0.15):
            try:
                q_h = self.sim.ik(arm, (hx, hy, top + dz), [self.sim.arms[arm].q])
                break
            except RuntimeError as exc:
                last = exc
        if q_h is None:
            raise last
        q_c = self.sim.ik(arm, (cx, cy, top + 0.01), [q_h])
        sched.ik_time[arm].add(time.time() - t0)
        # the IK took a while (and the other threads too): time the moves again from a fresh look at the
        # product, so the arm reaches the planned hover and contact points WITH the product
        tr = self._rv_refresh(arm, prod, plan.track, t0)
        now = time.time()
        o_h = plan.track.offset_at(arm, plan.t_hover)
        t_hover = tr.time_at_offset(arm, o_h, now)
        t_contact = tr.time_at_offset(arm, plan.o_contact, now)
        standing = t_contact is not None and t_contact >= tr.stops_at() - 1e-3   # it waits there for the arm
        if t_hover is None or t_contact is None or (t_contact < now + 0.6 * approach + 0.3 and not standing):
            raise _LateRendezvous(f"the IK took {now - t0:.1f} s; the product is past the planned point")
        if os.environ.get("RENDEZVOUS_DEBUG"):
            self.log(f"{arm}: DEBUG retimed hover {t_hover - plan.t_hover:+.2f} s, contact {t_contact - plan.t_contact:+.2f} s "
                     f"({tr.source}, {tr.v:.2f} m/s)")
        t_hover = max(t_hover, now + 0.6 * approach)
        t_contact = max(t_contact, t_hover + (move if standing else 0.25))
        dur = t_hover - now                                 # arrive WITH the product, not before it
        sched.stats[arm].stretched += max(0.0, dur - approach)
        sched.stats[arm].lead_used.append(t_hover - t0)
        self.sim.move_arm(arm, q_h, dur, self._stop, timed=True)
        self.sim.move_arm(arm, q_c, max(0.25, t_contact - time.time()), self._stop, profile="linear", timed=True)
        tip = self.sim.tip(arm)
        off = math.hypot(prod.x - tip[0], prod.y - tip[1])
        tol = self.timing["grasp_tolerance"]
        if os.environ.get("RENDEZVOUS_DEBUG"):
            self.log(f"{arm}: DEBUG at contact: late {time.time() - t_contact:+.2f} s, tip vs plan "
                     f"{math.hypot(tip[0] - cx, tip[1] - cy) * 100:.0f} cm, product vs plan "
                     f"{math.hypot(prod.x - cx, prod.y - cy) * 100:.0f} cm (along "
                     f"{(rv_offset(arm, plan.track.conv, P.path_s(plan.track.conv, prod.x, prod.y)) - plan.o_contact) * 100:+.0f} cm)")
        if off > tol and self.timing.get("locate", "assist") != "vision":
            self.assists[arm] = self.assists.get(arm, 0) + 1
            self.log(f"{arm}: assist #{self.assists[arm]}: the rendezvous put {prod.ptype} {off * 100:.0f} cm off; "
                     "following it with the simulation's position")
            t1 = time.time()
            tr = track_from_product(prod, t1)
            t_c = t1 + 0.5 + move * 0.6
            q = self.sim.ik(arm, (*tr.xy_at(t_c), top + 0.01), [self.sim.arms[arm].q])
            self.sim.move_arm(arm, q, max(0.25, t_c - time.time()), self._stop, profile="linear", timed=True)
            tip = self.sim.tip(arm)
            off = math.hypot(prod.x - tip[0], prod.y - tip[1])
        if off > tol:
            self._move(arm, (tip[0], tip[1], top + 0.3), move)
            raise MissedGrasp(f"the product was {off * 100:.0f} cm from the gripper at the rendezvous "
                              f"(tolerance {tol * 100:.0f} cm)")
        sched.stats[arm].contact_err.append(off)
        self.sim.attach(arm, prod.name)
        self._move_highest(arm, tip[0], tip[1], [min(top + 0.5, 2.35), top + 0.35, top + 0.2, top + 0.12], move)

    def _rv_refresh(self, arm: str, prod, track, since: float):
        """The product's track from a look newer than `since`: a camera frame (vision / assist) matched near
        where the track puts it, else (assist / truth) the simulation; else the old track."""
        locate = self.timing.get("locate", "assist")
        if locate == "truth":
            return track_from_product(prod)
        st = self._st(arm)
        if st.latest[0] <= since:
            self.wait_for_frame(0.6, arm)
        t = st.latest[0]
        if t > since:
            px, py = track.xy_at(t)
            best, bd = None, 0.3
            for d in self.detections(1, station=arm, fresh=2.0):
                dist = math.hypot(d.x - px, d.y - py)
                if dist < bd:
                    best, bd = d, dist
            if best is not None:
                new = track_from_detection(track.conv, best.x, best.y, t, track.v, track.half_len)
                if t - track.t >= 0.8:                    # speed over the long baseline old look -> new look
                    ds = (new.s - track.s) * (P.ring_dir(track.conv) if P.is_ring(track.conv) else 1)
                    if P.is_ring(track.conv):
                        ds %= P.length(track.conv)
                    v = ds / (t - track.t)
                    if 0.0 <= v <= 2.0 * max(self.belt_speed, 0.05):
                        new.v = v
                return new
        return track_from_product(prod) if locate == "assist" else track


class _LateRendezvous(RuntimeError):
    pass
