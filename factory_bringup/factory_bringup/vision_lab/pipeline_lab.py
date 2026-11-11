"""
PipelineLab: the vision lab in the pipeline world (pipeline_factory_lab.sdf): 13 conveyors, 12 BCR
arm stations A1 ... A12, each with its own camera, recycling bin and trash bin.

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
from factory_bringup.vision_lab.worlds import WorldProfile

# product types a "cardboard" rule takes (A6), and the size limits of "large" (A9) / "small" (A11)
CARDBOARD = {"shipping_box_large", "cereal_box", "shoe_box", "long_package", "pickup_box", "pizza_box", "toy_box",
             "gift_box", "tissue_box", "laptop_box", "kettle_box", "electronics_box", "book_parcel", "milk_carton"}
LARGE_M, SMALL_M = 0.33, 0.22


class MissedGrasp(RuntimeError):
    pass


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
                 feed: bool = True, feed_items: Optional[List[str]] = None, seed: Optional[int] = None, **_ignored):
        self.station = station.upper()
        if self.station not in P.STATIONS:
            raise ValueError(f"unknown station {station}; stations: {', '.join(P.STATIONS)}")
        self._detector_spec, self._detector_options = detector, dict(detector_options or {})
        self._backend_name = backend
        self._show_window = show_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir or "vision_lab_results"))
        self.recorder = YoloDatasetRecorder(record_dataset, **(record_options or {})) if record_dataset else None
        self.timing = {"move_duration": 0.9, "swing_duration": 2.5, "grasp_tolerance": 0.10, "roi_radius": 0.45,
                       "place_wait_sec": 30.0}
        self.timing.update({k: v for k, v in (arm_params or {}).items() if k in self.timing})
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
        if start:
            self.start()

    # ------------------------------------------------------------------ life cycle
    def start(self) -> "PipelineLab":
        if self._started:
            return self
        backend = FakeBackend() if self._backend_name == "fake" else GzBackend()
        if not backend.running():
            raise RuntimeError("The pipeline lab world is not running in Gazebo. Start it with\n"
                               "  ./run_vision_lab.sh sim --world pipeline\n"
                               "or try the lab without Gazebo: --fake")
        self.sim = PipelineSim(backend, self.belt_speed)
        self.sim.log = self.log
        for a in P.STATIONS:
            det = self._detector_spec
            if isinstance(det, Detector) and a != self.station:
                det = copy.deepcopy(det)
            self.stations[a] = _Station(self, a, det)
            backend.subscribe_camera(P.camera_topic(a), self._camera_cb(a))
        self.sim.start()
        self.activate(self.station)
        self._spawn(self._feed_loop, "pipeline_lab_feeder")
        if self._show_window:
            self._spawn(self._window_loop, "pipeline_lab_window")
        self._started = True
        self.log(f"Pipeline lab ready ({'no Gazebo, drawn cameras' if self._backend_name == 'fake' else 'Gazebo'}): "
                 f"station {self.station} ({P.station(self.station)['pick_conv']} -> {P.station(self.station)['place_conv']}), "
                 f"detector {self.stations[self.station].pipe.detector.name}")
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
        def cb(bgr, t):
            st = self.stations[arm]
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
                    tiles.append(cv2.resize(v, (640, 360)))
                cols = 1 if len(tiles) == 1 else (2 if len(tiles) <= 4 else 3)
                while len(tiles) % cols:
                    tiles.append(np.zeros_like(tiles[0]))
                grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
                cv2.imshow(win, grid)
            cv2.waitKey(200)
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
                if len(prods) >= keep or not self.sim.running_belts.get(conv, True):
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
                    self.sim.spawn_row([self._rng.choice(self.feed_items)], conv, 1.0, s0)

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

    def _approaching(self, st: _Station, skip: set):
        """(detection, product, frame time) of a product the camera sees close to the pick point."""
        px, py = P.station(st.arm)["pick"]
        best, bd = None, self.timing["roi_radius"]
        t = st.latest[0]
        for d in self.detections(1, station=st.arm, fresh=1.5):
            dist = math.hypot(d.x - px, d.y - py)
            if dist <= bd:
                x, y = self._predict(st, d, t)
                prod = self.sim.nearest(P.station(st.arm)["pick_conv"], x, y, 0.4)
                if prod is not None and prod.name not in skip:
                    best, bd = (d, prod, t), dist
        return best

    def _velocity(self, st: _Station, d: ObjectDetection, t: float):
        """The product's velocity from the camera: the same product in an earlier frame (0 if it stands)."""
        with st.cond:
            hist = list(st.history)
        for t0, dets in reversed(hist):
            if t - t0 < 0.35:
                continue
            if t - t0 > 1.5:
                break
            prev = min(dets, key=lambda e: math.hypot(e.x - d.x, e.y - d.y), default=None)
            limit = self.belt_speed * (t - t0) * 1.5 + 0.08
            if prev is not None and math.hypot(prev.x - d.x, prev.y - d.y) <= limit:
                return (d.x - prev.x) / (t - t0), (d.y - prev.y) / (t - t0)
            break
        return 0.0, 0.0

    def _predict(self, st: _Station, d: ObjectDetection, t: float, until: Optional[float] = None):
        """Where a product seen at (d.x, d.y) at time t is now, at the speed the camera measured."""
        vx, vy = self._velocity(st, d, t)
        dt = min(1.0, max(0.0, (until or time.time()) - t))
        return d.x + vx * dt, d.y + vy * dt

    def _remeasure(self, st: _Station, xy, radius: float = 0.4, frames: int = 4) -> Optional[ObjectDetection]:
        """The held (now standing) product's position from the next few frames, near where it should be
        (a detector can miss a product in one frame, so any of `frames` frames counts)."""
        best, bd = None, radius
        for i in range(frames):
            if not self.wait_for_frame(1.5, st.arm):
                continue
            for d in self.detections(1, station=st.arm):
                dist = math.hypot(d.x - xy[0], d.y - xy[1])
                if dist <= bd:
                    best, bd = d, dist
            if best is not None and i >= 1:
                break
        return best

    def _grab(self, arm: str, target: ObjectDetection, prod) -> None:
        top = BELT_TOP + prod.height
        tx, ty = target.x, target.y
        self._move_highest(arm, tx, ty, [top + 0.25, top + 0.15], self.timing["move_duration"] * 1.3)
        self._move(arm, (tx, ty, top + 0.01), self.timing["move_duration"])
        off = math.hypot(prod.x - tx, prod.y - ty)
        if off > self.timing["grasp_tolerance"]:
            self._move(arm, (tx, ty, top + 0.3), self.timing["move_duration"])
            raise MissedGrasp(f"vision put the product {off * 100:.0f} cm from where it is "
                              f"(tolerance {self.timing['grasp_tolerance'] * 100:.0f} cm)")
        self.sim.attach(arm, prod.name)
        self._move_highest(arm, tx, ty, [min(top + 0.5, 2.35), top + 0.35, top + 0.2], self.timing["move_duration"])

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
            while not self.sim.place_free(st["place_conv"], px, py) and time.time() < end:
                if self._stop.is_set():
                    raise InterruptedError("stopped")
                time.sleep(0.2)
            self._move(arm, (px, py, BELT_TOP + prod.height + 0.03), self.timing["move_duration"])
            self.sim.release_on_conveyor(arm, st["place_conv"], px, py)
            self._move_highest(arm, px, py, [2.0, 1.8, 1.6], self.timing["move_duration"] * 0.7)
        else:
            bx, by = prof.bins[dest]
            q_high = self._bin_ik(arm, bx, by, 2.1)
            self.sim.move_arm(arm, q_high, self.timing["swing_duration"], self._stop)
            q_drop = self._bin_ik(arm, bx, by, BIN_SIZE[2] + prod.height + 0.12)
            self.sim.move_arm(arm, q_drop, self.timing["move_duration"], self._stop)
            self.sim.release_into(arm, P.bin_name(arm, dest), bx, by)
        self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"], self._stop)

    def _cycle(self, arm: str, decide: Callable[[PickContext], str], deadline: Optional[float],
               skip: set, halt: Optional[threading.Event] = None) -> Optional[PickResult]:
        """Wait for the next product at this station, decide, pick, deliver. None on timeout / stop."""
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
                skip.add(prod.name)
                self._record(ctx, "skip", "skipped")
                continue
            self.sim.hold(prod.name, True)
            held_at = time.time()
            try:
                guess = self._predict(st, det, seen, held_at)
                target = self._remeasure(st, guess)
                if target is None:                         # not seen again: grasp where vision predicted it
                    self.log(f"{arm}: #{st.number} not seen again after the hold; grasping at the predicted spot")
                    target = copy.copy(det)
                    target.x, target.y = guess
                with self._arm_locks[arm]:
                    self.log(f"{arm}: #{st.number} {det.label} -> {dest}")
                    self._grab(arm, target, prod)
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
                skip.add(prod.name)
                self.log(f"{arm}: #{st.number} not reachable: {exc}")
                self.sim.move_arm(arm, self.sim.wait_posture(arm), self.timing["swing_duration"])
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
                           None if timeout is None else time.time() + timeout, set())

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
        if d is None:
            return "skip"
        size = max(d.size_x or 0.0, d.size_y or 0.0)
        ok = {"recyclable": d.label in RECYCLABLE, "cardboard": d.label in CARDBOARD,
              "not_large": size < LARGE_M, "large": size >= LARGE_M, "small": size <= SMALL_M}.get(rule, True)
        return "bent2" if ok else "skip"

    def run(self, policy: Union[Callable[[PickContext], str], Dict[str, str], str, None] = None,
            count: Optional[int] = None, timeout: Optional[float] = None, by: str = "label",
            stations: Union[str, Iterable[str], None] = None) -> List[PickResult]:
        """
        Keep catching products at the given stations (default: lab.station; "all" = A1 ... A12, each in
        its own thread) and let `policy` decide: a function (ctx) -> destination, a dict {label: dest,
        "*": default}, one destination, or None / "rules" = every station's rule from pipeline_layout.py.
        count = stop after that many delivered products in total.
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
            skip = set()
            while not self._stop.is_set() and not done.is_set():
                r = self._cycle(arm, decide, deadline, skip, halt=done)
                if r is None and (deadline is not None and time.time() >= deadline):
                    break
                if count and sum(1 for x in self._results[start:] if x.status == "delivered") >= count:
                    done.set()                       # the other arms finish their current delivery
        threads = [self._spawn(worker, f"station_{a}", a) for a in names]
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
                 f"detector label correct (vs simulation truth): {acc}",
                 "where products are now: " + ", ".join(f"{k} {v}" for k, v in self.summary().items())]
        for r in rs[-30:]:
            mark = {True: "ok", False: "WRONG", None: "--"}[r.label_correct]
            lines.append(f"  {r.station} #{r.number}: {r.status:9s} -> {r.destination:9s} label={r.label} "
                         f"truth={r.product_type} [{mark}]")
        return "\n".join(lines)
