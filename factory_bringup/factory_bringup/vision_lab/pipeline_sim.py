"""
Pipeline world engine of the vision lab (no ROS): moves what pipeline_factory_lab.sdf leaves still.

  - products: spawned on a conveyor, carried along it at the belt speed (straights from start to
    end, rings in pipeline_world.RING_DIRS), queued behind each other and at a belt end, dropped into
    the dock past an output line;
  - arms: joint values per arm, interpolated over time; the 8 link models of each arm are posed from
    the arm's forward kinematics; a product attached to an arm follows its gripper tip;
  - bins and docks: products released over them fall in and stay.

Everything is kinematic (set poses), like bcr_endpoint_object_attacher moves products in the other
worlds. Two backends:
  GzBackend    the running Gazebo (gz-transport Python bindings: python3-gz-transport13)
  FakeBackend  no Gazebo: the station cameras are drawn from the engine's state (synthetic.py), so
               labs, detectors and policies can be developed and tested anywhere.
"""

import itertools
import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from factory_bringup import pipeline_layout as L
from factory_bringup.product_catalog import BIN_SIZE, PRODUCT_TYPES, product_sdf
from factory_bringup.vision_lab import pipeline_world as P
from factory_bringup.vision_lab.arm_model import JOINTS, ArmModel, quat

RATE = 20.0                        # engine steps per second
BELT_TOP = L.BELT_TOP_Z
END_MARGIN = 0.15                  # products stop this far before the end of a straight belt
GAP = 0.25                         # free belt between two products in a queue (the camera must see them apart)
BIN_FLOOR = 0.04


@dataclass
class Product:
    name: str
    ptype: str
    conv: Optional[str]            # conveyor it rides, None when carried / binned / docked
    s: float = 0.0
    state: str = "belt"            # belt, carried, bin, dock
    held: bool = False             # stopped on the belt for a pick
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    where: str = ""                # bin / dock name once it is there

    @property
    def size(self) -> Tuple[float, float, float]:
        return PRODUCT_TYPES[self.ptype]["size"]

    @property
    def height(self) -> float:
        return self.size[2]


@dataclass
class Arm:
    name: str
    base: Tuple[float, float]
    q: Dict[str, float] = field(default_factory=dict)
    carrying: Optional[str] = None
    dirty: bool = True


# ------------------------------------------------------------------------------------------ backends
class GzBackend:
    """Gazebo through gz-transport (the Python bindings the attacher already uses)."""

    def __init__(self, world: str = P.WORLD_NAME):
        try:
            from gz.transport13 import Node
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            from gz.msgs10.entity_pb2 import Entity
            from gz.msgs10.image_pb2 import Image
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.pose_v_pb2 import Pose_V
        except ImportError as exc:
            raise ImportError("The pipeline lab needs the gz-transport Python bindings: "
                              "sudo apt install python3-gz-transport13 python3-gz-msgs10") from exc
        self.m = dict(Boolean=Boolean, EntityFactory=EntityFactory, Entity=Entity, Image=Image, Pose=Pose, Pose_V=Pose_V)
        self.node = Node()
        self.world = world
        self.pose_vector_ok = True

    def running(self) -> bool:
        try:
            services = self.node.service_list()
        except AttributeError:                       # older bindings: ask the CLI
            import subprocess
            try:
                services = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=5).stdout.split()
            except Exception:
                return False
        except Exception:
            return False
        return any(s.startswith(f"/world/{self.world}/") for s in services)

    def _req(self, service, msg, rep, timeout=3000):
        ok, res = self.node.request(f"/world/{self.world}/{service}", msg, type(msg), self.m[rep], timeout)
        return bool(ok and getattr(res, "data", True))

    def _pose(self, msg, x, y, z, q):
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q

    def create(self, name: str, sdf: str, x, y, z, q) -> bool:
        f = self.m["EntityFactory"]()
        f.sdf, f.name, f.allow_renaming = sdf, name, False
        self._pose(f.pose, x, y, z, q)
        return self._req("create", f, "Boolean")

    def remove(self, name: str) -> bool:
        e = self.m["Entity"]()
        e.name, e.type = name, self.m["Entity"].MODEL
        return self._req("remove", e, "Boolean")

    def set_poses(self, poses: List[Tuple[str, float, float, float, Tuple]]):
        if not poses:
            return
        if self.pose_vector_ok:
            v = self.m["Pose_V"]()
            for name, x, y, z, q in poses:
                p = v.pose.add()
                p.name = name
                self._pose(p, x, y, z, q)
            if self._req("set_pose_vector", v, "Boolean", 1000):
                return
            self.pose_vector_ok = False          # older Gazebo: one call per model
        for name, x, y, z, q in poses:
            p = self.m["Pose"]()
            p.name = name
            self._pose(p, x, y, z, q)
            self._req("set_pose", p, "Boolean", 500)

    def subscribe_camera(self, topic: str, callback: Callable[[np.ndarray, float], None]):
        Image = self.m["Image"]

        def on_image(msg):
            h, w = msg.height, msg.width
            data = np.frombuffer(msg.data, np.uint8)
            if data.size < h * w * 3:
                return
            rgb = data[: h * msg.step].reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3) if msg.step else data.reshape(h, w, 3)
            callback(rgb[:, :, ::-1].copy(), time.time())
        if not self.node.subscribe(Image, topic, on_image):
            raise RuntimeError(f"cannot subscribe to {topic}")

    def tick(self, engine):
        pass


class FakeBackend:
    """No Gazebo: keeps the poses, and draws each station camera from the engine state."""

    def __init__(self, noise: float = 3.0):
        self.noise = noise
        self.cams: List[Tuple[str, Callable]] = []
        self.engine = None
        self._thread: Optional[threading.Thread] = None

    def running(self) -> bool:
        return True

    def create(self, *a, **k) -> bool:
        return True

    def remove(self, name: str) -> bool:
        return True

    def set_poses(self, poses):
        pass

    def subscribe_camera(self, topic: str, callback):
        arm = topic.split("/")[2]
        self.cams.append((arm, callback))

    active: Optional[set] = None                          # stations whose camera is drawn (None = all)

    def tick(self, engine):
        self.engine = engine
        if self._thread is None:
            self._thread = threading.Thread(target=self._cameras, name="fake_cameras", daemon=True)
            self._thread.start()

    def close(self):
        self._closing = True
        if self._thread is not None:
            self._thread.join(3.0)

    def _cameras(self):
        while not getattr(self, "_closing", False):
            t0 = time.time()
            self._render(self.engine)
            time.sleep(max(0.01, 1.0 / P.CAM_RATE - (time.time() - t0)))

    def _render(self, engine):
        now = time.time()
        from factory_bringup.vision_lab.synthetic import render_view
        with engine.lock:
            items = [(p.ptype, p.x, p.y, p.yaw, p.z - p.height / 2) for p in engine.products.values()
                     if p.state in ("belt", "carried")]
        for arm, cb in self.cams:
            if self.active is not None and arm not in self.active:
                continue                                  # only the cameras the lab looks at
            prof = P.station_profile(arm)
            near = [it for it in items if math.hypot(it[1] - prof.camera_pose["x"], it[2] - prof.camera_pose["y"]) < 7.0]
            cb(render_view(prof, near, noise=self.noise), now)


# ------------------------------------------------------------------------------------------ engine
class PipelineSim:
    def __init__(self, backend=None, belt_speed: float = L.BELT_SPEED):
        self.backend = backend or GzBackend()
        self.speed = belt_speed
        self.lock = threading.RLock()
        self.products: Dict[str, Product] = {}
        self.running_belts: Dict[str, bool] = {c: True for c in list(L.STRAIGHTS) + list(L.CIRCLES)}
        self.model = ArmModel()
        self.arms: Dict[str, Arm] = {}
        for a in P.STATIONS:
            st = P.station(a)
            self.arms[a] = Arm(a, tuple(st["base"]), self.wait_posture(a))
        self.counts: Dict[str, int] = {}
        self._n = itertools.count(int(time.time()) % 100000 * 10)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.log: Callable[[str], None] = lambda text: None

    # ---------------------------------------------------------------- life cycle
    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name="pipeline_sim", daemon=True)
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(2.0)
        if hasattr(self.backend, "close"):
            self.backend.close()

    def _loop(self):
        dt = 1.0 / RATE
        t_prev = time.time()
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self.step(min(0.2, t0 - t_prev))
                self.backend.tick(self)
            except Exception as exc:                 # keep the engine alive
                self.log(f"engine step failed: {exc}")
            t_prev = t0
            self._stop.wait(max(0.0, dt - (time.time() - t0)))

    # ---------------------------------------------------------------- arms
    def base_T(self, arm: str) -> np.ndarray:
        T = np.eye(4)
        bx, by = self.arms[arm].base
        T[:3, 3] = (bx, by, L.ARM_BASE_Z)
        return T

    WAIT_BACK = 1.1     # the waiting gripper hangs this far from the pick point towards the base,
                        # so the arm does not hide the pick point from the station camera

    def wait_posture(self, arm: str) -> Dict[str, float]:
        st = P.station(arm)
        (px, py), (bx, by) = st["pick"], st["base"]
        d = math.hypot(bx - px, by - py)
        k = min(self.WAIT_BACK, 0.6 * d) / max(d, 1e-6)
        return self.ik(arm, (px + (bx - px) * k, py + (by - py) * k, 2.0), base=st["base"])

    def ik(self, arm: str, xyz, seeds: Optional[List[Dict]] = None, base=None, tol: float = 0.02) -> Dict[str, float]:
        bx, by = base or self.arms[arm].base
        t = (xyz[0] - bx, xyz[1] - by, xyz[2] - L.ARM_BASE_Z)
        q, err = self.model.solve(t, self.model.seeds_towards(t, seeds))
        if err > tol:
            raise RuntimeError(f"{arm}: ({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) is out of reach ({err:.3f} m off)")
        return q

    def tip(self, arm: str, q: Optional[Dict] = None) -> np.ndarray:
        q = q if q is not None else self.arms[arm].q
        return (self.base_T(arm) @ np.append(self.model.tip(q), 1.0))[:3]

    def arm_points(self, arm: str) -> List[Tuple[float, float, float]]:
        frames = self.model.link_frames(self.arms[arm].q)
        B = self.base_T(arm)
        return [tuple((B @ frames[n])[:3, 3]) for n in self.model.links] + [tuple(self.tip(arm))]

    def move_arm(self, arm: str, q_goal: Dict[str, float], duration: float, stop: Optional[threading.Event] = None):
        """Interpolate the arm's joints to q_goal over `duration` s (joint 1 the short way round)."""
        a = self.arms[arm]
        q0 = dict(a.q)
        goal = dict(q_goal)
        d = goal["joint1"] - q0["joint1"]
        goal["joint1"] = q0["joint1"] + math.atan2(math.sin(d), math.cos(d))
        steps = max(1, int(duration * RATE))
        for i in range(1, steps + 1):
            if stop is not None and stop.is_set():
                raise InterruptedError("stopped")
            f = i / steps
            f = f * f * (3 - 2 * f)                                  # smooth start and stop
            with self.lock:
                a.q = {j: q0[j] + (goal[j] - q0[j]) * f for j in JOINTS}
                a.dirty = True
            time.sleep(1.0 / RATE)

    def attach(self, arm: str, product: str):
        with self.lock:
            p = self.products[product]
            p.state, p.conv, p.held = "carried", None, False
            self.arms[arm].carrying = product

    def release_on_conveyor(self, arm: str, conv: str, x: float, y: float):
        with self.lock:
            name = self.arms[arm].carrying
            self.arms[arm].carrying = None
            p = self.products[name]
            p.conv, p.s, p.state, p.held = conv, P.path_s(conv, x, y), "belt", False
            self._place_on_path(p)

    def release_into(self, arm: str, where: str, x: float, y: float, floor: float = BIN_FLOOR):
        """Let the carried product fall into a bin (or dock) at (x, y): it stays there."""
        with self.lock:
            name = self.arms[arm].carrying
            self.arms[arm].carrying = None
            p = self.products[name]
            p.state, p.conv, p.where = "falling", None, where
            z0 = p.z
        jitter = (random.uniform(-0.15, 0.15), random.uniform(-0.15, 0.15))
        z1 = floor + p.height / 2
        t = 0.0
        while True:                                                  # free fall, ~0.3 s
            t += 1.0 / RATE
            z = max(z1, z0 - 0.5 * 9.81 * t * t)
            with self.lock:
                p.x, p.y, p.z = x + jitter[0], y + jitter[1], z
            if z <= z1:
                break
            time.sleep(1.0 / RATE)
        with self.lock:
            p.state = "bin"
            self.counts[where] = self.counts.get(where, 0) + 1

    # ---------------------------------------------------------------- products
    def spawn(self, ptype: str, conv: str, s: float) -> str:
        if ptype not in PRODUCT_TYPES:
            raise ValueError(f"unknown product type {ptype}")
        name = f"order_{ptype}_{next(self._n)}"
        p = Product(name, ptype, conv, s)
        self._place_on_path(p)
        if not self.backend.create(name, product_sdf(name, ptype), p.x, p.y, p.z, quat(_rz(p.yaw))):
            raise RuntimeError(f"Gazebo did not create {name} (is the pipeline lab world running?)")
        with self.lock:
            self.products[name] = p
        return name

    def spawn_row(self, items: List[str], conv: str, spacing: float = 1.0, start_s: Optional[float] = None) -> List[str]:
        """Products `spacing` m apart, the first one at start_s (default: the start of a straight, or where
        the arm feeding a ring puts its products)."""
        ring = P.is_ring(conv)
        sign = P.ring_dir(conv) if ring else 1
        if start_s is None:
            if ring:      # just past the point where the feeding arm puts products on this ring
                feed = [pl for _b, _pk, (c, pl), _r in L.ARMS.values() if c == conv]
                start_s = (P.path_s(conv, *feed[0]) if feed else 0.0) + sign * 0.6
            else:
                start_s = 0.4 + (len(items) - 1) * spacing
        names = []
        for i, t in enumerate(items):
            s = start_s - sign * i * spacing
            if not ring:
                s = max(0.3, s)
            names.append(self.spawn(t, conv, s))
        return names

    def remove(self, name: str):
        with self.lock:
            self.products.pop(name, None)
        self.backend.remove(name)

    def clear(self):
        for name in list(self.products):
            self.remove(name)

    def _place_on_path(self, p: Product):
        x, y, heading = P.path_xy(p.conv, p.s)
        p.x, p.y, p.z, p.yaw = x, y, BELT_TOP + p.height / 2, heading

    def on_conveyor(self, conv: str) -> List[Product]:
        return [p for p in list(self.products.values()) if p.state == "belt" and p.conv == conv]

    def nearest(self, conv: str, x: float, y: float, radius: float) -> Optional[Product]:
        with self.lock:
            best = min(self.on_conveyor(conv), key=lambda p: math.hypot(p.x - x, p.y - y), default=None)
        return best if best is not None and math.hypot(best.x - x, best.y - y) <= radius else None

    def hold(self, name: str, on: bool = True):
        with self.lock:
            if name in self.products:
                self.products[name].held = on

    def place_free(self, conv: str, x: float, y: float, clearance: float = 0.5) -> bool:
        with self.lock:
            return all(math.hypot(p.x - x, p.y - y) > clearance for p in self.on_conveyor(conv))

    # ---------------------------------------------------------------- one step
    def step(self, dt: float):
        poses = []
        with self.lock:
            for conv in self.running_belts:
                if self.running_belts[conv]:
                    self._advance(conv, dt)
            for a in self.arms.values():
                if a.carrying:
                    p = self.products[a.carrying]
                    tip = self.tip(a.name)
                    p.x, p.y, p.z = float(tip[0]), float(tip[1]), float(tip[2]) - p.height / 2 - 0.005
                    p.yaw = a.q.get("joint1", 0.0) + a.q.get("joint7", 0.0)
                    a.dirty = True
                if a.dirty:
                    frames = self.model.link_frames(a.q)
                    B = self.base_T(a.name)
                    for link in self.model.links:
                        T = B @ frames[link]
                        poses.append((f"arm_{a.name}__{link}", float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), quat(T[:3, :3])))
                    a.dirty = False
            for p in self.products.values():
                if p.state in ("belt", "carried", "falling"):
                    poses.append((p.name, p.x, p.y, p.z, quat(_rz(p.yaw))))
        self.backend.set_poses(poses)

    def _advance(self, conv: str, dt: float):
        ps = self.on_conveyor(conv)
        if not ps:
            return
        ring = P.is_ring(conv)
        n = P.length(conv)
        sign = P.ring_dir(conv) if ring else 1
        # leaders first: furthest along the travel direction
        ps.sort(key=lambda p: -sign * p.s)
        for i, p in enumerate(ps):
            if p.held:
                continue
            ds = self.speed * dt
            limit = None
            if not ring:
                limit = n - END_MARGIN - p.size[0] / 2
            ahead = [q for q in ps if q is not p]
            for q in ahead:
                gap_needed = (p.size[0] + q.size[0]) / 2 + GAP
                d = (sign * (q.s - p.s)) % n if ring else q.s - p.s
                if 0 <= d < gap_needed + ds:
                    ds = min(ds, max(0.0, d - gap_needed))
            new_s = p.s + sign * ds
            if limit is not None and new_s > limit:
                new_s = max(p.s, limit)
                dock = P.dock_point(conv)
                if dock and new_s >= limit - 1e-6:
                    self._to_dock(p, conv, dock)
                    continue
            p.s = new_s % n if ring else new_s
            self._place_on_path(p)

    def _to_dock(self, p: Product, conv: str, dock):
        p.state, p.conv, p.where = "bin", None, f"dock_{conv}"
        p.x, p.y = dock[0] + random.uniform(-0.8, 0.8), dock[1] + random.uniform(-0.8, 0.8)
        p.z = 0.135 + p.height / 2
        self.counts[p.where] = self.counts.get(p.where, 0) + 1
        self.backend.set_poses([(p.name, p.x, p.y, p.z, quat(_rz(p.yaw)))])

    def summary(self) -> Dict[str, int]:
        with self.lock:
            out = {c: len(self.on_conveyor(c)) for c in self.running_belts}
        out.update(self.counts)
        return {k: v for k, v in out.items() if v}


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
