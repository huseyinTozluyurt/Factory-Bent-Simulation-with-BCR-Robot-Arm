#!/usr/bin/env python3
"""
BCR Endpoint Object Attacher - V2.9 Gripper Top Attach + Auto Select

attach_mode "gripper_top": on attach the object's top face is placed centered
on the endpoint frame (the top of the gripper), it follows that pose while
carried, and on release it is dropped from exactly the pose it was carried at.

auto_select_target: on attach, grab whichever candidate model (the orbiting
products of the dual bent) is closest to the gripper instead of target_model,
using that product's own size and colour.
"""

import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
import rclpy.duration
import rclpy.parameter
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from factory_bringup.product_catalog import (
    ORDER_PREFIX, PRODUCT_TYPES, apply_world_layout, geometry_xml, max_items, order_poses, product_sdf, product_visuals,
    refresh_custom_products, type_of,
)
import tf2_ros

# Prefer native gz-transport bindings: one set_pose call takes ~1 ms instead of
# the ~50-100 ms a `gz service` subprocess needs, so the object no longer lags
# behind the gripper. Falls back to the CLI when the bindings are missing.
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
    from gz.msgs10.twist_pb2 import Twist as GzTwist
except ImportError:
    GzNode = None

# Products of dual_circular_factory_conveyor.sdf: size (x, y, z) and RGB colour.
PRODUCT_CATALOG = {
    "pickup_box": ((0.34, 0.26, 0.26), (0.90, 0.25, 0.10)),
    "product_shipping_box_large": ((0.34, 0.26, 0.26), (0.45, 0.29, 0.13)),
    "product_milk_carton": ((0.16, 0.10, 0.34), (0.97, 0.97, 0.98)),
    "product_bottle_boxy": ((0.12, 0.12, 0.42), (0.25, 0.60, 0.95)),
    "product_sneaker_pair": ((0.36, 0.20, 0.12), (0.12, 0.12, 0.15)),
    "product_packet_pouch": ((0.18, 0.06, 0.24), (0.88, 0.80, 0.15)),
    "product_shoe_box": ((0.34, 0.20, 0.14), (0.14, 0.20, 0.45)),
    "product_long_package": ((0.48, 0.18, 0.18), (0.72, 0.20, 0.20)),
}

Pose7 = Tuple[float, float, float, float, float, float, float]


def rotate_vector_by_quat(v: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    rx = vx + 2.0 * qw * (qy*vz - qz*vy) + 2.0 * (qx*(qx*vx + qy*vy + qz*vz) - vx*(qx*qx + qy*qy + qz*qz))
    ry = vy + 2.0 * qw * (qz*vx - qx*vz) + 2.0 * (qy*(qx*vx + qy*vy + qz*vz) - vy*(qx*qx + qy*qy + qz*qz))
    rz = vz + 2.0 * qw * (qx*vy - qy*vx) + 2.0 * (qz*(qx*vx + qy*vy + qz*vz) - vz*(qx*qx + qy*qy + qz*qz))
    return rx, ry, rz


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return quat_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return quat_normalize((
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ))


def fmt_pose(p: Optional[Pose7]) -> str:
    if p is None:
        return "None"
    return f"xyz=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), quat=({p[3]:.3f}, {p[4]:.3f}, {p[5]:.3f}, {p[6]:.3f})"


class BCREndpointObjectAttacher(Node):
    def __init__(self):
        super().__init__("bcr_endpoint_object_attacher")

        self.declare_parameter("world_name", "dual_circular_factory_world")
        self.declare_parameter("target_model", "pickup_box")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("endpoint_frame", "gripper_tip")

        self.declare_parameter("base_x", -0.200)
        self.declare_parameter("base_y", -0.154)
        self.declare_parameter("base_z", -0.508)
        self.declare_parameter("base_yaw", 0.0)

        # gripper_top: object top face sits on the endpoint frame (+ contact_gap).
        # endpoint: object center sits on the endpoint frame + attach_offset only.
        self.declare_parameter("attach_mode", "gripper_top")
        self.declare_parameter("contact_gap", 0.005)

        # Extra fine-tuning offset, applied in "world" or "endpoint" frame.
        self.declare_parameter("attach_offset_frame", "world")
        self.declare_parameter("attach_offset_x", 0.0)
        self.declare_parameter("attach_offset_y", 0.0)
        self.declare_parameter("attach_offset_z", 0.0)

        self.declare_parameter("attach_rpy_roll", 0.0)
        self.declare_parameter("attach_rpy_pitch", 0.0)
        self.declare_parameter("attach_rpy_yaw", 0.0)
        self.declare_parameter("keep_object_world_flat", True)

        self.declare_parameter("ghost_carry_on_attach", True)
        self.declare_parameter("physical_release_on_release", True)
        self.declare_parameter("carry_static", True)
        self.declare_parameter("carry_gravity", False)
        self.declare_parameter("carry_collision", False)
        self.declare_parameter("release_static", False)
        self.declare_parameter("release_gravity", True)
        self.declare_parameter("release_collision", True)

        self.declare_parameter("respawn_sleep_sec", 0.20)
        self.declare_parameter("object_size_x", 0.34)
        self.declare_parameter("object_size_y", 0.26)
        self.declare_parameter("object_size_z", 0.26)
        self.declare_parameter("object_mass", 0.20)
        self.declare_parameter("object_ixx", 0.001)
        self.declare_parameter("object_iyy", 0.001)
        self.declare_parameter("object_izz", 0.001)

        self.declare_parameter("pick_x", -2.175)
        self.declare_parameter("pick_y", 1.195)
        self.declare_parameter("pick_z", 2.006)
        self.declare_parameter("pick_qx", 0.0)
        self.declare_parameter("pick_qy", 0.0)
        self.declare_parameter("pick_qz", 0.0)
        self.declare_parameter("pick_qw", 1.0)

        self.declare_parameter("exact_release", False)
        self.declare_parameter("release_x", -2.175)
        self.declare_parameter("release_y", 1.195)
        self.declare_parameter("release_z", 1.90)
        self.declare_parameter("release_qx", 0.0)
        self.declare_parameter("release_qy", 0.0)
        self.declare_parameter("release_qz", 0.0)
        self.declare_parameter("release_qw", 1.0)

        self.declare_parameter("update_hz", 25.0)
        self.declare_parameter("gz_timeout_ms", 500)
        self.declare_parameter("attach_settle_repeats", 3)
        self.declare_parameter("attach_settle_sleep_sec", 0.03)
        self.declare_parameter("release_settle_repeats", 3)
        self.declare_parameter("release_settle_sleep_sec", 0.05)
        self.declare_parameter("carry_retry_on_failure", True)
        self.declare_parameter("carry_retry_sleep_sec", 0.015)

        self.declare_parameter("auto_select_target", False)
        self.declare_parameter("auto_select_radius", 1.0)
        # When select_near_point is true, auto-select looks around (select_x, select_y)
        # (the object position the camera predicted) instead of around the gripper tip.
        self.declare_parameter("select_near_point", False)
        # Products are driven by VelocityControl. On attach: zero their cmd_vel so they stop
        # where they are, then lock them to the gripper with set_pose (no delete/respawn,
        # which could race with Gazebo and leave nothing to hold).
        self.declare_parameter("stop_and_follow_products", True)
        self.declare_parameter("select_x", 0.0)
        self.declare_parameter("select_y", 0.0)
        self.declare_parameter("candidate_models", list(PRODUCT_CATALOG.keys()))
        # /resume_held_object restarts a held product with the twist of the world file
        # (VelocityControl initial_linear = 3.42 * omega, initial_angular = omega, omega 0.07).
        self.declare_parameter("resume_linear_x", 0.2394)
        self.declare_parameter("resume_angular_z", 0.07)
        self.declare_parameter("hold_max_drift", 0.01)   # m the held product may still move in 0.25 s
        # Pin the held product: the products have no gravity and no damping, so when the gripper
        # touches one while coming down onto it, the push made it drift and turn for a while (and
        # it was then grabbed off its place). While held, its pose from the moment of the hold is
        # set again on every timer tick until the arm attaches it or it is resumed.
        self.declare_parameter("pin_held_object", True)
        # The whole bent: /stop_bent and /start_bent stop or restart every product on it.
        # On a hold the whole bent stops; once the arm has the product the rest move on.
        self.declare_parameter("hold_stops_bent", True)
        # true: /hold_object_for_pick only answers which product it would hold ({"model", "peek"}),
        # nothing stops (vision_pick_place asks first in a world without bent 2)
        self.declare_parameter("hold_peek", False)
        self.declare_parameter("resume_bent_on_attach", True)
        # A single `gz topic -p` per product is sometimes lost (the publisher exits before Gazebo
        # has connected), so only the target product stopped and the rest rode on. After a stop
        # or start every product on the bent is checked and the command is sent again to the
        # ones that did not follow it (up to twist_verify_rounds times).
        self.declare_parameter("twist_verify", True)
        self.declare_parameter("twist_verify_rounds", 4)
        # After the attach the rest of the bent waits this long (the arm lifts the product
        # clear) before it moves on again.
        self.declare_parameter("resume_bent_delay_sec", 2.0)
        # Release onto the belt: the object is put down resting on release_surface_z (its bottom
        # on the belt, clear of the gripper) instead of being re-created where it hung under the
        # tip, overlapping the gripper, which made Gazebo throw it away from the place point.
        self.declare_parameter("release_on_surface", True)
        self.declare_parameter("release_surface_z", 1.375)
        self.declare_parameter("release_use_yaw", False)   # true: put it down with yaw release_yaw
        self.declare_parameter("release_yaw", 0.0)
        # Bin drop (set by vision_pick_place for one release): the product is let go straight over
        # (release_snap_x, release_snap_y), the bin centre, at the height it was carried, and is
        # moved down to the bin floor (release_fall_floor_z) like a free fall, by set_pose steps (it
        # does not wait for Gazebo gravity, which left it hanging), then becomes a normal body there.
        self.declare_parameter("release_snap_xy", False)
        self.declare_parameter("release_snap_x", 0.0)
        self.declare_parameter("release_snap_y", 0.0)
        self.declare_parameter("release_fall_floor_z", 0.04)
        # Second bent: an object released onto it rides it like the products ride the first
        # one (VelocityControl orbit around its centre, counter-clockwise like bent 1).
        # /stop_bent2 and /start_bent2 stop/restart only those; /stop_bent1 (= /stop_bent)
        # and /start_bent1 (= /start_bent) only the first bent.
        self.declare_parameter("ring2_moving", True)
        self.declare_parameter("ring2_center_x", 6.0)
        self.declare_parameter("ring2_center_y", 0.0)
        self.declare_parameter("ring2_inner_radius", 2.95)
        self.declare_parameter("ring2_outer_radius", 4.05)
        self.declare_parameter("ring2_omega", 0.07)   # unused now: both bents run at belt_speed
        # One belt speed for both bents (m/s along the belt). Every start / resume / release
        # uses it, so a product moves equally fast on bent 1 and bent 2 at any radius
        # (angular speed = belt_speed / radius). 0.2394 = the world file's 3.42 m * 0.07 rad/s.
        self.declare_parameter("belt_speed", 0.2394)
        # Orders for bent 2: products ride its product path, the first one start_offset before
        # the release point (angle of (2.2171, -0.3736) around (6, 0)).
        self.declare_parameter("ring2_product_radius", 3.42)
        self.declare_parameter("ring2_spawn_reference_angle", -3.0431)
        # Simulation time (/factory_settings from the GUI): Gazebo speed (x real time) and an
        # optional run time after which the arm and both bents stop (0 = no limit).
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("physics_step", 0.001)
        self.declare_parameter("run_minutes", 0.0)
        # Product orders (/spawn_order from the Product Order GUI): new products are put on
        # bent 1 on its product path, in order, equal distances apart, riding like the others.
        self.declare_parameter("ring1_center_x", -6.0)
        self.declare_parameter("ring1_center_y", 0.0)
        self.declare_parameter("ring1_radius", 3.42)
        self.declare_parameter("ring1_product_bottom_z", 1.42)   # products float 4.5 cm over the belt
        self.declare_parameter("spawn_reference_angle", 0.0)     # catch station on bent 1 (rad around its centre)
        # One-command start (run_factory.sh / factory_full.launch.py): the order chosen in the GUI
        # before the start is spawned as soon as Gazebo is up, then the vision pick and place starts.
        self.declare_parameter("startup_order_file", "")          # JSON as on /spawn_order; "" or "none" = none
        self.declare_parameter("start_vision_after_order", True)  # call /vision_pick_place/start afterwards
        self.declare_parameter("startup_wait_sec", 120.0)         # how long to wait for Gazebo / the controller
        # World layout (product_catalog.WORLD_LAYOUTS by world_name / $FACTORY_WORLD_NAME): the
        # single circular and the straight world have no bent 2; the straight belt ends over a
        # collection box, and a product past bent1_end_y drops into it.
        self.declare_parameter("has_bent2", True)
        self.declare_parameter("bent1_path_length", 0.0)          # m of belt before the station for orders; 0 = whole ring
        self.declare_parameter("bent1_end", False)
        self.declare_parameter("bent1_end_y", 0.0)
        self.declare_parameter("collection_box_x", 0.0)
        self.declare_parameter("collection_box_y", 0.0)
        self.declare_parameter("collection_box_floor_z", 0.0)
        # experimental worlds with a straight bent 2: it runs -y and ends over collection box 2
        self.declare_parameter("bent2_path_length", 0.0)          # m of belt before the place point for orders
        self.declare_parameter("bent2_end", False)
        self.declare_parameter("bent2_end_y", 0.0)
        self.declare_parameter("collection_box2_x", 0.0)
        self.declare_parameter("collection_box2_y", 0.0)
        self.declare_parameter("collection_box2_floor_z", 0.0)
        apply_world_layout(self)

        self.attached = False
        self.ghost_carry_enabled_now = False
        self.last_object_pose: Optional[Pose7] = None
        self.last_respawn_ok = True

        self.attach_count = 0
        self.release_count = 0
        self.update_count = 0
        self.failed_update_count = 0
        self.respawn_count = 0
        self.failed_respawn_count = 0

        # Model chosen by auto_select_target for the current attach/carry/release.
        self.active_model: Optional[str] = None
        # True between /hold_object_for_pick and attach/resume: the product is stopped on the bent
        self.held = False
        self.pin_pose = None                 # pose the held product is kept at (pin_held_object)
        self.bent_stopped = False
        self.carry_lock = threading.Lock()   # the carry timer never moves the object during a release
        self.ring2_models = {}               # products now riding bent 2: name -> orbit radius
        self.ring2_stopped = False
        self.spawn_extra = None              # VelocityControl / inner yaw for the next respawn
        self.order_models = []               # products spawned from orders (still on bent 1)
        self.removed_models = set()          # products taken off bent 1 by an order with "clear"
        self.order_counter = 0
        self.result_pub = self.create_publisher(String, "/spawn_order/result", 10)
        self.create_subscription(String, "/spawn_order", self.on_spawn_order, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(String, "/factory_settings", self.on_settings, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.run_timer = None                # threading.Timer of run_minutes
        self.model_poses = {}
        self.model_poses_lock = threading.Lock()

        self.gz_node = GzNode() if GzNode is not None else None
        if self.gz_node is not None:
            topic = f"/world/{self.get_str('world_name')}/pose/info"
            try:
                self.gz_node.subscribe(GzPoseV, topic, self.on_gz_poses)
            except Exception as exc:
                self.get_logger().warn(f"Could not subscribe to {topic}: {exc}")
        # cmd_vel publishers advertised up front so discovery is done before the first catch
        self.cmd_vel_pubs = {}
        if self.gz_node is not None:
            for name in self.get_parameter("candidate_models").value:
                try:
                    self.cmd_vel_pubs[name] = self.gz_node.advertise(f"/model/{name}/cmd_vel", GzTwist)
                except Exception as exc:
                    self.get_logger().warn(f"Could not advertise /model/{name}/cmd_vel: {exc}")
        self.get_logger().info(
            "Using gz-transport Python bindings for set_pose." if self.gz_node
            else "gz-transport Python bindings not found; using `gz service` CLI (slower follow)."
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services, the carry timer and TF each in their own callback group, spun by a
        # MultiThreadedExecutor (main): with the slow `gz service` CLI the timer kept the
        # single-threaded node busy, TF updates queued behind it and the carried object
        # stayed at the pick pose.
        srv_cb = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "/attach_object_to_endpoint", self.attach_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/release_object_from_endpoint", self.release_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/move_object_to_endpoint_pick", self.move_to_pick_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/get_endpoint_attach_status", self.status_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/hold_object_for_pick", self.hold_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/resume_held_object", self.resume_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/stop_bent", self.stop_bent_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/start_bent", self.start_bent_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/stop_bent1", self.stop_bent_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/start_bent1", self.start_bent_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/stop_bent2", self.stop_bent2_callback, callback_group=srv_cb)
        self.create_service(Trigger, "/start_bent2", self.start_bent2_callback, callback_group=srv_cb)

        update_hz = max(1.0, self.get_float("update_hz"))
        self.timer = self.create_timer(1.0 / update_hz, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.box_counts = {}                 # collection box parameter prefix -> products dropped into it
        self.boxed = {}                      # product in a collection box -> its resting pose
        self.belt_end_idle = 0               # belt end checks without any pose from Gazebo
        if self.get_bool("bent1_end") or self.get_bool("bent2_end"):
            self.create_timer(0.1 if self.gz_node is not None else 0.5, self.belt_end_tick,
                              callback_group=MutuallyExclusiveCallbackGroup())
            for bent in (1, 2):
                if self.get_bool(f"bent{bent}_end"):
                    self.get_logger().info(f"Straight bent {bent}: products past y = {self.get_float(f'bent{bent}_end_y'):.2f} "
                                           "fall into its collection box and stay there.")

        self.vision_start_client = self.create_client(Trigger, "/vision_pick_place/start",
                                                      callback_group=MutuallyExclusiveCallbackGroup())
        self.vision_stop_client = self.create_client(Trigger, "/vision_pick_place/stop",
                                                     callback_group=MutuallyExclusiveCallbackGroup())
        if self.get_str("startup_order_file") not in ("", "none"):
            threading.Thread(target=self.run_startup_order, daemon=True).start()

        self.get_logger().info("BCR Stable Endpoint Attacher Node Ready.")

    def gazebo_ready(self) -> bool:
        try:
            res = subprocess.run(["gz", "service", "-l"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, timeout=5.0)
        except subprocess.TimeoutExpired:
            return False
        return f"/world/{self.get_str('world_name')}/create" in res.stdout

    def run_startup_order(self):
        """Spawn the order of startup_order_file once Gazebo is up, then start the vision pick and place."""
        path = Path(self.get_str("startup_order_file")).expanduser()
        try:
            order = json.loads(path.read_text())
        except Exception as exc:
            self.get_logger().error(f"Start-up order {path} not read: {exc}")
            return
        deadline = time.monotonic() + self.get_float("startup_wait_sec")
        while not self.gazebo_ready():
            if time.monotonic() > deadline:
                self.get_logger().error("Start-up order: Gazebo did not come up, nothing spawned.")
                return
            time.sleep(1.0)
        time.sleep(3.0)                       # world loaded, products moving
        # file: {"bent1": order, "bent2": order, "settings": {...}}  (or one bent-1 order, older format)
        orders = [dict(order.get("bent1", {}), bent=1), dict(order.get("bent2", {}), bent=2)] \
            if ("bent1" in order or "bent2" in order) else [dict(order, bent=1)]
        settings = dict(order.get("settings", {}))
        run_minutes = settings.pop("run_minutes", None)      # the clock starts with the arm, below
        if settings:
            self.on_settings(String(data=json.dumps(settings)))
        for o in orders:
            if o["bent"] == 2 and o.get("items") and not self.get_bool("has_bent2"):
                self.get_logger().info("Start-up order: this world has no bent 2; its part of the order is left out.")
                continue
            if not o.get("items"):
                if o["bent"] == 1:
                    self.get_logger().info("Start-up order: nothing chosen for bent 1, keeping the world's own products.")
                continue
            try:
                text, ok = self.spawn_order(o), True
            except Exception as exc:
                text, ok = f"Start-up order for bent {o['bent']} not spawned: {exc}", False
                self.get_logger().error(text)
            self.result_pub.publish(String(data=json.dumps({"ok": ok, "message": text})))
        if run_minutes:
            self.on_settings(String(data=json.dumps({"run_minutes": run_minutes})))
        if not self.get_bool("start_vision_after_order"):
            return
        while not self.vision_start_client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline + 60.0:
                self.get_logger().error("Start-up order: /vision_pick_place/start not available.")
                return
        future = self.vision_start_client.call_async(Trigger.Request())
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.1)
        res = future.result() if future.done() else None
        self.get_logger().info("Vision pick and place: " + (res.message if res else "no answer to start"))

    def get_str(self, name): return str(self.get_parameter(name).value)
    def get_float(self, name): return float(self.get_parameter(name).value)
    def get_bool(self, name): return bool(self.get_parameter(name).value)
    def get_int(self, name): return int(self.get_parameter(name).value)

    def model_name(self) -> str:
        return self.active_model or self.get_str("target_model")

    def object_size(self) -> Tuple[float, float, float]:
        if self.active_model in PRODUCT_CATALOG:
            return PRODUCT_CATALOG[self.active_model][0]
        if type_of(self.active_model):
            return PRODUCT_TYPES[type_of(self.active_model)]["size"]
        return self.get_float("object_size_x"), self.get_float("object_size_y"), self.get_float("object_size_z")

    def object_color(self) -> Tuple[float, float, float]:
        if self.active_model in PRODUCT_CATALOG:
            return PRODUCT_CATALOG[self.active_model][1]
        if type_of(self.active_model):
            return PRODUCT_TYPES[type_of(self.active_model)]["rgb"]
        return 0.9, 0.2, 0.1

    def object_shape(self) -> str:
        t = type_of(self.active_model)
        return PRODUCT_TYPES[t]["shape"] if t else "box"

    def on_gz_poses(self, msg):
        with self.model_poses_lock:
            for p in msg.pose:
                if p.name:
                    self.model_poses[p.name] = (p.position.x, p.position.y, p.position.z)

    def read_model_pose7(self, name: str):
        """(x, y, z, qx, qy, qz, qw) of one model from one /world/<w>/pose/info sample, or None."""
        cmd = ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{self.get_str('world_name')}/pose/info"]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3.0)
        except subprocess.TimeoutExpired:
            return None
        m = re.search(r'name:\s*"' + re.escape(name) + r'"(.*?)(?=\n\s*pose\s*\{|\Z)', res.stdout, re.S)
        if not m:
            return None
        body = m.group(1)
        def block(key):
            b = re.search(key + r"\s*\{([^}]*)\}", body)
            return dict(re.findall(r"(\w):\s*(-?[\d.eE+-]+)", b.group(1))) if b else {}
        p, q = block("position"), block("orientation")
        return (float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0)),
                float(q.get("x", 0)), float(q.get("y", 0)), float(q.get("z", 0)), float(q.get("w", 1 if not q else 0)))

    def read_model_poses(self):
        """Model positions from gz-transport, or one `gz topic -e` sample as a fallback."""
        with self.model_poses_lock:
            if self.model_poses:
                return dict(self.model_poses)
        cmd = ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{self.get_str('world_name')}/pose/info"]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3.0)
        except subprocess.TimeoutExpired:     # Gazebo not up yet, or another world is running
            return {}
        poses = {}
        for m in re.finditer(r'name:\s*"([^"]+)".*?position\s*\{([^}]*)\}', res.stdout, re.S):
            vals = dict(re.findall(r'(\w):\s*(-?[\d.eE+-]+)', m.group(2)))
            poses[m.group(1)] = (float(vals.get("x", 0)), float(vals.get("y", 0)), float(vals.get("z", 0)))
        return poses

    def select_nearest_candidate(self) -> str:
        if self.get_bool("select_near_point"):
            ref, ref_name = (self.get_float("select_x"), self.get_float("select_y")), "the detected object"
        else:
            ref, ref_name = self.get_endpoint_pose_world(), "the gripper"
        tip = ref
        poses = self.read_model_poses()
        # products now on bent 1: the world's ones that are left plus ordered ones (after an order
        # with "clear" none of the world's product_* models exist any more)
        if not any(name in poses for name in self.bent_models()):
            raise RuntimeError("no product poses from Gazebo (/world/<world>/pose/info) for the products on bent 1 "
                               f"{self.bent_models()}; check world_name '{self.get_str('world_name')}'")
        best, best_d = None, float("inf")
        for name in self.bent_models():
            if name in poses:
                d = math.hypot(poses[name][0] - tip[0], poses[name][1] - tip[1])
                if d < best_d:
                    best, best_d = name, d
        if best is None or best_d > self.get_float("auto_select_radius"):
            raise RuntimeError(f"no candidate model within {self.get_float('auto_select_radius'):.2f} m of {ref_name} "
                               f"(nearest: {best}, {best_d:.2f} m)")
        self.get_logger().info(f"Auto-selected {best} ({best_d:.2f} m from {ref_name}).")
        return best

    def get_attach_quat(self):
        return rpy_to_quat(self.get_float("attach_rpy_roll"), self.get_float("attach_rpy_pitch"), self.get_float("attach_rpy_yaw"))

    def get_pick_pose(self) -> Pose7:
        return self.get_float("pick_x"), self.get_float("pick_y"), self.get_float("pick_z"), 0.0, 0.0, 0.0, 1.0

    def get_release_pose(self) -> Pose7:
        return self.get_float("release_x"), self.get_float("release_y"), self.get_float("release_z"), 0.0, 0.0, 0.0, 1.0

    def get_endpoint_pose_world(self) -> Pose7:
        tf = self.tf_buffer.lookup_transform(self.get_str("base_frame"), self.get_str("endpoint_frame"), rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
        t = tf.transform.translation
        r = tf.transform.rotation

        # base_frame -> world: rotate by the arm's spawn yaw, then translate.
        base_yaw = self.get_float("base_yaw")
        base_quat = (0.0, 0.0, math.sin(base_yaw * 0.5), math.cos(base_yaw * 0.5))
        wx, wy, wz = rotate_vector_by_quat((t.x, t.y, t.z), base_quat)
        q = quat_multiply(base_quat, (r.x, r.y, r.z, r.w))
        return (
            self.get_float("base_x") + wx,
            self.get_float("base_y") + wy,
            self.get_float("base_z") + wz,
            q[0], q[1], q[2], q[3]
        )

    def get_attached_object_pose(self) -> Pose7:
        endpoint = self.get_endpoint_pose_world()
        endpoint_quat = (endpoint[3], endpoint[4], endpoint[5], endpoint[6])
        x, y, z = endpoint[0], endpoint[1], endpoint[2]

        if self.get_str("attach_mode") == "gripper_top":
            # Model origin is the box center, so hang it half its height below the tip.
            z -= 0.5 * self.object_size()[2] + self.get_float("contact_gap")

        offset = (self.get_float("attach_offset_x"), self.get_float("attach_offset_y"), self.get_float("attach_offset_z"))
        if self.get_str("attach_offset_frame") == "endpoint":
            offset = rotate_vector_by_quat(offset, endpoint_quat)
        x += offset[0]
        y += offset[1]
        z += offset[2]

        if self.get_bool("keep_object_world_flat"):
            siny_cosp = 2.0 * (endpoint_quat[3] * endpoint_quat[2] + endpoint_quat[0] * endpoint_quat[1])
            cosy_cosp = 1.0 - 2.0 * (endpoint_quat[1] * endpoint_quat[1] + endpoint_quat[2] * endpoint_quat[2])
            gripper_yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, z, 0.0, 0.0, math.sin(gripper_yaw * 0.5), math.cos(gripper_yaw * 0.5)

        return x, y, z, endpoint[3], endpoint[4], endpoint[5], endpoint[6]

    def set_gazebo_model_pose_once(self, pose: Pose7, name: Optional[str] = None) -> bool:
        x, y, z, qx, qy, qz, qw = pose
        name = name or self.model_name()
        service = f"/world/{self.get_str('world_name')}/set_pose"
        if self.gz_node is not None:
            req = GzPose()
            req.name = name
            req.position.x, req.position.y, req.position.z = x, y, z
            req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
            try:
                ok, rep = self.gz_node.request(service, req, GzPose, GzBoolean, self.get_int("gz_timeout_ms"))
                return bool(ok and rep.data)
            except Exception:
                return False

        req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        cmd = ["gz", "service", "-s", service, "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", str(self.get_int("gz_timeout_ms")), "--req", req]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1.0)
            return res.returncode == 0 and "data: false" not in res.stdout
        except Exception:
            return False

    def set_pose_repeated(self, pose: Pose7, repeats: int, sleep_sec: float) -> bool:
        ok = True
        for _ in range(max(1, repeats)):
            ok = self.set_gazebo_model_pose_once(pose) and ok
            time.sleep(max(0.0, sleep_sec))
        return ok

    def object_sdf_text(self, static_enabled: bool, gravity_enabled: bool, collision_enabled: bool) -> str:
        model = self.model_name()
        sx, sy, sz = self.object_size()
        r, g, b = self.object_color()
        static_s = "true" if static_enabled else "false"
        gravity_s = "true" if gravity_enabled else "false"
        extra = self.spawn_extra or {}
        inner = f'<pose>0 0 0 0 0 {extra.get("inner_yaw", 0.0)}</pose>' if extra else ""
        plugin = ""
        if "twist" in extra:
            lin, ang = extra["twist"]
            plugin = (f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
                      f'<initial_linear>{lin} 0 0</initial_linear><initial_angular>0 0 {ang}</initial_angular></plugin>')
        geom = geometry_xml(self.object_shape(), (sx, sy, sz))
        visuals = product_visuals(type_of(model), (sx, sy, sz), (r, g, b), self.object_shape(), float(extra.get("inner_yaw", 0.0)))
        collision_block = f'<collision name="collision">{inner}<geometry>{geom}</geometry></collision>' if collision_enabled else ""
        return f'<?xml version="1.0" ?><sdf version="1.8"><model name="{model}"><static>{static_s}</static>{plugin}<link name="link"><gravity>{gravity_s}</gravity><velocity_decay><linear>0</linear><angular>0</angular></velocity_decay><inertial><mass>{self.get_float("object_mass")}</mass><inertia><ixx>{self.get_float("object_ixx")}</ixx><iyy>{self.get_float("object_iyy")}</iyy><izz>{self.get_float("object_izz")}</izz></inertia></inertial>{collision_block}{visuals}</link></model></sdf>'

    def stop_model_motion(self, name: str, linear_x: float = 0.0, angular_z: float = 0.0, use_cli: bool = False):
        """Set the product's VelocityControl command (zero = it stops moving along the bent)."""
        pub = self.cmd_vel_pubs.get(name)
        if pub is not None and not use_cli:
            msg = GzTwist()
            msg.linear.x, msg.angular.z = linear_x, angular_z
            for _ in range(5):
                pub.publish(msg)
                time.sleep(0.02)
            return
        cmd = ["gz", "topic", "-t", f"/model/{name}/cmd_vel", "-m", "gz.msgs.Twist",
               "-p", f"linear: {{x: {linear_x}, y: 0, z: 0}}, angular: {{x: 0, y: 0, z: {angular_z}}}"]
        subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.0)

    def set_models_twist(self, names, linear_x: float, angular_z: float):
        """Send the same VelocityControl command to several products at once."""
        names = list(names)
        cli = [n for n in names if n not in self.cmd_vel_pubs]
        for _ in range(5):
            for n in names:
                if n in self.cmd_vel_pubs:
                    msg = GzTwist()
                    msg.linear.x, msg.angular.z = linear_x, angular_z
                    self.cmd_vel_pubs[n].publish(msg)
            time.sleep(0.02)
        # without gz-transport bindings: one `gz topic` per product, all started together
        procs = [subprocess.Popen(["gz", "topic", "-t", f"/model/{n}/cmd_vel", "-m", "gz.msgs.Twist", "-p",
                                   f"linear: {{x: {linear_x}, y: 0, z: 0}}, angular: {{x: 0, y: 0, z: {angular_z}}}"],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for n in cli]
        for p in procs:
            try:
                p.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                p.kill()

    def bent_models(self, exclude=()):
        """Products still on the first bent (not moved to bent 2, not removed by an order)."""
        names = list(self.get_parameter("candidate_models").value) + list(self.order_models)
        return [n for n in names if n not in exclude and n not in self.ring2_models and n not in self.removed_models]

    def remove_model_named(self, name: str) -> bool:
        cmd = ["gz", "service", "-s", f"/world/{self.get_str('world_name')}/remove", "--reqtype", "gz.msgs.Entity",
               "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", f'name: "{name}", type: MODEL']
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.0)
        return res.returncode == 0 and "data: false" not in res.stdout

    def on_spawn_order(self, msg: String):
        try:
            text = self.spawn_order(json.loads(msg.data))
            ok = True
        except Exception as exc:
            text, ok = f"Order not spawned: {exc}", False
            self.get_logger().warn(text)
        self.result_pub.publish(String(data=json.dumps({"ok": ok, "message": text})))

    def spawn_order(self, order) -> str:
        """
        Put an ordered row of products on bent 1: order = {"items": [type, ...], "spacing": m,
        "start_offset": m, "clear": bool}. Item 0 is start_offset m before the catch station
        and arrives there first, the next ones follow `spacing` m apart along the belt.
        """
        items = list(order.get("items", []))
        bent = int(order.get("bent", 1))
        if bent == 2:
            if not self.get_bool("has_bent2"):
                raise RuntimeError(f"the world {self.get_str('world_name')} has no bent 2")
            return self.spawn_order_bent2(order)
        spacing = float(order.get("spacing", 1.0))
        offset = float(order.get("start_offset", 1.5))
        radius = self.get_float("ring1_radius")
        self.check_order(items, spacing, offset, radius)

        busy = self.busy_models()
        was_running = not self.bent_stopped
        self.stop_bent(exclude=busy)          # nothing moves while the row is laid out
        removed = []
        if order.get("clear", True):
            for name in self.bent_models(exclude=busy):
                self.remove_model_named(name)
                self.removed_models.add(name)
                removed.append(name)
            self.order_models = [n for n in self.order_models if n not in removed]
            time.sleep(self.get_float("respawn_sleep_sec"))

        center = (self.get_float("ring1_center_x"), self.get_float("ring1_center_y"))
        poses = order_poses(len(items), spacing, offset, center, radius, self.get_float("spawn_reference_angle"))
        spawned = []
        for ptype, (x, y, yaw) in zip(items, poses):
            name = self.create_product(ptype, x, y, yaw)
            if name:
                self.order_models.append(name)
                spawned.append(name)
        if was_running:
            self.start_bent(exclude=busy)
        text = (f"Spawned {len(spawned)}/{len(items)} products {spacing:.2f} m apart, first {offset:.2f} m before "
                f"the station" + (f"; removed {len(removed)} from bent 1" if removed else "") + f": {spawned}")
        self.get_logger().info(text)
        return text

    def check_order(self, items, spacing, offset, radius, length_param="bent1_path_length"):
        refresh_custom_products()                 # products users added since start-up
        unknown = [t for t in items if t not in PRODUCT_TYPES]
        if unknown:
            raise RuntimeError(f"unknown product types {unknown}")
        if not items:
            raise RuntimeError("the order is empty")
        if spacing < 0.5:
            raise RuntimeError("spacing must be at least 0.5 m (largest product is 0.48 m long)")
        n_max = max_items(spacing, offset, radius)
        length = self.get_float(length_param)
        if length > 0:                       # straight belt: only what lies before the station
            n_max = min(n_max, max(0, int((length - offset) // spacing) + 1))
        if len(items) > n_max:
            raise RuntimeError(f"{len(items)} products do not fit at {spacing:.2f} m spacing (at most {n_max})")

    def create_product(self, ptype: str, x: float, y: float, yaw: float) -> Optional[str]:
        """Create order_<type>_<n> standing still at (x, y) on a belt, heading yaw; its name or None."""
        self.order_counter += 1
        name = f"{ORDER_PREFIX}{ptype}_{self.order_counter}"
        path = Path(f"/tmp/{name}.sdf")
        path.write_text(product_sdf(name, ptype, twist=(0.0, 0.0), mass=self.get_float("object_mass")))
        z = self.get_float("ring1_product_bottom_z") + 0.5 * PRODUCT_TYPES[ptype]["size"][2]
        req = (f'name: "{name}", sdf_filename: "{path}", allow_renaming: false, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
        cmd = ["gz", "service", "-s", f"/world/{self.get_str('world_name')}/create", "--reqtype", "gz.msgs.EntityFactory",
               "--reptype", "gz.msgs.Boolean", "--timeout", "1500", "--req", req]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.5)
        if res.returncode != 0 or "data: false" in res.stdout:
            self.get_logger().warn(f"Could not create {name}: {res.stdout.strip()} {res.stderr.strip()}")
            return None
        if self.gz_node is not None:
            try:
                self.cmd_vel_pubs[name] = self.gz_node.advertise(f"/model/{name}/cmd_vel", GzTwist)
            except Exception:
                pass
        return name

    def spawn_order_bent2(self, order) -> str:
        """Like spawn_order, on bent 2: the first product start_offset m before the release point."""
        items = list(order.get("items", []))
        spacing = float(order.get("spacing", 1.0))
        offset = float(order.get("start_offset", 1.5))
        radius = self.get_float("ring2_product_radius")
        self.check_order(items, spacing, offset, radius, "bent2_path_length")
        busy = self.busy_models()
        was_running = not self.ring2_stopped
        self.stop_bent2()
        removed = []
        if order.get("clear", True):
            for name in [n for n in self.ring2_models if n not in busy]:
                self.remove_model_named(name)
                self.ring2_models.pop(name, None)
                removed.append(name)
            time.sleep(self.get_float("respawn_sleep_sec"))
        center = (self.get_float("ring2_center_x"), self.get_float("ring2_center_y"))
        poses = order_poses(len(items), spacing, offset, center, radius, self.get_float("ring2_spawn_reference_angle"))
        spawned = []
        for ptype, (x, y, yaw) in zip(items, poses):
            name = self.create_product(ptype, x, y, yaw)
            if name:
                self.ring2_models[name] = radius
                spawned.append(name)
        if was_running:
            self.start_bent2()
        text = (f"Bent 2: spawned {len(spawned)}/{len(items)} products {spacing:.2f} m apart, first {offset:.2f} m "
                f"before the release point" + (f"; removed {len(removed)}" if removed else "") + f": {spawned}")
        self.get_logger().info(text)
        return text

    # ------------------------------------------------------------- speed and time
    def on_settings(self, msg: String):
        try:
            text, ok = self.apply_settings(json.loads(msg.data)), True
        except Exception as exc:
            text, ok = f"Settings not applied: {exc}", False
            self.get_logger().warn(text)
        self.result_pub.publish(String(data=json.dumps({"ok": ok, "message": text})))

    def apply_settings(self, settings) -> str:
        """{"belt_speed": m/s, "real_time_factor": x, "run_minutes": min}; any of them."""
        done = []
        if "belt_speed" in settings:
            v = float(settings["belt_speed"])
            if not 0.0 < v <= 1.0:
                raise RuntimeError("belt speed must be between 0 and 1 m/s")
            self.set_parameters([rclpy.parameter.Parameter("belt_speed", value=v)])
            busy = self.busy_models()
            if not self.bent_stopped:
                self.start_bent(exclude=busy)
            if not self.ring2_stopped and self.ring2_models:
                self.start_bent2()
            done.append(f"belt speed {v:.3f} m/s on both bents")
        if "real_time_factor" in settings:
            rtf = float(settings["real_time_factor"])
            if not 0.05 <= rtf <= 10.0:
                raise RuntimeError("simulation speed must be between 0.05 and 10 x real time")
            req = f"max_step_size: {self.get_float('physics_step')}, real_time_factor: {rtf}"
            cmd = ["gz", "service", "-s", f"/world/{self.get_str('world_name')}/set_physics", "--reqtype", "gz.msgs.Physics",
                   "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3.0)
            if res.returncode != 0 or "data: true" not in res.stdout:
                raise RuntimeError(f"Gazebo did not accept the simulation speed: {res.stdout.strip()} {res.stderr.strip()}")
            self.set_parameters([rclpy.parameter.Parameter("real_time_factor", value=rtf)])
            done.append(f"simulation speed {rtf:.2f}x real time")
        if "run_minutes" in settings:
            minutes = float(settings["run_minutes"])
            if self.run_timer is not None:
                self.run_timer.cancel()
                self.run_timer = None
            if minutes > 0:
                self.run_timer = threading.Timer(minutes * 60.0, self.end_of_run)
                self.run_timer.daemon = True
                self.run_timer.start()
                done.append(f"run time {minutes:g} min from now")
            else:
                done.append("no run time limit")
            self.set_parameters([rclpy.parameter.Parameter("run_minutes", value=minutes)])
        text = "Settings: " + (", ".join(done) if done else "nothing to change")
        self.get_logger().info(text)
        return text

    def end_of_run(self):
        """Run time is over: the arm stops after its current move, both bents stop."""
        self.run_timer = None
        if self.vision_stop_client.wait_for_service(timeout_sec=1.0):
            self.vision_stop_client.call_async(Trigger.Request())
        self.stop_bent(exclude=self.busy_models())
        self.stop_bent2()
        text = "Run time over: pick and place stopped, both bents stopped."
        self.get_logger().info(text)
        self.result_pub.publish(String(data=json.dumps({"ok": True, "message": text})))

    def ring1_twist(self):
        """(linear, angular) of a product on bent 1 at belt_speed."""
        v = self.get_float("belt_speed")
        return v, v / self.get_float("ring1_radius")

    def ring2_twist(self, name: str):
        """(linear, angular) of an object on bent 2 at belt_speed, for its own orbit radius."""
        v = self.get_float("belt_speed")
        return v, v / max(0.5, self.ring2_models[name])

    def stop_bent2(self):
        """Stop the second bent: every object riding it stands still."""
        names = list(self.ring2_models)
        self.set_models_twist(names, 0.0, 0.0)
        self.ring2_stopped = True
        self.get_logger().info(f"Bent 2 stopped ({len(names)} objects).")
        return names

    def start_bent2(self):
        """Run the second bent again: every object on it moves on along the ring."""
        names = list(self.ring2_models)
        for n in names:              # orbit radius differs per object, so one command each
            lin, ang = self.ring2_twist(n)
            self.set_models_twist([n], lin, ang)
        self.ring2_stopped = False
        self.get_logger().info(f"Bent 2 running ({len(names)} objects).")
        return names

    def stop_bent2_callback(self, request, response):
        try:
            names = self.stop_bent2()
            response.success, response.message = True, f"Bent 2 stopped: {len(names)} objects."
        except Exception as exc:
            response.success, response.message = False, f"Error: {exc}"
        return response

    def start_bent2_callback(self, request, response):
        try:
            names = self.start_bent2()
            response.success, response.message = True, f"Bent 2 running: {len(names)} objects."
        except Exception as exc:
            response.success, response.message = False, f"Error: {exc}"
        return response

    def stop_bent(self, exclude=()):
        """Stop the bent: every product on it stands still (except `exclude`)."""
        names = self.bent_models(exclude)
        self.bent_stopped = True
        self.set_models_twist(names, 0.0, 0.0)
        self.verify_twist(names, moving=False)
        self.get_logger().info(f"Bent stopped ({len(names)} products).")
        return names

    def start_bent(self, exclude=()):
        """Restart the bent: every product (except `exclude`) moves on along the ring again."""
        names = self.bent_models(exclude)
        self.bent_stopped = False
        self.set_models_twist(names, *self.ring1_twist())
        self.verify_twist(names, moving=True)
        self.get_logger().info(f"Bent running again ({len(names)} products" + (f", not {list(exclude)}" if exclude else "") + ").")
        return names

    def verify_twist(self, names, moving: bool):
        """Check that `names` stand still (moving=False) or ride the belt (moving=True), and send
        the command again, one `gz topic` per product, to the ones that did not follow it."""
        if not self.get_bool("twist_verify") or not names:
            return
        twist = self.ring1_twist() if moving else (0.0, 0.0)
        tol = self.get_float("hold_max_drift")
        wrong = list(names)
        for _ in range(max(1, self.get_int("twist_verify_rounds"))):
            if moving != (not self.bent_stopped):
                return                        # stopped/started again meanwhile: that call checks
            t1, p1 = time.monotonic(), self.read_model_poses()
            time.sleep(0.25)
            t2, p2 = time.monotonic(), self.read_model_poses()
            expect = twist[0] * (t2 - t1)
            wrong = []
            for n in names:
                if n not in p1 or n not in p2:
                    continue
                d = math.hypot(p2[n][0] - p1[n][0], p2[n][1] - p1[n][1])
                if (d > tol) if not moving else (d < 0.3 * expect):
                    wrong.append(n)
            if not wrong:
                return
            self.get_logger().info(f"{len(wrong)} products did not {'start' if moving else 'stop'}: "
                                   f"sending the command again to {wrong}")
            procs = [subprocess.Popen(["gz", "topic", "-t", f"/model/{n}/cmd_vel", "-m", "gz.msgs.Twist", "-p",
                                       f"linear: {{x: {twist[0]}, y: 0, z: 0}}, angular: {{x: 0, y: 0, z: {twist[1]}}}"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for n in wrong]
            for p in procs:
                try:
                    p.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    p.kill()
            time.sleep(0.1)
        self.get_logger().warn(f"Products still not {'moving' if moving else 'stopped'}: {wrong}")

    def resume_bent_later(self, name: str):
        """After an attach: the rest of the bent moves on once the arm has lifted the product."""
        def run():
            time.sleep(max(0.0, self.get_float("resume_bent_delay_sec")))
            if self.bent_stopped and not self.held:
                self.start_bent(exclude=[name] + self.busy_models())
        threading.Thread(target=run, daemon=True).start()

    def busy_models(self):
        """The product the arm has or is holding still: never restarted by /start_bent."""
        return [self.active_model] if self.active_model and (self.attached or self.held) else []

    def stop_bent_callback(self, request, response):
        try:
            names = self.stop_bent()
            response.success, response.message = True, f"Bent stopped: {len(names)} products."
        except Exception as exc:
            response.success, response.message = False, f"Error: {exc}"
        return response

    def start_bent_callback(self, request, response):
        try:
            busy = self.busy_models()
            names = self.start_bent(exclude=busy)
            response.success = True
            response.message = f"Bent running: {len(names)} products" + (f" (kept {busy[0]} with the arm)" if busy else "") + "."
        except Exception as exc:
            response.success, response.message = False, f"Error: {exc}"
        return response

    def model_drift(self, name: str, wait: float = 0.25):
        """Pose of the model and how far it moved in `wait` seconds."""
        p1 = self.read_model_poses().get(name)
        time.sleep(wait)
        p2 = self.read_model_poses().get(name)
        if p1 is None or p2 is None:
            raise RuntimeError(f"no Gazebo pose for {name}")
        return p2, math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def hold_callback(self, request, response):
        """Stop the product nearest (select_x, select_y) on the bent so the arm can come down onto it."""
        try:
            if self.attached:
                raise RuntimeError(f"already carrying {self.model_name()}")
            if self.held and self.active_model:
                name = self.active_model
            else:
                name = self.select_nearest_candidate()
            if self.get_bool("hold_peek"):
                response.success = True
                response.message = json.dumps({"model": name, "peek": True})
                return response
            self.active_model = name
            if self.get_bool("hold_stops_bent"):
                self.stop_bent()          # the whole bent stops, not only this product
            else:
                self.stop_model_motion(name)
            time.sleep(0.1)
            pose, drift = self.model_drift(name)
            if drift > self.get_float("hold_max_drift"):
                # gz-transport publish did not reach VelocityControl: try the CLI once
                self.stop_model_motion(name, use_cli=True)
                time.sleep(0.2)
                pose, drift = self.model_drift(name)
            if drift > self.get_float("hold_max_drift"):
                raise RuntimeError(f"{name} keeps moving ({drift / 0.25:.2f} m/s) after a zero /model/{name}/cmd_vel; "
                                   "is its VelocityControl plugin listening on that topic?")
            if self.get_bool("pin_held_object"):
                full = self.read_model_pose7(name)
                self.pin_pose = full
                if full is None:
                    self.get_logger().warn(f"No full pose of {name} from Gazebo: it is held but not pinned.")
            self.held = True
            size = self.object_size()
            info = {"model": name, "x": pose[0], "y": pose[1], "z": pose[2], "size": list(size),
                    "top_z": pose[2] + 0.5 * size[2]}
            self.get_logger().info(f"Holding {name} still at ({pose[0]:.3f}, {pose[1]:.3f}), top z {info['top_z']:.3f}.")
            response.success = True
            response.message = json.dumps(info)
            return response
        except Exception as exc:
            if not self.held:
                self.active_model = None
            response.success = False
            response.message = f"Error: {exc}"
            return response

    def resume_callback(self, request, response):
        """Let a held (not attached) product carry on along the bent."""
        try:
            if self.attached:
                raise RuntimeError(f"{self.model_name()} is attached; release it instead")
            if not self.held or not self.active_model:
                raise RuntimeError("no product is held")
            name = self.active_model
            with self.carry_lock:
                self.pin_pose = None
            if self.bent_stopped:
                self.start_bent()
            else:
                self.stop_model_motion(name, *self.ring1_twist())
            self.held = False
            self.active_model = None
            response.success = True
            response.message = f"Resumed {name} along the bent."
            return response
        except Exception as exc:
            response.success = False
            response.message = f"Error: {exc}"
            return response

    def is_moving_product(self) -> bool:
        return self.get_bool("stop_and_follow_products") and (self.model_name() in PRODUCT_CATALOG or type_of(self.model_name()) is not None)

    def remove_model(self) -> bool:
        cmd = ["gz", "service", "-s", f"/world/{self.get_str('world_name')}/remove", "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", f'name: "{self.model_name()}", type: MODEL']
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.0)
        return res.returncode == 0 and "data: false" not in res.stdout

    def create_model_from_file(self, pose: Pose7, static_enabled: bool, gravity_enabled: bool, collision_enabled: bool) -> bool:
        mode = f"{'st' if static_enabled else 'dy'}_{'g1' if gravity_enabled else 'g0'}_{'c1' if collision_enabled else 'c0'}"
        if self.spawn_extra:
            mode += "_ring2"
        path = Path(f"/tmp/{self.model_name()}_{mode}.sdf")
        path.write_text(self.object_sdf_text(static_enabled, gravity_enabled, collision_enabled))
        req = f'name: "{self.model_name()}", sdf_filename: "{str(path)}", allow_renaming: false, pose: {{position: {{x: {pose[0]}, y: {pose[1]}, z: {pose[2]}}}, orientation: {{x: {pose[3]}, y: {pose[4]}, z: {pose[5]}, w: {pose[6]}}}}}'
        cmd = ["gz", "service", "-s", f"/world/{self.get_str('world_name')}/create", "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean", "--timeout", "1500", "--req", req]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.5)
        return res.returncode == 0 and "data: false" not in res.stdout

    def respawn_model(self, pose: Pose7, static_enabled: bool, gravity_enabled: bool, collision_enabled: bool) -> bool:
        self.remove_model()
        time.sleep(self.get_float("respawn_sleep_sec"))
        # create fails while Gazebo still has the old model under this name: retry until it is gone
        for _ in range(10):
            if self.create_model_from_file(pose, static_enabled, gravity_enabled, collision_enabled):
                time.sleep(self.get_float("respawn_sleep_sec"))
                return True
            time.sleep(0.2)
        self.get_logger().error(f"Could not re-create {self.model_name()} after removing it.")
        return False

    def status_message(self):
        return f"attached={self.attached}, held={self.held}, model={self.model_name()}, last_pose={fmt_pose(self.last_object_pose)}"

    def attach_callback(self, request, response):
        try:
            with self.carry_lock:
                self.pin_pose = None         # the gripper takes it from here
            # a product stopped by /hold_object_for_pick is grabbed as is, without re-selecting
            if self.get_bool("auto_select_target") and not (self.held and self.active_model):
                self.active_model = self.select_nearest_candidate()
            object_pose = self.get_attached_object_pose()

            if self.is_moving_product():
                # stop it on the bent first, then it is simply carried by set_pose each tick
                self.stop_model_motion(self.model_name())
            # Ghost first, so the teleport to the gripper cannot collide with the arm.
            elif self.get_bool("ghost_carry_on_attach"):
                self.respawn_model(object_pose, static_enabled=True, gravity_enabled=False, collision_enabled=False)

            # Re-read the tip: the arm may have moved during the respawn.
            object_pose = self.get_attached_object_pose()
            ok = self.set_pose_repeated(object_pose, self.get_int("attach_settle_repeats"), self.get_float("attach_settle_sleep_sec"))
            if not ok:
                raise RuntimeError(f"Gazebo set_pose failed for {self.model_name()} (world '{self.get_str('world_name')}')")
            self.last_object_pose = object_pose
            self.update_count = 0
            self.failed_update_count = 0
            self.attached = True
            self.held = False
            if self.bent_stopped and self.get_bool("resume_bent_on_attach"):
                self.resume_bent_later(self.model_name())      # the arm has it: the others move on

            response.success = True
            response.message = f"Attached {self.model_name()} to {self.get_str('endpoint_frame')} at {fmt_pose(object_pose)}"
            return response
        except Exception as exc:
            if not self.held:
                self.active_model = None   # a held product stays held so it can be resumed
            response.success = False
            response.message = f"Error: {exc}"
            return response

    def release_callback(self, request, response):
        try:
            # Stop following first (and wait for a carry update in flight), then freeze the pose.
            self.attached = False
            with self.carry_lock:
                pass
            try:
                carried = self.get_attached_object_pose()
            except Exception:
                carried = self.last_object_pose
            if self.get_bool("exact_release"):
                release_pose = self.get_release_pose()
            else:
                release_pose = carried
            if release_pose is None:
                raise RuntimeError("no carried pose known; attach first")
            x, y, z, qx, qy, qz, qw = release_pose
            snapped = self.get_bool("release_snap_xy")
            if snapped:
                x, y = self.get_float("release_snap_x"), self.get_float("release_snap_y")
            if self.get_bool("release_on_surface"):
                z = self.get_float("release_surface_z") + 0.5 * self.object_size()[2] + 0.003
            if self.get_bool("release_use_yaw"):
                yaw = self.get_float("release_yaw")
                qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
            release_pose = (x, y, z, qx, qy, qz, qw)
            offset = None if carried is None else math.hypot(carried[0] - x, carried[1] - y)

            self.set_pose_repeated(release_pose, self.get_int("release_settle_repeats"), self.get_float("release_settle_sleep_sec"))

            # Respawn as a dynamic body at the same pose; gravity then drops it straight down.
            if self.get_bool("physical_release_on_release"):
                cx, cy = self.get_float("ring2_center_x"), self.get_float("ring2_center_y")
                r2 = math.hypot(x - cx, y - cy)
                if self.get_bool("ring2_moving") and self.get_float("ring2_inner_radius") <= r2 <= self.get_float("ring2_outer_radius"):
                    # on bent 2: model heads along the ring (counter-clockwise tangent) and orbits it;
                    # the box itself keeps the yaw it was put down with (inner pose)
                    heading = math.atan2(y - cy, x - cx) + math.pi / 2
                    yaw = 2.0 * math.atan2(qz, qw)
                    name = self.model_name()
                    self.ring2_models[name] = r2
                    twist = (0.0, 0.0) if self.ring2_stopped else self.ring2_twist(name)
                    self.spawn_extra = {"twist": twist, "inner_yaw": yaw - heading}
                    try:
                        model_pose = (x, y, z, 0.0, 0.0, math.sin(heading / 2), math.cos(heading / 2))
                        self.respawn_model(model_pose, static_enabled=False, gravity_enabled=False, collision_enabled=True)
                    finally:
                        self.spawn_extra = None
                else:
                    if snapped:
                        release_pose = self.drop_into_bin(release_pose)
                    self.respawn_model(release_pose, static_enabled=False, gravity_enabled=True, collision_enabled=True)
                    # dropped off the bents (into a trash / recycling bin): no longer part of bent 1,
                    # so /stop_bent and /start_bent leave it alone
                    self.removed_models.add(self.model_name())
            self.last_object_pose = release_pose

            response.success = True
            response.message = (f"Released {self.model_name()} at {fmt_pose(release_pose)} "
                                + (f"({offset:.3f} m from where the gripper carried it) " if offset is not None else "")
                                + f"(carry updates ok {self.update_count}, failed {self.failed_update_count})")
            self.active_model = None
            self.held = False
            return response
        except Exception as exc:
            response.success = False
            response.message = f"Error: {exc}"
            return response

    def drop_into_bin(self, pose: Pose7) -> Pose7:
        """Let the product fall straight down from pose onto the bin floor (set_pose steps timed as a
        free fall), standing upright with its yaw. Returns the pose it ends at."""
        x, y, z0, qx, qy, qz, qw = pose
        z1 = self.get_float("release_fall_floor_z") + 0.5 * self.object_size()[2] + 0.005
        yaw = 2.0 * math.atan2(qz, qw)
        end = (x, y, z1, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        t_fall = math.sqrt(2.0 * max(0.0, z0 - z1) / 9.81)
        start = time.time()
        while time.time() - start < t_fall:
            t = time.time() - start
            self.set_gazebo_model_pose_once((x, y, z0 - 0.5 * 9.81 * t * t, qx, qy, qz, qw))
            time.sleep(0.02)
        self.set_pose_repeated(end, 3, 0.05)
        self.get_logger().info(f"Dropped {self.model_name()} into the bin: z {z0:.2f} -> {z1:.2f} in {t_fall:.2f} s.")
        return end

    def box_slot(self, box: str, direction: int, k: int, size) -> Tuple[float, float, float]:
        """Resting place of the k-th product in a collection box (parameters box_x / _y / _floor_z):
        rows from the wall next to the belt end inwards (the belt runs in direction +1 = +y, -1 = -y),
        3 across, 6 rows deep (0.6 .. 5.1 m from that wall), then a layer higher."""
        x = self.get_float(f"{box}_x") + (0, -1, 1)[k % 3] * 1.1      # under the belt first
        y = self.get_float(f"{box}_y") + direction * (-2.3 + (k // 3 % 6) * 0.9)
        z = self.get_float(f"{box}_floor_z") + (k // 18) * 0.45 + 0.5 * size[2] + 0.005
        return x, y, z

    def belt_end_tick(self):
        """Straight bents: a product whose centre passes the end of the belt tips off, falls into
        that belt's collection box (free fall, g = 9.81) and stops there; products in a box are held still.
        An error here is logged, never raised: it would stop the executor and with it the whole node,
        and the products would then ride off the belt end and hang in the air (they have no gravity)."""
        try:
            self.belt_end_check()
        except Exception as exc:
            self.get_logger().warn(f"Belt end check failed: {exc}", throttle_duration_sec=2.0)

    def belt_end_check(self):
        # only the gz-transport pose stream here: the `gz topic` fallback blocks for seconds
        if self.gz_node is not None:
            with self.model_poses_lock:
                poses = dict(self.model_poses)
        else:
            poses = self.read_model_poses()
        if not poses:
            self.belt_end_idle += 1
            if self.belt_end_idle == 300:        # 30 s at 0.1 s
                self.get_logger().warn(f"No model poses from /world/{self.get_str('world_name')}/pose/info yet: "
                                       "products cannot drop into the collection box. Is Gazebo running this world "
                                       "(and not paused)?")
            return
        # products already in a box: if anything moved one (a late belt command), stop it and put it back
        for name, rest in list(self.boxed.items()):
            p = poses.get(name)
            if p is not None and math.dist(p, rest[:3]) > 0.03:
                self.stop_model_motion(name)
                self.set_gazebo_model_pose_once(rest, name)
        busy = self.busy_models()
        ends = []                            # (bent, direction of travel along y, end y, box, products on it)
        if self.get_bool("bent1_end"):
            ends.append((1, 1, self.get_float("bent1_end_y"), "collection_box", self.bent_models(exclude=busy)))
        if self.get_bool("bent2_end"):
            ends.append((2, -1, self.get_float("bent2_end_y"), "collection_box2",
                         [n for n in list(self.ring2_models) if n not in busy]))
        for bent, direction, end, box, names in ends:
            for name in names:
                p = poses.get(name)
                if p is None or direction * (p[1] - end) <= 0 or name in self.boxed:
                    continue
                self.drop_into_box(name, p, bent, direction, box)

    def drop_into_box(self, name: str, p, bent: int, direction: int, box: str):
        self.removed_models.add(name)            # off the belt: bent stop / start leave it alone
        self.ring2_models.pop(name, None)
        self.stop_model_motion(name)
        k = self.box_counts.get(box, 0)
        self.box_counts[box] = k + 1
        size = PRODUCT_TYPES.get(type_of(name) or "", {}).get("size", (0.3, 0.3, 0.3))
        tx, ty, tz = self.box_slot(box, direction, k, size)
        x0, y0, z0 = p
        # free fall from the belt end to the box floor; forward and sideways at a steady speed (it
        # tips off the belt end and clears the box's wall), so the path is a parabola
        t_fall = math.sqrt(2.0 * max(0.05, z0 - tz) / 9.81)
        yaw = direction * math.pi / 2                    # still heading the way the belt ran
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        start = time.time()
        while True:
            t = time.time() - start
            if t >= t_fall:
                break
            f = t / t_fall
            self.set_gazebo_model_pose_once((x0 + (tx - x0) * f, y0 + (ty - y0) * f, z0 - 0.5 * 9.81 * t * t,
                                             0.0, 0.0, qz, qw), name)
            time.sleep(0.02)
        rest = (tx, ty, tz, 0.0, 0.0, qz, qw)
        self.boxed[name] = rest
        self.stop_model_motion(name)             # once more after landing, then hold it there
        landed = False
        for _ in range(10):
            landed = self.set_gazebo_model_pose_once(rest, name) or landed
            time.sleep(0.05)
            if landed:
                break
        if not landed:                           # the boxed check keeps trying every tick
            self.get_logger().warn(f"set_pose for {name} into the collection box failed "
                                   f"(world '{self.get_str('world_name')}'); retrying.")
        self.get_logger().info(f"{name} fell off the end of bent {bent} into its collection box "
                               f"(z {z0:.2f} -> {tz:.2f} in {t_fall:.2f} s, {k + 1} in that box).")

    def move_to_pick_callback(self, request, response):
        try:
            self.attached = False
            pose = self.get_pick_pose()
            self.respawn_model(pose, static_enabled=False, gravity_enabled=True, collision_enabled=True)
            response.success = self.set_pose_repeated(pose, 3, 0.03)
            return response
        except Exception as exc:
            response.success = False
            return response

    def status_callback(self, request, response):
        response.success = True
        response.message = self.status_message()
        return response

    def timer_callback(self):
        with self.carry_lock:
            self.carry_tick()

    def carry_tick(self):
        if not self.attached:
            if self.held and self.pin_pose is not None and self.active_model:
                self.set_gazebo_model_pose_once(self.pin_pose)     # undo any push of the gripper
            return
        try:
            pose = self.get_attached_object_pose()
        except Exception as exc:
            self.get_logger().warn(f"Carry: no gripper pose from TF ({exc}); object not moved.", throttle_duration_sec=2.0)
            return
        ok = self.set_gazebo_model_pose_once(pose)
        if not ok and self.get_bool("carry_retry_on_failure"):
            time.sleep(self.get_float("carry_retry_sleep_sec"))
            ok = self.set_gazebo_model_pose_once(pose)
        if ok:
            self.last_object_pose = pose
            self.update_count += 1
        else:
            self.failed_update_count += 1
            self.get_logger().warn(f"Carry: set_pose for {self.model_name()} failed "
                                   f"({self.failed_update_count} failures so far).", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = BCREndpointObjectAttacher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()