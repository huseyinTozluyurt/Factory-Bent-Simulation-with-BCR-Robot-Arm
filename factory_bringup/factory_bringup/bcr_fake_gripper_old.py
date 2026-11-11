#!/usr/bin/env python3
"""
BCR Fake Gripper V4.2 - Stable Hold + Pick Zone + Precise Place Version

Services:
  /pick_object   std_srvs/srv/Trigger
  /place_object  std_srvs/srv/Trigger

V4 purpose:
  - Stable object holding using the tested good offset:
      hold_offset_x=-0.12, hold_offset_y=0.0, hold_offset_z=-0.08
  - debug_mode=True ignores distance rejection for development.
  - debug_mode=False uses max_pick_distance for realistic demo safety.
  - Better response/debug messages for tool pose, object pose, distance, and hold offset.
  - Pick-zone support for picking from a fixed conveyor/gripper location.
  - Optional /move_object_to_pick_zone service for repeatable terminal/GUI tests.
"""

import math
import re
import subprocess
from typing import Optional, Tuple

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
import time


Pose7 = Tuple[float, float, float, float, float, float, float]


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return quat_normalize((
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ))


def quat_inverse(q):
    x, y, z, w = quat_normalize(q)
    return -x, -y, -z, w


def rotate_vector(q, v):
    q = quat_normalize(q)
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    q_conj = quat_inverse(q)
    out = quat_multiply(quat_multiply(q, qv), q_conj)
    return out[0], out[1], out[2]


def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return quat_normalize((qx, qy, qz, qw))


def compose_pose(a: Pose7, b: Pose7) -> Pose7:
    ax, ay, az, aqx, aqy, aqz, aqw = a
    bx, by, bz, bqx, bqy, bqz, bqw = b
    rx, ry, rz = rotate_vector((aqx, aqy, aqz, aqw), (bx, by, bz))
    q = quat_multiply((aqx, aqy, aqz, aqw), (bqx, bqy, bqz, bqw))
    return ax + rx, ay + ry, az + rz, q[0], q[1], q[2], q[3]


def inverse_pose(p: Pose7) -> Pose7:
    x, y, z, qx, qy, qz, qw = p
    q_inv = quat_inverse((qx, qy, qz, qw))
    tx, ty, tz = rotate_vector(q_inv, (-x, -y, -z))
    return tx, ty, tz, q_inv[0], q_inv[1], q_inv[2], q_inv[3]


def relative_pose(parent_world: Pose7, child_world: Pose7) -> Pose7:
    return compose_pose(inverse_pose(parent_world), child_world)


def pose_distance(a: Pose7, b: Pose7) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def fmt_xyz(p: Optional[Pose7]) -> str:
    if p is None:
        return "None"
    return f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"


def fmt_quat(p: Optional[Pose7]) -> str:
    if p is None:
        return "None"
    return f"({p[3]:.3f}, {p[4]:.3f}, {p[5]:.3f}, {p[6]:.3f})"


class BCRFakeGripper(Node):
    def __init__(self):
        super().__init__("bcr_fake_gripper")

        self.declare_parameter("world_name", "dual_circular_factory_world")
        self.declare_parameter("target_model", "pickup_box")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "gripper_tip")

        self.declare_parameter("base_x", 0.0)
        self.declare_parameter("base_y", 0.0)
        self.declare_parameter("base_z", 0.0)

        # Stable default from your successful tests.
        self.declare_parameter("hold_offset_x", -0.12)
        self.declare_parameter("hold_offset_y", 0.0)
        self.declare_parameter("hold_offset_z", -0.08)
        self.declare_parameter("hold_rpy_roll", 0.0)
        self.declare_parameter("hold_rpy_pitch", 0.0)
        self.declare_parameter("hold_rpy_yaw", 0.0)

        # Development mode: ignores distance check.
        # For final demo, set debug_mode=false and max_pick_distance=0.5.
        self.declare_parameter("debug_mode", True)
        self.declare_parameter("preserve_relative_pose", False)
        self.declare_parameter("snap_on_pick", True)
        self.declare_parameter("max_pick_distance", 0.0)

        # If true, object position follows the gripper but orientation stays world-flat.
        # Keep false by default because your working version followed gripper orientation.
        self.declare_parameter("flat_object_orientation", False)

        self.declare_parameter("exact_place_on_release", True)
        self.declare_parameter("place_x", 0.57)
        self.declare_parameter("place_y", 0.0)
        self.declare_parameter("place_z", 3.70)
        self.declare_parameter("place_qx", 0.0)
        self.declare_parameter("place_qy", 0.0)
        self.declare_parameter("place_qz", 0.0)
        self.declare_parameter("place_qw", 1.0)

        # Pick-zone target near the physical gripper/conveyor location.
        # Tune these to the red-rectangle location in Gazebo.
        self.declare_parameter("pick_x", 0.57)
        self.declare_parameter("pick_y", 0.0)
        self.declare_parameter("pick_z", 3.70)
        self.declare_parameter("pick_qx", 0.0)
        self.declare_parameter("pick_qy", 0.0)
        self.declare_parameter("pick_qz", 0.0)
        self.declare_parameter("pick_qw", 1.0)

        # If true, /pick_object first moves pickup_box to pick_x/y/z before attaching.
        # This is useful for repeatable demos/tests. For realistic conveyor picking, keep false.
        self.declare_parameter("auto_move_object_to_pick_zone", False)

        # If true, distance is checked from object to pick zone.
        # If false, distance is checked from object to gripper/tool frame.
        self.declare_parameter("use_pick_zone_for_distance", False)
        self.declare_parameter("pick_zone_max_distance", 0.35)

        self.declare_parameter("update_hz", 25.0)
        self.declare_parameter("gz_timeout_ms", 300)
        self.declare_parameter("verbose_debug", True)

        # V4.1 precise placement: repeat exact place set_pose a few times
        # so Gazebo settles the object at the requested final pose.
        self.declare_parameter("place_settle_repeats", 3)
        self.declare_parameter("place_settle_sleep_sec", 0.05)

        self.attached = False
        self.saved_tool_to_object: Pose7 = self.get_fallback_relative_pose()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "/pick_object", self.pick_callback)
        self.create_service(Trigger, "/place_object", self.place_callback)
        self.create_service(Trigger, "/move_object_to_pick_zone", self.move_object_to_pick_zone_callback)

        update_hz = self.get_param_float("update_hz")
        self.timer = self.create_timer(1.0 / max(update_hz, 1.0), self.on_timer)

        self.get_logger().info(
            "BCR fake gripper V4.2 ready. Stable hold + precise place defaults: "
            "debug_mode=True, max_pick_distance=0.0, "
            "hold_offset=(-0.12, 0.0, -0.08), snap_on_pick=True."
        )

    def get_param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def get_param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_param_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def get_hold_quat(self):
        return rpy_to_quat(
            self.get_param_float("hold_rpy_roll"),
            self.get_param_float("hold_rpy_pitch"),
            self.get_param_float("hold_rpy_yaw"),
        )

    def get_fallback_relative_pose(self) -> Pose7:
        qx, qy, qz, qw = self.get_hold_quat()
        return (
            self.get_param_float("hold_offset_x"),
            self.get_param_float("hold_offset_y"),
            self.get_param_float("hold_offset_z"),
            qx, qy, qz, qw,
        )

    def get_place_pose(self) -> Pose7:
        q = quat_normalize((
            self.get_param_float("place_qx"),
            self.get_param_float("place_qy"),
            self.get_param_float("place_qz"),
            self.get_param_float("place_qw"),
        ))
        return (
            self.get_param_float("place_x"),
            self.get_param_float("place_y"),
            self.get_param_float("place_z"),
            q[0], q[1], q[2], q[3],
        )

    def get_pick_pose(self) -> Pose7:
        q = quat_normalize((
            self.get_param_float("pick_qx"),
            self.get_param_float("pick_qy"),
            self.get_param_float("pick_qz"),
            self.get_param_float("pick_qw"),
        ))
        return (
            self.get_param_float("pick_x"),
            self.get_param_float("pick_y"),
            self.get_param_float("pick_z"),
            q[0], q[1], q[2], q[3],
        )


    def get_tool_pose_world(self) -> Pose7:
        tf = self.tf_buffer.lookup_transform(
            self.get_param_str("base_frame"),
            self.get_param_str("tool_frame"),
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.3),
        )

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        return (
            self.get_param_float("base_x") + tx,
            self.get_param_float("base_y") + ty,
            self.get_param_float("base_z") + tz,
            qx, qy, qz, qw,
        )

    def object_pose_from_gripper(self) -> Pose7:
        tool_world = self.get_tool_pose_world()

        if self.get_param_bool("flat_object_orientation"):
            ox = self.get_param_float("hold_offset_x")
            oy = self.get_param_float("hold_offset_y")
            oz = self.get_param_float("hold_offset_z")
            rx, ry, rz = rotate_vector(
                (tool_world[3], tool_world[4], tool_world[5], tool_world[6]),
                (ox, oy, oz),
            )
            qx, qy, qz, qw = self.get_hold_quat()
            return (
                tool_world[0] + rx,
                tool_world[1] + ry,
                tool_world[2] + rz,
                qx, qy, qz, qw,
            )

        return compose_pose(tool_world, self.saved_tool_to_object)

    def set_gazebo_model_pose(self, pose: Optional[Pose7] = None) -> bool:
        world_name = self.get_param_str("world_name")
        target_model = self.get_param_str("target_model")

        if pose is None:
            pose = self.object_pose_from_gripper()

        x, y, z, qx, qy, qz, qw = pose
        req = (
            f'name: "{target_model}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )

        cmd = [
            "gz", "service",
            "-s", f"/world/{world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(int(self.get_param_float("gz_timeout_ms"))),
            "--req", req,
        ]

        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        if result.returncode != 0:
            self.get_logger().warn(
                f"gz set_pose failed. model='{target_model}', world='{world_name}', "
                f"returncode={result.returncode}, stderr='{result.stderr.strip()}'"
            )
            return False

        if "data: false" in result.stdout:
            self.get_logger().warn(
                f"gz set_pose returned false. model='{target_model}', stdout='{result.stdout.strip()}'"
            )
            return False

        return True

    def get_gazebo_model_pose(self, model_name: str) -> Optional[Pose7]:
        world_name = self.get_param_str("world_name")
        cmd = ["gz", "topic", "-e", "-t", f"/world/{world_name}/pose/info", "-n", "1"]

        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1.5,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timed out while reading Gazebo pose/info.")
            return None

        if result.returncode != 0:
            self.get_logger().warn(f"gz topic pose/info failed: {result.stderr.strip()}")
            return None

        text = result.stdout
        idx = text.find(f'name: "{model_name}"')

        if idx < 0:
            self.get_logger().warn(f"Could not find model '{model_name}' in Gazebo pose/info.")
            return None

        block = text[idx: idx + 1400]

        def parse_float(pattern: str, default: float = 0.0) -> float:
            m = re.search(pattern, block)
            if not m:
                return default
            return float(m.group(1))

        px = parse_float(r"position\s*\{[^}]*x:\s*([-+0-9.eE]+)")
        py = parse_float(r"position\s*\{[^}]*y:\s*([-+0-9.eE]+)")
        pz = parse_float(r"position\s*\{[^}]*z:\s*([-+0-9.eE]+)")
        qx = parse_float(r"orientation\s*\{[^}]*x:\s*([-+0-9.eE]+)", 0.0)
        qy = parse_float(r"orientation\s*\{[^}]*y:\s*([-+0-9.eE]+)", 0.0)
        qz = parse_float(r"orientation\s*\{[^}]*z:\s*([-+0-9.eE]+)", 0.0)
        qw = parse_float(r"orientation\s*\{[^}]*w:\s*([-+0-9.eE]+)", 1.0)

        return px, py, pz, qx, qy, qz, qw

    def build_debug_text(
        self,
        tool_world: Optional[Pose7],
        object_world: Optional[Pose7],
        distance: Optional[float],
    ) -> str:
        hold = (
            self.get_param_float("hold_offset_x"),
            self.get_param_float("hold_offset_y"),
            self.get_param_float("hold_offset_z"),
        )
        dist_text = "None" if distance is None else f"{distance:.3f} m"

        return (
            f"tool_xyz={fmt_xyz(tool_world)}, "
            f"object_xyz={fmt_xyz(object_world)}, "
            f"distance={dist_text}, "
            f"hold_offset=({hold[0]:.3f}, {hold[1]:.3f}, {hold[2]:.3f}), "
            f"debug_mode={self.get_param_bool('debug_mode')}, "
            f"max_pick_distance={self.get_param_float('max_pick_distance'):.3f}"
        )

    def move_object_to_pick_zone_callback(self, request, response):
        target_model = self.get_param_str("target_model")
        pick_pose = self.get_pick_pose()

        try:
            # If object is currently attached, detach before moving it to pick zone.
            self.attached = False
            ok = self.set_gazebo_model_pose(pick_pose)

            response.success = ok
            response.message = (
                f"Moved '{target_model}' to pick zone. "
                f"pick_xyz={fmt_xyz(pick_pose)}, pick_quat={fmt_quat(pick_pose)}"
                if ok else
                f"Failed to move '{target_model}' to pick zone. pick_xyz={fmt_xyz(pick_pose)}"
            )
            return response

        except Exception as exc:
            response.success = False
            response.message = f"Move to pick zone failed: {exc}"
            return response

    def pick_callback(self, request, response):
        target_model = self.get_param_str("target_model")

        try:
            tool_world = self.get_tool_pose_world()

            # Optional repeatable test/demo behavior:
            # move the object to the tuned pick zone before attaching it.
            if self.get_param_bool("auto_move_object_to_pick_zone"):
                pick_pose = self.get_pick_pose()
                self.set_gazebo_model_pose(pick_pose)
                time.sleep(0.05)

            object_world = self.get_gazebo_model_pose(target_model)
            pick_pose = self.get_pick_pose()

            tool_distance = pose_distance(tool_world, object_world) if object_world is not None else None
            zone_distance = pose_distance(pick_pose, object_world) if object_world is not None else None

            # Use the normal debug text, but append pick-zone data.
            debug_text = self.build_debug_text(tool_world, object_world, tool_distance)
            debug_text += (
                f", pick_xyz={fmt_xyz(pick_pose)}, "
                f"zone_distance={'None' if zone_distance is None else f'{zone_distance:.3f} m'}, "
                f"use_pick_zone_for_distance={self.get_param_bool('use_pick_zone_for_distance')}, "
                f"auto_move_object_to_pick_zone={self.get_param_bool('auto_move_object_to_pick_zone')}"
            )

            if self.get_param_bool("verbose_debug"):
                self.get_logger().info(f"PICK request: {debug_text}")

            debug_mode = self.get_param_bool("debug_mode")

            if not debug_mode and object_world is not None:
                if self.get_param_bool("use_pick_zone_for_distance"):
                    max_dist = self.get_param_float("pick_zone_max_distance")
                    distance = zone_distance
                    reject_reason = "pick zone"
                else:
                    max_dist = self.get_param_float("max_pick_distance")
                    distance = tool_distance
                    reject_reason = "tool frame"

                if distance is not None and max_dist > 0.0 and distance > max_dist:
                    response.success = False
                    response.message = (
                        f"Pick rejected. Object '{target_model}' is too far from {reject_reason}. "
                        f"{debug_text}"
                    )
                    return response

            preserve_relative = self.get_param_bool("preserve_relative_pose")
            snap_on_pick = self.get_param_bool("snap_on_pick")

            if preserve_relative and not snap_on_pick and object_world is not None:
                self.saved_tool_to_object = relative_pose(tool_world, object_world)
                mode_text = "preserve_relative_pose"
            else:
                # Stable baseline: attach using the tuned hold offset.
                self.saved_tool_to_object = self.get_fallback_relative_pose()
                mode_text = "snap_to_stable_hold_offset"

            ok = self.set_gazebo_model_pose()
            self.attached = ok

            response.success = ok
            response.message = (
                f"Attached '{target_model}' to '{self.get_param_str('tool_frame')}'. "
                f"mode={mode_text}. {debug_text}"
                if ok else
                f"Failed to attach '{target_model}'. Check Gazebo model/world. {debug_text}"
            )
            return response

        except Exception as exc:
            response.success = False
            response.message = f"Pick failed: {exc}"
            return response

    def place_callback(self, request, response):
        target_model = self.get_param_str("target_model")

        try:
            if self.get_param_bool("exact_place_on_release"):
                # Precise place mode:
                # 1. Stop continuous following.
                # 2. Teleport the object to the exact target pose.
                # 3. Repeat set_pose briefly so Gazebo settles at the target.
                self.attached = False
                place_pose = self.get_place_pose()

                repeats = max(1, int(self.get_param_float("place_settle_repeats")))
                sleep_sec = max(0.0, self.get_param_float("place_settle_sleep_sec"))

                ok = True
                for i in range(repeats):
                    ok = self.set_gazebo_model_pose(place_pose) and ok
                    if sleep_sec > 0.0 and i < repeats - 1:
                        import time
                        time.sleep(sleep_sec)

                response.success = ok
                response.message = (
                    f"Released '{target_model}' at exact place pose. "
                    f"place_xyz={fmt_xyz(place_pose)}, place_quat={fmt_quat(place_pose)}, "
                    f"repeats={repeats}"
                    if ok else
                    f"Failed to place '{target_model}' at exact place pose. "
                    f"place_xyz={fmt_xyz(place_pose)}"
                )
                return response

            if self.attached:
                self.set_gazebo_model_pose()

            self.attached = False
            response.success = True
            response.message = f"Released '{target_model}' at current held pose."
            return response

        except Exception as exc:
            response.success = False
            response.message = f"Place failed: {exc}"
            return response

    def on_timer(self):
        if not self.attached:
            return

        try:
            ok = self.set_gazebo_model_pose()
            if not ok:
                self.get_logger().warn("Attach update failed: Gazebo set_pose returned false.")
        except Exception as exc:
            self.get_logger().warn(f"Attach update failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = BCRFakeGripper()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()