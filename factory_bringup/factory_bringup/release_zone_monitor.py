#!/usr/bin/env python3
"""
Release Zone Monitor - is the place on the second bent free to put an object down?

Camera /release_camera/image looks straight down at the release zone (see release_zone.py
for the OpenCV steps). The arm is masked out using TF, like in ring_object_detector.

Publishes:
  /vision/release_zone        std_msgs/String JSON {"stamp", "ready", "empty", "blobs": [[x, y, area], ...]}
  /vision/release_zone_image  sensor_msgs/Image annotated view (same as the window)
Service:
  /release_zone_monitor/relearn  std_srvs/Trigger  learn the empty belt again (zone must be empty)
"""

import json
import math
import os

import cv2

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from factory_bringup.release_zone import ReleaseZoneChecker, ReleaseZoneParams, compose_zone_view
from factory_bringup.vision_geometry import OverheadCamera, arm_mask_from_points

WINDOW_NAME = "Release Zone"


class ReleaseZoneMonitor(Node):
    def __init__(self):
        super().__init__("release_zone_monitor")
        self.declare_parameter("image_topic", "/release_camera/image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("save_debug_frames_dir", "~/ros2_ws/src/factory_bent_project/debug_frames")

        # Must match <model name="release_camera"> in the world file.
        self.declare_parameter("camera_x", 2.2171)
        self.declare_parameter("camera_y", -0.3736)
        self.declare_parameter("camera_z", 4.5)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 1.5708)
        self.declare_parameter("camera_yaw", 0.0)
        self.declare_parameter("camera_hfov", 1.0)
        for key, value in vars(ReleaseZoneParams()).items():
            self.declare_parameter(key, value)

        self.declare_parameter("mask_arm", True)
        self.declare_parameter("arm_base_frame", "base_link")
        self.declare_parameter("arm_mask_frames", ["link1", "link2", "link3", "link4", "link5", "link6", "flange", "gripper_tip"])
        self.declare_parameter("arm_mask_width", 0.45)
        self.declare_parameter("base_x", -0.200)
        self.declare_parameter("base_y", -0.154)
        self.declare_parameter("base_z", -0.508)
        self.declare_parameter("base_yaw", 0.0)

        self.bridge = CvBridge()
        self.checker = None
        self.checker_size = None
        self.status_text = ""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.zone_pub = self.create_publisher(String, "/vision/release_zone", 10)
        self.debug_pub = self.create_publisher(Image, "/vision/release_zone_image", 2)
        self.create_subscription(Image, self.p("image_topic"), self.image_callback, 2)
        self.create_subscription(String, "/vision_pick_place/status", self.on_status, 5)
        self.create_service(Trigger, "/release_zone_monitor/relearn", self.on_relearn)

        self.show_window = bool(self.p("show_window"))
        if self.show_window and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warn("No DISPLAY: the Release Zone window is disabled (debug image is still published).")
            self.show_window = False
        if self.show_window:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 800, 600)
        self.frame_count = 0
        self.last_empty = None
        self.get_logger().info(f"Release zone monitor ready, waiting for {self.p('image_topic')}.")

    def p(self, name):
        return self.get_parameter(name).value

    def on_status(self, msg: String):
        self.status_text = msg.data

    def on_relearn(self, request, response):
        if self.checker is not None:
            self.checker.relearn()
        response.success, response.message = True, "Learning the empty release zone again from the next frames."
        return response

    def arm_mask(self):
        if not self.p("mask_arm"):
            return None
        base = self.p("arm_base_frame")
        bx, by, bz, yaw = (float(self.p(k)) for k in ("base_x", "base_y", "base_z", "base_yaw"))
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(bx, by, bz)]
        for frame in self.p("arm_mask_frames"):
            try:
                tf = self.tf_buffer.lookup_transform(base, frame, rclpy.time.Time(),
                                                     timeout=rclpy.duration.Duration(seconds=0.0))
            except Exception:
                continue
            t = tf.transform.translation
            pts.append((bx + c * t.x - s * t.y, by + s * t.x + c * t.y, bz + t.z))
        if len(pts) < 3:
            return None
        return arm_mask_from_points(self.checker.camera, pts, float(self.p("arm_mask_width")))

    def build_checker(self, width: int, height: int):
        camera = OverheadCamera(
            x=float(self.p("camera_x")), y=float(self.p("camera_y")), z=float(self.p("camera_z")),
            roll=float(self.p("camera_roll")), pitch=float(self.p("camera_pitch")), yaw=float(self.p("camera_yaw")),
            hfov=float(self.p("camera_hfov")), width=width, height=height,
        )
        params = ReleaseZoneParams(**{k: type(v)(self.p(k)) for k, v in vars(ReleaseZoneParams()).items()})
        self.checker = ReleaseZoneChecker(camera, params)
        self.checker_size = (width, height)
        self.get_logger().info(f"Release zone checker built for {width}x{height}: zone ({params.zone_x:.3f}, "
                               f"{params.zone_y:.3f}) r={params.zone_radius:.2f} m = {self.checker.radius_px}px; "
                               f"learning the empty belt from {params.reference_frames} frames.")

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return
        h, w = bgr.shape[:2]
        if self.checker is None or self.checker_size != (w, h):
            self.build_checker(w, h)

        arm_mask = self.arm_mask()
        empty, blobs = self.checker.check(bgr, exclude_mask=arm_mask)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.zone_pub.publish(String(data=json.dumps({
            "stamp": stamp, "ready": empty is not None, "empty": bool(empty) if empty is not None else None,
            "zone": [self.checker.params.zone_x, self.checker.params.zone_y],
            "blobs": [[round(b.x, 3), round(b.y, 3), round(b.area_m2, 4)] for b in blobs],
        })))
        if empty is not None and empty != self.last_empty:
            self.get_logger().info("Release zone " + ("EMPTY." if empty else f"OCCUPIED ({len(blobs)} object(s))."))
            self.last_empty = empty

        if self.p("publish_debug_image") or self.show_window:
            view = compose_zone_view(self.checker, bgr, empty, blobs, self.status_text, arm_mask)
            if self.p("publish_debug_image"):
                img = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
                img.header = msg.header
                self.debug_pub.publish(img)
            if self.show_window:
                cv2.imshow(WINDOW_NAME, view)
                cv2.waitKey(1)

        self.frame_count += 1
        if self.frame_count == 40 and self.p("save_debug_frames_dir"):
            out_dir = os.path.expanduser(self.p("save_debug_frames_dir"))
            os.makedirs(out_dir, exist_ok=True)
            cv2.imwrite(os.path.join(out_dir, "release_raw.png"), bgr)
            cv2.imwrite(os.path.join(out_dir, "release_zone.png"),
                        compose_zone_view(self.checker, bgr, empty, blobs, self.status_text, arm_mask))
            self.get_logger().info(f"Saved release zone debug frames to {out_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = ReleaseZoneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
