#!/usr/bin/env python3
"""
Side Camera Checker - horizontal view of the catch station for cross-validation.

Pipeline on /side_camera/image (see side_vision.py):
  grayscale -> blur -> background subtraction -> Canny -> contours standing on the belt
  -> y along the belt and top height on the station plane.

Publishes:
  /vision/side_check        std_msgs/String  JSON {"stamp", "present", "y", "z_top", "z_bottom",
                                              "width", "height", "objects": [...]}
  /vision/side_debug_image  sensor_msgs/Image annotated view (same as the window)

With show_window:=true a "Side Camera" window shows the live view, the watched band,
each object's y / height and the controller's cross-check verdict
(from /vision_pick_place/cross_check).
"""

import json
import os

import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from factory_bringup.side_vision import SideCheckParams, SideObjectChecker, compose_side_view
from factory_bringup.vision_geometry import OverheadCamera

WINDOW_NAME = "Side Camera"


class SideCameraChecker(Node):
    def __init__(self):
        super().__init__("side_camera_checker")

        self.declare_parameter("image_topic", "/side_camera/image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("save_debug_frames_dir", "~/ros2_ws/src/factory_bent_project/debug_frames")

        # Must match <model name="side_camera"> in the world file.
        self.declare_parameter("camera_x", -1.50)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 1.80)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.17)
        self.declare_parameter("camera_yaw", 3.14159)
        self.declare_parameter("camera_hfov", 1.10)

        defaults = SideCheckParams()
        for key, value in vars(defaults).items():
            self.declare_parameter(key, value)

        self.bridge = CvBridge()
        self.checker = None
        self.checker_size = None
        self.status_text = ""
        self.confirm = None
        self.target_y = None

        self.check_pub = self.create_publisher(String, "/vision/side_check", 10)
        self.debug_pub = self.create_publisher(Image, "/vision/side_debug_image", 2)
        self.create_subscription(Image, self.p("image_topic"), self.image_callback, 2)
        self.create_subscription(String, "/vision_pick_place/status", self.on_status, 5)
        self.create_subscription(String, "/vision_pick_place/cross_check", self.on_cross_check, 5)

        self.show_window = bool(self.p("show_window"))
        if self.show_window and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warn("No DISPLAY: the Side Camera window is disabled (debug image is still published).")
            self.show_window = False
        if self.show_window:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 960, 540)
        self.frame_count = 0
        self.get_logger().info(f"Side camera checker ready, waiting for {self.p('image_topic')}.")

    def p(self, name):
        return self.get_parameter(name).value

    def on_status(self, msg: String):
        self.status_text = msg.data

    def on_cross_check(self, msg: String):
        try:
            self.confirm = json.loads(msg.data)
            self.target_y = self.confirm.get("top_y")
        except ValueError:
            self.confirm = None

    def build_checker(self, width: int, height: int):
        camera = OverheadCamera(
            x=float(self.p("camera_x")), y=float(self.p("camera_y")), z=float(self.p("camera_z")),
            roll=float(self.p("camera_roll")), pitch=float(self.p("camera_pitch")), yaw=float(self.p("camera_yaw")),
            hfov=float(self.p("camera_hfov")), width=width, height=height,
        )
        params = SideCheckParams(**{k: type(v)(self.p(k)) for k, v in vars(SideCheckParams()).items()})
        self.checker = SideObjectChecker(camera, params)
        self.checker_size = (width, height)
        self.get_logger().info(f"Side checker built for {width}x{height}, fx={camera.fx:.1f}px, "
                               f"station plane x={params.plane_x:.2f}, band y=[{params.band_y_min:.2f}, {params.band_y_max:.2f}].")

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return
        h, w = bgr.shape[:2]
        if self.checker is None or self.checker_size != (w, h):
            self.build_checker(w, h)

        dets = self.checker.check(bgr)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out = {"stamp": stamp, "present": bool(dets), "objects": [d.as_dict() for d in dets]}
        if dets:
            out.update({k: v for k, v in dets[0].as_dict().items() if k != "bbox"})
        self.check_pub.publish(String(data=json.dumps(out)))

        if self.p("publish_debug_image") or self.show_window:
            view = compose_side_view(self.checker, bgr, dets, self.status_text, self.target_y, self.confirm)
            if self.p("publish_debug_image"):
                img = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
                img.header = msg.header
                self.debug_pub.publish(img)
            if self.show_window:
                cv2.imshow(WINDOW_NAME, view)
                cv2.waitKey(1)

        self.frame_count += 1
        if self.frame_count == 30 and self.p("save_debug_frames_dir"):
            out_dir = os.path.expanduser(self.p("save_debug_frames_dir"))
            os.makedirs(out_dir, exist_ok=True)
            cv2.imwrite(os.path.join(out_dir, "side_raw.png"), bgr)
            cv2.imwrite(os.path.join(out_dir, "side_detections.png"),
                        compose_side_view(self.checker, bgr, dets, self.status_text, self.target_y, self.confirm))
            self.get_logger().info(f"Saved side camera debug frames to {out_dir}")
        if self.frame_count % 75 == 1:
            self.get_logger().info(f"frame {self.frame_count}: {len(dets)} object(s) on the belt at the station"
                                   + (f", nearest y={dets[0].y:+.2f} top z={dets[0].z_top:.2f}" if dets else ""))


def main(args=None):
    rclpy.init(args=args)
    node = SideCameraChecker()
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
