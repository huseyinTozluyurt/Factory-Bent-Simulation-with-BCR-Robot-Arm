#!/usr/bin/env python3
"""
Capture Product Images - photographs every product type in Gazebo for the order GUI.

The world has a photo camera (<model name="product_photo_camera">) at (0, 11, 1.6), away
from the bents. For each type of product_catalog.py this node:
  creates the product (photo_<type>) in front of the camera  ->  waits until the camera
  has rendered it  ->  saves the picture as <output_dir>/<type>.png (256 x 256)  ->  removes it.
The Product Order GUI shows these pictures (button "Reload pictures", or restart it).

Needs Gazebo (T1) running. Starts its own ros_gz_bridge for the camera.

  ros2 run factory_bringup capture_product_images
  ros2 run factory_bringup capture_product_images --ros-args -p types:="['basketball', 'gift_box']"
"""

import math
import os
import signal
import subprocess
import time
from pathlib import Path

import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from factory_bringup.product_catalog import PRODUCT_IMAGE_DIR, PRODUCT_TYPES, photo_product_pose, product_sdf


class ProductImageCapture(Node):
    def __init__(self):
        super().__init__("capture_product_images")
        self.declare_parameter("world_name", "dual_circular_factory_world")
        self.declare_parameter("image_topic", "/product_photo_camera/image")
        self.declare_parameter("start_bridge", True)
        self.declare_parameter("output_dir", PRODUCT_IMAGE_DIR)
        self.declare_parameter("types", [""])             # empty = every type
        self.declare_parameter("settle_sec", 1.0)          # after the product appears, before the picture
        self.declare_parameter("image_size", 256)

        self.bridge = CvBridge()
        self.last_image = None
        self.last_image_time = 0.0
        self.create_subscription(Image, self.p("image_topic"), self.on_image, 2)
        self.bridge_proc = None
        if self.p("start_bridge"):
            topic = self.p("image_topic")
            self.bridge_proc = subprocess.Popen(
                ["ros2", "run", "ros_gz_bridge", "parameter_bridge", f"{topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def p(self, name):
        return self.get_parameter(name).value

    def on_image(self, msg: Image):
        self.last_image = msg
        self.last_image_time = time.monotonic()

    def gz(self, service: str, reqtype: str, req: str, timeout: float = 3.0) -> bool:
        cmd = ["gz", "service", "-s", f"/world/{self.p('world_name')}/{service}", "--reqtype", reqtype,
               "--reptype", "gz.msgs.Boolean", "--timeout", str(int(timeout * 1000)), "--req", req]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout + 1)
        except subprocess.TimeoutExpired:
            return False
        return res.returncode == 0 and "data: false" not in res.stdout

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_first_image(self, timeout: float = 15.0) -> bool:
        end = time.monotonic() + timeout
        while self.last_image is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last_image is not None

    def capture(self, ptype: str, out_dir: Path) -> bool:
        name = f"photo_{ptype}"
        x, y, z, yaw = photo_product_pose(ptype)
        sdf = Path(f"/tmp/{name}.sdf")
        sdf.write_text(product_sdf(name, ptype))
        self.gz("remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
        req = (f'name: "{name}", sdf_filename: "{sdf}", allow_renaming: false, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
        if not self.gz("create", "gz.msgs.EntityFactory", req):
            self.get_logger().warn(f"Could not create {name}.")
            return False
        created = time.monotonic()
        self.spin_for(float(self.p("settle_sec")))
        while self.last_image_time < created + float(self.p("settle_sec")) and time.monotonic() < created + 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)        # a frame rendered after the product appeared
        ok = False
        if self.last_image_time >= created:
            bgr = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
            h, w = bgr.shape[:2]
            s = min(h, w)
            bgr = bgr[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
            size = int(self.p("image_size"))
            ok = cv2.imwrite(str(out_dir / f"{ptype}.png"), cv2.resize(bgr, (size, size), interpolation=cv2.INTER_AREA))
        self.gz("remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
        return bool(ok)

    def run(self) -> int:
        out_dir = Path(os.path.expanduser(self.p("output_dir")))
        out_dir.mkdir(parents=True, exist_ok=True)
        types = [t for t in self.p("types") if t] or list(PRODUCT_TYPES)
        unknown = [t for t in types if t not in PRODUCT_TYPES]
        if unknown:
            self.get_logger().error(f"Unknown product types: {unknown}")
            return 1
        self.get_logger().info(f"Waiting for {self.p('image_topic')} (is Gazebo running with the photo camera?) ...")
        if not self.wait_first_image():
            self.get_logger().error(f"No image on {self.p('image_topic')}. Restart Gazebo (T1) so the world "
                                    "loads with product_photo_camera.")
            return 1
        done = 0
        for i, ptype in enumerate(types, 1):
            ok = self.capture(ptype, out_dir)
            done += ok
            self.get_logger().info(f"[{i}/{len(types)}] {ptype}: " + ("saved" if ok else "FAILED"))
        self.get_logger().info(f"Saved {done}/{len(types)} pictures to {out_dir}")
        return 0 if done == len(types) else 1

    def close(self):
        if self.bridge_proc is not None:
            try:
                os.killpg(self.bridge_proc.pid, signal.SIGINT)     # ros2 run + the bridge it started
                self.bridge_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.bridge_proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ProductImageCapture()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
