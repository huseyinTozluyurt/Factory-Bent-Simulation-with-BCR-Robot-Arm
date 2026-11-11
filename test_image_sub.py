import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TestImageSub(Node):
    def __init__(self):
        super().__init__('test_image_sub')
        self.count = 0
        self.sub = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.cb,
            10
        )

    def cb(self, msg):
        self.count += 1
        self.get_logger().info(
            f"Image {self.count}: {msg.width}x{msg.height}, encoding={msg.encoding}"
        )

def main():
    rclpy.init()
    node = TestImageSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
