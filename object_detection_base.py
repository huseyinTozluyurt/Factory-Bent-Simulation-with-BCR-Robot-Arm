import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold to separate objects from background
        _, thresh = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        object_count = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Ignore very small noisy contours
            if area < 500:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            object_count += 1

        # Show object count on the frame
        cv2.putText(
            frame,
            f'Objects: {object_count}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

        # Optional debug windows
        cv2.imshow("Threshold", thresh)
        cv2.imshow("Factory Object Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()