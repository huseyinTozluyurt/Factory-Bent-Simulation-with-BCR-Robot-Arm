import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# Object detection node that focuses on a specific ROI in the image and counts objects entering that ROI.
# Grayscale based detection

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

        # Total number of objects that have entered the ROI
        self.total_counter = 0

        # Number of objects currently detected inside the ROI
        self.current_objects = 0

        # State variable:
        # False = ROI empty
        # True  = ROI currently occupied by at least one object
        self.roi_occupied = False

        self.get_logger().info('Object detection with ROI counter started.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame_height, frame_width = frame.shape[:2]

        # -----------------------------
        # ROI definition
        # -----------------------------
        # Lower-middle region of the image
        roi_x1 = int(frame_width * 0.42)
        roi_x2 = int(frame_width * 0.58)
        roi_y1 = int(frame_height * 0.75)
        roi_y2 = int(frame_height * 0.90)

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

        # -----------------------------
        # Grayscale detection in ROI
        # -----------------------------
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

        # Since you said objects appear as white in gray detection
        _, thresh_roi = cv2.threshold(blurred_roi, 120, 255, cv2.THRESH_BINARY)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh_roi = cv2.morphologyEx(thresh_roi, cv2.MORPH_OPEN, kernel)
        thresh_roi = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            thresh_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected_objects_in_roi = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Ignore very small noisy contours
            if area < 100:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Draw bounding boxes on the original full frame
            cv2.rectangle(
                frame,
                (roi_x1 + x, roi_y1 + y),
                (roi_x1 + x + w, roi_y1 + y + h),
                (0, 255, 0),
                2
            )

            detected_objects_in_roi += 1

        # Update current objects
        self.current_objects = detected_objects_in_roi

        # -----------------------------
        # Counting logic
        # -----------------------------
        # If ROI was empty before and now at least one object exists:
        # count this as a NEW object arrival event
        if not self.roi_occupied and self.current_objects > 0:
            self.total_counter += 1
            self.roi_occupied = True
            self.get_logger().info(
                f'New object entered ROI. Total counter: {self.total_counter}'
            )

        # If ROI becomes empty, reset occupancy state
        elif self.current_objects == 0:
            self.roi_occupied = False

        # -----------------------------
        # Drawing
        # -----------------------------
        # Draw ROI rectangle
        cv2.rectangle(
            frame,
            (roi_x1, roi_y1),
            (roi_x2, roi_y2),
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            'ROI',
            (roi_x1, roi_y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        # Display current objects in ROI
        cv2.putText(
            frame,
            f'Current objects: {self.current_objects}',
            (20, 340),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (175,214,17),
            2
        )

        # Display total counter
        cv2.putText(
            frame,
            f'Total counter: {self.total_counter}',
            (20, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (201, 53, 8),
            2
        )

        # Debug windows
        cv2.namedWindow("ROI Threshold", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Threshold", 700, 500)

        cv2.namedWindow("Factory Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Factory Object Detection", 1200, 800)


        cv2.imshow("ROI Threshold", thresh_roi)
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