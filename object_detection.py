import os
from datetime import datetime
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


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

        self.total_counter = 0
        self.current_objects = 0
    
        self.measured_objects = []
        self.object_name_counter = 1
        self.output_dir = "measurement_logs"
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.measurement_log_path = os.path.join(
            self.output_dir,
            f"factory_measurements_{timestamp}.txt"
        )

        with open(self.measurement_log_path, "w") as f:
            f.write("Factory Measurement Log\n")
            f.write("=" * 50 + "\n\n")

        self.get_logger().info(
            f"Measurement log file created: {self.measurement_log_path}"
        )
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500,
            varThreshold=28,
            detectShadows=False
        )

        self.next_track_id = 0
        self.tracks = {}

        self.count_line_ratio = 0.55
        self.frame_count = 0
        self.warmup_frames = 20

        self.get_logger().info(
            'Shadow-robust object detection, counting, and measurement started.'
        )

    def _get_roi(self, frame):
        frame_height, frame_width = frame.shape[:2]

        roi_x1 = int(frame_width * 0.42)
        roi_x2 = int(frame_width * 0.58)
        roi_y1 = int(frame_height * 0.62)
        roi_y2 = int(frame_height * 0.92)

        return roi_x1, roi_y1, roi_x2, roi_y2, frame[roi_y1:roi_y2, roi_x1:roi_x2]

    def _preprocess_roi(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        fg_mask = self.bg_subtractor.apply(blur)

        adaptive = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21,
            5
        )

        edges = cv2.Canny(blur, 35, 100)

        fused = cv2.bitwise_and(adaptive, fg_mask)
        fused = cv2.bitwise_or(fused, edges)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        fused = cv2.morphologyEx(fused, cv2.MORPH_CLOSE, kernel_close)
        fused = cv2.morphologyEx(fused, cv2.MORPH_OPEN, kernel_open)
        fused = cv2.dilate(fused, kernel_open, iterations=1)

        return gray, blur, fg_mask, adaptive, edges, fused

    def _extract_detections(self, fused_roi):
        contours, _ = cv2.findContours(
            fused_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 90:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w < 12 or h < 12:
                continue

            rect_area = w * h
            fill_ratio = area / float(rect_area) if rect_area > 0 else 0.0

            if fill_ratio < 0.22:
                continue

            aspect_ratio = w / float(h)
            if aspect_ratio < 0.20 or aspect_ratio > 5.5:
                continue

            cx = x + w // 2
            cy = y + h // 2

            detections.append({
                'bbox': (x, y, w, h),
                'centroid': (cx, cy),
                'area': area
            })

        return detections


    def _is_object_ready_for_measurement(self, det, roi_w, roi_h):
        x, y, w, h = det['bbox']
        cx, cy = det['centroid']

        # Smaller margin so objects are accepted easier
        margin = 3

        fully_inside_roi = (
            x > margin and
            y > margin and
            x + w < roi_w - margin and
            y + h < roi_h - margin
        )

        roi_center_x = roi_w // 2
        roi_center_y = roi_h // 2

        # More relaxed center condition
        center_tolerance_x = int(roi_w * 0.42)
        center_tolerance_y = int(roi_h * 0.48)

        near_center = (
            abs(cx - roi_center_x) < center_tolerance_x and
            abs(cy - roi_center_y) < center_tolerance_y
        )

        # Reject tiny noisy detections
        enough_size = (w > 18 and h > 18)

        return fully_inside_roi and near_center and enough_size


    def _match_tracks(self, detections):
        max_distance = 45

        unmatched_detection_indices = set(range(len(detections)))
        updated_tracks = {}

        existing_ids = list(self.tracks.keys())
        used_track_ids = set()

        for det_idx, det in enumerate(detections):
            dcx, dcy = det['centroid']

            best_track_id = None
            best_distance = float('inf')

            for track_id in existing_ids:
                if track_id in used_track_ids:
                    continue

                tx = self.tracks[track_id]['cx']
                ty = self.tracks[track_id]['cy']

                dist = math.hypot(dcx - tx, dcy - ty)

                if dist < best_distance and dist < max_distance:
                    best_distance = dist
                    best_track_id = track_id

            if best_track_id is not None:
                prev_track = self.tracks[best_track_id]

                updated_tracks[best_track_id] = {
                    'cx': dcx,
                    'cy': dcy,
                    'prev_cx': prev_track['cx'],
                    'prev_cy': prev_track['cy'],
                    'missed': 0,
                    'counted': prev_track.get('counted', False),
                    'age': prev_track.get('age', 1) + 1,
                    'measured': prev_track.get('measured', False),
                    'name': prev_track.get('name', None)
                }

                used_track_ids.add(best_track_id)
                unmatched_detection_indices.discard(det_idx)

        for track_id in existing_ids:
            if track_id not in updated_tracks:
                prev_track = self.tracks[track_id]
                missed = prev_track.get('missed', 0) + 1

                if missed <= 6:
                    updated_tracks[track_id] = {
                        'cx': prev_track['cx'],
                        'cy': prev_track['cy'],
                        'prev_cx': prev_track.get('prev_cx', prev_track['cx']),
                        'prev_cy': prev_track.get('prev_cy', prev_track['cy']),
                        'missed': missed,
                        'counted': prev_track.get('counted', False),
                        'age': prev_track.get('age', 1),
                        'measured': prev_track.get('measured', False),
                        'name': prev_track.get('name', None)
                    }

        for det_idx in unmatched_detection_indices:
            dcx, dcy = detections[det_idx]['centroid']

            updated_tracks[self.next_track_id] = {
                'cx': dcx,
                'cy': dcy,
                'prev_cx': dcx,
                'prev_cy': dcy,
                'missed': 0,
                'counted': False,
                'age': 1,
                'measured': False,
                'name': None
            }

            self.next_track_id += 1

        self.tracks = updated_tracks

    def _update_counter(self):
        min_stable_age = 3

        for track_id, track in self.tracks.items():
            if track['missed'] > 0:
                continue

            if track['counted']:
                continue

            if track.get('age', 1) >= min_stable_age:
                self.total_counter += 1
                track['counted'] = True

                self.get_logger().info(
                    f'Object counted after stable tracking. '
                    f'Track ID: {track_id}, Total: {self.total_counter}'
                )

    def _find_nearest_track_id(self, cx, cy):
        best_track_id = None
        best_distance = float('inf')

        for track_id, track in self.tracks.items():
            if track.get('missed', 0) > 0:
                continue

            dist = math.hypot(cx - track['cx'], cy - track['cy'])

            if dist < best_distance:
                best_distance = dist
                best_track_id = track_id

        if best_distance <= 45:
            return best_track_id

        return None

    def _register_measured_object(
        self,
        track_id,
        frame_cx,
        frame_cy,
        object_width_px,
        object_height_px,
        object_area_px
    ):
        track = self.tracks[track_id]

        if track.get('measured', False):
            return

        object_name = f'OBJ-{self.object_name_counter}'
        self.object_name_counter += 1

        track['measured'] = True
        track['name'] = object_name

        object_info = {
            'name': object_name,
            'track_id': track_id,
            'cx': frame_cx,
            'cy': frame_cy,
            'width': object_width_px,
            'height': object_height_px,
            'area': int(object_area_px)
        }

        self.measured_objects.append(object_info)
        self._save_measurement_to_file(object_info)
        max_objects_to_show = 6
        if len(self.measured_objects) > max_objects_to_show:
            self.measured_objects.pop(0)

        self.get_logger().info(
            f'Registered {object_name} | '
            f'Track ID={track_id} | '
            f'center=({frame_cx},{frame_cy}) | '
            f'width={object_width_px}px | '
            f'height={object_height_px}px | '
            f'area={int(object_area_px)}px'
        )


    def _save_measurement_to_file(self, object_info):
        with open(self.measurement_log_path, "a") as f:
            f.write(f"Object Name: {object_info['name']}\n")
            f.write(f"Track ID: {object_info['track_id']}\n")
            f.write(f"Center: ({object_info['cx']}, {object_info['cy']})\n")
            f.write(f"Width: {object_info['width']} px\n")
            f.write(f"Height: {object_info['height']} px\n")
            f.write(f"Area: {object_info['area']} px\n")
            f.write("-" * 50 + "\n")

    def _draw_measurement_panel(self, frame):
        panel_x = 20
        panel_y = 420

        cv2.putText(
            frame,
            'Measured Objects:',
            (panel_x, panel_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        if len(self.measured_objects) == 0:
            cv2.putText(
                frame,
                'No completed measurements yet',
                (panel_x, panel_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (200, 200, 200),
                1
            )
            return

        for idx, obj in enumerate(self.measured_objects):
            line_y = panel_y + 30 + (idx * 28)

            text = (
                f"{obj['name']} | "
                f"W:{obj['width']}px "
                f"H:{obj['height']}px "
                f"C:({obj['cx']},{obj['cy']}) "
                f"A:{obj['area']}px"
            )

            cv2.putText(
                frame,
                text,
                (panel_x, line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1
            )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_count += 1

        roi_x1, roi_y1, roi_x2, roi_y2, roi = self._get_roi(frame)
        roi_h, roi_w = roi.shape[:2]

        gray_roi, blur_roi, fg_mask, adaptive_roi, edges_roi, fused_roi = self._preprocess_roi(roi)

        count_line_y = int(roi_h * self.count_line_ratio)

        if self.frame_count <= self.warmup_frames:
            cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)

            cv2.line(
                frame,
                (roi_x1, roi_y1 + count_line_y),
                (roi_x2, roi_y1 + count_line_y),
                (255, 0, 255),
                2
            )

            cv2.putText(
                frame,
                'Warming up background model...',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f'Current objects: 0',
                (20, 340),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (175, 214, 17),
                2
            )

            cv2.putText(
                frame,
                f'Total counter: {self.total_counter}',
                (20, 300),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (201, 53, 8),
                2
            )

            self._draw_measurement_panel(frame)

            cv2.imshow("ROI Foreground", fg_mask)
            cv2.imshow("ROI Adaptive", adaptive_roi)
            cv2.imshow("ROI Fused", fused_roi)
            cv2.imshow("Factory Object Detection", frame)
            cv2.waitKey(1)
            return

        detections = self._extract_detections(fused_roi)
        self._match_tracks(detections)
        self._update_counter()

        self.current_objects = sum(
            1 for t in self.tracks.values() if t.get('missed', 0) == 0
        )

        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)

        cv2.putText(
            frame,
            'ROI',
            (roi_x1, roi_y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        cv2.line(
            frame,
            (roi_x1, roi_y1 + count_line_y),
            (roi_x2, roi_y1 + count_line_y),
            (255, 0, 255),
            2
        )

        cv2.putText(
            frame,
            'Count line',
            (roi_x1 + 8, roi_y1 + count_line_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            2
        )

        for det in detections:
            x, y, w, h = det['bbox']
            cx, cy = det['centroid']

            frame_x = roi_x1 + x
            frame_y = roi_y1 + y
            frame_cx = roi_x1 + cx
            frame_cy = roi_y1 + cy

            object_width_px = w
            object_height_px = h
            object_area_px = det['area']

            measurement_ready = self._is_object_ready_for_measurement(det, roi_w, roi_h)

            track_id = self._find_nearest_track_id(cx, cy)
            object_name = None

            if track_id is not None:
                object_name = self.tracks[track_id].get('name', None)

            box_color = (0, 255, 0)
            if measurement_ready:
                box_color = (0, 255, 255)

            cv2.rectangle(
                frame,
                (frame_x, frame_y),
                (frame_x + w, frame_y + h),
                box_color,
                2
            )

            cv2.circle(frame, (frame_cx, frame_cy), 4, (0, 255, 255), -1)

            label = f'W:{object_width_px}px H:{object_height_px}px'
            if object_name is not None:
                label = f'{object_name} | ' + label

            cv2.putText(
                frame,
                label,
                (frame_x, max(frame_y - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                box_color,
                1
            )

            cv2.putText(
                frame,
                f'C:({frame_cx},{frame_cy})',
                (frame_x, frame_y + h + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                box_color,
                1
            )

            if measurement_ready:
                cv2.putText(
                    frame,
                    'MEASUREMENT READY',
                    (frame_x, frame_y + h + 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    1
                )

                if track_id is not None:
                    self._register_measured_object(
                        track_id,
                        frame_cx,
                        frame_cy,
                        object_width_px,
                        object_height_px,
                        object_area_px
                    )
                    cv2.putText(frame, 'REGISTERED', (frame_x, frame_y + h + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        for track_id, track in self.tracks.items():
            if track.get('missed', 0) > 0:
                continue

            tx, ty = track['cx'], track['cy']
            track_name = track.get('name', None)

            id_text = f'ID {track_id}'
            if track_name is not None:
                id_text = f'{track_name} / ID {track_id}'

            cv2.putText(
                frame,
                id_text,
                (roi_x1 + tx + 6, roi_y1 + ty - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                1
            )

        cv2.putText(
            frame,
            f'Total counter: {self.total_counter}',
            (20, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (201, 53, 8),
            2
        )

        cv2.putText(
            frame,
            f'Current objects: {self.current_objects}',
            (20, 340),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (175, 214, 17),
            2
        )

        self._draw_measurement_panel(frame)

        cv2.namedWindow("ROI Foreground", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Foreground", 700, 500)

        cv2.namedWindow("ROI Adaptive", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Adaptive", 700, 500)

        cv2.namedWindow("ROI Edges", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Edges", 700, 500)

        cv2.namedWindow("ROI Fused", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Fused", 700, 500)

        cv2.namedWindow("Factory Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Factory Object Detection", 1200, 800)

        cv2.imshow("ROI Foreground", fg_mask)
        cv2.imshow("ROI Adaptive", adaptive_roi)
        cv2.imshow("ROI Edges", edges_roi)
        cv2.imshow("ROI Fused", fused_roi)
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