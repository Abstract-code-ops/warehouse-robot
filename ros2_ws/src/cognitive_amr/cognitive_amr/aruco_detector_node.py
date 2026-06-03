#!/usr/bin/env python3
"""
aruco_detector_node.py
─────────────────────────────────────────────────────────────────────────────
Subscribes to /camera/image_raw, detects ArUco markers, and publishes:
    • /camera/detected_markers             std_msgs/String          JSON list of detections
    • /camera/image_annotated/compressed    sensor_msgs/CompressedImage  JPEG feed for Foxglove

Uses DICT_4X4_50 and standard simulated camera intrinsics.
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import cv2
    import cv2.aruco as aruco
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

# Simulated 640×480 camera matrix (60° H-FOV)
FX = FY = 554.0
CX, CY = 320.0, 240.0
CAM_MATRIX = np.array([[FX, 0, CX],
                        [0, FY, CY],
                        [0,  0,  1]], dtype=np.float64)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)

MARKER_SIZE_M = 0.28   # matches aruco_face visual in SDF


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_every_n_frames', 3)
        self.declare_parameter('jpeg_quality', 60)
        image_topic = self.get_parameter('image_topic').value
        if not isinstance(image_topic, str) or not image_topic:
            image_topic = '/camera/color/image_raw'
        frame_skip = int(self.get_parameter('publish_every_n_frames').value)
        self._publish_every_n = max(1, frame_skip)
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._jpeg_quality = max(20, min(95, jpeg_quality))
        self._frame_count = 0

        if not _CV2_OK:
            self.get_logger().error(
                "opencv-contrib-python not found. "
                "Install: pip install opencv-contrib-python"
            )

        self._dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self._params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._dictionary, self._params)

        self._sub = self.create_subscription(
            Image, image_topic, self._on_image, 5)

        self._pub_markers = self.create_publisher(
            String, '/camera/detected_markers', 10)
        self._pub_annotated = self.create_publisher(
            CompressedImage, '/camera/image_annotated/compressed', 5)

        self.get_logger().info(
            f'ArUco detector ready (DICT_4X4_50, 640x480) on {image_topic} '
            f'| publish_every_n_frames={self._publish_every_n} '
            f'| jpeg_quality={self._jpeg_quality}')

    def _on_image(self, msg: Image):
        if not _CV2_OK:
            return

        self._frame_count += 1
        if self._frame_count % self._publish_every_n != 0:
            return

        # Convert ROS Image → OpenCV BGR
        try:
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ('bgr8', 'bgr888'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}",
                                       throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().warn(f"Image decode error: {e}",
                                   throttle_duration_sec=2.0)
            return

        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(grey)

        detections = []
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE_M, CAM_MATRIX, DIST_COEFFS)

            for i, marker_id in enumerate(ids.flatten()):
                t = tvecs[i][0]   # [tx, ty, tz] in camera frame
                r = rvecs[i][0]
                detections.append({
                    'aruco_id': int(marker_id),
                    'distance_m': float(np.linalg.norm(t)),
                    'camera_frame': msg.header.frame_id,
                    'tvec': t.tolist(),
                    'rvec': r.tolist(),
                })

            # Annotate
            annotated = frame.copy()
            aruco.drawDetectedMarkers(annotated, corners, ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(annotated, CAM_MATRIX, DIST_COEFFS,
                                  rvecs[i], tvecs[i], 0.1)
            self._pub_annotated.publish(
                self._cv2_to_compressed_ros(annotated, msg.header, self._jpeg_quality))
        else:
            # Still publish the current frame so Foxglove always shows feed.
            self._pub_annotated.publish(
                self._cv2_to_compressed_ros(frame, msg.header, self._jpeg_quality))

        self._pub_markers.publish(String(data=json.dumps({
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'camera_frame': msg.header.frame_id,
            'detections': detections
        })))

    @staticmethod
    def _cv2_to_compressed_ros(frame_bgr, header, jpeg_quality: int) -> CompressedImage:
        ok, encoded = cv2.imencode('.jpg', frame_bgr, [
            int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality),
        ])
        if not ok:
            raise RuntimeError('Failed to encode camera frame as JPEG')

        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
