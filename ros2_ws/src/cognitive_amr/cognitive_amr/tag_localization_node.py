#!/usr/bin/env python3
"""
tag_localization_node.py
─────────────────────────────────────────────────────────────────────────────
Consumes ArUco detections and performs two tasks:
  1) Landmark tags: publish AMCL /initialpose corrections to reduce drift.
  2) Product tags: publish dynamic product_<aruco_id> TFs in map frame.

Inputs:
  SUB /camera/detected_markers  std_msgs/String JSON from aruco_detector_node

Outputs:
  PUB /initialpose              geometry_msgs/PoseWithCovarianceStamped
  PUB /products/registry        std_msgs/String JSON registry of seen products
  TF  map -> product_<id>
"""

import json
import math
import time
from typing import Dict, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String

from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from cognitive_amr.warehouse_constants import LANDMARK_TAGS


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(a: Tuple[float, float, float, float],
                  b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vec(q: Tuple[float, float, float, float],
               v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    vq = (v[0], v[1], v[2], 0.0)
    qr = quat_multiply(quat_multiply(q, vq), quat_conjugate(q))
    return (qr[0], qr[1], qr[2])


class TagLocalizationNode(Node):

    def __init__(self):
        super().__init__('tag_localization_node')

        # landmark_tag_map is a nested dict — not a valid ROS2 parameter type;
        # use the constant from warehouse_constants directly.
        self._landmarks: Dict[str, Dict[str, float]] = LANDMARK_TAGS
        self.declare_parameter('landmark_correction_period_sec', 1.0)
        self.declare_parameter('product_stale_sec', 3.0)
        self.declare_parameter('max_landmark_range_m', 8.0)

        self._correction_period = float(self.get_parameter('landmark_correction_period_sec').value)
        self._product_stale_sec = float(self.get_parameter('product_stale_sec').value)
        self._max_landmark_range_m = float(self.get_parameter('max_landmark_range_m').value)

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._product_registry_pub = self.create_publisher(String, '/products/registry', 10)

        self.create_subscription(String, '/camera/detected_markers', self._on_detections, 10)
        self.create_timer(0.25, self._publish_registry)

        self._products: Dict[str, Dict[str, float]] = {}
        self._last_landmark_correction_t = 0.0

        self.get_logger().info(
            f'Tag localization ready: landmarks={len(self._landmarks)} stale={self._product_stale_sec:.1f}s')

    def _on_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        detections = data.get('detections', [])
        if not detections:
            return

        for det in detections:
            try:
                aruco_id = int(det.get('aruco_id'))
                tvec = det.get('tvec', [0.0, 0.0, 0.0])
                cam_frame = det.get('camera_frame') or data.get('camera_frame') or 'camera_link'
                now_t = time.time()
            except (TypeError, ValueError):
                continue

            if str(aruco_id) in self._landmarks:
                self._process_landmark(aruco_id, tvec, cam_frame, now_t)
            else:
                self._process_product(aruco_id, tvec, cam_frame, now_t)

    def _process_landmark(self, aruco_id: int, tvec: list, cam_frame: str, now_t: float):
        if now_t - self._last_landmark_correction_t < self._correction_period:
            return

        if len(tvec) != 3:
            return

        rel_cam = (float(tvec[0]), float(tvec[1]), float(tvec[2]))
        rng = math.sqrt(rel_cam[0] ** 2 + rel_cam[1] ** 2 + rel_cam[2] ** 2)
        if rng > self._max_landmark_range_m:
            return

        landmark = self._landmarks[str(aruco_id)]
        p_lm_map = (float(landmark['x']), float(landmark['y']), float(landmark.get('z', 0.0)))

        try:
            t_base_from_cam = self._tf_buf.lookup_transform(
                'base_link', cam_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
            t_map_from_base = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        q_bc = (
            t_base_from_cam.transform.rotation.x,
            t_base_from_cam.transform.rotation.y,
            t_base_from_cam.transform.rotation.z,
            t_base_from_cam.transform.rotation.w,
        )
        t_bc = (
            t_base_from_cam.transform.translation.x,
            t_base_from_cam.transform.translation.y,
            t_base_from_cam.transform.translation.z,
        )

        rel_base_rot = rotate_vec(q_bc, rel_cam)
        rel_base = (
            t_bc[0] + rel_base_rot[0],
            t_bc[1] + rel_base_rot[1],
            t_bc[2] + rel_base_rot[2],
        )

        q_mb = (
            t_map_from_base.transform.rotation.x,
            t_map_from_base.transform.rotation.y,
            t_map_from_base.transform.rotation.z,
            t_map_from_base.transform.rotation.w,
        )
        rel_map = rotate_vec(q_mb, rel_base)

        corrected_x = p_lm_map[0] - rel_map[0]
        corrected_y = p_lm_map[1] - rel_map[1]
        corrected_yaw = quat_to_yaw(*q_mb)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = corrected_x
        pose_msg.pose.pose.position.y = corrected_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(corrected_yaw * 0.5)
        pose_msg.pose.pose.orientation.w = math.cos(corrected_yaw * 0.5)
        pose_msg.pose.covariance[0] = 0.04
        pose_msg.pose.covariance[7] = 0.04
        pose_msg.pose.covariance[35] = 0.08
        self._initial_pose_pub.publish(pose_msg)

        self._last_landmark_correction_t = now_t
        self.get_logger().info(
            f'Landmark {aruco_id} correction -> x={corrected_x:.2f} y={corrected_y:.2f}',
            throttle_duration_sec=1.0
        )

    def _process_product(self, aruco_id: int, tvec: list, cam_frame: str, now_t: float):
        if len(tvec) != 3:
            return

        rel_cam = (float(tvec[0]), float(tvec[1]), float(tvec[2]))

        try:
            t_map_from_cam = self._tf_buf.lookup_transform(
                'map', cam_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        q_mc = (
            t_map_from_cam.transform.rotation.x,
            t_map_from_cam.transform.rotation.y,
            t_map_from_cam.transform.rotation.z,
            t_map_from_cam.transform.rotation.w,
        )
        p_mc = (
            t_map_from_cam.transform.translation.x,
            t_map_from_cam.transform.translation.y,
            t_map_from_cam.transform.translation.z,
        )

        rel_map = rotate_vec(q_mc, rel_cam)
        prod_map = (
            p_mc[0] + rel_map[0],
            p_mc[1] + rel_map[1],
            p_mc[2] + rel_map[2],
        )

        child = f'product_{aruco_id}'
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'map'
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = prod_map[0]
        tf_msg.transform.translation.y = prod_map[1]
        tf_msg.transform.translation.z = prod_map[2]
        tf_msg.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_msg)

        self._products[child] = {
            'aruco_id': aruco_id,
            'x': prod_map[0],
            'y': prod_map[1],
            'z': prod_map[2],
            'last_seen': now_t,
        }

    def _publish_registry(self):
        now_t = time.time()
        fresh = {}
        for frame, info in self._products.items():
            if now_t - info['last_seen'] <= self._product_stale_sec:
                fresh[frame] = info

        self._product_registry_pub.publish(String(data=json.dumps({
            'stamp': now_t,
            'stale_after_sec': self._product_stale_sec,
            'products': fresh,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
