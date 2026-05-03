#!/usr/bin/env python3
"""
Simple dual-LiDAR merger node.
Merges /scan/lidar_front_left and /scan/lidar_back_right into /scan
No external dependencies beyond rclpy and sensor_msgs.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException


class LaserMerger(Node):

    def __init__(self):
        super().__init__('laser_merger')

        self.declare_parameter('destination_frame', 'base_link')
        self.declare_parameter('angle_min',  -math.pi)
        self.declare_parameter('angle_max',   math.pi)
        self.declare_parameter('angle_increment', 0.00581)
        self.declare_parameter('range_min',  0.10)
        self.declare_parameter('range_max',  20.0)

        self.dest_frame   = self.get_parameter('destination_frame').value
        self.angle_min    = self.get_parameter('angle_min').value
        self.angle_max    = self.get_parameter('angle_max').value
        self.angle_inc    = self.get_parameter('angle_increment').value
        self.range_min    = self.get_parameter('range_min').value
        self.range_max    = self.get_parameter('range_max').value

        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_inc)

        # TF buffer for transforming scans into base_link frame
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store latest scan from each lidar
        self._scans = {}

        # Subscribers
        self.create_subscription(
            LaserScan, '/scan/lidar_front_left',
            lambda msg: self._scan_cb(msg, 'lidar_front_left'), 10)

        self.create_subscription(
            LaserScan, '/scan/lidar_back_right',
            lambda msg: self._scan_cb(msg, 'lidar_back_right'), 10)

        # Publisher
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Merge at 10Hz
        self.create_timer(0.1, self._merge_and_publish)

        self.get_logger().info('Laser merger ready — merging to /scan')

    def _scan_cb(self, msg: LaserScan, name: str):
        self._scans[name] = msg

    def _merge_and_publish(self):
        if len(self._scans) < 1:
            return   # Wait until we have at least one scan

        # Output array — start with max range (no obstacle)
        merged = np.full(self.num_readings, self.range_max)

        for name, scan in self._scans.items():
            self._project_scan(scan, merged)

        # Build output message
        out = LaserScan()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.dest_frame
        out.angle_min       = self.angle_min
        out.angle_max       = self.angle_max
        out.angle_increment = self.angle_inc
        out.time_increment  = 0.0
        out.scan_time       = 0.1
        out.range_min       = self.range_min
        out.range_max       = self.range_max
        out.ranges          = merged.tolist()
        self.pub.publish(out)

    def _project_scan(self, scan: LaserScan, output: np.ndarray):
        """
        Project a LaserScan into the output array.
        Uses the scan's own frame_id to determine angular offset via TF.
        Falls back to using raw angles if TF is not yet available.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.dest_frame,
                scan.header.frame_id,
                rclpy.time.Time())
            # Extract yaw from quaternion
            q = tf.transform.rotation
            yaw_offset = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except TransformException:
            yaw_offset = 0.0   # Fallback: no rotation correction

        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            # Angle of this reading in the scan's own frame
            angle_in_scan = scan.angle_min + i * scan.angle_increment
            # Angle in destination frame
            angle_in_dest = angle_in_scan + yaw_offset
            # Wrap to [-pi, pi]
            angle_in_dest = math.atan2(math.sin(angle_in_dest),
                                        math.cos(angle_in_dest))
            # Map to output index
            if not (self.angle_min <= angle_in_dest <= self.angle_max):
                continue
            idx = int((angle_in_dest - self.angle_min) / self.angle_inc)
            if 0 <= idx < len(output):
                output[idx] = min(output[idx], r)   # Take closest reading


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LaserMerger())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
