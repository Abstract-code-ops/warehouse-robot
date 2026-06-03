#!/usr/bin/env python3
"""
pick_simulator_node.py
─────────────────────────────────────────────────────────────────────────────
Simulates the physical pick: teleports a product box from the shelf into the
robot's cargo bin, then carries it to the dispatch zone.

Uses the Gazebo Sim `gz service` CLI to call the set_entity_pose service.
This avoids needing gz-transport Python bindings.

Topics:
  SUB  /robot/pick_event       std_msgs/String  JSON pick event
  PUB  /robot/deposit_event    std_msgs/String  JSON deposit confirmation
  SUB  /tf                     tf2_msgs/TFMessage  (via tf2_ros buffer)
"""

import json
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from cognitive_amr.warehouse_constants import (
    PRODUCTS, DISPATCH_ZONES_BOTTOM_LEFT
)

WORLD_NAME = 'tugbot_style_ai_warehouse'

# Cargo bin centre offset from base_link (mast at x=0, slider default pos=0)
# mast_joint z=0.05, slider_joint z=0.40, cargo_bin_joint xyz="0.10 0 0.02"
# bin bottom plate half-thickness = 0.02 → centre z ≈ 0.05+0.40+0.02 = 0.47
BIN_OFFSET_X =  0.10
BIN_OFFSET_Y =  0.0
BIN_OFFSET_Z =  0.47


def _gz_set_pose(model_name: str, x: float, y: float, z: float,
                 qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> bool:
    """Call gz service to teleport a model in Gazebo Sim."""
    req = (
        f"name: '{model_name}', "
        f"pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
        f"orientation: {{x: {qx:.4f}, y: {qy:.4f}, "
        f"z: {qz:.4f}, w: {qw:.4f}}}}}"
    )
    cmd = [
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/set_entity_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '500',
        '--req', req,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def _sku_to_model_name(product_id: str) -> str:
    return f"product_{product_id.replace('-', '_')}"


class PickSimulatorNode(Node):

    def __init__(self):
        super().__init__('pick_simulator_node')

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self.create_subscription(
            String, '/robot/pick_event', self._on_pick_event, 10)

        self._deposit_pub = self.create_publisher(
            String, '/robot/deposit_event', 10)
        self._viz_pub = self.create_publisher(
            MarkerArray, '/hmi/pick_sim_markers', 10)

        # Currently carried products: list of (product_id, model_name)
        self._carrying: list[tuple[str, str]] = []
        self._carry_lock = threading.Lock()
        self._deposited: dict[str, tuple[float, float, float]] = {}
        self._carry_thread: threading.Thread | None = None
        self.create_timer(0.2, self._publish_markers)

        self.get_logger().info('Pick simulator ready')

    def _on_pick_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        product_id = ev.get('product_id', '')
        if not product_id:
            return

        model_name = _sku_to_model_name(product_id)

        with self._carry_lock:
            if any(pid == product_id for pid, _ in self._carrying):
                self.get_logger().warn(f"{product_id} already in cargo bin, ignoring duplicate pick")
                return
            self._carrying.append((product_id, model_name))
            self._deposited.pop(product_id, None)

        self.get_logger().info(f"Pick: loading {model_name} into cargo bin")
        self._snap_all_to_bin()

        if not self._carry_thread or not self._carry_thread.is_alive():
            self._carry_thread = threading.Thread(
                target=self._carry_loop,
                daemon=True
            )
            self._carry_thread.start()

    def _carry_loop(self):
        t_end = time.time() + 180.0
        while time.time() < t_end:
            time.sleep(0.1)
            with self._carry_lock:
                if not self._carrying:
                    return

            self._snap_all_to_bin()

            # Check for deposit trigger: is robot near dispatch zone?
            rx, ry = self._get_robot_xy()
            if rx is not None:
                for dx, dy in DISPATCH_ZONES_BOTTOM_LEFT:
                    dist = math.hypot(rx - dx, ry - dy)
                    if dist < 1.5:
                        self._deposit_all(dx, dy)
                        return

        # Timeout — drop all at current position
        self.get_logger().warn('Carry timeout — depositing all carried products in place')
        self._deposit_all(None, None)

    def _snap_all_to_bin(self):
        bx, by, _ = self._get_bin_world_pos()
        if bx is None:
            self.get_logger().warn('Could not get robot pose — using approx cargo pose')
            bx, by = 0.0, 0.0

        with self._carry_lock:
            carrying = list(self._carrying)

        for idx, (_, model_name) in enumerate(carrying):
            z = BIN_OFFSET_Z + 0.10 * idx
            _gz_set_pose(model_name, bx, by, z)

    def _deposit_all(self, dx: float | None, dy: float | None):
        with self._carry_lock:
            carrying = list(self._carrying)
            self._carrying.clear()

        if not carrying:
            return

        if dx is None or dy is None:
            rx, ry = self._get_robot_xy()
            if rx is None:
                rx, ry = 0.0, 0.0
            dx, dy = rx, ry

        for idx, (product_id, model_name) in enumerate(carrying):
            deposit_x = dx + 0.5 + 0.18 * (idx % 3)
            deposit_y = dy + 0.18 * (idx // 3)
            deposit_z = 0.35
            self.get_logger().info(
                f"Deposit: {model_name} → ({deposit_x:.2f}, {deposit_y:.2f})")
            _gz_set_pose(model_name, deposit_x, deposit_y, deposit_z)
            self._deposited[product_id] = (deposit_x, deposit_y, deposit_z)
            self._deposit_pub.publish(String(data=json.dumps({
                'product_id': product_id,
                'deposit_x': deposit_x,
                'deposit_y': deposit_y,
            })))

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Carrying markers follow cargo bin.
        bx, by, _ = self._get_bin_world_pos()
        with self._carry_lock:
            carrying = list(self._carrying)

        for idx, (product_id, _) in enumerate(carrying):
            if bx is None:
                continue
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'pick_carry'
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = bx
            m.pose.position.y = by
            m.pose.position.z = BIN_OFFSET_Z + 0.12 * idx
            m.pose.orientation.w = 1.0
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.10
            m.color.r = 0.10
            m.color.g = 0.75
            m.color.b = 0.95
            m.color.a = 0.95
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now
            t.ns = 'pick_carry_text'
            t.id = idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = bx
            t.pose.position.y = by
            t.pose.position.z = BIN_OFFSET_Z + 0.22 + 0.12 * idx
            t.pose.orientation.w = 1.0
            t.scale.z = 0.10
            t.color.r = 0.15
            t.color.g = 0.95
            t.color.b = 1.0
            t.color.a = 0.95
            t.text = product_id
            ma.markers.append(t)

        # Deposited markers stay at dispatch floor.
        base = 1000
        for idx, (product_id, pos) in enumerate(sorted(self._deposited.items())):
            x, y, z = pos
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'pick_deposited'
            m.id = base + idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = 0.12
            m.color.r = 0.95
            m.color.g = 0.75
            m.color.b = 0.20
            m.color.a = 0.95
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now
            t.ns = 'pick_deposited_text'
            t.id = base + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = z + 0.15
            t.pose.orientation.w = 1.0
            t.scale.z = 0.09
            t.color.r = 1.0
            t.color.g = 0.9
            t.color.b = 0.4
            t.color.a = 0.95
            t.text = product_id
            ma.markers.append(t)

        self._viz_pub.publish(ma)

    # ── TF helpers ────────────────────────────────────────────────────────

    def _get_robot_xy(self) -> tuple:
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
            return (t.transform.translation.x,
                    t.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return (None, None)

    def _get_bin_world_pos(self) -> tuple:
        """Compute cargo bin world position from base_link TF."""
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
            tr = t.transform.translation
            rot = t.transform.rotation

            # Apply cargo_bin_joint offset in robot frame, then rotate
            # For 2D: only yaw matters
            siny = 2 * (rot.w * rot.z + rot.x * rot.y)
            cosy = 1 - 2 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny, cosy)

            bx = tr.x + BIN_OFFSET_X * math.cos(yaw) - BIN_OFFSET_Y * math.sin(yaw)
            by = tr.y + BIN_OFFSET_X * math.sin(yaw) + BIN_OFFSET_Y * math.cos(yaw)
            bz = BIN_OFFSET_Z
            return bx, by, bz
        except (LookupException, ConnectivityException, ExtrapolationException):
            return (None, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = PickSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
