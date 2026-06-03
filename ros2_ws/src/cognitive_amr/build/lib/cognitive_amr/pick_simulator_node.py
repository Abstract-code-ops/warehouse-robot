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
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from cognitive_amr.warehouse_constants import (
    PRODUCTS, DISPATCH_ZONE_1
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

        # Currently carried product (model name)
        self._carrying: str | None = None
        self._carry_thread: threading.Thread | None = None

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

        if self._carry_thread and self._carry_thread.is_alive():
            self.get_logger().warn(
                "Already carrying a product — ignoring pick event")
            return

        self._carrying = model_name
        self._carry_thread = threading.Thread(
            target=self._carry_product,
            args=(product_id, model_name),
            daemon=True
        )
        self._carry_thread.start()

    def _carry_product(self, product_id: str, model_name: str):
        self.get_logger().info(
            f"Pick: teleporting {model_name} into cargo bin")

        # 1. Snap to cargo bin
        bx, by, bz = self._get_bin_world_pos()
        if bx is None:
            self.get_logger().warn("Could not get robot pose — using approx")
            bx, by, bz = 0.0, 0.0, 0.5

        _gz_set_pose(model_name, bx, by, bz)

        # 2. Follow bin while robot moves (10 Hz update for ~60s max)
        t_end = time.time() + 120.0
        while time.time() < t_end:
            time.sleep(0.1)
            nx, ny, nz = self._get_bin_world_pos()
            if nx is not None:
                _gz_set_pose(model_name, nx, ny, nz)

            # Check for deposit trigger: is robot near dispatch zone?
            rx, ry = self._get_robot_xy()
            if rx is not None:
                dx, dy = DISPATCH_ZONE_1
                dist = math.hypot(rx - dx, ry - dy)
                if dist < 1.5:
                    self._deposit(product_id, model_name, dx, dy)
                    return

        # Timeout — drop at current position
        self.get_logger().warn(
            f"Carry timeout for {product_id} — depositing in place")
        nx, ny, nz = self._get_bin_world_pos()
        if nx is not None:
            _gz_set_pose(model_name, nx, ny, 0.95)
        self._carrying = None

    def _deposit(self, product_id: str, model_name: str,
                 dx: float, dy: float):
        deposit_x = dx + 0.5
        deposit_y = dy
        deposit_z = 0.35
        self.get_logger().info(
            f"Deposit: {model_name} → ({deposit_x:.2f}, {deposit_y:.2f})")
        _gz_set_pose(model_name, deposit_x, deposit_y, deposit_z)
        self._carrying = None
        self._deposit_pub.publish(String(data=json.dumps({
            'product_id': product_id,
            'deposit_x': deposit_x,
            'deposit_y': deposit_y,
        })))

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
