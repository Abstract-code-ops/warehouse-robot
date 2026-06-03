#!/usr/bin/env python3
"""
inventory_manager_node.py
─────────────────────────────────────────────────────────────────────────────
Single database interface node. All other nodes query inventory through this
one — no other node touches the SQLite file directly.

Topics:
  SUB  /inventory/request   std_msgs/String  JSON request envelope
  PUB  /inventory/response  std_msgs/String  JSON response envelope

Request schema:
  { "type": "lookup_product"   | "record_verification" |
             "update_location" | "get_audit_summary",
    "request_id": str,
    "params": { ... } }

Response schema:
  { "request_id": str, "ok": bool, "data": { ... }, "error": str|null }
"""

import os
import sqlite3
import json
import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory
from cognitive_amr.warehouse_constants import get_approach_pose


DB_PATH_DEFAULT = '/ros2_ws/src/cognitive_amr/config/inventory.db'


def _conn(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


class InventoryManagerNode(Node):

    def __init__(self):
        super().__init__('inventory_manager_node')

        self.declare_parameter('db_path', DB_PATH_DEFAULT)
        self._db_path = self.get_parameter('db_path').value or ""

        if not os.path.exists(self._db_path):
            self.get_logger().error(
                f"Database not found: {self._db_path}\n"
                "Run: python3 scripts/init_database.py"
            )
        else:
            self.get_logger().info(f"Inventory DB: {self._db_path}")

        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            String, '/inventory/request', self._on_request, 10)
        self._pub = self.create_publisher(String, '/inventory/response', 10)

    # ── Dispatch ──────────────────────────────────────────────────────────

    def _on_request(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON in inventory request: {e}")
            return

        req_id = req.get('request_id', 'unknown')
        req_type = req.get('type', '')
        params = req.get('params', {})

        handlers = {
            'lookup_product':     self._lookup_product,
            'record_verification': self._record_verification,
            'update_location':    self._update_location,
            'get_audit_summary':  self._get_audit_summary,
        }

        if req_type not in handlers:
            self._respond(req_id, ok=False, error=f"Unknown type: {req_type}")
            return

        try:
            data = handlers[req_type](params)
            self._respond(req_id, ok=True, data=data)
        except Exception as e:
            self.get_logger().error(f"Inventory error [{req_type}]: {e}")
            self._respond(req_id, ok=False, error=str(e))

    def _respond(self, request_id: str, ok: bool,
                 data: dict = None, error: str = None):
        payload = {
            'request_id': request_id,
            'ok': ok,
            'data': data or {},
            'error': error
        }
        self._pub.publish(String(data=json.dumps(payload)))

    # ── Handlers ──────────────────────────────────────────────────────────

    def _lookup_product(self, params: dict) -> dict:
        """Return inventory record for a product by id or name."""
        product_id = params.get('product_id')
        product_name = params.get('product_name')

        with self._lock:
            conn = _conn(self._db_path)
            if product_id:
                row = conn.execute(
                    """SELECT i.*, p.product_name, p.aruco_id
                       FROM inventory i JOIN products p USING(product_id)
                       WHERE i.product_id = ?""",
                    (product_id,)
                ).fetchone()
            elif product_name:
                row = conn.execute(
                    """SELECT i.*, p.product_name, p.aruco_id
                       FROM inventory i JOIN products p USING(product_id)
                       WHERE LOWER(p.product_name) LIKE LOWER(?)""",
                    (f'%{product_name}%',)
                ).fetchone()
            else:
                raise ValueError("Provide product_id or product_name")
            conn.close()

        if not row:
            raise LookupError(f"Product not found: {product_id or product_name}")

        return dict(row)

    def _record_verification(self, params: dict) -> dict:
        """Write a scan event to audit_log and update inventory confidence."""
        required = ('product_id', 'shelf_scanned', 'slot_scanned',
                    'aruco_detected', 'outcome', 'robot_pose_x', 'robot_pose_y')
        for k in required:
            if k not in params:
                raise ValueError(f"Missing param: {k}")

        ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
        outcome = params['outcome']   # 'match' | 'mismatch' | 'not_found'

        with self._lock:
            conn = _conn(self._db_path)
            conn.execute(
                """INSERT INTO audit_log
                   (timestamp, product_id, shelf_scanned, slot_scanned,
                    aruco_detected, outcome, robot_pose_x, robot_pose_y)
                   VALUES (?,?,?,?,?,?,?,?)""",
                (ts, params['product_id'], params['shelf_scanned'],
                 params['slot_scanned'], params.get('aruco_detected'),
                 outcome, params['robot_pose_x'], params['robot_pose_y'])
            )
            # Update confidence in inventory table
            if outcome == 'match':
                new_conf = 'confirmed'
            elif outcome in ('mismatch', 'not_found'):
                new_conf = 'discrepancy'
            else:
                new_conf = 'unverified'

            conn.execute(
                "UPDATE inventory SET confidence=?, last_verified=? WHERE product_id=?",
                (new_conf, ts, params['product_id'])
            )
            conn.commit()
            event_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]
            conn.close()

        self.get_logger().info(
            f"Audit logged: {params['product_id']} {outcome}"
        )
        return {'event_id': event_id}

    def _update_location(self, params: dict) -> dict:
        """Move a product to a new shelf/slot in the inventory table."""
        required = ('product_id', 'shelf_id', 'slot_index')
        for k in required:
            if k not in params:
                raise ValueError(f"Missing param: {k}")

        product_id = params['product_id']
        shelf_id   = params['shelf_id']
        slot_index = int(params['slot_index'])

        ax, ay, ayaw = get_approach_pose(shelf_id, slot_index)
        ts = datetime.datetime.now(datetime.timezone.utc).isoformat()

        with self._lock:
            conn = _conn(self._db_path)
            conn.execute(
                """UPDATE inventory
                   SET shelf_id=?, slot_index=?,
                       expected_x=?, expected_y=?, expected_yaw=?,
                       last_verified=?, confidence='confirmed'
                   WHERE product_id=?""",
                (shelf_id, slot_index, ax, ay, ayaw, ts, product_id)
            )
            # Also log to audit_log
            conn.execute(
                """INSERT INTO audit_log
                   (timestamp, product_id, shelf_scanned, slot_scanned,
                    outcome, operator_action)
                   VALUES (?,?,?,?,'location_update','operator_confirmed')""",
                (ts, product_id, shelf_id, slot_index)
            )
            conn.commit()
            conn.close()

        self.get_logger().info(
            f"Location updated: {product_id} → {shelf_id}-slot{slot_index}"
        )
        return {'product_id': product_id, 'shelf_id': shelf_id,
                'slot_index': slot_index,
                'approach_x': ax, 'approach_y': ay, 'approach_yaw': ayaw}

    def _get_audit_summary(self, params: dict) -> dict:
        """Return last N audit events and discrepancy count."""
        limit = int(params.get('limit', 10))
        with self._lock:
            conn = _conn(self._db_path)
            rows = conn.execute(
                """SELECT * FROM audit_log ORDER BY event_id DESC LIMIT ?""",
                (limit,)
            ).fetchall()
            disc_count = conn.execute(
                "SELECT COUNT(*) FROM inventory WHERE confidence='discrepancy'"
            ).fetchone()[0]
            unverified = conn.execute(
                "SELECT COUNT(*) FROM inventory WHERE confidence='unverified'"
            ).fetchone()[0]
            total = conn.execute(
                "SELECT COUNT(*) FROM inventory"
            ).fetchone()[0]
            conn.close()

        return {
            'events': [dict(r) for r in rows],
            'discrepancy_count': disc_count,
            'unverified_count': unverified,
            'total_products': total
        }


def main(args=None):
    rclpy.init(args=args)
    node = InventoryManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
