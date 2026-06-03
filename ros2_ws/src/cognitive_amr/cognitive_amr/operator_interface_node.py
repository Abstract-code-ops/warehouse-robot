#!/usr/bin/env python3
"""
operator_interface_node.py
─────────────────────────────────────────────────────────────────────────────
HMI layer — publishes live status topics for Foxglove panels.

Publishes (30 Hz unless otherwise noted):
  /hmi/mission_status   std_msgs/String  — current state text (from task_planner)
  /hmi/inventory_panel  std_msgs/String  — formatted table string (2 Hz)
  /hmi/audit_log        std_msgs/String  — last 10 audit events (2 Hz)
  /hmi/slot_markers     visualization_msgs/MarkerArray  — coloured cubes per slot
  /hmi/operator_alert   std_msgs/String  — forwarded mismatch dialog

Sources:
  /hmi/mission_status   — forwarded from /scan/status + /hmi/mission_status
  /inventory/request    — to query audit summary
  /inventory/response   — from inventory_manager
"""

import json
import math
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from cognitive_amr.warehouse_constants import (
    SHELVES, PRODUCTS, DB_INITIAL_INVENTORY,
    SLOT_X_OFFSETS, PRODUCT_Z, get_approach_pose
)

# Slot marker colours
_GREEN  = (0.10, 0.85, 0.20, 0.85)
_YELLOW = (0.95, 0.85, 0.05, 0.85)
_RED    = (0.95, 0.10, 0.10, 0.85)
_BLUE   = (0.10, 0.50, 0.95, 0.85)
_GREY   = (0.55, 0.55, 0.55, 0.40)

CONFIDENCE_COLOUR = {
    'confirmed':   _GREEN,
    'unverified':  _YELLOW,
    'discrepancy': _RED,
    'scanning':    _BLUE,
    'unknown':     _GREY,
}


class OperatorInterfaceNode(Node):

    def __init__(self):
        super().__init__('operator_interface_node')

        # ── Cache ─────────────────────────────────────────────────────────
        # product_id → {'shelf_id', 'slot', 'confidence'}
        self._slot_state: dict = {}
        self._current_scanning: str | None = None   # product_id being scanned
        self._lock = threading.Lock()

        # Seed from compile-time DB_INITIAL_INVENTORY (updated live via
        # /inventory/response events)
        for sku, rec in DB_INITIAL_INVENTORY.items():
            self._slot_state[sku] = {
                'shelf_id':   rec['shelf'],
                'slot_index': rec['slot'],
                'confidence': rec.get('confidence', 'unverified'),
            }

        # ── Publishers ───────────────────────────────────────────────────
        self._status_pub   = self.create_publisher(String, '/hmi/mission_status', 10)
        self._inv_pub      = self.create_publisher(String, '/hmi/inventory_panel', 10)
        self._audit_pub    = self.create_publisher(String, '/hmi/audit_log', 10)
        self._marker_pub   = self.create_publisher(MarkerArray, '/hmi/slot_markers', 10)
        self._alert_fwd    = self.create_publisher(String, '/hmi/operator_alert', 10)
        self._inv_req_pub  = self.create_publisher(String, '/inventory/request', 10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            String, '/hmi/mission_status', self._on_status, 10)
        self.create_subscription(
            String, '/scan/status', self._on_status, 10)
        self.create_subscription(
            String, '/hmi/operator_alert', self._on_alert, 10)
        self.create_subscription(
            String, '/inventory/response', self._on_inv_response, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(0.5, self._publish_slot_markers)
        self.create_timer(2.0, self._request_audit_summary)

        # Correlate audit summary responses
        self._pending: dict[str, threading.Event] = {}
        self._audit_data: dict = {}

        self.get_logger().info('Operator interface ready')

    # ── Pass-through status ───────────────────────────────────────────────

    def _on_status(self, msg: String):
        # Update scanning state hint from status text
        text = msg.data
        with self._lock:
            if '[SCANNING]' in text or 'scanning' in text.lower():
                # Try to extract product from status
                pass
        # Relay to /hmi/mission_status (avoid loop: the source may differ)
        # (already republished from task_planner — just pass through)

    def _on_alert(self, msg: String):
        # Forwarded by task_planner; this node also publishes to same topic
        # — nothing to do here, just for the subscription side
        pass

    def _on_inv_response(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rid = data.get('request_id')
        if rid and rid in self._pending:
            self._audit_data[rid] = data
            self._pending[rid].set()
            return

        # Detect update_location responses to refresh slot colours.
        # lookup_product also contains shelf_id/slot_index, so require
        # approach fields that are unique to update_location responses.
        d = data.get('data', {})
        if (
            data.get('ok')
            and 'shelf_id' in d
            and 'slot_index' in d
            and 'approach_x' in d
            and 'approach_y' in d
            and 'approach_yaw' in d
        ):
            pid = d.get('product_id', '')
            if pid:
                with self._lock:
                    self._slot_state[pid] = {
                        'shelf_id':   d['shelf_id'],
                        'slot_index': d['slot_index'],
                        'confidence': 'confirmed',
                    }

    # ── Audit summary request ─────────────────────────────────────────────

    def _request_audit_summary(self):
        rid = str(uuid.uuid4())
        ev = threading.Event()
        self._pending[rid] = ev

        self._inv_req_pub.publish(String(data=json.dumps({
            'type': 'get_audit_summary',
            'request_id': rid,
            'params': {'limit': 10}
        })))

        # Non-blocking: just schedule to process in next callback
        def _delayed():
            triggered = ev.wait(timeout=3.0)
            if triggered and rid in self._audit_data:
                d = self._audit_data.pop(rid)
                self._pending.pop(rid, None)
                if d.get('ok'):
                    self._publish_audit_log(d['data'])
                    self._publish_inventory_panel(d['data'])
            else:
                self._pending.pop(rid, None)

        threading.Thread(target=_delayed, daemon=True).start()

    # ── Slot markers ──────────────────────────────────────────────────────

    def _publish_slot_markers(self):
        ma = MarkerArray()
        marker_id = 0

        with self._lock:
            slot_state_copy = dict(self._slot_state)
            scanning = self._current_scanning

        for sku, state in slot_state_copy.items():
            shelf_id   = state['shelf_id']
            slot_index = state['slot_index']
            conf       = state['confidence']

            if shelf_id not in SHELVES:
                continue

            s = SHELVES[shelf_id]
            wx = s['cx'] + SLOT_X_OFFSETS[slot_index]
            wy = s['cy']
            wz = PRODUCT_Z + 0.45   # slightly above box

            # Override colour if currently scanning this slot
            if sku == scanning:
                colour = _BLUE
            else:
                colour = CONFIDENCE_COLOUR.get(conf, _GREY)

            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'slot_markers'
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = wz
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=0.20, y=0.20, z=0.20)
            m.color = ColorRGBA(r=colour[0], g=colour[1],
                                b=colour[2], a=colour[3])
            ma.markers.append(m)
            marker_id += 1

        self._marker_pub.publish(ma)

    # ── Audit log panel ───────────────────────────────────────────────────

    def _publish_audit_log(self, summary: dict):
        events = summary.get('events', [])
        lines = ["AUDIT LOG — LAST 10 EVENTS",
                 "─" * 54]
        for ev in events:
            ts = ev.get('timestamp', '')[:19]
            outcome = ev.get('outcome', '?').upper()[:10].ljust(10)
            pid = ev.get('product_id', '?')
            shelf = ev.get('shelf_scanned', '?')
            slot  = ev.get('slot_scanned', '?')
            lines.append(f"[{ts}] {outcome} | {pid} | {shelf}-slot{slot}")
        disc = summary.get('discrepancy_count', 0)
        unv  = summary.get('unverified_count', 0)
        tot  = summary.get('total_products', 0)
        lines += ['─' * 54,
                  f"Confirmed: {tot - disc - unv}/{tot}  "
                  f"Discrepancies: {disc}  Unverified: {unv}"]
        self._audit_pub.publish(String(data='\n'.join(lines)))

    # ── Inventory panel ───────────────────────────────────────────────────

    def _publish_inventory_panel(self, summary: dict):
        header = (f"{'SKU':<12} | {'PRODUCT':<22} | "
                  f"{'LOCATION':<10} | STATUS")
        sep = '─' * len(header)
        lines = ['LIVE INVENTORY STATE', sep, header, sep]

        with self._lock:
            ss = dict(self._slot_state)

        for sku, pinfo in PRODUCTS.items():
            state = ss.get(sku, {})
            shelf  = state.get('shelf_id', '?')
            slot   = state.get('slot_index', '?')
            conf   = state.get('confidence', 'unverified')
            icon = {'confirmed': '✓', 'discrepancy': '✗',
                    'unverified': '?', 'scanning': '⟳'}.get(conf, '?')
            name = pinfo['name'][:22]
            loc  = f"{shelf}-slot{slot}"
            lines.append(
                f"{sku:<12} | {name:<22} | {loc:<10} | {icon} {conf.upper()}")

        lines.append(sep)
        self._inv_pub.publish(String(data='\n'.join(lines)))

    # ── Helpers ───────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OperatorInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
