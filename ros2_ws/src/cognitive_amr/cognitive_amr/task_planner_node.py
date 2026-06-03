#!/usr/bin/env python3
"""
task_planner_node.py — Simplified mission state machine.

Flow:  IDLE → RESOLVING → NAVIGATING → SCANNING → PICKING → DISPATCHING → IDLE

Scan logic (aisle waypoint sweep + ArUco detection) runs inline.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import Buffer, TransformListener

from cognitive_amr.warehouse_constants import (
    SHELVES, SLOT_X_OFFSETS, get_approach_pose,
)
from cognitive_amr.node_helpers import (
    JsonRequestClient, go_to_xyyaw, make_pose_stamped, parse_json_msg, publish_json,
)

# ── States ─────────────────────────────────────────────────────────────────────
IDLE, RESOLVING, NAVIGATING, SCANNING, PICKING, DISPATCHING = range(6)
STATE_NAMES = {
    0: 'IDLE', 1: 'RESOLVING', 2: 'NAVIGATING',
    3: 'SCANNING', 4: 'PICKING', 5: 'DISPATCHING',
}


class TaskPlannerNode(Node):

    _PICK_STANDOFF_EXTRA = 0.30
    _DISPATCH_ZONES = ((1.7, 1.1), (1.7, 2.8))
    _MAX_RETRIES = 3
    _PREEMPT_CONSECUTIVE_DETECTIONS = 3
    _PICK_CONFIRM_CONSECUTIVE_DETECTIONS = 2
    _PICK_CONFIRM_TIMEOUT_SEC = 8.0
    _PICK_CONFIRM_CENTER_TVEC_X_M = 0.22
    _PICK_CONFIRM_MAX_DISTANCE_M = 2.8

    def __init__(self):
        super().__init__('task_planner_node')

        # Nav2
        self._nav = BasicNavigator()
        self._nav.waitUntilNav2Active()

        # TF (for robot pose during scan)
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # Latest ArUco detections from the camera
        self._detections: list = []
        self._det_lock = threading.Lock()
        self._last_scan_miss_log_t = 0.0

        self._state = IDLE
        self._ready = False   # set True after startup escape finishes

        # Publishers
        self._status_pub  = self.create_publisher(String, '/hmi/mission_status', 10)
        self._inv_req_pub = self.create_publisher(String, '/inventory/request', 10)
        self._pick_pub    = self.create_publisher(String, '/robot/pick_event', 10)
        self._alert_pub   = self.create_publisher(String, '/hmi/operator_alert', 10)

        # Subscribers
        self.create_subscription(
            String, '/task_request_parsed', self._on_task_request, 10)
        self.create_subscription(
            String, '/inventory/response', self._on_inventory_response, 10)
        self.create_subscription(
            String, '/camera/detected_markers', self._on_markers, 10)

        self._rpc = JsonRequestClient(self.get_logger())

        # Deferred mismatch memory: ArUco id -> expected location context.
        # If a product is missing at its expected slot/aisle, keep it here and
        # reconcile once seen elsewhere during the same run.
        self._missing_lock = threading.Lock()
        self._missing_products: dict[int, dict] = {}

        # Move away from the charging dock before accepting tasks
        threading.Thread(target=self._startup_escape, daemon=True).start()

    # ── Subscriptions ──────────────────────────────────────────────────────────

    def _on_markers(self, msg: String):
        data = parse_json_msg(msg, self.get_logger(), '/camera/detected_markers')
        if data:
            raw = data.get('detections', [])
            normalized = []
            for det in raw:
                if not isinstance(det, dict):
                    continue
                entry = dict(det)
                try:
                    # Normalize id type once so scan matching is deterministic.
                    raw_id = entry.get('aruco_id')
                    if raw_id is not None:
                        entry['aruco_id'] = int(raw_id)
                except (TypeError, ValueError):
                    pass
                normalized.append(entry)
            with self._det_lock:
                self._detections = normalized

            self._reconcile_missing_from_detections(normalized)

    def _on_inventory_response(self, msg: String):
        self._rpc.handle_response_msg(msg, '/inventory/response')

    def _on_task_request(self, msg: String):
        req = parse_json_msg(msg, self.get_logger(), '/task_request_parsed')
        if not req or req.get('intent') == 'clarify':
            return
        if not self._ready:
            self.get_logger().warn('Not ready yet — startup escape still running')
            return
        if self._state != IDLE:
            self.get_logger().warn(
                f'Task received while {STATE_NAMES[self._state]} — ignoring')
            return
        threading.Thread(target=self._run_mission, args=(req,), daemon=True).start()

    # ── Startup ────────────────────────────────────────────────────────────────

    def _startup_escape(self):
        import math

        self._set_status('[STARTUP] Clearing charging station — moving to main aisle')
        self.get_logger().info('Startup escape: driving 2.0 m straight via odometry')

        cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        done_evt = threading.Event()
        start_pos: list[tuple[float, float] | None] = [None]

        def _odom_cb(msg):
            p = msg.pose.pose.position
            if start_pos[0] is None:
                start_pos[0] = (p.x, p.y)
                return
            dist = math.hypot(p.x - start_pos[0][0], p.y - start_pos[0][1])
            if dist >= 2.0:
                cmd_pub.publish(Twist())
                done_evt.set()
            else:
                tw = Twist()
                tw.linear.x = 0.3
                cmd_pub.publish(tw)

        odom_sub = self.create_subscription(Odometry, '/odom', _odom_cb, 10)

        reached = done_evt.wait(timeout=30.0)
        cmd_pub.publish(Twist())
        self.destroy_subscription(odom_sub)

        if reached:
            self.get_logger().info('Startup escape complete — ready for tasks')
        else:
            self.get_logger().warn('Startup escape timed out — proceeding anyway')

        self._ready = True
        self._set_status('[IDLE] Ready — awaiting orders')

    # ── Mission ────────────────────────────────────────────────────────────────

    def _run_mission(self, task: dict):
        requested_products = self._extract_requested_products(task)
        if not requested_products:
            self._set_status('[ERROR] No product_id found in parsed request')
            self._transition(IDLE)
            return

        pending = requested_products[:]
        picked_count = 0

        while pending:
            self._transition(RESOLVING)
            product_id, record = self._choose_next_product(pending)
            pending.remove(product_id)

            if record is None:
                self._set_status(f'[ERROR] {product_id} not found in inventory, skipping')
                continue

            shelf_id = record['shelf_id']
            slot_index = int(record['slot_index'])
            aruco_id = int(record['aruco_id'])
            remaining = len(pending)
            self._set_status(
                f'[RESOLVING] {product_id} → {shelf_id} slot {slot_index} (ArUco {aruco_id}), '
                f'{remaining} item(s) left')

            # NAVIGATING TO SHELF
            self._transition(NAVIGATING)
            sx, sy, syaw = get_approach_pose(shelf_id)
            self._set_status(f'[NAVIGATING] Heading to {shelf_id} entry ({sx:.2f}, {sy:.2f})')

            tag_xy = None
            nav_result, detected_xy = 'failed', None
            for attempt in range(1, self._MAX_RETRIES + 1):
                nav_result, detected_xy = self._go_to_entry_with_tag_preemption(
                    sx, sy, syaw, aruco_id)
                if nav_result != 'failed':
                    break
                if attempt < self._MAX_RETRIES:
                    self._set_status(
                        f'[NAVIGATING] Retry {attempt + 1}/{self._MAX_RETRIES} to reach {shelf_id} entry')
            if nav_result == 'detected' and detected_xy is not None:
                tag_xy = detected_xy
                self._transition(SCANNING)
                self._set_status(
                    f'[SCANNING] Target ArUco {aruco_id} seen during approach — skipping full aisle sweep')
            elif nav_result != 'arrived':
                self._set_status(f'[ERROR] Navigation to {shelf_id} failed, skipping {product_id}')
                continue

            # SCANNING — sweep aisle waypoints and watch for the target ArUco tag
            if tag_xy is None:
                self._transition(SCANNING)
                self._set_status(f'[SCANNING] Sweeping {shelf_id} for ArUco {aruco_id}')
                tag_xy = self._scan_for_tag(shelf_id, aruco_id)
                if tag_xy is None:
                    self._register_missing_product(
                        product_id=product_id,
                        aruco_id=aruco_id,
                        expected_shelf=shelf_id,
                        expected_slot=slot_index,
                        reason='missing_in_expected_aisle',
                    )
            pick_x, pick_y, syaw = self._safe_pick_pose(shelf_id, slot_index)
            pick_yaw = float(syaw)
            if tag_xy:
                self._set_status(
                    f'[SCANNING] Tag found — staging at safe pick stand-off '
                    f'({pick_x:.2f}, {pick_y:.2f})')
            else:
                self._set_status(
                    f'[SCANNING] Tag not found — using DB slot position '
                    f'({pick_x:.2f}, {pick_y:.2f})')

            # PICKING
            self._transition(PICKING)
            self._set_status(f'[PICKING] Navigating to safe pick stand-off')
            if not self._go_to_with_retry(
                pick_x, pick_y, pick_yaw,
                label='[PICKING] stand-off',
                retries=self._MAX_RETRIES,
            ):
                self._set_status(f'[ERROR] Could not reach pick stand-off for {product_id}, skipping')
                continue

            if not self._confirm_pick_alignment(aruco_id):
                self._set_status(
                    f'[PICKING] Tag {aruco_id} not centered at stand-off, rescanning {shelf_id}')
                tag_xy = self._scan_for_tag(shelf_id, aruco_id)
                if tag_xy is None or not self._go_to_with_retry(
                    pick_x, pick_y, pick_yaw,
                    label='[PICKING] stand-off realign',
                    retries=2,
                ) or not self._confirm_pick_alignment(aruco_id):
                    self._register_missing_product(
                        product_id=product_id,
                        aruco_id=aruco_id,
                        expected_shelf=shelf_id,
                        expected_slot=slot_index,
                        reason='unable_to_confirm_alignment',
                    )
                    self._set_status(
                        f'[ERROR] Could not confirm aligned view of ArUco {aruco_id}, skipping {product_id}')
                    continue

            publish_json(self._pick_pub, {
                'product_id': product_id,
                'shelf_id':   shelf_id,
                'slot_index': slot_index,
                'pick_x':     pick_x,
                'pick_y':     pick_y,
            })
            picked_count += 1
            self._set_status(f'[PICKED] {product_id} from {shelf_id}-slot{slot_index}')

        if picked_count == 0:
            self._set_status('[IDLE] No items picked — mission complete')
            self._transition(IDLE)
            return

        # DISPATCHING (once after full pick list)
        self._transition(DISPATCHING)
        dx, dy = self._select_dispatch_zone()
        self._set_status(f'[DISPATCHING] En route to dispatch zone ({dx:.2f}, {dy:.2f})')
        self._go_to_with_retry(
            dx, dy, 0.0,
            label='[DISPATCHING] dispatch zone',
            retries=self._MAX_RETRIES,
        )

        self._set_status('[IDLE] Mission complete — awaiting next order')
        self._transition(IDLE)

    # ── Scan logic ─────────────────────────────────────────────────────────────

    def _go_to_entry_with_tag_preemption(self, x: float, y: float, yaw: float,
                                         target_aruco_id: int,
                                         timeout: float = 120.0,
                                         poll_period: float = 0.1):
        """Navigate to shelf entry, but preempt once target ArUco is visible.

        Returns ('arrived', None), ('detected', (rx, ry)) or ('failed', None).
        """
        goal = make_pose_stamped(self._nav.get_clock(), x, y, yaw)
        self._nav.goToPose(goal)

        t0 = time.time()
        consecutive_hits = 0
        while not self._nav.isTaskComplete():
            with self._det_lock:
                ids_seen = []
                for d in self._detections:
                    try:
                        ids_seen.append(int(d.get('aruco_id')))
                    except (TypeError, ValueError):
                        continue

            if any(marker_id == target_aruco_id for marker_id in ids_seen):
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            if consecutive_hits >= self._PREEMPT_CONSECUTIVE_DETECTIONS:
                self._nav.cancelTask()
                rx, ry = self._robot_xy()
                if rx is None:
                    rx, ry = x, y
                self.get_logger().info(
                    f'[SCAN] ArUco {target_aruco_id} confirmed '
                    f'{self._PREEMPT_CONSECUTIVE_DETECTIONS}x during approach at ({rx:.2f}, {ry:.2f})')
                return 'detected', (rx, ry)

            if ids_seen:
                now_t = time.time()
                if now_t - self._last_scan_miss_log_t > 1.0:
                    self.get_logger().info(
                        f'[SCAN] Seen tags {ids_seen}, target ArUco {target_aruco_id} '
                        f'consecutive_hits={consecutive_hits}/{self._PREEMPT_CONSECUTIVE_DETECTIONS}')
                    self._last_scan_miss_log_t = now_t

            if time.time() - t0 > timeout:
                self._nav.cancelTask()
                return 'failed', None
            time.sleep(poll_period)

        if self._nav.getResult() == TaskResult.SUCCEEDED:
            return 'arrived', None
        return 'failed', None

    def _scan_for_tag(self, shelf_id: str, aruco_id: int):
        """
        Drive waypoints along the shelf aisle.
        Returns (robot_x, robot_y) when the target ArUco is first spotted,
        or None if the tag is not seen during the sweep.
        """
        shelf = SHELVES[shelf_id]
        _, entry_y, entry_yaw = get_approach_pose(shelf_id)
        cx = shelf['cx']
        xs = [
            cx + SLOT_X_OFFSETS[1] - 0.35,
            cx + SLOT_X_OFFSETS[1],
            cx + SLOT_X_OFFSETS[2],
            cx + SLOT_X_OFFSETS[3],
            cx + SLOT_X_OFFSETS[3] + 0.35,
        ]
        rx, _ = self._robot_xy()
        scan_xs = xs
        if rx is not None:
            # Continue from the robot's current aisle progress and avoid re-covering
            # waypoints already crossed toward the shelf scan direction.
            progress_eps = 0.12
            remaining = [x for x in xs if x >= (rx - progress_eps)]
            if remaining:
                scan_xs = remaining
            else:
                scan_xs = [xs[-1]]

        poses = [
            make_pose_stamped(self._nav.get_clock(), x, entry_y, entry_yaw)
            for x in scan_xs
        ]
        self._nav.goThroughPoses(poses)

        while not self._nav.isTaskComplete():
            with self._det_lock:
                ids_seen = []
                for d in self._detections:
                    try:
                        ids_seen.append(int(d.get('aruco_id')))
                    except (TypeError, ValueError):
                        continue
                visible = any(marker_id == aruco_id for marker_id in ids_seen)
            if visible:
                rx, ry = self._robot_xy()
                self._nav.cancelTask()
                if rx is None:
                    rx, ry = xs[0], entry_y
                self.get_logger().info(
                    f'[SCAN] ArUco {aruco_id} spotted at ({rx:.2f}, {ry:.2f})')
                return rx, ry
            if ids_seen:
                now_t = time.time()
                if now_t - self._last_scan_miss_log_t > 1.0:
                    self.get_logger().info(
                        f'[SCAN] Seen tags {ids_seen}, still waiting for target ArUco {aruco_id}')
                    self._last_scan_miss_log_t = now_t
            time.sleep(0.1)

        return None

    def _safe_pick_pose(self, shelf_id: str, slot_index: int):
        """Return a pick staging pose with extra stand-off for turning room."""
        px, py, pyaw = get_approach_pose(shelf_id, slot_index)
        row = SHELVES[shelf_id]['row']
        if row == 'A':
            py -= self._PICK_STANDOFF_EXTRA
        else:
            py += self._PICK_STANDOFF_EXTRA
        return px, py, pyaw

    def _robot_xy(self):
        """Return current robot (x, y) in map frame, or (None, None) on failure."""
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_link', Time(),
                timeout=Duration(seconds=1))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None

    def _select_dispatch_zone(self):
        """Pick nearest configured bottom-left dispatch zone."""
        rx, ry = self._robot_xy()
        if rx is None or ry is None:
            return self._DISPATCH_ZONES[0]

        return min(
            self._DISPATCH_ZONES,
            key=lambda d: (d[0] - rx) ** 2 + (d[1] - ry) ** 2,
        )

    def _go_to_with_retry(self, x: float, y: float, yaw: float,
                          label: str, retries: int) -> bool:
        for attempt in range(1, retries + 1):
            if go_to_xyyaw(self._nav, x, y, yaw):
                return True
            if attempt < retries:
                self._set_status(f'{label} failed, retry {attempt + 1}/{retries}')
        return False

    def _confirm_pick_alignment(self, target_aruco_id: int) -> bool:
        """Require stable, centered target visibility before simulating a pick."""
        deadline = time.time() + self._PICK_CONFIRM_TIMEOUT_SEC
        consecutive_hits = 0

        while time.time() < deadline:
            with self._det_lock:
                target_det = None
                for det in self._detections:
                    try:
                        if int(det.get('aruco_id')) == target_aruco_id:
                            target_det = det
                            break
                    except (TypeError, ValueError):
                        continue

            if target_det is not None:
                tvec = target_det.get('tvec', [0.0, 0.0, 0.0])
                try:
                    tx = float(tvec[0]) if len(tvec) >= 1 else 0.0
                    dist = float(target_det.get('distance_m', 999.0))
                    centered = abs(tx) <= self._PICK_CONFIRM_CENTER_TVEC_X_M
                    close_enough = dist <= self._PICK_CONFIRM_MAX_DISTANCE_M
                    if centered and close_enough:
                        consecutive_hits += 1
                    else:
                        consecutive_hits = 0
                except (TypeError, ValueError):
                    consecutive_hits = 0
            else:
                consecutive_hits = 0

            if consecutive_hits >= self._PICK_CONFIRM_CONSECUTIVE_DETECTIONS:
                return True
            time.sleep(0.08)

        return False

    def _register_missing_product(self, product_id: str, aruco_id: int,
                                  expected_shelf: str, expected_slot: int,
                                  reason: str):
        """Store unresolved mismatch context for later cross-aisle reconciliation."""
        first_seen = False
        with self._missing_lock:
            rec = self._missing_products.get(aruco_id)
            if rec is None:
                self._missing_products[aruco_id] = {
                    'product_id': product_id,
                    'expected_shelf': expected_shelf,
                    'expected_slot': int(expected_slot),
                    'reason': reason,
                    'first_missing_ts': time.time(),
                    'last_reconcile_attempt_ts': 0.0,
                }
                first_seen = True

        if not first_seen:
            return

        self._record_inventory_verification(
            product_id=product_id,
            shelf_id=expected_shelf,
            slot_index=int(expected_slot),
            aruco_id=aruco_id,
            outcome='not_found',
        )
        self._emit_operator_alert(
            f'Missing product noted: {product_id} (ArUco {aruco_id}) not found at '
            f'{expected_shelf}-slot{expected_slot}. Will continue mission and auto-reconcile if detected elsewhere.'
        )

    def _reconcile_missing_from_detections(self, detections: list[dict]):
        """When a previously missing SKU is seen, update its inventory location."""
        if self._state == IDLE:
            return

        with self._missing_lock:
            if not self._missing_products:
                return
            missing_snapshot = {
                aid: dict(rec) for aid, rec in self._missing_products.items()
            }

        inferred = self._infer_detected_location()
        if inferred is None:
            return
        found_shelf, found_slot = inferred

        now = time.time()
        for det in detections:
            try:
                aid = int(det.get('aruco_id'))
            except (TypeError, ValueError):
                continue
            if aid not in missing_snapshot:
                continue

            with self._missing_lock:
                live = self._missing_products.get(aid)
                if live is None:
                    continue
                if now - float(live.get('last_reconcile_attempt_ts', 0.0)) < 2.0:
                    continue
                live['last_reconcile_attempt_ts'] = now

            threading.Thread(
                target=self._resolve_missing_product,
                args=(aid, found_shelf, int(found_slot)),
                daemon=True,
            ).start()

    def _resolve_missing_product(self, aruco_id: int, shelf_id: str, slot_index: int):
        with self._missing_lock:
            rec = self._missing_products.get(aruco_id)
            if rec is None:
                return
            product_id = rec['product_id']
            expected_shelf = rec['expected_shelf']
            expected_slot = int(rec['expected_slot'])

        moved = (shelf_id != expected_shelf) or (int(slot_index) != expected_slot)
        self._record_inventory_verification(
            product_id=product_id,
            shelf_id=shelf_id,
            slot_index=int(slot_index),
            aruco_id=aruco_id,
            outcome='mismatch' if moved else 'match',
        )

        update = self._rpc.request(
            self._inv_req_pub,
            {
                'type': 'update_location',
                'params': {
                    'product_id': product_id,
                    'shelf_id': shelf_id,
                    'slot_index': int(slot_index),
                },
            },
            timeout=6.0,
            label=f'update location {product_id}',
        )

        if not update or not update.get('ok'):
            return

        with self._missing_lock:
            self._missing_products.pop(aruco_id, None)

        if moved:
            self._emit_operator_alert(
                f'Inventory auto-updated: {product_id} (ArUco {aruco_id}) moved '
                f'from {expected_shelf}-slot{expected_slot} to {shelf_id}-slot{slot_index}.'
            )
        else:
            self._emit_operator_alert(
                f'Inventory confirmed: {product_id} (ArUco {aruco_id}) re-detected at '
                f'{shelf_id}-slot{slot_index}.'
            )

    def _infer_detected_location(self):
        """Infer shelf/slot from current robot pose while a tag is visible."""
        rx, ry = self._robot_xy()
        if rx is None or ry is None:
            return None

        best_shelf = None
        best_metric = float('inf')
        best_y_err = float('inf')
        for shelf_id, s in SHELVES.items():
            _, aisle_y, _ = get_approach_pose(shelf_id)
            y_err = abs(ry - aisle_y)
            metric = y_err + 0.25 * abs(rx - s['cx'])
            if metric < best_metric:
                best_metric = metric
                best_shelf = shelf_id
                best_y_err = y_err

        if best_shelf is None or best_y_err > 1.3:
            return None

        shelf = SHELVES[best_shelf]
        slot_index = min(
            SLOT_X_OFFSETS.keys(),
            key=lambda slot: abs((shelf['cx'] + SLOT_X_OFFSETS[slot]) - rx),
        )
        x_err = abs((shelf['cx'] + SLOT_X_OFFSETS[slot_index]) - rx)
        if x_err > 1.5:
            return None

        return best_shelf, int(slot_index)

    def _record_inventory_verification(self, product_id: str, shelf_id: str,
                                       slot_index: int, aruco_id: int,
                                       outcome: str):
        rx, ry = self._robot_xy()
        if rx is None:
            rx, ry = 0.0, 0.0

        self._rpc.request(
            self._inv_req_pub,
            {
                'type': 'record_verification',
                'params': {
                    'product_id': product_id,
                    'shelf_scanned': shelf_id,
                    'slot_scanned': int(slot_index),
                    'aruco_detected': int(aruco_id),
                    'outcome': outcome,
                    'robot_pose_x': float(rx),
                    'robot_pose_y': float(ry),
                },
            },
            timeout=5.0,
            label=f'record verification {product_id}',
        )

    def _emit_operator_alert(self, text: str):
        self._alert_pub.publish(String(data=text))
        self.get_logger().warn(text)

    def _lookup_product(self, product_id: str):
        inv_data = None
        for attempt in range(1, self._MAX_RETRIES + 1):
            inv_data = self._rpc.request(
                self._inv_req_pub,
                {'type': 'lookup_product', 'params': {'product_id': product_id}},
                timeout=10.0, label=f'inventory lookup {product_id}')
            if inv_data and inv_data.get('ok'):
                return inv_data['data']
            if attempt < self._MAX_RETRIES:
                self._set_status(
                    f'[RESOLVING] Lookup retry {attempt + 1}/{self._MAX_RETRIES} for {product_id}')
        return None

    def _choose_next_product(self, pending_products: list[str]):
        records = []
        for pid in pending_products:
            record = self._lookup_product(pid)
            if record is not None:
                records.append((pid, record))

        if not records:
            # Return the first pending product to consume it and avoid deadlocks.
            return pending_products[0], None

        rx, ry = self._robot_xy()
        if rx is None or ry is None:
            return records[0]

        def _entry_distance(item):
            _, rec = item
            ex, ey, _ = get_approach_pose(rec['shelf_id'])
            return (ex - rx) ** 2 + (ey - ry) ** 2

        return min(records, key=_entry_distance)

    @staticmethod
    def _extract_requested_products(task: dict) -> list[str]:
        ordered = []

        pid = task.get('product_id')
        if isinstance(pid, str) and pid.strip():
            ordered.append(pid.strip().upper())

        pids = task.get('product_ids')
        if isinstance(pids, list):
            for item in pids:
                if isinstance(item, str) and item.strip():
                    ordered.append(item.strip().upper())

        # De-duplicate while preserving request order.
        seen = set()
        out = []
        for item in ordered:
            if item in seen:
                continue
            seen.add(item)
            out.append(item)
        return out

    # ── Utilities ──────────────────────────────────────────────────────────────

    def _transition(self, state: int):
        self._state = state
        self.get_logger().info(f'State → {STATE_NAMES[state]}')

    def _set_status(self, text: str):
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    # MultiThreadedExecutor keeps the node callbacks running while the
    # background mission thread drives BasicNavigator via the global executor.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
