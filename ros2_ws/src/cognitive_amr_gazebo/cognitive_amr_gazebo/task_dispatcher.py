#!/usr/bin/env python3
"""
Task dispatcher node for cognitive AMR demo.
Loads waypoints.yaml and tasks.yaml, scores tasks using priority/distance/
battery, and sends NavigateToPose goals to Nav2 one at a time.

Publishes:
  /task_status   (std_msgs/String)  — JSON string for Foxglove table panel
  /task_markers  (visualization_msgs/MarkerArray) — waypoint labels in 3D view
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import yaml
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# ── scoring weights (tune for demo) ─────────────────────────────────────────
W_PRIORITY   = 10.0   # per priority level (1=highest gets most points)
W_DISTANCE   = -0.5   # per metre — penalise far tasks
W_BATTERY    = -5.0   # per battery % below 50 when task requires travel > 5 m
BATTERY_SOFT = 0.30   # insert charge task below this level
BATTERY_HARD = 0.18   # refuse new task below this level


class TaskDispatcher(Node):

    def __init__(self):
        super().__init__('task_dispatcher')

        # ── parameters ──────────────────────────────────────────────────────
        pkg = get_package_share_directory('cognitive_amr_gazebo')
        self.declare_parameter('waypoints_file',
                               os.path.join(pkg, 'config', 'waypoints.yaml'))
        self.declare_parameter('tasks_file',
                               os.path.join(pkg, 'config', 'tasks.yaml'))
        self.declare_parameter('battery_topic', '/battery_state')

        wp_file = self.get_parameter('waypoints_file').value
        tk_file = self.get_parameter('tasks_file').value

        # ── load config ─────────────────────────────────────────────────────
        with open(wp_file) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']
        with open(tk_file) as f:
            self.task_queue = yaml.safe_load(f)['tasks']

        # Track which tasks are pending / done
        for t in self.task_queue:
            t.setdefault('status', 'pending')

        # ── state ────────────────────────────────────────────────────────────
        self.battery_pct = 1.0          # 0.0–1.0; update from real topic
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.active_task = None
        self.demo_start_time = time.time()

        # ── publishers ───────────────────────────────────────────────────────
        self.status_pub  = self.create_publisher(String,      '/task_status',  10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/task_markers', 10)

        # ── Nav2 action client ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── timer: evaluate queue every 2 s ──────────────────────────────────
        self.create_timer(2.0, self._dispatch_loop)
        self.create_timer(5.0, self._publish_markers)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('TaskDispatcher ready — waiting for Nav2 action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 connected. Demo starts now.')

    # ── scoring ───────────────────────────────────────────────────────────────
    def _score_task(self, task):
        if task['status'] != 'pending':
            return -9999.0

        # Priority: level 1 → highest score
        priority_score = W_PRIORITY * (4 - task['priority'])

        # Distance to source waypoint
        src = self.waypoints.get(task['source'])
        if src is None:
            return -9999.0
        dist = math.hypot(src['x'] - self.robot_x, src['y'] - self.robot_y)
        distance_score = W_DISTANCE * dist

        # Battery penalty for heavy / long missions
        battery_score = 0.0
        if dist > 5.0 and self.battery_pct < 0.50:
            shortage = 0.50 - self.battery_pct
            battery_score = W_BATTERY * shortage * 100

        # Deadline urgency bonus
        elapsed = time.time() - self.demo_start_time
        deadline = task.get('deadline_s', 9999)
        time_left = max(deadline - elapsed, 1)
        urgency = 500.0 / time_left   # bigger bonus as deadline approaches

        return priority_score + distance_score + battery_score + urgency

    def _best_task(self):
        if self.battery_pct <= BATTERY_HARD:
            self.get_logger().warn('Battery critical — only charge tasks allowed.')
            charges = [t for t in self.task_queue
                       if t['type'] == 'charge' and t['status'] == 'pending']
            return charges[0] if charges else None

        if self.battery_pct <= BATTERY_SOFT:
            self.get_logger().info('Battery low — inserting charge task.')
            charges = [t for t in self.task_queue
                       if t['type'] == 'charge' and t['status'] == 'pending']
            if charges:
                return charges[0]

        scored = [(self._score_task(t), t) for t in self.task_queue]
        scored.sort(key=lambda x: x[0], reverse=True)
        best_score, best_task = scored[0]

        if best_score <= -9999.0:
            return None  # all done

        self.get_logger().info(
            f'Selected task {best_task["id"]} (score={best_score:.1f}): '
            f'{best_task["source"]} → {best_task["destination"]}'
        )
        return best_task

    # ── dispatch loop ─────────────────────────────────────────────────────────
    def _dispatch_loop(self):
        if self.active_task is not None:
            return  # wait for current task to finish

        task = self._best_task()
        if task is None:
            self.get_logger().info('All tasks complete.')
            return

        self._send_nav_goal(task)

    def _send_nav_goal(self, task):
        wp_name = task['source']
        wp = self.waypoints.get(wp_name)
        if wp is None:
            self.get_logger().error(f'Unknown waypoint: {wp_name}')
            task['status'] = 'error'
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        # Convert yaw to quaternion (z-axis rotation only)
        yaw = float(wp.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        task['status'] = 'active'
        self.active_task = task

        self.get_logger().info(f'Navigating to {wp_name} for task {task["id"]}')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal.')
            if self.active_task:
                self.active_task['status'] = 'failed'
                self.active_task = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        task = self.active_task
        if task is None:
            return
        result = future.result()
        # NavigateToPose result status 4 = SUCCEEDED
        if result.status == 4:
            self.get_logger().info(
                f'Task {task["id"]} source reached — '
                f'simulating pick at {task["source"]}'
            )
            task['status'] = 'picked'
            # Now navigate to destination
            self._send_delivery_goal(task)
        else:
            self.get_logger().warn(f'Task {task["id"]} navigation failed (status={result.status})')
            task['status'] = 'failed'
            self.active_task = None

    def _send_delivery_goal(self, task):
        wp_name = task['destination']
        wp = self.waypoints.get(wp_name)
        if wp is None:
            self.get_logger().error(f'Unknown destination waypoint: {wp_name}')
            task['status'] = 'error'
            self.active_task = None
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Delivering to {wp_name} for task {task["id"]}')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._delivery_response_cb)

    def _delivery_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected delivery goal.')
            if self.active_task:
                self.active_task['status'] = 'failed'
                self.active_task = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._delivery_result_cb)

    def _delivery_result_cb(self, future):
        task = self.active_task
        if task is None:
            return
        result = future.result()
        if result.status == 4:
            self.get_logger().info(f'Task {task["id"]} COMPLETE: {task["sku"]} delivered.')
            task['status'] = 'complete'
        else:
            self.get_logger().warn(f'Task {task["id"]} delivery failed.')
            task['status'] = 'failed'
        self.active_task = None

    # ── publishers ────────────────────────────────────────────────────────────
    def _publish_status(self):
        elapsed = time.time() - self.demo_start_time
        summary = {
            'elapsed_s':   round(elapsed, 1),
            'battery_pct': round(self.battery_pct * 100, 1),
            'active':      self.active_task['id'] if self.active_task else None,
            'pending':     sum(1 for t in self.task_queue if t['status'] == 'pending'),
            'complete':    sum(1 for t in self.task_queue if t['status'] == 'complete'),
            'failed':      sum(1 for t in self.task_queue if t['status'] == 'failed'),
            'queue': [
                {
                    'id':       t['id'],
                    'sku':      t['sku'],
                    'priority': t['priority'],
                    'status':   t['status'],
                    'deadline': t.get('deadline_s', 0),
                    'notes':    t.get('notes', ''),
                }
                for t in self.task_queue
            ]
        }
        msg = String()
        msg.data = json.dumps(summary)
        self.status_pub.publish(msg)

    def _publish_markers(self):
        """Publish waypoint labels as text markers visible in Foxglove 3D view."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (name, wp) in enumerate(self.waypoints.items()):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(wp['x'])
            m.pose.position.y = float(wp['y'])
            m.pose.position.z = 1.8   # float above the floor
            m.pose.orientation.w = 1.0
            m.scale.z = 0.25          # text height in metres
            m.text = name
            # Colour by zone
            zone = wp.get('zone', 'storage')
            if zone == 'charging':
                m.color.r, m.color.g, m.color.b = 1.0, 0.85, 0.0
            elif zone == 'packing':
                m.color.r, m.color.g, m.color.b = 0.1, 0.5, 1.0
            elif zone == 'dispatch':
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.2
            elif zone == 'priority':
                m.color.r, m.color.g, m.color.b = 1.0, 0.1, 0.1
            else:
                m.color.r, m.color.g, m.color.b = 0.9, 0.9, 0.9
            m.color.a = 1.0
            m.lifetime = Duration(seconds=6).to_msg()
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
