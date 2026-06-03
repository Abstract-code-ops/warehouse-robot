#!/usr/bin/env python3
"""
planner_node.py

Topics subscribed:
  /semantic_goal       std_msgs/String  — single location name
  /semantic_task_list  std_msgs/String  — comma-separated location names
  /cancel_tasks        std_msgs/String  — any message clears queue + cancels nav

"""

import os
import time
import yaml
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from ament_index_python.packages import get_package_share_directory

from cognitive_amr.task_manager import TaskManager


class SemanticPlanner(Node):
    """
    ROS 2 node: converts semantic location names → Nav2 NavigateToPose goals.

    Supports a FIFO task queue — goals are executed in the order they were
    received. New goals enqueue while the robot is moving; the queue drains
    automatically as each goal completes.
    """

    def __init__(self):
        super().__init__('cognitive_amr_planner')

        # ── Load location database ─────────────────────────────────────────
        config_path = os.path.join(
            get_package_share_directory('cognitive_amr'),
            'config',
            'locations.yaml'
        )
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        self.locations: dict = data['locations']
        self.frame_id: str   = data['metadata']['frame_id']

        self.get_logger().info(
            f"Loaded {len(self.locations)} locations from {config_path}"
        )

        # ── TaskManager ────────────────────────────────────────────────────
        self.task_manager = TaskManager()

        # ── Task queue ─────────────────────────────────────────────────────
        # deque stores task dicts waiting to be executed.
        # _executing is True while Nav2 is actively working on a goal.
        # _goal_handle keeps a reference to the active Nav2 goal so we can
        # cancel it when /cancel_tasks is received.
        self._queue: deque        = deque()
        self._executing: bool     = False
        self._active_task: dict   = {}
        self._goal_handle         = None
        self._dispatch_time: float = 0.0

        # After a successful goal, the robot waits this many seconds
        # I will simulate the robot picking a product, maybe spawn a small item in Gazebo,
        # or just give the operator a moment to see the result before
        # moving to the next task (simulates picking/placing an item).

        # Override the time at launch: --ros-args -p dwell_time:=5.0
        self.declare_parameter('dwell_time', 3.0)
        self._dwell_time: float = (
            self.get_parameter('dwell_time').get_parameter_value().double_value
        )
        self._dwell_timer = None   # holds the active one-shot timer, if any
        self.get_logger().info(f"Dwell time between tasks: {self._dwell_time:.1f}s")

        # ── Nav2 Action Client ─────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Subscribers ────────────────────────────────────────────────────

        # Single location name
        self.create_subscription(String, '/semantic_goal',
                                 self._semantic_goal_callback, 10)

        # Comma-separated list: "shelf_A1, dispatch_zone, home"
        self.create_subscription(String, '/semantic_task_list',
                                 self._semantic_task_list_callback, 10)

        # Any message on this topic clears the queue and cancels current nav, 
        # maybe I will add a specific stop command
        self.create_subscription(String, '/cancel_tasks',
                                 self._cancel_callback, 10)

        # AMCL pose — forwarded to TaskManager for proximity planning
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_pose_callback, 10)

        self.get_logger().info(
            "SemanticPlanner ready.\n"
            "  /semantic_goal      — single location\n"
            "  /semantic_task_list — comma-separated list\n"
            "  /cancel_tasks       — clear queue and stop"
        )

    # ── Subscriber callbacks ───────────────────────────────────────────────

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.task_manager.update_pose(x, y)

    def _semantic_goal_callback(self, msg: String) -> None:
        """Single location name → validate and enqueue."""
        name = msg.data.strip()
        task = self._build_task(name)
        if task is None:
            return
        self._queue.append(task)
        self.get_logger().info(
            f"Queued '{name}' — queue length: {len(self._queue)}"
        )
        self._execute_next()

    def _semantic_task_list_callback(self, msg: String) -> None:
        """
        Comma-separated location names → validate all, then enqueue in order.
        The entire batch is rejected if any name is unknown, so the robot
        never starts a list that it cannot complete.

        Example message:  "storage_shelf_A1, dispatch_zone, charging_dock_1"
        """
        names = [n.strip() for n in msg.data.split(',') if n.strip()]

        if not names:
            self.get_logger().warn("Received empty task list — ignoring.")
            return

        # Validate all names before enqueuing any
        unknown = [n for n in names if n not in self.locations]
        if unknown:
            self.get_logger().error(
                f"Unknown location(s) in task list: {unknown}. "
                f"Entire list rejected. Valid: {list(self.locations.keys())}"
            )
            return

        # Build task dicts and run through TaskManager pipeline
        tasks = [self._build_task(n) for n in names]
        tasks = self.task_manager.prioritize(tasks)
        tasks = self.task_manager.nearest_first(tasks)

        for task in tasks:
            self._queue.append(task)

        self.get_logger().info(
            f"Queued {len(tasks)} tasks: {[t['name'] for t in tasks]} "
            f"— total queue length: {len(self._queue)}"
        )
        self._execute_next()

    def _cancel_callback(self, msg: String) -> None:
        """
        Clear the entire queue and cancel the active Nav2 goal immediately.
        The robot will stop after finishing its current motion primitive.
        """
        cleared = len(self._queue)
        self._queue.clear()
        self.get_logger().warn(
            f"Cancel received — cleared {cleared} queued task(s)."
        )
        # Also cancel any pending dwell timer so the robot doesn't
        # silently resume after the dwell period expires
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer.destroy()
            self._dwell_timer = None
            self.get_logger().warn("Dwell timer cancelled.")
        # Cancel the goal currently being executed in Nav2
        if self._executing and self._goal_handle is not None:
            self.get_logger().warn(
                f"Cancelling active goal '{self._active_task.get('name')}'."
            )
            self._goal_handle.cancel_goal_async()

    # ── Queue helpers ──────────────────────────────────────────────────────

    def _build_task(self, name: str) -> dict | None:
        """Look up a location name and return a task dict, or None if unknown."""
        if name not in self.locations:
            self.get_logger().error(
                f"Unknown location: '{name}'. "
                f"Valid locations: {list(self.locations.keys())}"
            )
            return None
        loc = self.locations[name]
        return {
            'name':     name,
            'priority': 1,
            'x':        loc['x'],
            'y':        loc['y'],
            'z_orient': loc['z_orient'],
            'w_orient': loc['w_orient'],
        }

    def _execute_next(self) -> None:
        """
        Pop the next task from the queue and send it to Nav2.
        Does nothing if the robot is already executing a goal or the queue
        is empty. Called after every enqueue and after every result callback.
        """
        if self._executing:
            return  # result callback will call us again when current goal ends

        if not self._queue:
            self.get_logger().info("Queue empty — robot idle.")
            return

        # Battery check: inject charging task at front if needed
        if self.task_manager.needs_charging():
            charging_loc = self.locations.get('charging_dock_1')
            if charging_loc:
                charge_task = self._build_task('charging_dock_1')
                self._queue.appendleft(charge_task)
                self.get_logger().warn(
                    "Battery low — inserting charging_dock_1 at front of queue."
                )

        task = self._queue.popleft()

        if not self.task_manager.check_traffic(task):
            self.get_logger().warn(
                f"Path to '{task['name']}' appears blocked — skipping."
            )
            self._execute_next()   # try the next task instead
            return

        self._send_nav_goal(task)

    # ── Nav2 dispatch ──────────────────────────────────────────────────────

    def _send_nav_goal(self, task: dict) -> None:
        """Build a NavigateToPose goal and send it asynchronously to Nav2."""

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "NavigateToPose action server not available after 5s. "
                "Is Nav2 running? Re-queueing task."
            )
            self._queue.appendleft(task)   # put it back so it isn't lost
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        # sec=0 / nanosec=0 → bypass tf2 extrapolation / sim-time sync errors
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0
        goal_msg.pose.pose.position.x    = task['x']
        goal_msg.pose.pose.position.y    = task['y']
        goal_msg.pose.pose.position.z    = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = task['z_orient']
        goal_msg.pose.pose.orientation.w = task['w_orient']

        self._executing    = True
        self._active_task  = task
        self._goal_handle  = None
        self._dispatch_time = time.monotonic()

        remaining = len(self._queue)
        self.get_logger().info(
            f"[{remaining} in queue] Dispatching → '{task['name']}' "
            f"at map ({task['x']}, {task['y']})"
        )

        future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    # ── Action callbacks ───────────────────────────────────────────────────

    def _goal_response_callback(self, future) -> None:
        """Nav2 accepted or rejected the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal '{self._active_task.get('name')}' REJECTED by Nav2. "
                "Check Nav2 is fully initialised and the pose is reachable."
            )
            self._executing = False
            self._execute_next()   # try the next task in the queue
            return

        # Store handle so _cancel_callback can cancel this goal mid-flight
        self._goal_handle = goal_handle
        self.get_logger().info(
            f"Goal '{self._active_task.get('name')}' accepted by Nav2."
        )
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        """Fires ~1 Hz while the robot is navigating."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  → '{self._active_task.get('name')}' | "
            f"distance remaining: {fb.distance_remaining:.2f} m | "
            f"{len(self._queue)} task(s) still in queue"
        )

    def _result_callback(self, future) -> None:
        """
        Fires when Nav2 finishes (success, abort, or cancel).
        On success: wait dwell_time seconds (simulating item pickup), then
        pull the next task. On failure: proceed immediately.
        """
        status   = future.result().status
        duration = time.monotonic() - self._dispatch_time
        name     = self._active_task.get('name')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"SUCCEEDED '{name}' in {duration:.1f}s. "
                f"Waiting {self._dwell_time:.1f}s before next task..."
            )
            self.task_manager.record_result(self._active_task, True, duration)
            self._executing   = False
            self._goal_handle = None
            # One-shot timer: fires once after dwell_time, then cancels itself
            self._dwell_timer = self.create_timer(
                self._dwell_time, self._dwell_expired
            )

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f"ABORTED '{name}' after {duration:.1f}s. "
                "Path may be blocked or goal unreachable."
            )
            self.task_manager.record_result(self._active_task, False, duration)
            self._executing   = False
            self._goal_handle = None
            self._execute_next()

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"CANCELED '{name}' after {duration:.1f}s.")
            self.task_manager.record_result(self._active_task, False, duration)
            self._executing   = False
            self._goal_handle = None
            # Queue was already cleared by _cancel_callback; nothing to do

        else:
            self.get_logger().warn(
                f"'{name}' ended with unexpected status {status}."
            )
            self._executing   = False
            self._goal_handle = None
            self._execute_next()

    def _dwell_expired(self) -> None:
        """Called once after dwell_time seconds. Cancels itself, then starts next task."""
        # Destroy the timer so it doesn't repeat
        self._dwell_timer.cancel()
        self._dwell_timer.destroy()
        self._dwell_timer = None
        self.get_logger().info("Dwell complete — moving to next task.")
        self._execute_next()


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    node = SemanticPlanner()

    try:
        # spin() keeps the node alive, processing callbacks as they arrive
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
