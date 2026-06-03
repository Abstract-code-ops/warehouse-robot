"""
task_manager.py
────────────────────────────────────────────────────────────────────────────
Handles all planning logic: deciding WHAT the robot should do next and in
what order. Intentionally decoupled from ROS 2 communication — the planner
node handles talking to Nav2, this class handles thinking.

Current state: all methods are stubs that pass through or return safe
defaults. Each stub is marked with # TODO and describes exactly what it
should do when implemented.

A "task" throughout this file is a plain Python dict with these keys:
    {
        "name":     str,    # semantic location name, e.g. "storage_shelf_A3"
        "priority": int,    # 1 = highest urgency, higher number = lower
        "x":        float,  # map-frame x (filled in by planner from YAML)
        "y":        float,  # map-frame y (filled in by planner from YAML)
    }
"""

from __future__ import annotations

import math
import logging
from typing import TYPE_CHECKING

# Only import for type hints — avoids importing rclpy in unit tests
if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class TaskManager:
    """
    Central planning authority for the Cognitive AMR.

    The planner node creates one instance of this class and passes tasks
    through it before dispatching them to Nav2. Each method in this class
    is one planning concern (battery, priority, proximity, traffic).
    They are designed to be composed — you can chain them:

        tasks = self.task_manager.prioritize(tasks)
        tasks = self.task_manager.nearest_first(tasks, current_pose)
        if self.task_manager.needs_charging(battery_pct):
            tasks.insert(0, charging_task)
    """

    def __init__(self):
        # Battery percentage below which the robot should go charge first
        self.battery_low_threshold: float = 20.0

        # Battery percentage considered "full" — stop charging above this
        self.battery_full_threshold: float = 90.0

        # Current cached battery level (updated by planner from /battery_state)
        self._battery_pct: float = 100.0

        # Current cached robot pose (updated by planner from /amcl_pose)
        self._current_pose: tuple[float, float] = (0.0, 0.0)

        logger.info("TaskManager initialised (all planning features are stubs)")

    # ── Battery ────────────────────────────────────────────────────────────

    def update_battery(self, percentage: float) -> None:
        """
        Called by the planner node whenever a /battery_state message arrives.
        Keeps the TaskManager's battery knowledge current without it needing
        a ROS 2 subscription of its own.
        """
        self._battery_pct = percentage

    def needs_charging(self) -> bool:
        """
        Returns True if the robot should interrupt the current task queue
        and go to a charging dock before continuing.

        TODO: implement properly once a /battery_state topic exists.
              For a simulated battery, publish a std_msgs/Float32 on
              /battery_state and have the planner call update_battery().
        """
        return False

    # ── Pose ───────────────────────────────────────────────────────────────

    def update_pose(self, x: float, y: float) -> None:
        """
        Called by the planner node whenever an /amcl_pose message arrives.
        Keeps the TaskManager's position knowledge current.
        """
        self._current_pose = (x, y)

    # ── Prioritisation ─────────────────────────────────────────────────────

    def prioritize(self, tasks: list[dict]) -> list[dict]:
        """
        Sort tasks so that the most urgent run first.
        Lower 'priority' value = higher urgency (priority 1 runs before 2).

        TODO: implement once tasks carry a 'priority' field.
              Input tasks will come from an external queue (ROS 2 topic,
              REST API, etc.) and carry a priority integer.

        Example future implementation:
            return sorted(tasks, key=lambda t: t.get('priority', 99))
        """
        return tasks

    # ── Proximity sorting ──────────────────────────────────────────────────

    def nearest_first(self, tasks: list[dict]) -> list[dict]:
        """
        Re-order tasks so the robot visits the closest location first,
        minimising total travel distance (greedy nearest-neighbour).

        Uses self._current_pose as the starting point.

        TODO: implement once tasks carry x/y coordinates and the planner
              is calling update_pose() from /amcl_pose callbacks.

        Example future implementation:
            remaining = list(tasks)
            ordered = []
            pos = self._current_pose
            while remaining:
                closest = min(remaining, key=lambda t: self._dist(pos, t))
                ordered.append(closest)
                pos = (closest['x'], closest['y'])
                remaining.remove(closest)
            return ordered
        """
        # Stub: return tasks in original order
        return tasks

    # ── Traffic / obstacle awareness ───────────────────────────────────────

    def check_traffic(self, task: dict) -> bool:
        """
        Returns True if the path to this task's location appears clear,
        False if a significant obstacle is blocking the route.

        TODO: implement by querying Nav2's /compute_path_to_pose action or
              reading the global costmap to detect blocked routes before
              dispatching. If blocked, the planner can delay and retry.

        Args:
            task: a task dict with 'x' and 'y' keys.

        Example future implementation:
            cost = self._query_costmap(task['x'], task['y'])
            return cost < LETHAL_OBSTACLE_COST
        """
        return True

    # ── Outcome recording ──────────────────────────────────────────────────

    def record_result(self, task: dict, success: bool, duration_sec: float) -> None:
        """
        Persist the outcome of a completed task for analytics / audit trail.

        TODO: implement once a logging backend is chosen.
              Options: append to a CSV file, write to a SQLite database,
              or publish a custom ROS 2 message on /task_results.

        Args:
            task:         the task dict that was executed.
            success:      True if Nav2 reported SUCCESS, False otherwise.
            duration_sec: wall-clock seconds from goal dispatch to result.
        """
        # Stub: just log to the ROS 2 / Python logger for now
        status = "SUCCESS" if success else "FAILED"
        logger.info(
            f"Task result | location={task.get('name','?')} "
            f"status={status} duration={duration_sec:.1f}s"
        )

    # ── Internal helpers ───────────────────────────────────────────────────

    @staticmethod
    def _dist(pos: tuple[float, float], task: dict) -> float:
        """Euclidean distance from pos to a task's (x, y)."""
        return math.sqrt(
            (task['x'] - pos[0]) ** 2 +
            (task['y'] - pos[1]) ** 2
        )
