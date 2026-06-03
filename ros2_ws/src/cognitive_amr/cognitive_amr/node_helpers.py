#!/usr/bin/env python3

import json
import math
import threading
import time
import uuid
from typing import Any

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from std_msgs.msg import String


def parse_json_msg(msg: String, logger, context: str) -> dict | None:
    try:
        return json.loads(msg.data)
    except json.JSONDecodeError:
        logger.warn(f'Ignoring malformed JSON on {context}', throttle_duration_sec=2.0)
        return None


def publish_json(pub, payload: dict):
    pub.publish(String(data=json.dumps(payload)))


def make_pose_stamped(clock, x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = clock.now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def go_to_xyyaw(navigator, x: float, y: float, yaw: float,
                timeout: float = 120.0, poll_period: float = 0.1) -> bool:
    goal = make_pose_stamped(navigator.get_clock(), x, y, yaw)
    navigator.goToPose(goal)

    t0 = time.time()
    while not navigator.isTaskComplete():
        if time.time() - t0 > timeout:
            navigator.cancelTask()
            return False
        time.sleep(poll_period)

    return navigator.getResult() == TaskResult.SUCCEEDED


class JsonRequestClient:
    def __init__(self, logger):
        self._logger = logger
        self._pending: dict[str, threading.Event] = {}
        self._responses: dict[str, dict] = {}
        self._lock = threading.Lock()

    def handle_response_msg(self, msg: String, context: str) -> bool:
        data = parse_json_msg(msg, self._logger, context)
        if not data:
            return False

        rid = data.get('request_id')
        if not rid:
            return False

        with self._lock:
            ev = self._pending.get(rid)
            if ev is None:
                return False
            self._responses[rid] = data
            ev.set()
        return True

    def request(self, pub, payload: dict, timeout: float, label: str) -> dict | None:
        rid = str(payload.get('request_id') or uuid.uuid4())
        payload['request_id'] = rid
        ev = threading.Event()

        with self._lock:
            self._pending[rid] = ev

        publish_json(pub, payload)
        triggered = ev.wait(timeout=timeout)

        with self._lock:
            data = self._responses.pop(rid, None)
            self._pending.pop(rid, None)

        if not triggered:
            self._logger.warn(f"{label} timed out")
            return None
        return data