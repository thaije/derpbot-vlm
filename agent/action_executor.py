import logging
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist

logger = logging.getLogger(__name__)


class ActionExecutor:
    def __init__(self, node, config: dict):
        self.node = node
        self.actions = config["actions"]
        topic = config["ros"]["cmd_vel_topic"]
        self.publisher = node.create_publisher(Twist, topic, 10)
        self._current_twist: Optional[Twist] = None
        self._lock = threading.Lock()
        self._safety_override = False
        self._stopped = False
        self._publish_interval = config.get("actions", {}).get("_publish_interval_s", 0.1)
        self._last_action: str = "stop"

    def set_action(self, action: str):
        if action not in self.actions:
            logger.warning("Unknown action '%s', stopping", action)
            action = "stop"

        self._last_action = action
        params = self.actions[action]
        twist = Twist()
        twist.linear.x = float(params["linear_x"])
        twist.angular.z = float(params["angular_z"])

        with self._lock:
            self._current_twist = twist

    def publish_current(self):
        with self._lock:
            if self._stopped or self._current_twist is None:
                self.publisher.publish(Twist())
                return
            if self._safety_override:
                self.publisher.publish(Twist())
                return
            self.publisher.publish(self._current_twist)

    @property
    def last_action(self) -> str:
        return self._last_action

    @property
    def safety_active(self) -> bool:
        with self._lock:
            return self._safety_override

    def _is_forward_action(self, action: str) -> bool:
        params = self.actions.get(action, {})
        return params.get("linear_x", 0) > 0

    def publish_current(self):
        with self._lock:
            if self._stopped or self._current_twist is None:
                self.publisher.publish(Twist())
                return
            if self._safety_override and self._is_forward_action(self._last_action):
                self.publisher.publish(Twist())
                return
            self.publisher.publish(self._current_twist)

    def set_safety_override(self, active: bool):
        with self._lock:
            self._safety_override = active
            if active and self._current_twist is not None and self._is_forward_action(self._last_action):
                self.publisher.publish(Twist())

    def stop(self):
        with self._lock:
            self._stopped = True
            self._current_twist = Twist()
            self.publisher.publish(Twist())