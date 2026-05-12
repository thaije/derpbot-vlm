import logging
import threading
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

    def execute(self, action: str, duration_s: float = 1.0):
        if action not in self.actions:
            logger.warning("Unknown action '%s', stopping", action)
            action = "stop"

        params = self.actions[action]
        twist = Twist()
        twist.linear.x = float(params["linear_x"])
        twist.angular.z = float(params["angular_z"])

        with self._lock:
            self._current_twist = twist

        rate = self.node.create_rate(10)
        sim_clock = self.node.get_clock()
        start = sim_clock.now().nanoseconds / 1e9
        while sim_clock.now().nanoseconds / 1e9 - start < duration_s:
            with self._lock:
                if self._safety_override:
                    self.publisher.publish(Twist())
                    break
                self.publisher.publish(self._current_twist)
            rate.sleep()

    def set_safety_override(self, active: bool):
        with self._lock:
            self._safety_override = active
            if active and self._current_twist is not None:
                zero = Twist()
                self.publisher.publish(zero)

    def stop(self):
        with self._lock:
            self._current_twist = Twist()
            self.publisher.publish(Twist())