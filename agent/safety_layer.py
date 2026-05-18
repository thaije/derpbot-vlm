import logging
import math
import threading
from typing import Optional

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

logger = logging.getLogger(__name__)


class SafetyLayer:
    FORWARDED_ARC_DEG = 30
    MIN_RANGE_M = 0.3

    def __init__(self, node, config: dict):
        self.node = node
        ros_cfg = config["ros"]
        safety_cfg = config.get("safety", {})

        self.min_range_m = safety_cfg.get("min_range_m", self.MIN_RANGE_M)
        self.forward_arc_deg = safety_cfg.get("forward_arc_deg", self.FORWARDED_ARC_DEG)
        self.cmd_vel_topic = ros_cfg["cmd_vel_topic"]
        self.lidar_topic = ros_cfg["lidar_topic"]

        self._safety_active = False
        self._lock = threading.Lock()
        self._action_executor = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        self._cmd_vel_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.node.create_subscription(LaserScan, self.lidar_topic, self._scan_callback, qos)
        logger.info("Safety layer started (min_range=%.2fm, arc=±%d°)", self.min_range_m, self.forward_arc_deg)

    def set_action_executor(self, executor):
        self._action_executor = executor

    def _scan_callback(self, msg: LaserScan):
        if msg.ranges is None or len(msg.ranges) == 0:
            return

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        arc_rad = math.radians(self.forward_arc_deg)

        obstacle_in_arc = False
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min:
                continue
            angle = angle_min + i * angle_increment
            if abs(self._normalize_angle(angle)) <= arc_rad:
                if r < self.min_range_m:
                    obstacle_in_arc = True
                    break

        with self._lock:
            was_active = self._safety_active
            self._safety_active = obstacle_in_arc

        if obstacle_in_arc:
            self._cmd_vel_pub.publish(Twist())
            if self._action_executor:
                self._action_executor.set_safety_override(True)

        elif not obstacle_in_arc and was_active:
            logger.info("SAFETY: forward arc clear, resuming")
            if self._action_executor:
                self._action_executor.set_safety_override(False)

        if obstacle_in_arc and not was_active:
            logger.warning("SAFETY: obstacle detected in forward arc (< %.2fm), stopping", self.min_range_m)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._safety_active