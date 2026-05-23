"""Reactive safety layer — Phase 1 of #9.

Owns /cmd_vel. Upstream callers (wall-follow, VLM planner) issue desired
velocities via command(); the safety layer applies LiDAR forward veto and
bumper-triggered back-off, then publishes at a fixed rate.

Behaviours:
  - Bumper contact (non-ground) → enter back-off for BACKUP_DURATION_S:
    drive backwards + rotate toward the more open side. Upstream commands
    ignored while backing off.
  - LiDAR forward arc < min_range_m → zero linear component, keep angular
    (or rotate toward open side if upstream wasn't already turning).
  - Otherwise pass through.

Ground-plane filter mirrors metrics/collision_count.py exactly: string-match
on 'ground_plane' in the collision entity names. (A per-contact-normal filter
was tried but over-rejected wall hits because `normals[]` is sometimes empty
or has non-horizontal entries for legitimate wall contacts.)
"""

import logging
import math
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class ReactiveSafetyLayer:
    PUBLISH_HZ = 20.0
    FORWARD_VETO_RANGE_M = 0.25
    FORWARD_VETO_ARC_DEG = 30.0
    BACKUP_LINEAR_M_S = -0.2
    BACKUP_ANGULAR_RAD_S = 0.6
    BACKUP_DURATION_S = 1.5
    CONTACT_DEBOUNCE_S = 0.5
    DEFAULT_BUMPER_TOPIC = "/derpbot_0/bumper_contact"

    def __init__(self, node, config: dict):
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.node = node
        ros_cfg = config["ros"]
        safety_cfg = config.get("safety", {})

        self.cmd_vel_topic = ros_cfg["cmd_vel_topic"]
        self.lidar_topic = ros_cfg["lidar_topic"]
        self.bumper_topic = ros_cfg.get("bumper_topic", self.DEFAULT_BUMPER_TOPIC)
        self.min_range_m = float(safety_cfg.get("min_range_m", self.FORWARD_VETO_RANGE_M))
        self.forward_arc_deg = float(safety_cfg.get("forward_arc_deg", self.FORWARD_VETO_ARC_DEG))

        qos_be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cmd_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)
        node.create_subscription(LaserScan, self.lidar_topic, self._scan_cb, qos_be)

        try:
            from ros_gz_interfaces.msg import Contacts
            node.create_subscription(Contacts, self.bumper_topic, self._bumper_cb, 10)
            logger.info("Bumper subscribed: %s", self.bumper_topic)
        except Exception as e:
            logger.warning("Bumper unavailable (%s); running LiDAR-only safety", e)

        self._lock = threading.Lock()
        self._desired_lin = 0.0
        self._desired_ang = 0.0
        self._scan_min_front = math.inf
        self._scan_min_left = math.inf
        self._scan_min_right = math.inf

        self._backup_until_sim_s: Optional[float] = None
        self._backup_angular_dir = 1.0
        self._last_contact_sim_s = -1e9
        self._collision_events = 0
        self._lidar_veto_active = False

        node.create_timer(1.0 / self.PUBLISH_HZ, self._publish_tick)

        logger.info(
            "ReactiveSafetyLayer started (min_range=%.2fm, arc=±%.0f°, %.0fHz)",
            self.min_range_m, self.forward_arc_deg, self.PUBLISH_HZ,
        )

    # ------------------------------------------------------------------ API

    def command(self, linear_x: float, angular_z: float) -> None:
        """Upstream desired velocity — applied subject to safety filters."""
        with self._lock:
            self._desired_lin = float(linear_x)
            self._desired_ang = float(angular_z)

    def is_blocked(self) -> bool:
        """True while a bumper-triggered back-off is in progress."""
        with self._lock:
            return self._backup_until_sim_s is not None

    def is_veto_active(self) -> bool:
        with self._lock:
            return self._lidar_veto_active

    def front_clearance_m(self) -> float:
        with self._lock:
            return self._scan_min_front

    @property
    def collision_events(self) -> int:
        with self._lock:
            return self._collision_events

    def stop(self) -> None:
        from geometry_msgs.msg import Twist
        with self._lock:
            self._desired_lin = 0.0
            self._desired_ang = 0.0
            self._backup_until_sim_s = None
        self._cmd_pub.publish(Twist())

    # ----------------------------------------------------------- callbacks

    def _scan_cb(self, msg) -> None:
        if not msg.ranges:
            return
        amin = msg.angle_min
        ai = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max
        arc_rad = math.radians(self.forward_arc_deg)

        front = math.inf
        left = math.inf
        right = math.inf

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
                continue
            a = self._normalize(amin + i * ai)
            if abs(a) <= arc_rad and r < front:
                front = r
            if 0 < a <= math.pi / 2 and r < left:
                left = r
            elif -math.pi / 2 <= a < 0 and r < right:
                right = r

        with self._lock:
            self._scan_min_front = front
            self._scan_min_left = left
            self._scan_min_right = right

    def _bumper_cb(self, msg) -> None:
        raw = list(getattr(msg, "contacts", []))
        real = [c for c in raw if self._is_real_contact(c)]
        if not real:
            if raw:
                logger.debug("BUMPER raw=%d filtered=0 (all ground)", len(raw))
            return

        sim_now = self._sim_now()
        with self._lock:
            if sim_now - self._last_contact_sim_s < self.CONTACT_DEBOUNCE_S:
                self._last_contact_sim_s = sim_now
                return
            self._last_contact_sim_s = sim_now
            left = self._scan_min_left
            right = self._scan_min_right
            self._backup_angular_dir = 1.0 if left >= right else -1.0
            self._backup_until_sim_s = sim_now + self.BACKUP_DURATION_S
            self._collision_events += 1
            events = self._collision_events
            ang_dir = self._backup_angular_dir

        sample = real[0]
        logger.warning(
            "BUMPER: contact #%d raw=%d real=%d c1=%s c2=%s rot=%s",
            events, len(raw), len(real),
            self._entity_name(getattr(sample, "collision1", None)),
            self._entity_name(getattr(sample, "collision2", None)),
            "left" if ang_dir > 0 else "right",
        )

    @staticmethod
    def _entity_name(e) -> str:
        if e is None:
            return "?"
        name = getattr(e, "name", None)
        return name if name else str(e)[:60].replace("\n", " ")

    def _is_real_contact(self, c) -> bool:
        # Mirror metrics/collision_count.py: string-match exclusion of ground_plane only.
        if "ground_plane" in str(getattr(c, "collision1", c)):
            return False
        if "ground_plane" in str(getattr(c, "collision2", c)):
            return False
        return True

    # ------------------------------------------------------------ publish

    def _publish_tick(self) -> None:
        from geometry_msgs.msg import Twist
        sim_now = self._sim_now()

        with self._lock:
            desired_lin = self._desired_lin
            desired_ang = self._desired_ang
            front = self._scan_min_front
            left = self._scan_min_left
            right = self._scan_min_right
            backup_end = self._backup_until_sim_s
            backup_dir = self._backup_angular_dir

        twist = Twist()

        if backup_end is not None:
            if sim_now < backup_end:
                twist.linear.x = self.BACKUP_LINEAR_M_S
                twist.angular.z = self.BACKUP_ANGULAR_RAD_S * backup_dir
                self._cmd_pub.publish(twist)
                return
            with self._lock:
                self._backup_until_sim_s = None
            logger.info("BUMPER: back-off complete, resuming upstream commands")

        veto = (desired_lin > 0.0) and (front < self.min_range_m)
        if veto:
            # Zero forward. If the upstream wasn't already turning, slide toward
            # the more open side so the robot doesn't deadlock facing a wall.
            ang = desired_ang
            if abs(ang) < 1e-3:
                ang = 0.7 if left >= right else -0.7
            twist.linear.x = 0.0
            twist.angular.z = ang
        else:
            twist.linear.x = desired_lin
            twist.angular.z = desired_ang

        with self._lock:
            self._lidar_veto_active = veto

        self._cmd_pub.publish(twist)

    def _sim_now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _normalize(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
