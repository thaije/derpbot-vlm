"""Reactive safety layer.

Owns /cmd_vel. Upstream callers (planner, teleop) issue desired velocities
via command(); the safety layer applies bumper-triggered back-off, then
publishes at a fixed rate.

Geometry veto (LiDAR clearance caps + rotation veto) was removed in #14
because it caused oscillation near walls. The VLM sees LiDAR clearance in
the prompt and chooses its own distances/headers. Bumper back-off remains
as the safety backstop.

Bumper back-off: reverse capped by rear clearance, turn unconditional.
The reverse is gated by rear clearance so we never drive straight back
into another obstacle; the recovery turn is left unconditional because
rotating the heading away from the wall is the reliable escape from a
wedge.
"""

import logging
import math
import threading
from typing import List, Optional, Sequence

logger = logging.getLogger(__name__)


class ReactiveSafetyLayer:
    PUBLISH_HZ = 20.0

    # Robot geometry from robot-sandbox/robots/derpbot/urdf/derpbot.urdf:
    # base_link collision box = 0.30 x 0.20 x 0.10. Wheels sit at y=±0.115 m
    # extending the lateral footprint slightly past the body — fold that
    # into SIDE so the cushion covers the wheels too.
    ROBOT_FRONT_M = 0.15
    ROBOT_SIDE_M = 0.13          # body 0.10 + wheel hub 0.03
    ROBOT_CORNER_M = math.hypot(ROBOT_FRONT_M, ROBOT_SIDE_M)  # ≈0.198
    # Padding beyond the robot perimeter. The LiDAR scan window + control
    # tick latency is now charged to the velocity cap (REACTION_TIME_S), so
    # the cushion is a pure geometric margin rather than a fudge for braking
    # lag. A larger cushion (0.15 m) made cells worse in practice because the
    # planner then thrashes in the bigger no-go zone and pushes the robot
    # into adjacent walls. Cushion 0.10 is the practical sweet spot.
    SAFETY_CUSHION_M = 0.10
    # Velocity cap: the robot must be able to stop within (clear − cushion).
    # Stopping distance has two parts: a reaction phase where the robot keeps
    # moving at the commanded speed before the veto can respond, then a
    # braking phase at MAX_LINEAR_DECEL_M_S2. Solving
    #   v·t_react + v²/(2a) ≤ (clear − cushion)
    # for v gives the cap (see _safe_linear_cap). a=2.0; a gentler value
    # (0.8) starts capping earlier but produced MORE contacts in practice —
    # the robot crawled into the cushion zone while the planner re-issued
    # commits. Keep a=2.0.
    MAX_LINEAR_DECEL_M_S2 = 2.0
    # Reaction latency before braking actually starts: one 10 Hz LiDAR period
    # (≤0.10 s of scan age) + one 0.05 s control tick. Without this term the
    # cap assumed instantaneous braking, so at speed the robot overran the
    # cushion and the bumper fired — the documented "inertia carries the
    # robot the last few cm into contact" (#12). It scales with speed, so
    # unlike a bigger fixed cushion it does NOT enlarge the no-go zone at
    # crawl speed (where the cushion=0.15 experiment caused planner thrash).
    REACTION_TIME_S = 0.15

    # Bumper back-off parameters.
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
        self.cushion_m = float(safety_cfg.get("cushion_m", self.SAFETY_CUSHION_M))
        self.reaction_time_s = float(
            safety_cfg.get("reaction_time_s", self.REACTION_TIME_S)
        )

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
        # Cached scan: list of (angle_rad, range_m), filtered to finite/in-range.
        self._scan_points: List[tuple[float, float]] = []
        # Convenience minima used by the existing UI (front_clearance_m()) +
        # the auto-slide direction selector.
        self._scan_min_front = math.inf
        self._scan_min_left = math.inf
        self._scan_min_right = math.inf

        self._backup_until_sim_s: Optional[float] = None
        self._backup_angular_dir = 1.0
        self._last_contact_sim_s = -1e9
        self._collision_events = 0
        # Passthrough: publish the commanded twist unfiltered. Off by default;
        # only the #13 debug harness's --no-safety flag flips it so the user can
        # feel raw control. Still publishes via THIS timer (no /cmd_vel race).
        self._passthrough = False
        # Change-detection for cmd_vel logging (last published values).
        self._last_pub_lin: Optional[float] = None
        self._last_pub_ang: Optional[float] = None
        self._last_cmd_src: str = ""

        node.create_timer(1.0 / self.PUBLISH_HZ, self._publish_tick)

        logger.info(
            "ReactiveSafetyLayer started "
            "(robot %0.2fx%0.2f m, cushion=%.2fm, corner=%.2fm, %.0fHz)",
            2 * self.ROBOT_FRONT_M, 2 * self.ROBOT_SIDE_M,
            self.cushion_m, self.ROBOT_CORNER_M, self.PUBLISH_HZ,
        )

    # ------------------------------------------------------------------ API

    def command(self, linear_x: float, angular_z: float, src: str = "") -> None:
        """Upstream desired velocity — applied subject to safety filters."""
        with self._lock:
            self._desired_lin = float(linear_x)
            self._desired_ang = float(angular_z)
            if src:
                self._last_cmd_src = src

    def set_passthrough(self, enabled: bool) -> None:
        """Debug-only (#13): when True, the publish tick emits the commanded
        twist with NO bumper filtering. Off by default."""
        with self._lock:
            self._passthrough = bool(enabled)
        logger.warning("Safety passthrough %s — collision filtering %s",
                       "ON" if enabled else "OFF",
                       "DISABLED" if enabled else "active")

    def is_blocked(self) -> bool:
        """True while a bumper-triggered back-off is in progress."""
        with self._lock:
            return self._backup_until_sim_s is not None

    def front_clearance_m(self) -> float:
        with self._lock:
            return self._scan_min_front

    def rotation_clearance_m(self) -> float:
        """Worst-case clearance for pure in-place rotation: min(r − CORNER) over
        the scan. ≤ 0 means a wall is inside the corner-sweep circle and the
        robot cannot spin freely. Callers (e.g. the agent's scan) use this to
        avoid attempting a sweep in a corridor too tight to turn in."""
        with self._lock:
            return self._rotation_clearance_m(self._scan_points)

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

        points: List[tuple[float, float]] = []
        front = math.inf
        left = math.inf
        right = math.inf
        fwd_arc = math.radians(30.0)  # legacy: front_clearance_m() reports this

        for i, r in enumerate(msg.ranges):
            if math.isnan(r):
                continue
            if math.isinf(r) or r > rmax:
                # Too far / no return — that direction is clear.
                continue
            a = self._normalize(amin + i * ai)
            if r < rmin:
                # In the LiDAR's blind zone — there IS something but we can't
                # measure how close. Treat as an obstacle right at rmin so the
                # safety check still vetoes motion in that direction. Without
                # this, a robot pressed against a wall sees the wall vanish
                # from the scan and forward motion gets re-allowed.
                r = rmin
            points.append((a, r))
            if abs(a) <= fwd_arc and r < front:
                front = r
            if 0 < a <= math.pi / 2 and r < left:
                left = r
            elif -math.pi / 2 <= a < 0 and r < right:
                right = r

        with self._lock:
            self._scan_points = points
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

    # ------------------------------------------------------- clearance utils

    def _directional_clearance_m(self, points: Sequence[tuple[float, float]],
                                   direction_is_forward: bool) -> float:
        """Closest obstacle clearance to the FRONT (or REAR) face of the
        robot for motion along ±x.

        For each scan point (theta, r) with theta in robot frame, 0 = +x:
          x = r·cos(theta) — flipped if checking rearward motion
          y = r·sin(theta)
        Obstacles outside the robot's lateral band (|y| > SIDE + cushion)
        can't be hit by axial motion alone. Among the rest, the minimum
        (x − FRONT) is the worst-case clearance. Returns inf if nothing
        is in the corridor.
        """
        side_limit = self.ROBOT_SIDE_M + self.cushion_m
        front_extent = self.ROBOT_FRONT_M
        sign = 1.0 if direction_is_forward else -1.0
        min_clear = math.inf
        for theta, r in points:
            x = r * math.cos(theta) * sign
            y = r * math.sin(theta)
            if x <= 0:
                continue
            if abs(y) > side_limit:
                continue
            clear = x - front_extent
            if clear < min_clear:
                min_clear = clear
        return min_clear

    def _rotation_clearance_m(self, points: Sequence[tuple[float, float]]) -> float:
        """For pure in-place rotation, the worst-case extent is the corner
        radius. Returns the smallest `r - CORNER` over the scan."""
        min_clear = math.inf
        for _theta, r in points:
            clear = r - self.ROBOT_CORNER_M
            if clear < min_clear:
                min_clear = clear
        return min_clear

    def _safe_linear_cap(self, clearance: float) -> float:
        """Maximum |linear velocity| from which the robot can still stop inside
        `clearance - cushion`, accounting for reaction latency THEN braking.

        Stopping distance = v·t_react + v²/(2a). Requiring it ≤ d gives a
        quadratic whose positive root is √(a²t² + 2·a·d) − a·t. With t=0 this
        reduces to the old √(2·a·d). Returns 0.0 when clearance ≤ cushion.
        Smooth ramp instead of a hard binary veto so the robot bleeds off
        speed before it reaches the contact zone."""
        d = clearance - self.cushion_m
        if d <= 0.0:
            return 0.0
        a = self.MAX_LINEAR_DECEL_M_S2
        t = self.reaction_time_s
        return math.sqrt(a * a * t * t + 2.0 * a * d) - a * t

    # ------------------------------------------------------------ publish

    def _publish_tick(self) -> None:
        from geometry_msgs.msg import Twist
        sim_now = self._sim_now()

        with self._lock:
            desired_lin = self._desired_lin
            desired_ang = self._desired_ang
            points = list(self._scan_points)
            backup_end = self._backup_until_sim_s
            backup_dir = self._backup_angular_dir
            passthrough = self._passthrough

        twist = Twist()

        if passthrough:
            # Raw control for the debug harness — no vetoes, no back-off.
            twist.linear.x = desired_lin
            twist.angular.z = desired_ang
            self._cmd_pub.publish(twist)
            return

        # Bumper back-off has highest priority — it overrides the upstream
        # command with a reverse-AND-turn recovery. The reverse is capped by
        # rear clearance so we never drive straight back into another obstacle
        # (a box/pillar behind), but the turn is left UNCONDITIONAL: rotating
        # the heading away from the wall is the reliable escape from a wedge,
        # and for a robot already touching a wall the contact corner only
        # slides tangentially. Gating the turn on corner clearance (an earlier
        # attempt) removed the escape and the robot froze in sustained contact
        # against thin divider walls — far worse than the tangential scrape it
        # was meant to avoid.
        if backup_end is not None:
            if sim_now < backup_end:
                rear_clear = self._directional_clearance_m(
                    points, direction_is_forward=False
                )
                rev_cap = self._safe_linear_cap(rear_clear)
                twist.linear.x = -min(abs(self.BACKUP_LINEAR_M_S), rev_cap)
                twist.angular.z = self.BACKUP_ANGULAR_RAD_S * backup_dir
                self._cmd_pub.publish(twist)
                self._log_cmd_vel_change(twist.linear.x, twist.angular.z, "bumper")
                return
            with self._lock:
                self._backup_until_sim_s = None
            logger.info("BUMPER: back-off complete, resuming upstream commands")

        twist.linear.x = desired_lin
        twist.angular.z = desired_ang
        self._cmd_pub.publish(twist)
        self._log_cmd_vel_change(desired_lin, desired_ang, self._last_cmd_src)

    def _sim_now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _log_cmd_vel_change(self, lin: float, ang: float, src: str) -> None:
        """Log when the published (lin, ang) transitions to a new value."""
        rounded_lin = round(lin, 3)
        rounded_ang = round(ang, 3)
        if (self._last_pub_lin is None
                or rounded_lin != self._last_pub_lin
                or rounded_ang != self._last_pub_ang):
            src_tag = f" [{src}]" if src else ""
            logger.info("CMD_VEL: lin=%+.2f ang=%+.2f%s",
                        rounded_lin, rounded_ang, src_tag)
        self._last_pub_lin = rounded_lin
        self._last_pub_ang = rounded_ang

    @staticmethod
    def _normalize(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a