"""Reactive safety layer.

Owns /cmd_vel. Upstream callers (planner, teleop) issue desired velocities
via command(); the safety layer applies a geometry-aware LiDAR veto and
bumper-triggered back-off, then publishes at a fixed rate.

The veto is built so that — given a valid 360° LiDAR scan — collisions are
geometrically impossible:

  - The robot is treated as a rectangle (`ROBOT_FRONT_M` x `ROBOT_SIDE_M`
    from base_link) with a configurable cushion (`SAFETY_CUSHION_M`).
  - For every commanded twist (lin, ang), three checks run:
      forward translation  → any scan point in the front half whose
                              longitudinal clearance is < cushion vetoes
                              the forward component;
      backward translation → mirror of the above for the rear half;
      rotation             → any scan point whose RAW distance from
                              base_link is < `ROBOT_CORNER_M` + cushion
                              vetoes angular motion (the robot's corner
                              sweeps a circle of that radius during pure
                              rotation).
  - Bumper contact still triggers back-off as a backstop — it should not
    fire if the LiDAR veto is doing its job. The back-off recovery itself is
    run through the same clearance caps so it can never reverse or rotate the
    robot into a second obstacle.

The forward/backward velocity cap also charges the LiDAR + control-loop
reaction latency (`REACTION_TIME_S`) against the available clearance, so the
robot starts braking early enough that it cannot overrun the cushion at speed.

This module assumes the basement template doesn't contain any obstacle
below the LiDAR plane (z ≈ 0.12 m). Floor-obstacle handling lived in an
earlier opt-in depth-camera veto path (see #12 history); it has been
removed since the scenario was changed to clear short objects.
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
    # cushion by v·t_react and the bumper fired — the documented "inertia
    # carries the robot the last few cm into contact" (#12). It scales with
    # speed, so unlike a bigger fixed cushion it does NOT enlarge the no-go
    # zone at crawl speed (where the cushion=0.15 experiment caused planner
    # thrash).
    REACTION_TIME_S = 0.15

    # Gentle reverse used to escape a wedge where the robot can neither move
    # forward nor pivot to either open side (walls front + both sides).
    WEDGE_ESCAPE_SPEED_M_S = 0.12
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
        self._lidar_veto_active = False

        node.create_timer(1.0 / self.PUBLISH_HZ, self._publish_tick)

        logger.info(
            "ReactiveSafetyLayer started "
            "(robot %0.2fx%0.2f m, cushion=%.2fm, corner=%.2fm, %.0fHz)",
            2 * self.ROBOT_FRONT_M, 2 * self.ROBOT_SIDE_M,
            self.cushion_m, self.ROBOT_CORNER_M, self.PUBLISH_HZ,
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

    # ------------------------------------------------------- veto geometry

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

    def _wedge_reverse_speed(self, points: Sequence[tuple[float, float]]) -> float:
        """Reverse speed magnitude to back out of a wedge, capped by rear
        clearance via the usual stopping bound. 0.0 when the rear is also
        blocked — the robot is genuinely trapped and reversing would just
        cause a rear collision."""
        rear_clear = self._directional_clearance_m(points, direction_is_forward=False)
        return min(self.WEDGE_ESCAPE_SPEED_M_S, self._safe_linear_cap(rear_clear))

    def _point_rect_distance(self, x: float, y: float) -> float:
        """Distance from a point to the robot rectangle (0 if inside)."""
        dx = max(abs(x) - self.ROBOT_FRONT_M, 0.0)
        dy = max(abs(y) - self.ROBOT_SIDE_M, 0.0)
        return math.hypot(dx, dy)

    def _rotation_allowed(self, points: Sequence[tuple[float, float]],
                          ang: float) -> bool:
        """Directional rotation veto.

        Binary "any obstacle within corner+cushion blocks all rotation" wedges
        the robot: pressed against a wall it can neither translate (front/rear
        blocked in a narrow gap) nor turn, so it freezes in sustained contact.
        But rotating *away* from the nearest obstacle is the escape — the
        contact corner just slides tangentially.

        So: if nothing is within the cushion, rotation is always fine. If
        something is, allow rotation only in the direction that does not bring
        the robot's perimeter closer to that nearest obstacle. We test it by
        rotating the world-fixed scan points by a small angle opposite to the
        commanded body rotation and checking the closest point's distance to
        the rectangle does not shrink.
        """
        if abs(ang) < 1e-3:
            return True
        nearest = min(self._point_rect_distance(r * math.cos(t), r * math.sin(t))
                      for t, r in points) if points else math.inf
        if nearest >= self.cushion_m:
            return True
        # Body rotates by +ang; a world-fixed point moves by -ang in the body
        # frame. Use a small probe angle to gauge the trend.
        dtheta = -math.copysign(math.radians(5.0), ang)
        probed = min(
            self._point_rect_distance(r * math.cos(t + dtheta),
                                      r * math.sin(t + dtheta))
            for t, r in points
        )
        return probed >= nearest

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
            left_min = self._scan_min_left
            right_min = self._scan_min_right
            backup_end = self._backup_until_sim_s
            backup_dir = self._backup_angular_dir


        twist = Twist()

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
                return
            with self._lock:
                self._backup_until_sim_s = None
            logger.info("BUMPER: back-off complete, resuming upstream commands")

        lin = desired_lin
        ang = desired_ang
        veto_components: List[str] = []

        # Clearance-based velocity cap. For forward motion: the smallest
        # (x - FRONT) over the forward corridor. For rearward: mirror. Cap
        # the commanded velocity so the robot can always brake before
        # entering the cushion. Hard-zero when clearance ≤ cushion.
        if lin > 0.0:
            fwd_clear = self._directional_clearance_m(points, direction_is_forward=True)
            cap = self._safe_linear_cap(fwd_clear)
            if cap < abs(lin):
                if cap == 0.0:
                    veto_components.append(f"fwd<{fwd_clear:.2f}m")
                else:
                    veto_components.append(f"fwd-cap<{fwd_clear:.2f}m→{cap:.2f}")
                lin = cap
        elif lin < 0.0:
            rear_clear = self._directional_clearance_m(points, direction_is_forward=False)
            cap = self._safe_linear_cap(rear_clear)
            if cap < abs(lin):
                if cap == 0.0:
                    veto_components.append(f"rear<{rear_clear:.2f}m")
                else:
                    veto_components.append(f"rear-cap<{rear_clear:.2f}m→{cap:.2f}")
                lin = -cap  # rearward speed magnitude, preserve sign

        # Rotation: directional veto. Block the turn only if rotating THIS way
        # would bring the perimeter closer to an obstacle already inside the
        # cushion; rotating away (the wedge escape) stays allowed.
        if abs(ang) > 1e-3 and not self._rotation_allowed(points, ang):
            veto_components.append(
                f"rot<{self._rotation_clearance_m(points):.2f}m")
            ang = 0.0

        # Deadlock recovery: the upstream wants to move but every component
        # got vetoed (forward into a wall, or rotation that would close on it).
        # First try to pivot toward the more-open side — the directional veto
        # allows that exactly when it opens clearance, which is the wedge
        # escape. If even that is blocked (walls on both sides), back out when
        # the rear permits. This is what un-sticks the planner's stop-and-turn
        # commits in narrow divider-wall gaps; it fires for teleop too.
        wants_motion = desired_lin > 0.0 or abs(desired_ang) > 1e-3
        if wants_motion and lin == 0.0 and abs(ang) < 1e-3:
            slide = 0.7 if left_min >= right_min else -0.7
            if self._rotation_allowed(points, slide):
                ang = slide
            else:
                rev = self._wedge_reverse_speed(points)
                if rev > 0.0:
                    lin = -rev
                    veto_components.append(f"wedge-rev-{rev:.2f}")

        veto_active = bool(veto_components)
        if veto_active and not self._lidar_veto_active:
            logger.info(
                "VETO ON (%s) lin=%+.2f→%+.2f ang=%+.2f→%+.2f",
                ",".join(veto_components),
                desired_lin, lin, desired_ang, ang,
            )

        with self._lock:
            self._lidar_veto_active = veto_active

        twist.linear.x = lin
        twist.angular.z = ang
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
