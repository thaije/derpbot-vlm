"""iRobot Create 3 transport via ROS 2 (#25).

Implements ``RobotTransport`` using native ROS 2 topics:
- Drive:   ``/cmd_vel`` (``geometry_msgs/Twist``)
- LED:     ``/cmd_lightring`` (``irobot_create_msgs/LightringLeds``)
- Audio:   ``/cmd_audio`` (``irobot_create_msgs/AudioNoteVector``)
- IMU:     ``/imu`` (``sensor_msgs/Imu``) — yaw for rotate-to-target
- Odom:    ``/odom`` (``nav_msgs/Odometry``) — pose for move_linear distance
- Battery: ``/battery_state`` (``sensor_msgs/BatteryState``)
- Hazard:  ``/hazard_detection`` (``irobot_create_msgs/HazardDetectionStamped``)

The rclpy node runs in a background thread with a ``SingleThreadedExecutor``
(STATE.md #15 — MultiThreadedExecutor busy-spins).  The agent's main thread
stays asyncio; rclpy↔asyncio bridging uses ``asyncio.run_coroutine_threadsafe``.

Create 3 firmware is ROS 2 Iron + FastDDS, confirmed Jazzy-compatible by
iRobot dev.  ``use_sim_time=False`` (real hardware).
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Optional

from PIL import Image

from robot_agent.transport import BatteryState, HazardEvent, RobotPose, RobotTransport
from rvr_bridge.server import PhoneRelay

logger = logging.getLogger(__name__)

# Drive constants
DRIVE_SPEED_MPS = 0.4
ROTATE_SPEED_RAD = 0.8
YAW_TOLERANCE_RAD = math.radians(8.0)  # same as agent/planner.py
TELEOP_MAX_LIN = 0.4
TELEOP_MAX_ANG = 1.5

# ROS topic names (Create 3 standard — may need namespace prefix on some setups)
CMD_VEL_TOPIC = "/cmd_vel"
CMD_LIGHTRING_TOPIC = "/cmd_lightring"
CMD_AUDIO_TOPIC = "/cmd_audio"
IMU_TOPIC = "/imu"
ODOM_TOPIC = "/odom"
BATTERY_TOPIC = "/battery_state"
HAZARD_TOPIC = "/hazard_detection"

# Hazard type constants (irobot_create_msgs/HazardDetection)
HAZARD_BACKUP_LIMIT = 0
HAZARD_BUMP = 1
HAZARD_CLIFF = 2
HAZARD_STALL = 3
HAZARD_WHEEL_DROP = 4
HAZARD_OBJECT_PROXIMITY = 5

_HAZARD_KINDS = {
    HAZARD_BUMP: "bump",
    HAZARD_CLIFF: "cliff",
    HAZARD_STALL: "stall",
    HAZARD_WHEEL_DROP: "wheel_drop",
    HAZARD_OBJECT_PROXIMITY: "object_proximity",
    HAZARD_BACKUP_LIMIT: "backup_limit",
}


class Create3Transport(RobotTransport):
    """iRobot Create 3 transport via ROS 2."""

    backend_name = "create3"
    capabilities = ["led_ring", "audio", "dock", "hazard_display"]

    def __init__(
        self,
        relay: PhoneRelay,
        *,
        ros_domain_id: int = 0,
        drive_speed_mps: float = DRIVE_SPEED_MPS,
        rotate_speed_rad: float = ROTATE_SPEED_RAD,
    ):
        self.relay = relay
        self.ros_domain_id = ros_domain_id
        self.drive_speed_mps = drive_speed_mps
        self.rotate_speed_rad = rotate_speed_rad

        self._rclpy_thread: Optional[threading.Thread] = None
        self._node: Optional["_Create3Node"] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._connected = False

        # Wire relay callbacks for camera frames (phone in camera-only mode)
        self.relay.on_frame = self._on_frame
        self._latest_frame: Optional[Image.Image] = None
        self._on_frame_cb: Optional[callable] = None  # type: ignore[assignment]

    @property
    def connection_state(self) -> str:
        return "ready" if self._connected else "disconnected"

    @property
    def has_real_heading(self) -> bool:
        return True  # /imu provides real yaw

    @property
    def has_hazard_sensor(self) -> bool:
        return True  # /hazard_detection is a discrete sensor

    async def start(self) -> None:
        # Start the phone relay (camera frames)
        await self.relay.start()
        self._loop = asyncio.get_event_loop()

        # Start rclpy in a background thread
        self._node = _Create3Node(
            ros_domain_id=self.ros_domain_id,
            on_hazard=self._on_ros_hazard,
            on_battery=self._on_ros_battery,
        )
        self._rclpy_thread = threading.Thread(
            target=self._node.spin_forever, daemon=True, name="create3-rclpy"
        )
        self._rclpy_thread.start()

        # Wait briefly for rclpy to discover the robot
        await asyncio.sleep(1.0)
        self._connected = True
        logger.info("Create 3 transport started (ROS_DOMAIN_ID=%d)", self.ros_domain_id)

    async def stop(self) -> None:
        self._connected = False
        if self._node:
            self._node.shutdown()
        if self._rclpy_thread:
            self._rclpy_thread.join(timeout=5.0)
        await self.relay.stop()

    async def capture_frame(self) -> Optional[Image.Image]:
        return await self.relay.capture_frame()

    async def move_linear(self, distance_m: float, *, timeout_s: float) -> None:
        """Drive straight by ``distance_m`` using /odom path-length."""
        if abs(distance_m) < 0.01:
            await self.halt()
            return

        start_pose = self._node.get_pose() if self._node else None
        if start_pose is None:
            logger.warning("No /odom yet — cannot do closed-loop linear drive")
            await self._publish_cmd_vel(distance_m * 0.4, 0.0)
            await asyncio.sleep(min(abs(distance_m) / self.drive_speed_mps, timeout_s))
            await self.halt()
            return

        speed = self.drive_speed_mps if distance_m > 0 else -self.drive_speed_mps
        target = abs(distance_m)
        deadline = time.monotonic() + timeout_s

        await self._publish_cmd_vel(speed, 0.0)
        while time.monotonic() < deadline:
            await asyncio.sleep(0.05)
            pose = self._node.get_pose()
            if pose is None:
                continue
            dx = pose.x - start_pose.x
            dy = pose.y - start_pose.y
            traveled = math.sqrt(dx * dx + dy * dy)
            if traveled >= target:
                break
        await self.halt()

    async def rotate(self, angle_deg: float, *, timeout_s: float) -> None:
        """Rotate in place by ``angle_deg`` (positive = left/CCW) using /imu yaw."""
        if abs(angle_deg) < 1.0:
            await self.halt()
            return

        start_yaw = self._node.get_yaw() if self._node else None
        if start_yaw is None:
            logger.warning("No /imu yet — cannot do closed-loop rotation")
            # Fallback: timed rotation
            duration_s = min(abs(angle_deg) / 90.0, timeout_s)
            ang = self.rotate_speed_rad if angle_deg > 0 else -self.rotate_speed_rad
            await self._publish_cmd_vel(0.0, ang)
            await asyncio.sleep(duration_s)
            await self.halt()
            return

        target_delta = math.radians(angle_deg)
        speed = self.rotate_speed_rad if angle_deg > 0 else -self.rotate_speed_rad
        deadline = time.monotonic() + timeout_s

        await self._publish_cmd_vel(0.0, speed)
        while time.monotonic() < deadline:
            await asyncio.sleep(0.05)
            yaw = self._node.get_yaw()
            if yaw is None:
                continue
            # Compute the shortest angular distance traveled
            delta = _angle_diff(yaw, start_yaw)
            if abs(delta) >= abs(target_delta) - YAW_TOLERANCE_RAD:
                break
        await self.halt()

    async def teleop_step(self, lin: float, turn: float) -> None:
        """Direct Twist: linear.x = lin * max, angular.z = turn * max."""
        vx = lin * TELEOP_MAX_LIN
        wz = turn * TELEOP_MAX_ANG
        await self._publish_cmd_vel(vx, wz)

    async def halt(self) -> None:
        await self._publish_cmd_vel(0.0, 0.0)

    async def set_status(self, kind: str, **kw) -> None:
        if kind == "led":
            self._node.set_led(kw.get("r", 0), kw.get("g", 0), kw.get("b", 0))
        elif kind == "audio":
            self._node.play_beep(kw.get("beep_type", "found"))
        elif kind == "dock":
            self._node.dock()
        elif kind == "undock":
            self._node.undock()
        elif kind == "get_battery":
            pass  # battery is pushed via /battery_state subscription
        elif kind == "reset_yaw":
            pass  # Create 3 has real yaw — no-op
        else:
            logger.debug("Create 3: unknown status kind: %s", kind)

    async def get_battery(self) -> BatteryState:
        if self._node:
            return self._node.get_battery()
        return BatteryState()

    async def get_pose(self) -> RobotPose:
        if self._node:
            pose = self._node.get_pose()
            yaw = self._node.get_yaw()
            if pose:
                return RobotPose(
                    x=pose.x, y=pose.y,
                    heading_deg=math.degrees(yaw) if yaw is not None else None,
                )
        return RobotPose(heading_deg=None)

    # ── Internal helpers ────────────────────────────────────────────────

    async def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if self._node:
            self._node.publish_cmd_vel(linear_x, angular_z)

    def _on_frame(self, img: Image.Image) -> None:
        self._latest_frame = img
        if self._on_frame_cb:
            self._on_frame_cb(img)

    def _on_ros_hazard(self, event: HazardEvent) -> None:
        """Called from the rclpy thread — forward to the agent via the
        transport's hazard sink.  Thread-safe: just sets the event, the
        agent reads it on the next tick."""
        self.emit_hazard(event)

    def _on_ros_battery(self, battery: BatteryState) -> None:
        """Called from the rclpy thread — cache for get_battery()."""
        # Could also forward to the panel via a callback


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a - b, normalized to [-pi, pi]."""
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


# ── rclpy node (runs in a background thread) ────────────────────────────

class _Create3Node:
    """rclpy node for Create 3 ROS 2 communication.

    Runs in a background thread with a SingleThreadedExecutor.  All sensor
    data is cached in thread-safe variables (GIL-protected @Volatile reads).
    Commands are published directly (rclpy publish is thread-safe).
    """

    def __init__(
        self,
        ros_domain_id: int,
        on_hazard: callable,
        on_battery: callable,
    ):
        import os
        os.environ.setdefault("ROS_DOMAIN_ID", str(ros_domain_id))

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._executor = SingleThreadedExecutor()

        self._node = rclpy.create_node("derpbot_create3_bridge")
        self._on_hazard = on_hazard
        self._on_battery = on_battery

        # Cached sensor state (written by callbacks, read by transport)
        self._pose_x: float = 0.0
        self._pose_y: float = 0.0
        self._yaw: Optional[float] = None
        self._battery: BatteryState = BatteryState()

        # Publishers
        from geometry_msgs.msg import Twist
        self._Twist = Twist
        self._cmd_vel_pub = self._node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # Subscribers — QoS matches Create 3 sensor publishers (BEST_EFFORT)
        from sensor_msgs.msg import Imu, BatteryState as BatteryMsg
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._node.create_subscription(Imu, IMU_TOPIC, self._imu_cb, sensor_qos)
        self._node.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, sensor_qos)
        self._node.create_subscription(BatteryMsg, BATTERY_TOPIC, self._battery_cb, sensor_qos)

        # Hazard detection — Create 3 publishes HazardDetectionVector on
        # /hazard_detection (Jazzy uses the Vector type, not Stamped)
        try:
            from irobot_create_msgs.msg import HazardDetectionVector
            self._node.create_subscription(
                HazardDetectionVector, HAZARD_TOPIC, self._hazard_cb, sensor_qos)
        except ImportError:
            logger.warning("irobot_create_msgs not available — hazard detection disabled")

        # LED + Audio publishers (lazy: msg types imported on first use)
        self._lightring_pub = None
        self._audio_pub = None
        try:
            from irobot_create_msgs.msg import LightringLeds, AudioNoteVector
            self._lightring_pub = self._node.create_publisher(LightringLeds, CMD_LIGHTRING_TOPIC, 10)
            self._audio_pub = self._node.create_publisher(AudioNoteVector, CMD_AUDIO_TOPIC, 10)
        except ImportError:
            logger.warning("irobot_create_msgs not available — LED + audio disabled")

        self._running = False

    def spin_forever(self) -> None:
        self._running = True
        self._executor.add_node(self._node)
        try:
            while self._running:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            logger.error("rclpy spin error: %s", e)
        finally:
            try:
                self._node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass

    def shutdown(self) -> None:
        self._running = False

    # ── Publishers ──────────────────────────────────────────────────────

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = self._Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)

    def set_led(self, r: int, g: int, b: int) -> None:
        if not self._lightring_pub:
            return
        try:
            from irobot_create_msgs.msg import LightringLeds, LedColor
            msg = LightringLeds()
            msg.override_system = True
            color = LedColor(red=min(255, max(0, r)),
                             green=min(255, max(0, g)),
                             blue=min(255, max(0, b)))
            msg.leds = [color] * 6  # all 6 LEDs the same color
            self._lightring_pub.publish(msg)
        except Exception as e:
            logger.warning("LED publish failed: %s", e)

    def play_beep(self, beep_type: str = "found") -> None:
        if not self._audio_pub:
            return
        try:
            from irobot_create_msgs.msg import AudioNoteVector, AudioNote
            from builtin_interfaces.msg import Duration
            msg = AudioNoteVector()
            msg.append = False

            # Define note sequences (frequency Hz, duration ms)
            if beep_type == "found":
                notes = [(523, 100), (659, 100), (784, 200)]  # C-E-G ascending
            elif beep_type == "bump":
                notes = [(200, 150), (150, 200)]  # descending buzz
            elif beep_type == "error":
                notes = [(150, 300)]  # harsh low
            else:
                notes = [(440, 100)]  # default beep

            for freq, ms in notes:
                note = AudioNote()
                note.frequency = freq
                note.max_runtime = Duration(sec=0, nanosec=ms * 1_000_000)
                msg.notes.append(note)

            self._audio_pub.publish(msg)
        except Exception as e:
            logger.warning("Audio publish failed: %s", e)

    def dock(self) -> None:
        try:
            from irobot_create_msgs.action import Dock
            self._node.get_logger().info("Dock action requested (not yet implemented)")
            # TODO: action client for Dock — needs rclpy action client in the thread
        except ImportError:
            pass

    def undock(self) -> None:
        try:
            from irobot_create_msgs.action import Undock
            self._node.get_logger().info("Undock action requested (not yet implemented)")
            # TODO: action client for Undock
        except ImportError:
            pass

    # ── Sensor state accessors ──────────────────────────────────────────

    def get_pose(self) -> Optional[RobotPose]:
        if self._yaw is None:
            return None
        return RobotPose(x=self._pose_x, y=self._pose_y, heading_deg=math.degrees(self._yaw))

    def get_yaw(self) -> Optional[float]:
        return self._yaw

    def get_battery(self) -> BatteryState:
        return self._battery

    # ── Subscription callbacks (called by rclpy executor) ───────────────

    def _imu_cb(self, msg) -> None:
        # Extract yaw from quaternion
        q = msg.orientation
        # quaternion (x, y, z, w) → yaw (rotation about z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y

    def _battery_cb(self, msg) -> None:
        pct = int(msg.percentage * 100) if hasattr(msg, 'percentage') else 0
        self._battery = BatteryState(pct=pct, voltage_v=msg.voltage if hasattr(msg, 'voltage') else None)
        self._on_battery(self._battery)

    def _hazard_cb(self, msg) -> None:
        # HazardDetectionVector has a .detections list (each with .type)
        # On older firmware it might be a single detection — handle both
        detections = msg.detections if hasattr(msg, 'detections') else [msg]
        for det in detections:
            kind_num = det.type
            kind = _HAZARD_KINDS.get(kind_num, "unknown")
            event = HazardEvent(kind=kind, ts=time.time())
            logger.info("Create 3 hazard: kind=%s (type=%d)", kind, kind_num)
            self._on_hazard(event)