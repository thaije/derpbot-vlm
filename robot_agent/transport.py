"""Transport interface for real-robot backends (#25).

A ``RobotTransport`` decouples the domain logic in ``BaseRealAgent`` (VLM
loop, teleop state machine, bump-recovery protocol, panel hooks) from the
locomotion API of a specific robot.  Each backend implements the contract:

    RvrTransport       — Sphero RVR via phone-as-BLE-shell (speed byte + heading)
    Create3Transport   — iRobot Create 3 via ROS 2 /cmd_vel Twist + /odom

All methods are async; the agent's main thread stays asyncio.  Backends
that use a different concurrency model (e.g. rclpy) bridge into asyncio via
``asyncio.run_coroutine_threadsafe``.
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field
from typing import Optional

from PIL import Image


@dataclass
class RobotPose:
    """2D pose in the robot's odometry frame.

    ``heading_deg`` is None when the transport cannot report a real heading
    (e.g. RVR's dead-reckoned byte counter is exposed via ``BaseRealAgent``
    instead).  Create 3 fills it from ``/imu`` quaternion.
    """
    x: float = 0.0
    y: float = 0.0
    heading_deg: Optional[float] = None


@dataclass
class BatteryState:
    pct: int = 0
    voltage_v: Optional[float] = None
    ts: float = field(default_factory=time.time)


@dataclass
class HazardEvent:
    """A single hazard detection (bump, cliff, stall, ...).

    ``kind`` is the transport-normalised string; the agent uses it to pick
    a recovery profile.  RVR's IMU spike detector only emits ``bump``;
    Create 3's ``/hazard_detection`` can also emit ``cliff``/``stall``/...
    """
    kind: str = "bump"
    magnitude: float = 0.0
    ts: float = field(default_factory=time.time)


class RobotTransport(abc.ABC):
    """Contract between ``BaseRealAgent`` and a robot locomotion API.

    Lifecycle: ``start()`` → (capture/move/rotate/...) → ``stop()``.
    All methods are async; blocking hardware I/O must be offloaded.
    """

    backend_name: str = "?"

    @abc.abstractmethod
    async def start(self) -> None: ...

    @abc.abstractmethod
    async def stop(self) -> None: ...

    @abc.abstractmethod
    async def capture_frame(self) -> Optional[Image.Image]: ...

    @abc.abstractmethod
    async def move_linear(self, distance_m: float, *, timeout_s: float) -> None:
        """Drive straight by ``distance_m`` (forward positive). Blocks until
        done, bumped, or ``timeout_s`` elapses.  Implementations must
        publish halt on cancellation."""

    @abc.abstractmethod
    async def rotate(self, angle_deg: float, *, timeout_s: float) -> None:
        """Rotate in place by ``angle_deg`` (positive = left/CCW). Blocks
        until done, bumped, or ``timeout_s`` elapses."""

    @abc.abstractmethod
    async def teleop_step(self, lin: float, turn: float) -> None:
        """One tick of teleop drive. ``lin`` ∈ [-1, 1] forward, ``turn``
        ∈ [-1, 1] left.  Called at ~20 Hz; both zero → halt."""

    @abc.abstractmethod
    async def halt(self) -> None: ...

    @abc.abstractmethod
    async def set_status(self, kind: str, **kw) -> None:
        """Backend-specific status output: ``torch`` (RVR), ``led`` /
        ``audio`` / ``dock`` / ``undock`` (Create 3). Unknown kinds are
        no-ops."""

    @abc.abstractmethod
    async def get_battery(self) -> BatteryState: ...

    @abc.abstractmethod
    async def get_pose(self) -> RobotPose: ...

    @property
    @abc.abstractmethod
    def connection_state(self) -> str:
        """Human-readable state for the panel: ``ready`` when drivable,
        ``disconnected`` / ``scanning`` / ... otherwise."""

    @property
    @abc.abstractmethod
    def has_real_heading(self) -> bool:
        """True when ``get_pose().heading_deg`` is a real sensor reading
        (Create 3 /imu).  False when the agent must keep its own
        dead-reckoned heading counter (RVR)."""

    @property
    @abc.abstractmethod
    def has_hazard_sensor(self) -> bool:
        """True when the transport emits ``HazardEvent``s from a discrete
        sensor (Create 3 ``/hazard_detection``).  False when bump is
        inferred from an IMU spike detector (RVR)."""

    # Bump event sink — set by BaseRealAgent; transport pushes events here.
    # RvrTransport leaves this unset and the agent's BumpDetector feeds it
    # from the phone IMU stream; Create3Transport pushes /hazard_detection.
    on_hazard: Optional[callable] = None  # type: ignore[assignment]

    def emit_hazard(self, event: HazardEvent) -> None:
        if self.on_hazard is not None:
            self.on_hazard(event)