"""RVR transport: Sphero RVR via phone-as-BLE-shell (#25).

Wraps ``PhoneRelay`` (the WebSocket server the Android app connects to) and
implements the ``RobotTransport`` contract using RVR BLE protocol messages
(``DriveMessage``, ``RawMotorsMessage``, ``StopMessage``).

RVR-specific drive mechanics:
- **Linear drive**: ``DriveMessage(speed_byte, heading, flags)`` at a fixed
  speed byte; distance is timed from ``speed_mps``.  The RVR's firmware
  handles closed-loop heading hold.
- **Rotate**: ``RawMotorsMessage`` with opposite wheel directions for true
  in-place pivot.  ``driveWithHeading`` drives one track forward while the
  other drags — not a pivot.  Speed must be high enough to overcome
  wheel-scrub friction (64 stalls, 100+ works).
- **Heading**: dead-reckoned byte counter (``_desired_heading``), incremented
  by the agent.  No real odometry — ``has_real_heading = False``.
- **Bump**: IMU spike detector (``BumpDetector``) over the phone's
  accelerometer stream — no discrete hazard sensor.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional

from PIL import Image

from .bump_detect import BumpDetector
from .protocol import (DriveMessage, GetBatteryMessage, GetBleStateMessage,
                       GetPhoneBatteryMessage, RawMotorsMessage, ResetYawMessage,
                       SleepMessage, StopMessage, WakeMessage,
                       TorchMessage, BeepMessage)
from .server import PhoneRelay
from robot_agent.transport import BatteryState, HazardEvent, RobotPose, RobotTransport

logger = logging.getLogger(__name__)

DRIVE_FLAGS_FORWARD = 0x00
DRIVE_FLAGS_REVERSE = 0x01

TURN_SPEED = 100         # raw_motors pivot speed — 64 stalls, 100+ overcomes scrub
DRIVE_SPEED_BYTE = 64
SPEED_MPS = 0.35
MAX_DRIVE_MS = 4000
BUMP_REVERSE_MS = 1500
BUMP_TURN_DEG = 90
TELEOP_TURN_DEG_PER_TICK = 5  # ≈100°/s at 20 Hz tick rate


class RvrTransport(RobotTransport):
    """Sphero RVR transport via the phone-as-BLE-shell relay."""

    backend_name = "rvr"
    capabilities = ["wake_sleep", "reset_yaw", "torch"]

    def __init__(
        self,
        relay: PhoneRelay,
        *,
        drive_speed_byte: int = DRIVE_SPEED_BYTE,
        speed_mps: float = SPEED_MPS,
        max_drive_ms: int = MAX_DRIVE_MS,
        bump_threshold_factor: float = 2.5,
    ):
        self.relay = relay
        self.drive_speed_byte = drive_speed_byte
        self.speed_mps = speed_mps
        self.max_drive_ms = max_drive_ms
        self.bump_detector = BumpDetector(threshold_factor=bump_threshold_factor)
        self._bump_enabled = True

        # Wire relay callbacks
        self.relay.on_imu = self._on_imu
        self.relay.on_ble_state = self._on_ble_state
        self.relay.on_battery = self._on_battery
        self.relay.on_phone_battery = self._on_phone_battery
        self.relay.on_frame = self._on_frame

        # Battery state cache (updated by relay callbacks)
        self._rvr_battery: Optional[BatteryState] = None
        self._phone_battery_pct: Optional[int] = None

        # Latest frame cache (push-mode)
        self._latest_frame: Optional[Image.Image] = None
        self._on_frame_cb: Optional[callable] = None  # type: ignore[assignment]

    @property
    def connection_state(self) -> str:
        return self.relay.ble_state

    @property
    def has_real_heading(self) -> bool:
        return False

    @property
    def has_hazard_sensor(self) -> bool:
        return False

    async def start(self) -> None:
        await self.relay.start()

    async def stop(self) -> None:
        await self.relay.stop()

    async def capture_frame(self) -> Optional[Image.Image]:
        return await self.relay.capture_frame()

    async def move_linear(self, distance_m: float, *, timeout_s: float) -> None:
        """Drive straight by ``distance_m`` (forward positive).  RVR uses
        a fixed speed byte + timed duration; distance is approximate."""
        if distance_m <= 0.0:
            await self.halt()
            return

        duration_ms = min(
            max(int(distance_m / self.speed_mps * 1000), 200),
            self.max_drive_ms,
        )
        await self.relay.send(DriveMessage(
            speed=self.drive_speed_byte,
            heading=0,  # heading is managed by the agent's counter
            flags=DRIVE_FLAGS_FORWARD if distance_m > 0 else DRIVE_FLAGS_REVERSE,
        ))

        step_ms = 50
        elapsed_ms = 0
        while elapsed_ms < duration_ms:
            await asyncio.sleep(step_ms / 1000.0)
            elapsed_ms += step_ms
            # Bump check handled by the agent via _hazard_event

        await self.relay.send(StopMessage(heading=0))

    async def rotate(self, angle_deg: float, *, timeout_s: float) -> None:
        """In-place pivot via raw_motors with opposite wheel directions.
        ``angle_deg`` positive = left/CCW.  Timed (no real odometry)."""
        if abs(angle_deg) < 1.0:
            await self.halt()
            return

        # Rough timing: TURN_SPEED at ~100°/s for pivot
        duration_s = min(abs(angle_deg) / 100.0, timeout_s)
        turn_speed = TURN_SPEED
        if angle_deg > 0:
            # Turn left: left reverse, right forward
            await self.relay.send(RawMotorsMessage(
                l_mode=2, l_speed=turn_speed, r_mode=1, r_speed=turn_speed))
        else:
            # Turn right: left forward, right reverse
            await self.relay.send(RawMotorsMessage(
                l_mode=1, l_speed=turn_speed, r_mode=2, r_speed=turn_speed))

        await asyncio.sleep(duration_s)
        await self.relay.send(StopMessage(heading=0))

    async def teleop_step(self, lin: float, turn: float) -> None:
        """One tick of teleop drive (~20 Hz).  Both zero → halt.

        Turn: raw_motors opposite wheels for true pivot.
        Linear: driveWithHeading at proportional speed.
        """
        if abs(turn) > 0.05 and abs(lin) < 0.05:
            turn_speed = max(int(abs(turn) * 100), 80)
            if turn > 0:
                await self.relay.send(RawMotorsMessage(
                    l_mode=2, l_speed=turn_speed, r_mode=1, r_speed=turn_speed))
            else:
                await self.relay.send(RawMotorsMessage(
                    l_mode=1, l_speed=turn_speed, r_mode=2, r_speed=turn_speed))
            return

        if abs(lin) > 0.05:
            speed = max(int(abs(lin) * self.drive_speed_byte), 1)
            flags = DRIVE_FLAGS_REVERSE if lin < 0 else DRIVE_FLAGS_FORWARD
            await self.relay.send(DriveMessage(
                speed=speed, heading=0, flags=flags))
            return

        await self.relay.send(StopMessage(heading=0))

    async def halt(self) -> None:
        await self.relay.send(StopMessage(heading=0))

    async def set_status(self, kind: str, **kw) -> None:
        if kind == "torch":
            await self.relay.send(TorchMessage(on=kw.get("on", False)))
        elif kind == "audio":
            # RVR beeps via the phone (BeepMessage)
            await self.relay.send(BeepMessage(
                beep_type=kw.get("beep_type", "found"),
                volume=kw.get("volume", 80)))
        elif kind == "wake":
            await self.relay.send(WakeMessage())
        elif kind == "sleep":
            await self.relay.send(SleepMessage())
        elif kind == "reset_yaw":
            await self.relay.send(ResetYawMessage())
        elif kind == "get_battery":
            await self.relay.send(GetBatteryMessage())
            await self.relay.send(GetPhoneBatteryMessage())
        else:
            logger.debug("RVR: unknown status kind: %s", kind)

    async def get_battery(self) -> BatteryState:
        if self._rvr_battery:
            return self._rvr_battery
        await self.relay.send(GetBatteryMessage())
        # Callback will fill _rvr_battery; return empty for now
        return BatteryState()

    async def get_pose(self) -> RobotPose:
        # RVR has no real odometry — heading is dead-reckoned by the agent
        return RobotPose(heading_deg=None)

    def set_bump_enabled(self, enabled: bool) -> None:
        self._bump_enabled = enabled

    # ── Relay callbacks ─────────────────────────────────────────────────

    def _on_imu(self, msg) -> None:
        # Forward to agent's IMU event callback (for panel display)
        if hasattr(self, '_imu_cb') and self._imu_cb:
            self._imu_cb(msg)
        if not self._bump_enabled:
            return
        event = self.bump_detector.feed(msg)
        if event:
            logger.info("RVR bump detected: mag=%.1f", event.magnitude)
            self.emit_hazard(HazardEvent(
                kind="bump", magnitude=event.magnitude, ts=event.timestamp))

    def _on_ble_state(self, msg) -> None:
        logger.info("BLE state: %s", msg.state)

    def _on_battery(self, msg) -> None:
        logger.info("RVR battery: %d%%", msg.pct)
        self._rvr_battery = BatteryState(pct=msg.pct)

    def _on_phone_battery(self, msg) -> None:
        logger.info("Phone battery: %d%%", msg.pct)
        self._phone_battery_pct = msg.pct

    def _on_frame(self, img: Image.Image) -> None:
        self._latest_frame = img
        if self._on_frame_cb:
            self._on_frame_cb(img)