"""RVR transport: Sphero RVR via phone-as-BLE-shell (#25).

Wraps ``PhoneRelay`` (the WebSocket server the Android app connects to) and
implements the ``RobotTransport`` contract using RVR BLE protocol messages
(``DriveMessage``, ``RawMotorsMessage``, ``StopMessage``).

RVR-specific drive mechanics:
- **Linear drive**: ``DriveMessage(speed_byte, heading, flags)`` at a fixed
  speed byte; distance is timed from ``speed_mps``.  The RVR's firmware
  handles closed-loop heading hold.
- **Rotate**: ``driveWithHeading(speed=0)`` by default (firmware yaw
  controller, gentle).  Optional ``raw_motors`` pivot mode for faster
  turns when ``rotate_speed > 0`` — speed must be ≥100 to overcome
  wheel-scrub friction.
- **Heading**: dead-reckoned byte counter (``_desired_heading``), incremented
  by the agent.  No real odometry — ``has_real_heading = False``.
- **Bump**: IMU spike detector (``BumpDetector``) over the phone's
  accelerometer stream — no discrete hazard sensor.
"""

from __future__ import annotations

import asyncio
import logging
import math
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
ROTATE_SPEED = 0            # 0 = driveWithHeading (firmware yaw), >0 = raw_motors pivot speed


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
        teleop_turn_deg_per_tick: int = TELEOP_TURN_DEG_PER_TICK,
        rotate_speed: int = ROTATE_SPEED,
    ):
        self.relay = relay
        self.drive_speed_byte = drive_speed_byte
        self.speed_mps = speed_mps
        self.max_drive_ms = max_drive_ms
        self.teleop_turn_deg_per_tick = teleop_turn_deg_per_tick
        self.rotate_speed = rotate_speed
        self.bump_detector = BumpDetector(threshold_factor=bump_threshold_factor)
        self._bump_enabled = True

        # Wire relay callbacks
        self.relay.on_imu = self._on_imu
        self.relay.on_ble_state = self._on_ble_state
        self.relay.on_battery = self._on_battery
        self.relay.on_phone_battery = self._on_phone_battery
        self.relay.on_frame = self._on_frame
        self.relay.on_phone_connect = self._on_phone_connect
        self.relay.on_phone_disconnect = self._on_phone_disconnect

        # Battery state cache (updated by relay callbacks)
        self._rvr_battery: Optional[BatteryState] = None
        self._phone_battery_pct: Optional[int] = None

        # Latest frame cache (push-mode)
        self._latest_frame: Optional[Image.Image] = None
        self._on_frame_cb: Optional[callable] = None  # type: ignore[assignment]

        # Last stop heading — dedupes redundant StopMessage sends (the teleop
        # idle tick fires ~20 Hz; without this the phone log fills with
        # "STOP hdg=0". The RVR firmware holds heading after a single stop, so
        # repeating an identical stop is pure noise.
        self._last_stop_heading: Optional[int] = None

    @property
    def connection_state(self) -> str:
        return self.relay.ble_state

    @property
    def has_real_heading(self) -> bool:
        return False

    @property
    def has_hazard_sensor(self) -> bool:
        return False

    # ── Heading helpers (dead-reckoned counter lives on the agent) ──────

    def _agent_heading(self) -> int:
        """Read the agent's dead-reckoned heading counter."""
        a = getattr(self, "_agent", None)
        if a is not None and a._desired_heading is not None:
            return a._desired_heading
        return 0

    def _agent_set_heading(self, h: int) -> int:
        """Write back the heading counter and return it."""
        a = getattr(self, "_agent", None)
        if a is not None and a._desired_heading is not None:
            a._desired_heading = h
        return h

    @staticmethod
    def _norm_heading(h: int) -> int:
        return h % 360

    async def _send_stop(self, heading: int) -> None:
        """Send a StopMessage, deduping identical consecutive stops.

        The RVR firmware's yaw-hold keeps the chassis at the last commanded
        heading after a single stop, so repeating ``StopMessage(h)`` with the
        same ``h`` is redundant. The teleop idle tick (20 Hz) would otherwise
        flood the phone log with "STOP hdg=0". A drive/rotate command clears
        the dedup so the first stop after motion always sends.
        """
        if self._last_stop_heading == heading:
            return
        self._last_stop_heading = heading
        await self.relay.send(StopMessage(heading=heading))

    def _clear_stop_dedup(self) -> None:
        """Invalidate the stop-dedup so the next stop always sends."""
        self._last_stop_heading = None

    async def start(self) -> None:
        await self.relay.start()

    async def stop(self) -> None:
        await self.relay.stop()

    async def capture_frame(self) -> Optional[Image.Image]:
        return await self.relay.capture_frame()

    async def wait_standstill(
        self,
        *,
        timeout_s: float = 3.0,
        gyro_threshold: float = 0.15,
        settle_s: float = 0.15,
    ) -> bool:
        """Wait until the robot's gyro indicates it has stopped moving.

        Uses the phone IMU gyroscope (rad/s). Returns True if standstill
        was detected within ``timeout_s``, False on timeout. ``settle_s``
        is how long the gyro must stay below threshold before we consider
        the robot truly stopped (avoids transient zeros mid-motion).
        """
        deadline = time.monotonic() + timeout_s
        still_since: Optional[float] = None
        while time.monotonic() < deadline:
            imu = self.relay._latest_imu
            if imu is not None:
                gyro_mag = math.sqrt(sum(g * g for g in imu.gyro))
                if gyro_mag < gyro_threshold:
                    if still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since >= settle_s:
                        return True
                else:
                    still_since = None
            await asyncio.sleep(0.03)
        logger.warning("wait_standstill timed out (%.1fs)", timeout_s)
        return False

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
        self._clear_stop_dedup()
        await self.relay.send(DriveMessage(
            speed=self.drive_speed_byte,
            heading=self._agent_heading(),
            flags=DRIVE_FLAGS_FORWARD if distance_m > 0 else DRIVE_FLAGS_REVERSE,
        ))

        step_ms = 50
        elapsed_ms = 0
        while elapsed_ms < duration_ms:
            await asyncio.sleep(step_ms / 1000.0)
            elapsed_ms += step_ms
            # Bump check handled by the agent via _hazard_event

        await self._send_stop(heading=0)

    async def rotate(self, angle_deg: float, *, timeout_s: float) -> None:
        """In-place turn.

        Two modes (``rotate_speed`` constructor arg):
        - ``0`` (default): ``driveWithHeading(speed=0)`` — RVR firmware
          closed-loop yaw controller.  Gentle, no snap-back, but rotation
          speed is not directly controllable.
        - ``>0``: ``raw_motors`` pivot at the given speed byte.  Direct
          wheel control — faster, more aggressive turns.  May snap back
          slightly on stop if the yaw-hold re-asserts.

        ``angle_deg`` positive = left/CCW.  The heading counter is already
        updated by ``_apply_heading_delta`` before this is called.
        """
        if abs(angle_deg) < 1.0:
            await self.halt()
            return

        if self.rotate_speed > 0:
            await self._rotate_raw(angle_deg, timeout_s)
        else:
            await self._rotate_heading(timeout_s)

    async def _rotate_heading(self, timeout_s: float) -> None:
        """Closed-loop rotate via firmware yaw controller."""
        heading = self._agent_heading()
        self._clear_stop_dedup()
        await self.relay.send(DriveMessage(
            speed=0, heading=heading, flags=DRIVE_FLAGS_FORWARD))
        await self.wait_standstill(timeout_s=timeout_s, settle_s=0.2)
        await self._send_stop(heading=heading)

    async def _rotate_raw(self, angle_deg: float, timeout_s: float) -> None:
        """Raw-motors pivot at ``rotate_speed``.  Timed turn — duration
        scales with angle magnitude and inversely with speed."""
        speed = self.rotate_speed
        # Empirical: ~700 ms per 90° at speed 100 → 7.8 ms/°
        duration_ms = max(int(abs(angle_deg) * 7800 / speed), 300)
        # positive = left/CCW: left wheels reverse, right wheels forward
        if angle_deg > 0:
            l_mode, r_mode = 2, 1   # reverse, forward
        else:
            l_mode, r_mode = 1, 2   # forward, reverse
        self._clear_stop_dedup()
        await self.relay.send(RawMotorsMessage(
            l_mode=l_mode, l_speed=speed,
            r_mode=r_mode, r_speed=speed))
        await asyncio.sleep(duration_ms / 1000.0)
        await self._send_stop(heading=self._agent_heading())

    async def teleop_step(self, lin: float, turn: float) -> None:
        """One tick of teleop drive (~20 Hz).  Both zero → halt.

        Uses ``driveWithHeading`` (closed-loop) for everything — the RVR's
        internal yaw-hold fights open-loop ``raw_motors``, causing the body
        to snap back when the key is released.  Turning is done by
        incrementing the dead-reckoned heading counter each tick and letting
        the firmware's controller rotate to it.
        """
        heading = self._agent_heading()

        if abs(turn) > 0.05:
            # Increment the heading target so the RVR's closed-loop controller
            # rotates to it.  teleop_turn_deg_per_tick at 20 Hz, scaled by turn.
            delta = int(self.teleop_turn_deg_per_tick * turn)
            heading = self._agent_set_heading(
                self._norm_heading(heading + delta))

        if abs(lin) > 0.05:
            speed = max(int(abs(lin) * self.drive_speed_byte), 1)
            flags = DRIVE_FLAGS_REVERSE if lin < 0 else DRIVE_FLAGS_FORWARD
            self._clear_stop_dedup()
            await self.relay.send(DriveMessage(
                speed=speed, heading=heading, flags=flags))
            return

        if abs(turn) > 0.05:
            # Turn only: speed 0 holds the heading while rotating to it.
            self._clear_stop_dedup()
            await self.relay.send(DriveMessage(
                speed=0, heading=heading, flags=DRIVE_FLAGS_FORWARD))
            return

        await self._send_stop(heading=heading)

    async def halt(self) -> None:
        await self._send_stop(heading=self._agent_heading())

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
        # Push to the agent's BLE event callback (→ panel) and emit a fresh
        # state snapshot so the panel reflects the new link state immediately.
        if hasattr(self, '_agent') and self._agent is not None:
            self._agent.on_ble_event({'state': msg.state})
            self._agent._emit_state()
            self._agent._log_entry({"event": "ble_state", "state": msg.state})

    def _on_battery(self, msg) -> None:
        logger.info("RVR battery: %d%%", msg.pct)
        self._rvr_battery = BatteryState(pct=msg.pct)
        if hasattr(self, '_agent') and self._agent is not None:
            self._agent.on_battery_event({'pct': msg.pct})

    def _on_phone_battery(self, msg) -> None:
        logger.info("Phone battery: %d%%", msg.pct)
        self._phone_battery_pct = msg.pct
        if hasattr(self, '_agent') and self._agent is not None:
            self._agent.on_phone_battery_event({'pct': msg.pct})

    def _on_phone_connect(self) -> None:
        if hasattr(self, '_agent') and self._agent is not None:
            self._agent._log_entry({"event": "phone_connect"})

    def _on_phone_disconnect(self) -> None:
        if hasattr(self, '_agent') and self._agent is not None:
            self._agent._log_entry({"event": "phone_disconnect"})

    def _on_frame(self, img: Image.Image) -> None:
        self._latest_frame = img
        if self._on_frame_cb:
            self._on_frame_cb(img)