"""RVR real-robot agent (#21, refactored #25).

Slim subclass of ``BaseRealAgent`` with RVR-specific overrides:
- BLE connection wait (polls ``GetBleStateMessage`` until ``ready``)
- Wake + reset_yaw on connect
- IMU/battery/phone-battery event forwarding to the panel
- Torch + beep via the phone relay (not the transport's ``set_status``)
- Battery polling (RVR battery is pull, not push)

All domain logic (VLM loop, teleop, bump recovery) lives in the base class.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional

from PIL import Image

from robot_agent.base_agent import BaseRealAgent
from .protocol import (GetBatteryMessage, GetBleStateMessage,
                       GetPhoneBatteryMessage, ResetYawMessage,
                       WakeMessage)
from .transport import RvrTransport

logger = logging.getLogger(__name__)

BUMP_REVERSE_MS = 1500
BUMP_TURN_DEG = 90
TELEOP_TURN_DEG_PER_TICK = 5


class RvrAgent(BaseRealAgent):
    """RVR-specific real-robot agent."""

    def __init__(self, config, transport: RvrTransport):
        super().__init__(config, transport)
        self.rvr_transport = transport

        # RVR uses a dead-reckoned heading counter (no real odometry)
        self._desired_heading: Optional[int] = 0
        self._last_battery_poll: float = 0.0

        # Wire transport IMU callback → agent panel event
        self.rvr_transport._imu_cb = self._on_imu

    def _is_connected(self) -> bool:
        # For RVR, "connected" means the phone is connected AND BLE is ready
        # (or teleop-only where camera works without BLE)
        if self.rvr_transport.relay.phone_connected:
            if self._teleop_only:
                return True
            return self.rvr_transport.relay.ble_state == "ready"
        return False

    async def _on_connected(self) -> None:
        """RVR: wait for BLE ready, then wake + reset_yaw."""
        if self._teleop_only:
            logger.info("Teleop-only: skipping BLE wait (camera-only ok)")
            return

        logger.info("Phone connected. Waiting for BLE ready...")
        while self._running and self.rvr_transport.relay.ble_state != "ready":
            await self.rvr_transport.relay.send(GetBleStateMessage())
            await asyncio.sleep(0.5)

        if not self._running:
            return

        if self.rvr_transport.relay.ble_state == "ready":
            logger.info("BLE ready. Waking RVR and zeroing heading.")
            await self.rvr_transport.relay.send(WakeMessage())
            await asyncio.sleep(3.0)
            await self.rvr_transport.relay.send(ResetYawMessage())
            await asyncio.sleep(0.5)
        else:
            logger.info("BLE not ready (RVR unavailable). Camera-only mode.")

    async def _poll_battery(self) -> None:
        """Request battery % from the phone every 60 s.

        The phone only sends ``battery`` messages in response to
        ``get_battery`` commands.
        """
        now = time.monotonic()
        if now - self._last_battery_poll < 60.0:
            return
        self._last_battery_poll = now
        await self.rvr_transport.relay.send(GetBatteryMessage())
        await self.rvr_transport.relay.send(GetPhoneBatteryMessage())

    def _on_imu(self, msg) -> None:
        """Forward IMU data to the panel."""
        if self.on_imu_event:
            self.on_imu_event({
                "accel": list(msg.accel),
                "gyro": list(msg.gyro),
                "ts": msg.ts,
            })

    async def beep(self, beep_type: str = "found", **kw) -> None:
        """RVR beeps via the phone (BeepMessage), not the transport."""
        from .protocol import BeepMessage
        await self.rvr_transport.relay.send(BeepMessage(
            beep_type=beep_type, volume=kw.get("volume", 80)))

    async def set_torch(self, on: bool) -> None:
        """RVR torch via the phone (TorchMessage)."""
        from .protocol import TorchMessage
        await self.rvr_transport.relay.send(TorchMessage(on=on))

    def _state_snapshot(self) -> dict:
        s = super()._state_snapshot()
        s["ble_state"] = self.rvr_transport.relay.ble_state
        s["phone_connected"] = self.rvr_transport.relay.phone_connected
        return s