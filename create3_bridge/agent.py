"""Create 3 real-robot agent (#25).

Slim subclass of ``BaseRealAgent`` with Create 3-specific overrides:
- Odom-based heading: no dead-reckoned counter (transport has /imu yaw)
- LED ring: autonomous colour signals (green=auto, blue=teleop, red=bump, yellow=search)
- Beep via ROS /cmd_audio (not the phone)
- Battery pushed via /battery_state subscription (no polling needed)
- Hazard events from /hazard_detection (bump, cliff, stall, ...)

All domain logic (VLM loop, teleop, bump recovery) lives in the base class.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Optional

from PIL import Image

from robot_agent.base_agent import BaseRealAgent
from robot_agent.transport import HazardEvent
from .transport import Create3Transport

logger = logging.getLogger(__name__)


class Create3Agent(BaseRealAgent):
    """Create 3-specific real-robot agent."""

    def __init__(self, config, transport: Create3Transport):
        super().__init__(config, transport)
        self.create3_transport = transport

        # Create 3 has real /imu yaw — no dead-reckoned heading counter
        self._desired_heading: Optional[int] = None

        # Wire transport frame callback → agent panel event
        self.create3_transport._on_frame_cb = self._on_frame

        # LED colour tracking
        self._current_led: str = "off"

    def _is_connected(self) -> bool:
        # For Create 3, "connected" means the transport (rclpy) is up
        # AND the phone relay (camera) is connected
        return self.create3_transport.connection_state == "ready"

    async def _on_connected(self) -> None:
        """Create 3: set LED to teleop colour (blue) on connect."""
        await self._set_led_blue()
        logger.info("Create 3 connected. LED → blue (teleop).")

    async def _on_arrived(self) -> None:
        """Override: flash green + beep via /cmd_audio."""
        await self._set_led_green()
        await self.beep("found")
        await self.transport.halt()
        logger.info("ARRIVED at '%s' (confirmed)", self.config.target)
        self._log_entry({"event": "arrived", "target": self.config.target})

    async def _execute_drive(self, distance_m: float, turn_angle_deg: int = 0) -> None:
        """Create 3: rotate first (real /imu yaw), then drive linear (/odom)."""
        if turn_angle_deg != 0:
            await self.transport.rotate(turn_angle_deg, timeout_s=10.0)

        if distance_m > 0.0:
            await self._set_led_yellow()  # searching/driving
            await self.transport.move_linear(distance_m, timeout_s=8.0)
            await self._set_led_green()  # back to auto
        else:
            await self.transport.halt()

    def _apply_heading_delta(self, turn_angle_deg: int) -> None:
        """No-op for Create 3 — the transport tracks yaw via /imu.
        The heading delta is applied during _execute_drive → transport.rotate."""
        pass

    async def _handle_bump_recovery(self) -> None:
        """Override: flash red, beep bump, reverse via /cmd_vel, halt."""
        logger.info("Create 3 hazard — emergency stop + recovery")
        await self._set_led_red()
        await self.beep("bump")
        await self.transport.halt()
        # Reverse via direct cmd_vel
        await self.transport.teleop_step(-0.3, 0.0)
        await asyncio.sleep(1.5)
        await self.transport.halt()
        # Turn 90° using real /imu
        await self.transport.rotate(90.0, timeout_s=5.0)
        self._hazard_event = None
        await self._set_led_green()

    async def beep(self, beep_type: str = "found", **kw) -> None:
        """Create 3 beeps via /cmd_audio (ROS), not the phone."""
        await self.transport.set_status("audio", beep_type=beep_type)

    async def _poll_battery(self) -> None:
        """No-op for Create 3 — /battery_state subscription pushes updates.
        The transport caches the latest battery state; the panel reads it
        via get_battery() on demand."""
        pass

    def _on_frame(self, img: Image.Image) -> None:
        """Forward phone relay frames to the panel."""
        self._latest_frame = img
        self._emit_frame(img)

    # ── LED helpers ─────────────────────────────────────────────────────

    async def _set_led_green(self) -> None:
        await self.transport.set_status("led", r=0, g=255, b=0)
        self._current_led = "green"

    async def _set_led_blue(self) -> None:
        await self.transport.set_status("led", r=0, g=0, b=255)
        self._current_led = "blue"

    async def _set_led_red(self) -> None:
        await self.transport.set_status("led", r=255, g=0, b=0)
        self._current_led = "red"

    async def _set_led_yellow(self) -> None:
        await self.transport.set_status("led", r=255, g=200, b=0)
        self._current_led = "yellow"

    def _state_snapshot(self) -> dict:
        s = super()._state_snapshot()
        s["led_color"] = self._current_led
        return s