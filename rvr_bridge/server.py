"""WebSocket server for phone-as-BLE-shell relay (#21).

Runs on the computer. The phone connects, streams camera frames and IMU data,
and receives motor commands. The server exposes callbacks for the agent loop
to send commands and receive sensor data.
"""

from __future__ import annotations

import asyncio
import base64
import io
import logging
import time
from typing import Callable, Optional

import websockets
from PIL import Image

from websockets.protocol import State as WsState

from .protocol import (
    BatteryMessage,
    BleStateMessage,
    CaptureFrameMessage,
    DriveMessage,
    FrameMessage,
    GetBatteryMessage,
    ImuMessage,
    RawMotorsMessage,
    ResetYawMessage,
    SleepMessage,
    StopMessage,
    WakeMessage,
    decode,
    encode,
)

logger = logging.getLogger(__name__)


class PhoneRelay:
    """Manages the WebSocket connection to the phone.

    Usage:
        relay = PhoneRelay(host="0.0.0.0", port=8765)
        relay.on_frame = lambda img: ...
        relay.on_imu = lambda imu: ...
        relay.on_ble_state = lambda state: ...
        relay.on_battery = lambda pct: ...
        await relay.start()

        # Later, from the agent loop:
        await relay.send(DriveMessage(speed=64, heading=90))
        img = await relay.capture_frame()
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self._ws: Optional[websockets.asyncio.server.ServerConnection] = None
        self._server: Optional[websockets.asyncio.server.Server] = None

        self.on_frame: Optional[Callable[[Image.Image], None]] = None
        self.on_imu: Optional[Callable[[ImuMessage], None]] = None
        self.on_ble_state: Optional[Callable[[BleStateMessage], None]] = None
        self.on_battery: Optional[Callable[[BatteryMessage], None]] = None

        self._pending_frame_future: Optional[asyncio.Future] = None
        self._latest_imu: Optional[ImuMessage] = None
        self._ble_state: str = "disconnected"

    @property
    def ble_state(self) -> str:
        return self._ble_state

    @property
    def phone_connected(self) -> bool:
        return self._ws is not None and self._ws.state == WsState.OPEN

    async def start(self) -> None:
        self._server = await websockets.serve(
            self._handler, self.host, self.port
        )
        logger.info("Phone relay listening on ws://%s:%d", self.host, self.port)

    async def stop(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        logger.info("Phone relay stopped")

    async def send(self, msg) -> None:
        if self._ws and not self._ws.closed:
            await self._ws.send(encode(msg))

    async def capture_frame(self) -> Optional[Image.Image]:
        """Request a frame from the phone and wait for the response."""
        if not self.phone_connected:
            return None
        loop = asyncio.get_event_loop()
        self._pending_frame_future = loop.create_future()
        await self.send(CaptureFrameMessage())
        try:
            return await asyncio.wait_for(self._pending_frame_future, timeout=10.0)
        except asyncio.TimeoutError:
            logger.warning("Frame capture timed out")
            return None
        finally:
            self._pending_frame_future = None

    async def _handler(self, websocket: websockets.asyncio.server.ServerConnection) -> None:
        logger.info("Phone connected from %s", websocket.remote_address)
        self._ws = websocket
        try:
            async for raw in websocket:
                try:
                    msg = decode(raw)
                except (ValueError, Exception) as e:
                    logger.warning("Failed to decode message: %s", e)
                    continue

                if isinstance(msg, FrameMessage):
                    img = self._decode_frame(msg)
                    if self._pending_frame_future and not self._pending_frame_future.done():
                        self._pending_frame_future.set_result(img)
                    if self.on_frame and img:
                        self.on_frame(img)
                elif isinstance(msg, ImuMessage):
                    self._latest_imu = msg
                    if self.on_imu:
                        self.on_imu(msg)
                elif isinstance(msg, BleStateMessage):
                    self._ble_state = msg.state
                    if self.on_ble_state:
                        self.on_ble_state(msg)
                    logger.info("BLE state: %s", msg.state)
                elif isinstance(msg, BatteryMessage):
                    if self.on_battery:
                        self.on_battery(msg)
                    logger.info("Battery: %d%%", msg.pct)
                else:
                    logger.debug("Unhandled message type: %s", type(msg).__name__)
        except websockets.ConnectionClosed:
            logger.info("Phone disconnected")
        except Exception as e:
            logger.error("Phone connection error: %s", e)
        finally:
            self._ws = None
            self._ble_state = "disconnected"

    @staticmethod
    def _decode_frame(msg: FrameMessage) -> Optional[Image.Image]:
        try:
            data = base64.b64decode(msg.jpeg_b64)
            img = Image.open(io.BytesIO(data))
            if msg.rotation and msg.rotation != 0:
                from PIL import Image as PILImage
                img = img.rotate(-msg.rotation, expand=True)
            return img
        except Exception as e:
            logger.error("Failed to decode frame: %s", e)
            return None