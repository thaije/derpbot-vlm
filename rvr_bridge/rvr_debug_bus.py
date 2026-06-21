"""Debug bus WebSocket server for the RVR agent (#24).

Runs inside the agent's asyncio loop. Wires RvrAgent callbacks to a
WebSocket server that the panel process connects to as a client.

Wire protocol (bus → panel, JSON unless noted):
    hello, frame_meta, state, decision, verifier, imu, bump, ble, battery
    binary: JPEG frames (1-byte prefix 0x01)

Wire protocol (panel → bus, JSON):
    teleop {x, y}, stop, manual_query, toggle {which, value?},
    set_target {target, description?}, rvr {cmd}, get_state, get_frame

The bus holds zero domain logic — it translates between RvrAgent methods
and the wire protocol. All prompts/schema/verifier stay in shared/ + agent/.
"""

from __future__ import annotations

import asyncio
import io
import json
import logging
import time
from typing import Any, Optional

from PIL import Image
from websockets.asyncio.server import ServerConnection
from websockets.protocol import State as WsState
import websockets

logger = logging.getLogger(__name__)

FRAME_PREFIX = b"\x01"  # binary frame marker


class RvrDebugBus:
    """WebSocket server exposing RvrAgent state + commands to a panel client.

    Usage (inside agent.run(), after RvrAgent is constructed):
        bus = RvrDebugBus(agent, host="0.0.0.0", port=8770)
        await bus.start()
        # ... agent loop runs normally; bus pushes events as they arrive
        await bus.stop()
    """

    def __init__(self, agent, host: str = "0.0.0.0", port: int = 8770):
        self.agent = agent
        self.host = host
        self.port = port
        self._server: Optional[websockets.asyncio.server.Server] = None
        self._clients: set[ServerConnection] = set()
        self._frame_seq: int = 0
        self._last_frame_jpeg: Optional[bytes] = None
        self._last_state: Optional[dict] = None

    async def start(self) -> None:
        self._server = await websockets.serve(self._handler, self.host, self.port)
        self._wire_agent_callbacks()
        logger.info("Debug bus listening on ws://%s:%d", self.host, self.port)

    async def stop(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        logger.info("Debug bus stopped")

    def _wire_agent_callbacks(self) -> None:
        """Wire RvrAgent callback hooks to bus publish methods."""
        self.agent.on_frame = self._on_agent_frame
        self.agent.on_decision = self._on_agent_decision
        self.agent.on_verifier = self._on_agent_verifier
        self.agent.on_imu_event = self._on_agent_imu
        self.agent.on_bump_event = self._on_agent_bump
        self.agent.on_ble_event = self._on_agent_ble
        self.agent.on_battery_event = self._on_agent_battery
        self.agent.on_phone_battery_event = self._on_agent_phone_battery
        self.agent.on_state_change = self._on_agent_state

    # ── Agent callback → broadcast ──────────────────────────────────────

    def _on_agent_frame(self, img: Image.Image) -> None:
        jpeg = self._encode_jpeg(img)
        if jpeg is None:
            return
        self._last_frame_jpeg = jpeg
        self._frame_seq += 1
        meta = {
            "type": "frame_meta",
            "seq": self._frame_seq,
            "ts": time.time(),
        }
        asyncio.ensure_future(self._broadcast_json(meta))

        async def _send_binary():
            payload = FRAME_PREFIX + jpeg
            await self._broadcast_binary(payload)
        asyncio.ensure_future(_send_binary())

    def _on_agent_decision(self, data: dict) -> None:
        msg = {"type": "decision", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_verifier(self, data: dict) -> None:
        msg = {"type": "verifier", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_imu(self, data: dict) -> None:
        msg = {"type": "imu", **data}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_bump(self, data: dict) -> None:
        msg = {"type": "bump", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_ble(self, data: dict) -> None:
        msg = {"type": "ble", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_battery(self, data: dict) -> None:
        msg = {"type": "battery", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_phone_battery(self, data: dict) -> None:
        msg = {"type": "phone_battery", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    def _on_agent_state(self, data: dict) -> None:
        self._last_state = data
        msg = {"type": "state", **data, "ts": time.time()}
        asyncio.ensure_future(self._broadcast_json(msg))

    # ── WS handler (panel → bus commands) ───────────────────────────────

    async def _handler(self, ws: ServerConnection) -> None:
        self._clients.add(ws)
        logger.info("Panel client connected from %s", ws.remote_address)
        try:
            # Send hello + last known state/frame
            hello = {
                "type": "hello",
                "backend": "rvr",
                "capabilities": ["teleop", "manual_query", "stop",
                                 "set_target", "toggle", "rvr"],
                "teleop_schema": "normalized",
            }
            await ws.send(json.dumps(hello))
            if self._last_state:
                await ws.send(json.dumps({"type": "state", **self._last_state}))
            if self._last_frame_jpeg:
                await ws.send(FRAME_PREFIX + self._last_frame_jpeg)

            async for raw in ws:
                await self._handle_message(ws, raw)
        except websockets.ConnectionClosed:
            pass
        except Exception as e:
            logger.error("Panel client error: %s", e)
        finally:
            self._clients.discard(ws)
            logger.info("Panel client disconnected")

    async def _handle_message(self, ws: ServerConnection, raw: Any) -> None:
        if isinstance(raw, bytes):
            return  # browser shouldn't send binary; ignore

        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON from panel: %s", raw[:200])
            return

        mtype = msg.get("type", "")
        try:
            if mtype == "teleop":
                await self._cmd_teleop(msg)
            elif mtype == "stop":
                await self.agent.manual_stop()
            elif mtype == "manual_query":
                await self.agent.manual_query()
            elif mtype == "toggle":
                which = msg.get("which", "")
                value = msg.get("value")
                new = self.agent.toggle(which, value)
                await ws.send(json.dumps({"type": "toggle_ack", "which": which,
                                          "value": new}))
            elif mtype == "set_target":
                self.agent.set_target(
                    msg.get("target", ""),
                    msg.get("description", ""),
                )
            elif mtype == "rvr":
                await self._cmd_rvr(msg)
            elif mtype == "torch":
                await self.agent.set_torch(msg.get("on", False))
            elif mtype == "beep":
                await self.agent.beep(msg.get("beep_type", "found"),
                                      msg.get("volume", 80))
            elif mtype == "get_state":
                await ws.send(json.dumps({"type": "state",
                                          **self.agent._state_snapshot()}))
            elif mtype == "get_frame":
                if self._last_frame_jpeg:
                    await ws.send(FRAME_PREFIX + self._last_frame_jpeg)
            elif mtype == "ping":
                await ws.send(json.dumps({"type": "pong", "ts": time.time()}))
            else:
                logger.warning("Unknown panel command: %s", mtype)
        except Exception as e:
            logger.error("Error handling '%s': %s", mtype, e)

    async def _cmd_teleop(self, msg: dict) -> None:
        x = float(msg.get("x", 0.0))
        y = float(msg.get("y", 0.0))
        # x = turn (-1 left, +1 right), y = forward (-1 reverse, +1 forward)
        # RVR heading increases clockwise (right), so +x → +heading
        self.agent.teleop_drive(lin=y, turn=x)

    async def _cmd_rvr(self, msg: dict) -> None:
        from .protocol import (DriveMessage, ResetYawMessage, SleepMessage,
                               WakeMessage, GetBatteryMessage)
        cmd = msg.get("cmd", "")
        if cmd == "wake":
            await self.agent.relay.send(WakeMessage())
        elif cmd == "sleep":
            await self.agent.relay.send(SleepMessage())
        elif cmd == "reset_yaw":
            await self.agent.relay.send(ResetYawMessage())
            self.agent._desired_heading = 0
            self.agent._emit_state()
        elif cmd == "get_battery":
            await self.agent.relay.send(GetBatteryMessage())
        else:
            logger.warning("Unknown rvr cmd: %s", cmd)

    # ── Broadcast helpers ───────────────────────────────────────────────

    async def _broadcast_json(self, msg: dict) -> None:
        if not self._clients:
            return
        data = json.dumps(msg)
        dead = set()
        for ws in self._clients:
            if ws.state == WsState.OPEN:
                try:
                    await ws.send(data)
                except websockets.ConnectionClosed:
                    dead.add(ws)
            else:
                dead.add(ws)
        self._clients -= dead

    async def _broadcast_binary(self, data: bytes) -> None:
        if not self._clients:
            return
        dead = set()
        for ws in self._clients:
            if ws.state == WsState.OPEN:
                try:
                    await ws.send(data)
                except websockets.ConnectionClosed:
                    dead.add(ws)
            else:
                dead.add(ws)
        self._clients -= dead

    @staticmethod
    def _encode_jpeg(img: Image.Image, quality: int = 75) -> Optional[bytes]:
        try:
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=quality)
            return buf.getvalue()
        except Exception as e:
            logger.warning("JPEG encode failed: %s", e)
            return None