"""Panel proxy: connects to agent debug bus, fans out to N browsers (#24).

Architecture:
    Agent DebugBus ←WS→ PanelProxy ←WS→ N Browsers
                        ↑
                   static HTTP (index.html, separate port via stdlib http.server)

The proxy holds no domain logic. It:
  - Maintains one WS connection to the agent debug bus.
  - Caches the last frame + state so newly-connected browsers get instant data.
  - Fans out bus messages to all connected browsers.
  - Forwards browser commands upstream to the bus.
  - Serves the static HTML/CSS/JS via stdlib http.server on a separate port.

Two ports are used (WS_PORT = http_port, HTTP_PORT = http_port + 1) because
websockets v16 doesn't reliably serve HTTP responses (transport is aborted
before the response is flushed). The browser loads the HTML from the HTTP
port, which then connects to the WS port.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional, Set

import websockets
from websockets.asyncio.client import connect
from websockets.asyncio.server import ServerConnection
from websockets.protocol import State as WsState

logger = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_FRAME_PREFIX = b"\x01"


class PanelProxy:
    """Fan-out proxy between agent debug bus and browser clients."""

    def __init__(self, agent_url: str, http_host: str, http_port: int):
        self.agent_url = agent_url
        self.http_host = http_host
        self.http_port = http_port
        # WS on http_port, static HTTP on http_port+1
        self.ws_port = http_port
        self.static_port = http_port + 1
        self._browsers: Set[ServerConnection] = set()
        self._bus_ws: Optional[ServerConnection] = None
        self._last_frame: Optional[bytes] = None
        self._last_state: dict = {}
        self._last_decision: Optional[dict] = None
        self._last_verifier: Optional[dict] = None
        self._ws_server: Optional[websockets.asyncio.server.Server] = None
        self._http_server: Optional[HTTPServer] = None
        self._running = False

    async def run(self) -> None:
        self._running = True
        # WS server (for browser connections)
        self._ws_server = await websockets.serve(
            self._browser_handler,
            self.http_host,
            self.ws_port,
            max_size=None,
        )
        # Static HTTP server (for serving index.html) on port+1
        self._http_server = await self._start_static_server()
        logger.info("Panel WS on ws://%s:%d, HTTP on http://%s:%d",
                    self.http_host, self.ws_port,
                    self.http_host, self.static_port)
        logger.info("Open http://%s:%d in your browser", self.http_host, self.static_port)
        logger.info("Connecting to agent debug bus at %s ...", self.agent_url)
        await self._bus_loop()

    async def stop(self) -> None:
        self._running = False
        if self._ws_server:
            self._ws_server.close()
            await self._ws_server.wait_closed()
        if self._http_server:
            # Shutdown in a thread to avoid blocking
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._http_server.shutdown)
            self._http_server.server_close()

    # ── Static file serving (stdlib http.server, separate port) ────────

    async def _start_static_server(self) -> HTTPServer:
        """Start a stdlib HTTP server in a thread for static files."""
        static_dir = str(_STATIC_DIR)
        ws_port = self.ws_port

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=static_dir, **kwargs)

            def log_message(self, fmt, *args):
                pass  # silence

            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    file_path = Path(static_dir) / "index.html"
                    if file_path.is_file():
                        html = file_path.read_text()
                        # Inject WS port as a JS global before </head>
                        inject = f'<script>window.__WS_PORT__={ws_port};</script>'
                        html = html.replace("</head>", f"{inject}\n</head>")
                        body = html.encode()
                        self.send_response(200)
                        self.send_header("Content-Type", "text/html; charset=utf-8")
                        self.send_header("Content-Length", str(len(body)))
                        self.send_header("Cache-Control", "no-cache")
                        self.end_headers()
                        self.wfile.write(body)
                        return
                super().do_GET(self)

        server = HTTPServer((self.http_host, self.static_port), Handler)
        loop = asyncio.get_event_loop()
        loop.run_in_executor(None, server.serve_forever)
        return server

    # ── Agent debug bus client ──────────────────────────────────────────

    async def _bus_loop(self) -> None:
        """Maintain a persistent WS connection to the agent debug bus.

        Reconnects on disconnect with a short backoff. The agent may not be
        up yet when the panel starts.
        """
        while self._running:
            try:
                async with connect(self.agent_url, max_size=None) as ws:
                    self._bus_ws = ws
                    logger.info("Connected to agent debug bus")
                    await ws.send(json.dumps({"type": "get_state"}))
                    await ws.send(json.dumps({"type": "get_frame"}))
                    async for raw in ws:
                        await self._on_bus_message(raw)
            except (OSError, websockets.ConnectionClosed) as e:
                logger.warning("Bus disconnected: %s; reconnecting in 2s", e)
            except Exception as e:
                logger.error("Bus error: %s; reconnecting in 2s", e)
            finally:
                self._bus_ws = None
            if self._running:
                await asyncio.sleep(2.0)

    async def _on_bus_message(self, raw) -> None:
        """Handle a message from the agent debug bus; fan out to browsers."""
        if isinstance(raw, bytes):
            if raw[:1] == _FRAME_PREFIX:
                self._last_frame = raw[1:]
                await self._broadcast_binary(raw)
            return

        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON from bus: %s", raw[:200])
            return

        mtype = msg.get("type", "")
        if mtype == "state":
            self._last_state = {k: v for k, v in msg.items() if k != "type"}
        elif mtype == "decision":
            self._last_decision = {k: v for k, v in msg.items() if k != "type"}
        elif mtype == "verifier":
            self._last_verifier = {k: v for k, v in msg.items() if k != "type"}

        await self._broadcast_json(msg)

    async def _send_to_bus(self, msg: dict) -> None:
        if self._bus_ws and self._bus_ws.state == WsState.OPEN:
            try:
                await self._bus_ws.send(json.dumps(msg))
            except websockets.ConnectionClosed:
                logger.warning("Bus send failed (disconnected)")

    # ── Browser WS handler ──────────────────────────────────────────────

    async def _browser_handler(self, ws: ServerConnection) -> None:
        """Handle a browser WebSocket connection."""
        self._browsers.add(ws)
        logger.info("Browser connected (%d total)", len(self._browsers))
        try:
            if self._last_state:
                await ws.send(json.dumps({"type": "state", **self._last_state}))
            if self._last_decision:
                await ws.send(json.dumps({"type": "decision", **self._last_decision}))
            if self._last_verifier:
                await ws.send(json.dumps({"type": "verifier", **self._last_verifier}))
            if self._last_frame:
                await ws.send(_FRAME_PREFIX + self._last_frame)

            async for raw in ws:
                await self._on_browser_message(ws, raw)
        except websockets.ConnectionClosed:
            pass
        except Exception as e:
            logger.error("Browser error: %s", e)
        finally:
            self._browsers.discard(ws)
            logger.info("Browser disconnected (%d total)", len(self._browsers))

    async def _on_browser_message(self, ws: ServerConnection, raw) -> None:
        """Forward browser commands upstream to the agent debug bus."""
        if isinstance(raw, bytes):
            return
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return

        mtype = msg.get("type", "")
        if mtype == "ping":
            await ws.send(json.dumps({"type": "pong", "ts": time.time()}))
            return

        await self._send_to_bus(msg)

    # ── Broadcast helpers ───────────────────────────────────────────────

    async def _broadcast_json(self, msg: dict) -> None:
        if not self._browsers:
            return
        data = json.dumps(msg)
        dead = set()
        for ws in self._browsers:
            if ws.state == WsState.OPEN:
                try:
                    await ws.send(data)
                except websockets.ConnectionClosed:
                    dead.add(ws)
            else:
                dead.add(ws)
        self._browsers -= dead

    async def _broadcast_binary(self, data: bytes) -> None:
        if not self._browsers:
            return
        dead = set()
        for ws in self._browsers:
            if ws.state == WsState.OPEN:
                try:
                    await ws.send(data)
                except websockets.ConnectionClosed:
                    dead.add(ws)
            else:
                dead.add(ws)
        self._browsers -= dead