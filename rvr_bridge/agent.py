"""Real-robot agent loop for phone-as-BLE-shell (#21).

Runs on the computer. Uses the phone relay for camera frames and motor
commands, and the VLM client (shared with the sim agent) for decisions.
Adapts agent/agent_node.py's decide→commit cycle for the RVR (no ROS,
no LiDAR, no depth — camera only, IMU for bump detection).
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from PIL import Image

from .bump_detect import BumpDetector, BumpEvent
from .protocol import (DriveMessage, GetBleStateMessage, ResetYawMessage,
                        SleepMessage, StopMessage, WakeMessage)
from .server import PhoneRelay

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_SHARED_DIR = _PROJECT_ROOT / "shared"

TURN_STEP_DEG = 30
DRIVE_SPEED_BYTE = 64
SPEED_MPS = 0.35
ARRIVE_DIST_M = 0.4
MAX_DRIVE_MS = 4000
VLM_INTERVAL_S = 0.3
BUMP_REVERSE_MS = 1500
BUMP_TURN_DEG = 90

DRIVE_FLAGS_FORWARD = 0x00
DRIVE_FLAGS_REVERSE = 0x01

TELEOP_TURN_DEG_PER_TICK = 5  # ≈100°/s at the 20 Hz teleop tick rate
TELEOP_FRAME_MIN_GAP_S = 0.15  # caps request rate; real fps is bounded lower by
                                # phone capture+encode+transfer time


@dataclass
class BridgeConfig:
    ollama_url: str = "http://localhost:11434"
    model: str = "gemma4:31b-cloud"
    api_key: Optional[str] = None
    target: str = "fire_extinguisher"
    target_description: str = ""
    ws_host: str = "::"  # dual-stack: adb reverse tunnel + OkHttp uses IPv6 ::1
    ws_port: int = 8765
    drive_speed_byte: int = DRIVE_SPEED_BYTE
    speed_mps: float = SPEED_MPS
    arrive_dist_m: float = ARRIVE_DIST_M
    max_drive_ms: int = MAX_DRIVE_MS
    vlm_interval_s: float = VLM_INTERVAL_S
    bump_threshold_factor: float = 2.5
    log_file: Optional[str] = None
    debug_bus_port: Optional[int] = None  # #24: if set, start RvrDebugBus
    teleop_only: bool = False  # #24: start in teleop mode; never autonomously drive


class RvrAgent:
    """Autonomous object-finding agent for the real RVR.

    Lifecycle:
        1. Connect to phone relay (WebSocket)
        2. Wait for BLE ready
        3. Wake + reset_yaw
        4. Loop: capture_frame → VLM query → verify → drive
        5. On arrival or timeout: stop + report
    """

    def __init__(self, config: BridgeConfig):
        self.config = config
        self.relay = PhoneRelay(host=config.ws_host, port=config.ws_port)
        self.bump_detector = BumpDetector(threshold_factor=config.bump_threshold_factor)
        self._desired_heading: int = 0
        self._running = False
        self._confirmed_count = 0
        self._vlm_client = None
        self._log_fh = None
        self._bump_event: Optional[BumpEvent] = None
        self._debug_bus = None  # #24: set if config.debug_bus_port is not None
        self._teleop_only = config.teleop_only  # #24: start paused if True
        self._last_battery_poll: float = 0.0  # monotonic; 0 → poll immediately

        # Teleop state (#24): when teleop is active the autonomous loop pauses
        # and the panel owns drive commands.
        self._teleop_active: bool = False
        self._teleop_lin: float = 0.0  # -1..1 forward
        self._teleop_turn: float = 0.0  # -1..1 left
        self._auto_mode: bool = False  # auto-observe toggle (panel parity with debug_node)
        self._bump_enabled: bool = True  # bump detector armed (toggle: 'bump')
        self._latest_frame: Optional[Image.Image] = None  # cached for panel/manual query

        # Callbacks (set by RvrDebugBus if --debug-bus is active; None otherwise).
        # Each receives a dict of event data; the bus serialises and pushes to browsers.
        self.on_frame: Optional[Callable[[Image.Image], None]] = None
        self.on_decision: Optional[Callable[[dict], None]] = None
        self.on_verifier: Optional[Callable[[dict], None]] = None
        self.on_imu_event: Optional[Callable[[dict], None]] = None
        self.on_bump_event: Optional[Callable[[dict], None]] = None
        self.on_ble_event: Optional[Callable[[dict], None]] = None
        self.on_battery_event: Optional[Callable[[dict], None]] = None
        self.on_phone_battery_event: Optional[Callable[[dict], None]] = None
        self.on_state_change: Optional[Callable[[dict], None]] = None

        self.relay.on_imu = self._on_imu
        self.relay.on_ble_state = self._on_ble_state
        self.relay.on_battery = self._on_battery
        self.relay.on_phone_battery = self._on_phone_battery
        self.relay.on_frame = self._on_frame

    async def run(self) -> None:
        self._running = True
        await self.relay.start()

        if self.config.debug_bus_port:
            from .rvr_debug_bus import RvrDebugBus
            self._debug_bus = RvrDebugBus(
                self, host="0.0.0.0", port=self.config.debug_bus_port)
            await self._debug_bus.start()

        if self.config.log_file:
            self._log_fh = open(self.config.log_file, "a")
            self._log_entry({"event": "agent_start", "target": self.config.target})

        logger.info("Waiting for phone to connect on ws://%s:%d ...", self.config.ws_host, self.config.ws_port)
        while self._running and not self.relay.phone_connected:
            await asyncio.sleep(0.5)

        if not self._running:
            return

        logger.info("Phone connected. Waiting for BLE ready...")
        ble_ready = False
        while self._running and self.relay.ble_state != "ready":
            # The phone only pushes ble_state on a transition (onStateChange);
            # if BLE was already up before this process (re)started, no
            # transition ever fires and this loop would hang forever. Poll
            # for the current state instead of waiting passively.
            await self.relay.send(GetBleStateMessage())
            await asyncio.sleep(0.5)
            # In teleop-only mode, don't block forever on BLE — the camera
            # works without the RVR. Stream frames while waiting.
            if self._teleop_only:
                break

        if not self._running:
            return

        ble_ready = self.relay.ble_state == "ready"
        if ble_ready:
            logger.info("BLE ready. Waking RVR and zeroing heading.")
            await self.relay.send(WakeMessage())
            # RVR needs ~3s to wake from soft-sleep before accepting drive commands.
            await asyncio.sleep(3.0)
            await self.relay.send(ResetYawMessage())
            await asyncio.sleep(0.5)
        else:
            logger.info("BLE not ready (RVR unavailable). Camera-only mode.")

        self._vlm_client = self._make_vlm_client()

        if self._teleop_only:
            logger.info("Teleop-only mode: autonomous loop paused. Drive via panel.")
            self._teleop_active = True
            self._emit_state()

        frame_task = asyncio.ensure_future(self._teleop_frame_loop())
        try:
            await self._loop()
        finally:
            frame_task.cancel()
            if self._debug_bus:
                await self._debug_bus.stop()
            await self.relay.send(StopMessage(heading=self._desired_heading))
            await self.relay.stop()
            if self._log_fh:
                self._log_entry({"event": "agent_stop"})
                self._log_fh.close()

    def stop(self) -> None:
        self._running = False

    def _make_vlm_client(self):
        import sys
        sys.path.insert(0, str(_PROJECT_ROOT))
        from agent.vlm_client import VLMClient

        vlm_config = {
            "model": {
                "name": self.config.model,
                "backend": "ollama-cloud" if "cloud" in self.config.model else "ollama",
            },
            "inference": {
                "max_retries": 3,
                "timeout_s": 60.0,
            },
        }
        client = VLMClient(vlm_config)
        client.start()
        return client

    async def _loop(self) -> None:
        while self._running:
            # Periodic battery poll (every 30 s; phone only sends on request)
            await self._poll_battery()

            # Teleop override (#24): panel owns drive commands; loop idles.
            if self._teleop_active:
                await self._teleop_tick()
                await asyncio.sleep(0.05)
                continue

            img = await self.relay.capture_frame()
            if img is None:
                logger.warning("Frame capture failed; retrying")
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            self._latest_frame = img
            self._emit_frame(img)

            loop = asyncio.get_event_loop()
            prompt = self._build_prompt()
            t0 = time.time()
            decision = await loop.run_in_executor(None, self._vlm_client.query, img, prompt)
            latency_ms = (time.time() - t0) * 1000
            if decision is None:
                logger.warning("VLM returned None; stopping for a cycle")
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            logger.info("VLM: vis=%s hdg=%s turn=%+d° dist=%.2f loc=%s | %s",
                        decision.target_visible, decision.heading,
                        decision.turn_angle_deg,
                        decision.drive_distance_m, decision.target_location,
                        decision.reason[:80])

            self._emit_decision(decision, latency_ms)

            confirmed = False
            if decision.target_visible and decision.target_location:
                t0 = time.time()
                verify = await loop.run_in_executor(
                    None, self._vlm_client.verify_candidate,
                    img, self.config.target, decision.target_location
                )
                v_latency_ms = (time.time() - t0) * 1000
                if verify:
                    confirmed = verify.confirmed
                    logger.info("VERIFY: confirmed=%s | %s",
                                verify.confirmed, verify.reason[:80])
                    self._emit_verifier(verify, v_latency_ms)
                    if confirmed:
                        self._confirmed_count += 1
                        if decision.drive_distance_m <= self.config.arrive_dist_m:
                            await self.beep("found")
                            await self.relay.send(StopMessage(heading=self._desired_heading))
                            logger.info("ARRIVED at '%s' (confirmed, dist≈%.2f m)",
                                        self.config.target, decision.drive_distance_m)
                            self._log_entry({"event": "arrived", "target": self.config.target})
                            self._running = False
                            return
                else:
                    logger.warning("Verifier returned None; treating as unconfirmed")

            self._log_entry({
                "event": "decision",
                "vis": decision.target_visible,
                "heading": decision.heading,
                "dist": decision.drive_distance_m,
                "loc": decision.target_location,
                "confirmed": confirmed,
                "reason": decision.reason[:120],
            })

            self._desired_heading = self._norm_heading(
                self._desired_heading + decision.turn_angle_deg
            )
            self._emit_state()

            await self._execute_drive(decision.drive_distance_m)

            await asyncio.sleep(self.config.vlm_interval_s)

    async def _execute_drive(self, distance_m: float) -> None:
        if distance_m <= 0.0:
            await self.relay.send(DriveMessage(
                speed=0, heading=self._desired_heading, flags=DRIVE_FLAGS_FORWARD
            ))
            return

        duration_ms = min(
            max(int(distance_m / self.config.speed_mps * 1000), 200),
            self.config.max_drive_ms,
        )
        await self.relay.send(DriveMessage(
            speed=self.config.drive_speed_byte,
            heading=self._desired_heading,
            flags=DRIVE_FLAGS_FORWARD,
        ))

        step_ms = 50
        elapsed_ms = 0
        while elapsed_ms < duration_ms and self._running:
            await asyncio.sleep(step_ms / 1000.0)
            elapsed_ms += step_ms
            if self._bump_event is not None:
                logger.info("Bump during drive — emergency stop")
                await self.relay.send(StopMessage(heading=self._desired_heading))
                await self.relay.send(DriveMessage(
                    speed=self.config.drive_speed_byte // 2,
                    heading=self._desired_heading,
                    flags=DRIVE_FLAGS_REVERSE,
                ))
                await asyncio.sleep(BUMP_REVERSE_MS / 1000.0)
                await self.relay.send(StopMessage(heading=self._desired_heading))
                self._desired_heading = self._norm_heading(self._desired_heading + BUMP_TURN_DEG)
                await self.relay.send(DriveMessage(
                    speed=self.config.drive_speed_byte // 2,
                    heading=self._desired_heading,
                    flags=DRIVE_FLAGS_FORWARD,
                ))
                await asyncio.sleep(0.5)
                await self.relay.send(StopMessage(heading=self._desired_heading))
                self._bump_event = None
                return

        if self._running:
            await self.relay.send(StopMessage(heading=self._desired_heading))

    def _on_imu(self, msg) -> None:
        if self.on_imu_event:
            self.on_imu_event({
                "accel": list(msg.accel),
                "gyro": list(msg.gyro),
                "ts": msg.ts,
            })
        if not self._bump_enabled:
            return
        event = self.bump_detector.feed(msg)
        if event:
            logger.info("Bump detected: mag=%.1f", event.magnitude)
            self._log_entry({"event": "bump", "magnitude": event.magnitude})
            self._bump_event = event
            if self.on_bump_event:
                self.on_bump_event({
                    "magnitude": event.magnitude,
                    "timestamp": event.timestamp,
                })

    def _on_ble_state(self, msg) -> None:
        logger.info("BLE state: %s", msg.state)
        if self.on_ble_event:
            self.on_ble_event({"state": msg.state})
        self._emit_state()

    def _on_battery(self, msg) -> None:
        logger.info("Battery: %d%%", msg.pct)
        if self.on_battery_event:
            self.on_battery_event({"pct": msg.pct})

    def _on_phone_battery(self, msg) -> None:
        logger.info("Phone battery: %d%%", msg.pct)
        if self.on_phone_battery_event:
            self.on_phone_battery_event({"pct": msg.pct})

    async def _poll_battery(self) -> None:
        """Request battery % from the phone every 30 s.

        The phone only sends `battery` messages in response to `get_battery`
        commands (it reads the RVR BLE response and relays it). Without this
        poll the battery widget in the panel never gets data.
        """
        now = time.monotonic()
        if now - self._last_battery_poll < 60.0:
            return
        self._last_battery_poll = now
        from .protocol import GetBatteryMessage, GetPhoneBatteryMessage
        await self.relay.send(GetBatteryMessage())
        await self.relay.send(GetPhoneBatteryMessage())

    def _on_frame(self, img: Image.Image) -> None:
        """Relay push-mode frame callback (when phone streams continuously)."""
        self._latest_frame = img
        self._emit_frame(img)

    def _build_prompt(self) -> str:
        natural = self.config.target.replace("_", " ")
        lines = [
            f"Target: {self.config.target}  (natural language: \"{natural}\")",
        ]
        if self.config.target_description:
            lines.append(f"Description: {self.config.target_description}")
        lines += [
            "",
            "Look at the image. Decide:",
            "  - Is the target visible? Scan floor, corners, walls, edges. The target may be",
            "    small or low-contrast. If you see ANY object that plausibly matches, set",
            "    target_visible=true and fill target_location.",
            "  - How much to turn (turn_angle_deg: -90/-60/-30/0/30/60/90, +=right) and how",
            "    far to drive (0.0-2.0 m). Use 0.0 m + a large turn when facing a wall.",
            "Reply JSON only.",
        ]
        return "\n".join(lines)

    @staticmethod
    def _norm_heading(h: int) -> int:
        return (h % 360 + 360) % 360

    # ── Panel command handlers (#24) ────────────────────────────────────

    def set_teleop(self, active: bool) -> None:
        """Enable/disable teleop override. When active, autonomous loop pauses."""
        if active and not self._teleop_active:
            self._teleop_lin = 0.0
            self._teleop_turn = 0.0
        self._teleop_active = active
        if not active:
            # Stop on handoff
            asyncio.ensure_future(self.relay.send(StopMessage(heading=self._desired_heading)))
        self._emit_state()

    def teleop_drive(self, lin: float, turn: float) -> None:
        """Set teleop velocities (normalized -1..1). lin=forward, turn=left."""
        self._teleop_lin = max(-1.0, min(1.0, lin))
        self._teleop_turn = max(-1.0, min(1.0, turn))
        if not self._teleop_active:
            self.set_teleop(True)

    async def _teleop_frame_loop(self) -> None:
        """Stream camera frames to the panel while teleop is active (#24).

        The autonomous loop only requests a frame once per VLM cycle
        (~0.3-1s, often slower), too sparse to drive by. Runs as its own
        task so the frame round-trip to the phone never blocks the 20 Hz
        drive-command loop in `_teleop_tick`.
        """
        while self._running:
            if self._teleop_active:
                img = await self.relay.capture_frame()
                if img is not None:
                    self._latest_frame = img
                    self._emit_frame(img)
            await asyncio.sleep(TELEOP_FRAME_MIN_GAP_S)

    async def _teleop_tick(self) -> None:
        """Called every loop iteration while in teleop; sends drive commands.

        Forward/backward: driveWithHeading at proportional speed.
        Turn: driveWithHeading at speed=0 with a continuously incremented
        heading target — the firmware's closed-loop yaw control rotates the
        chassis to track it, so holding the key keeps nudging the target
        ahead and the robot keeps turning. (raw_motors was tried first: it's
        open-loop with no torque compensation, and 64/255 — fine for rolling
        forward — wasn't enough to overcome the higher friction of pivoting
        in place, so the wheels spun without turning the chassis.)
        Both are hold-to-move, release-to-stop.

        Bump detector stays armed: a bump triggers emergency stop + reverse
        (same recovery as autonomous drive), then clears the teleop command.
        """
        if self._bump_event is not None:
            await self._handle_bump_recovery()
            self._teleop_lin = 0.0
            self._teleop_turn = 0.0
            return

        lin = self._teleop_lin
        turn = self._teleop_turn

        # Turning: continuously advance the heading target (hold to rotate)
        if abs(turn) > 0.05 and abs(lin) < 0.05:
            self._desired_heading = self._norm_heading(
                self._desired_heading + int(turn * TELEOP_TURN_DEG_PER_TICK)
            )
            await self.relay.send(DriveMessage(
                speed=0,
                heading=self._desired_heading,
                flags=DRIVE_FLAGS_FORWARD,
            ))
            return

        # Forward/backward: driveWithHeading (hold to drive)
        if abs(lin) > 0.05:
            speed = max(int(abs(lin) * self.config.drive_speed_byte), 1)
            flags = DRIVE_FLAGS_REVERSE if lin < 0 else DRIVE_FLAGS_FORWARD
            await self.relay.send(DriveMessage(
                speed=speed,
                heading=self._desired_heading,
                flags=flags,
            ))
            return

        # Nothing pressed — stop
        await self.relay.send(StopMessage(heading=self._desired_heading))

    async def _handle_bump_recovery(self) -> None:
        """Emergency stop + reverse + turn, shared by teleop and autonomous."""
        logger.info("Bump during teleop — emergency stop")
        await self.beep("bump")
        await self.relay.send(StopMessage(heading=self._desired_heading))
        await self.relay.send(DriveMessage(
            speed=self.config.drive_speed_byte // 2,
            heading=self._desired_heading,
            flags=DRIVE_FLAGS_REVERSE,
        ))
        await asyncio.sleep(BUMP_REVERSE_MS / 1000.0)
        await self.relay.send(StopMessage(heading=self._desired_heading))
        self._desired_heading = self._norm_heading(self._desired_heading + BUMP_TURN_DEG)
        self._bump_event = None

    async def manual_stop(self) -> None:
        """Emergency stop — halt motors immediately. Stays in teleop mode."""
        self._teleop_lin = 0.0
        self._teleop_turn = 0.0
        await self.relay.send(StopMessage(heading=self._desired_heading))
        self._emit_state()

    async def manual_query(self) -> None:
        """Stop, capture frame, run VLM decision + verifier, emit results.

        Mirrors agent/debug_node._manual_query for RVR parity. Does NOT drive.
        """
        if self._vlm_client is None:
            logger.warning("VLM client not ready; cannot run manual query")
            return

        img = await self.relay.capture_frame()
        if img is None:
            logger.warning("Manual query: frame capture failed")
            return
        self._latest_frame = img
        self._emit_frame(img)

        loop = asyncio.get_event_loop()
        prompt = self._build_prompt()
        t0 = time.time()
        decision = await loop.run_in_executor(None, self._vlm_client.query, img, prompt)
        latency_ms = (time.time() - t0) * 1000
        if decision is None:
            logger.warning("Manual query: VLM returned None")
            return

        logger.info("MANUAL VLM: vis=%s hdg=%s turn=%+d° dist=%.2f loc=%s | %s",
                    decision.target_visible, decision.heading,
                    decision.turn_angle_deg,
                    decision.drive_distance_m, decision.target_location,
                    decision.reason[:80])
        self._emit_decision(decision, latency_ms)

        if decision.target_visible and decision.target_location:
            t0 = time.time()
            verify = await loop.run_in_executor(
                None, self._vlm_client.verify_candidate,
                img, self.config.target, decision.target_location
            )
            v_latency_ms = (time.time() - t0) * 1000
            if verify:
                self._emit_verifier(verify, v_latency_ms)
                logger.info("MANUAL VERIFY: confirmed=%s | %s",
                            verify.confirmed, verify.reason[:80])

    def set_target(self, target: str, description: str = "") -> None:
        self.config.target = target
        self.config.target_description = description
        self._confirmed_count = 0
        self._emit_state()

    async def set_torch(self, on: bool) -> None:
        from .protocol import TorchMessage
        await self.relay.send(TorchMessage(on=on))

    async def beep(self, beep_type: str = "found", volume: int = 80) -> None:
        from .protocol import BeepMessage
        await self.relay.send(BeepMessage(beep_type=beep_type, volume=volume))

    def toggle(self, which: str, value: Optional[bool] = None) -> bool:
        """Toggle a panel-controlled flag. Returns the new state."""
        if which == "auto":
            self._auto_mode = not self._auto_mode if value is None else value
            new = self._auto_mode
        elif which == "bump":
            self._bump_enabled = not self._bump_enabled if value is None else value
            new = self._bump_enabled
        elif which == "teleop":
            self.set_teleop(not self._teleop_active if value is None else value)
            new = self._teleop_active
        else:
            return False
        self._emit_state()
        return new

    # ── Panel event emitters (#24) ──────────────────────────────────────

    def _emit_frame(self, img: Image.Image) -> None:
        if self.on_frame:
            self.on_frame(img)

    def _emit_decision(self, decision, latency_ms: float) -> None:
        if self.on_decision:
            self.on_decision({
                "target_visible": decision.target_visible,
                "heading": decision.heading,
                "turn_angle_deg": decision.turn_angle_deg,
                "drive_distance_m": decision.drive_distance_m,
                "target_location": decision.target_location,
                "reason": decision.reason,
                "latency_ms": latency_ms,
            })

    def _emit_verifier(self, verify, latency_ms: float) -> None:
        if self.on_verifier:
            self.on_verifier({
                "confirmed": verify.confirmed,
                "matches": verify.matches,
                "mismatches": verify.mismatches,
                "reason": verify.reason,
                "latency_ms": latency_ms,
            })

    def _emit_state(self) -> None:
        if self.on_state_change:
            self.on_state_change(self._state_snapshot())

    def _state_snapshot(self) -> dict:
        return {
            "target": self.config.target,
            "target_description": self.config.target_description,
            "desired_heading": self._desired_heading,
            "ble_state": self.relay.ble_state,
            "phone_connected": self.relay.phone_connected,
            "teleop_active": self._teleop_active,
            "auto_mode": self._auto_mode,
            "bump_enabled": self._bump_enabled,
            "confirmed_count": self._confirmed_count,
            "running": self._running,
            "vlm_ready": self._vlm_client is not None,
        }

    def _log_entry(self, entry: dict) -> None:
        if self._log_fh:
            entry["t"] = time.time()
            self._log_fh.write(json.dumps(entry) + "\n")
            self._log_fh.flush()