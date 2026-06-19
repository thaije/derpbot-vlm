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
from typing import Optional

from PIL import Image

from .bump_detect import BumpDetector, BumpEvent
from .protocol import DriveMessage, ResetYawMessage, SleepMessage, StopMessage, WakeMessage
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

        self.relay.on_imu = self._on_imu
        self.relay.on_ble_state = self._on_ble_state

    async def run(self) -> None:
        self._running = True
        await self.relay.start()

        if self.config.log_file:
            self._log_fh = open(self.config.log_file, "a")
            self._log_entry({"event": "agent_start", "target": self.config.target})

        logger.info("Waiting for phone to connect on ws://%s:%d ...", self.config.ws_host, self.config.ws_port)
        while self._running and not self.relay.phone_connected:
            await asyncio.sleep(0.5)

        if not self._running:
            return

        logger.info("Phone connected. Waiting for BLE ready...")
        while self._running and self.relay.ble_state != "ready":
            await asyncio.sleep(0.2)

        if not self._running:
            return

        logger.info("BLE ready. Waking RVR and zeroing heading.")
        await self.relay.send(WakeMessage())
        # RVR needs ~3s to wake from soft-sleep before accepting drive commands.
        await asyncio.sleep(3.0)
        await self.relay.send(ResetYawMessage())
        await asyncio.sleep(0.5)

        self._vlm_client = self._make_vlm_client()

        try:
            await self._loop()
        finally:
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
            img = await self.relay.capture_frame()
            if img is None:
                logger.warning("Frame capture failed; retrying")
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            loop = asyncio.get_event_loop()
            prompt = self._build_prompt()
            decision = await loop.run_in_executor(None, self._vlm_client.query, img, prompt)
            if decision is None:
                logger.warning("VLM returned None; stopping for a cycle")
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            logger.info("VLM: vis=%s hdg=%s dist=%.2f loc=%s | %s",
                        decision.target_visible, decision.heading,
                        decision.drive_distance_m, decision.target_location,
                        decision.reason[:80])

            confirmed = False
            if decision.target_visible and decision.target_location:
                verify = await loop.run_in_executor(
                    None, self._vlm_client.verify_candidate,
                    img, self.config.target, decision.target_location
                )
                if verify:
                    confirmed = verify.confirmed
                    logger.info("VERIFY: confirmed=%s | %s",
                                verify.confirmed, verify.reason[:80])
                    if confirmed:
                        self._confirmed_count += 1
                        if decision.drive_distance_m <= self.config.arrive_dist_m:
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

            if decision.heading == "left":
                self._desired_heading -= TURN_STEP_DEG
            elif decision.heading == "right":
                self._desired_heading += TURN_STEP_DEG
            self._desired_heading = self._norm_heading(self._desired_heading)

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
        event = self.bump_detector.feed(msg)
        if event:
            logger.info("Bump detected: mag=%.1f", event.magnitude)
            self._log_entry({"event": "bump", "magnitude": event.magnitude})
            self._bump_event = event

    def _on_ble_state(self, msg) -> None:
        logger.info("BLE state: %s", msg.state)

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
            "  - Which heading (left/center/right) leads toward the target or open space?",
            "  - How far to drive in that heading (0.0-2.0 m). In tight/uncertain scenes pick",
            "    a short distance (<= 0.5 m).",
            "Reply JSON only.",
        ]
        return "\n".join(lines)

    @staticmethod
    def _norm_heading(h: int) -> int:
        return (h % 360 + 360) % 360

    def _log_entry(self, entry: dict) -> None:
        if self._log_fh:
            entry["t"] = time.time()
            self._log_fh.write(json.dumps(entry) + "\n")
            self._log_fh.flush()