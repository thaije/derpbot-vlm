"""Base class for real-robot VLM agents (#25).

Extracted from ``rvr_bridge/agent.py``: holds the transport-agnostic domain
logic (VLM decide→verify→commit loop, teleop state machine, bump-recovery
protocol, panel callback hooks, logging).  The locomotion API is delegated
to a ``RobotTransport`` implementation:

    RvrTransport       — Sphero RVR via phone-as-BLE-shell
    Create3Transport   — iRobot Create 3 via ROS 2 /cmd_vel

Subclasses may override hooks (``_on_battery``, ``set_status``, etc.) for
backend-specific behaviour, but the loop, teleop state machine, and panel
event emitters live here.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from PIL import Image

from .transport import BatteryState, HazardEvent, RobotTransport

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_RUNS_DIR = _PROJECT_ROOT / "runs"
_FRAME_DIR = _RUNS_DIR / "_current" / "frames"

# Defaults — backends may override via their config dataclass.
ARRIVE_DIST_M = 0.4
VLM_INTERVAL_S = 0.3
BUMP_REVERSE_MS = 1500
BUMP_TURN_DEG = 90
TELEOP_FRAME_MIN_GAP_S = 0.15

# Scan sweep — step-stop-shoot rotation to acquire full environmental context.
SCAN_STEPS = 6              # 6 × 60° = 360°
SCAN_STEP_DEG = 60
SCAN_ROTATE_TIMEOUT_S = 10.0
# Loop detection — after N consecutive turns without seeing the target,
# trigger a scan sweep.
LOOP_TURNS_TRIGGER = 3


@dataclass
class BaseAgentConfig:
    ollama_url: str = "http://localhost:11434"
    model: str = "gemma4:31b-cloud"
    api_key: Optional[str] = None
    target: str = "fire_extinguisher"
    target_description: str = ""
    arrive_dist_m: float = ARRIVE_DIST_M
    vlm_interval_s: float = VLM_INTERVAL_S
    log_file: Optional[str] = None
    debug_bus_port: Optional[int] = None
    teleop_only: bool = False
    run_dir: Optional[str] = None
    confirm_with_user: bool = True


class BaseRealAgent:
    """Transport-agnostic real-robot VLM agent.

    Lifecycle:
        1. ``run()`` → transport.start() → wait for connection
        2. Loop: capture_frame → VLM query → verify → drive (via transport)
        3. On arrival or timeout: stop + report

    The agent owns:
        - VLM client (reuses ``agent/vlm_client.py`` + ``shared/``)
        - Teleop state machine (lin/turn, hold-to-move)
        - Bump recovery protocol (halt → reverse → turn)
        - Panel event emitters (on_frame, on_decision, ...)
        - JSONL logging

    The transport owns:
        - Frame capture (phone relay / ROS camera)
        - Locomotion (RVR bytes / ROS Twist)
        - Battery / pose / hazard sensing
        - Status outputs (torch / LED / audio)
    """

    def __init__(self, config: BaseAgentConfig, transport: RobotTransport):
        self.config = config
        self.transport = transport

        self._running = False
        self._confirmed_count = 0
        self._vlm_client = None
        self._log_fh = None
        self._hazard_event: Optional[HazardEvent] = None
        self._debug_bus = None
        self._teleop_only = config.teleop_only

        # Run directory: timestamped subdir under runs/.  VLM frames and the
        # JSONL decision log go here.  Symlink runs/_current → this run so the
        # panel proxy can serve the latest frames without a path update.
        global _FRAME_DIR
        if config.run_dir:
            run_path = Path(config.run_dir)
        else:
            from datetime import datetime
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_path = _RUNS_DIR / stamp
        run_path = run_path.resolve()
        (run_path / "frames").mkdir(parents=True, exist_ok=True)
        _FRAME_DIR = run_path / "frames"
        self.run_dir = run_path
        # Symlink runs/_current → this run (best-effort; ignore if it exists)
        current_link = _RUNS_DIR / "_current"
        try:
            if current_link.is_symlink() or current_link.exists():
                current_link.unlink()
            current_link.symlink_to(run_path, target_is_directory=True)
        except OSError:
            pass  # not fatal — panel falls back to _FRAME_DIR directly
        if not config.log_file:
            config.log_file = str(run_path / "decisions.jsonl")
        logger.info("Run dir: %s", run_path)

        # Heading: kept by the agent when the transport has no real heading
        # (RVR dead-reckoned byte counter).  None when the transport reports
        # a real heading from /imu (Create 3).
        self._desired_heading: Optional[int] = 0

        # Teleop state (#24): when teleop is active the autonomous loop
        # pauses and the panel owns drive commands.
        self._teleop_active: bool = False
        self._teleop_lin: float = 0.0
        self._teleop_turn: float = 0.0
        self._auto_mode: bool = False
        self._bump_enabled: bool = True
        self._latest_frame: Optional[Image.Image] = None
        self._frame_seq: int = 0

        # Recent decision history (for anti-loop awareness in the prompt).
        # Capped — only the last N decisions are kept.
        self._decision_history: list[dict] = []
        self._decision_history_len: int = 8

        # Loop detection: count consecutive turns without vis=True.
        self._consecutive_turns: int = 0
        self._scanning: bool = False
        self._initial_scan_done: bool = False

        # User confirmation: when confirm_with_user is True, the agent pauses
        # after a verifier-confirmed sighting and waits for the user to
        # acknowledge via the panel before declaring arrival.
        self._confirm_future: Optional[asyncio.Future] = None

        # Callbacks (set by DebugBus if --debug-bus is active; None otherwise).
        self.on_frame: Optional[Callable[[Image.Image], None]] = None
        self.on_decision: Optional[Callable[[dict], None]] = None
        self.on_verifier: Optional[Callable[[dict], None]] = None
        self.on_imu_event: Optional[Callable[[dict], None]] = None
        self.on_bump_event: Optional[Callable[[dict], None]] = None
        self.on_ble_event: Optional[Callable[[dict], None]] = None
        self.on_battery_event: Optional[Callable[[dict], None]] = None
        self.on_phone_battery_event: Optional[Callable[[dict], None]] = None
        self.on_state_change: Optional[Callable[[dict], None]] = None
        self.on_confirm_request: Optional[Callable[[dict], None]] = None

        # Wire transport hazard sink → agent
        self.transport.on_hazard = self._on_hazard
        # Back-reference so transports can emit BLE/battery state changes
        # and trigger a state snapshot for the panel (teleop-only mode only
        # emits state once at startup; relay callbacks need to refresh it).
        self.transport._agent = self

    # ── Lifecycle ────────────────────────────────────────────────────────

    async def run(self) -> None:
        self._running = True
        await self.transport.start()

        if self.config.debug_bus_port:
            from .debug_bus import DebugBus
            self._debug_bus = DebugBus(
                self, host="0.0.0.0", port=self.config.debug_bus_port)
            await self._debug_bus.start()

        if self.config.log_file:
            self._log_fh = open(self.config.log_file, "a")
            self._log_entry({
                "event": "agent_start",
                "target": self.config.target,
                "model": self.config.model,
                "run_dir": str(self.run_dir),
            })

        logger.info("Waiting for %s connection...", self.transport.backend_name)
        while self._running and not self._is_connected():
            await asyncio.sleep(0.5)

        if not self._running:
            return

        await self._on_connected()

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
            await self.transport.halt()
            await self.transport.stop()
            if self._log_fh:
                self._log_entry({"event": "agent_stop"})
                self._log_fh.close()

    def stop(self) -> None:
        self._running = False

    def _is_connected(self) -> bool:
        """Override in subclass if connection check differs from
        ``transport.connection_state == "ready"``."""
        return self.transport.connection_state == "ready"

    async def _on_connected(self) -> None:
        """Hook called once the transport reports connected.  Override in
        subclasses for backend-specific init (RVR: wake + reset_yaw)."""
        pass

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

    # ── Scan sweep ───────────────────────────────────────────────────────

    async def _scan_sweep(self, reason: str = "loop") -> Optional[dict]:
        """360° step-stop-shoot scan: rotate SCAN_STEP_DEG, settle, capture,
        VLM query at each of SCAN_STEPS positions.  Breaks early if the
        target is spotted and confirmed.

        Returns the last VLM decision dict if the scan completed without
        finding the target (so the caller can use it for a forced drive),
        or None if the scan was interrupted / target found.
        """
        self._scanning = True
        self._log_entry({"event": "scan_start", "reason": reason,
                         "steps": SCAN_STEPS, "step_deg": SCAN_STEP_DEG})
        logger.info("SCAN: starting 360° sweep (%s)", reason)

        loop = asyncio.get_event_loop()
        last_decision = None

        for step in range(SCAN_STEPS):
            if not self._running or self._teleop_active:
                break

            await self.transport.wait_standstill()
            img = await self.transport.capture_frame()
            if img is None:
                logger.warning("SCAN: frame capture failed at step %d", step)
                continue

            self._latest_frame = img
            self._emit_frame(img)
            frame_path = self._save_frame(img, tag=f"scan{step}")

            prompt = self._build_prompt()
            t0 = time.time()
            decision = await loop.run_in_executor(
                None, self._vlm_client.query, img, prompt)
            latency_ms = (time.time() - t0) * 1000

            if decision is None:
                logger.warning("SCAN: VLM returned None at step %d", step)
                self._log_entry({"event": "vlm_error", "stage": "scan",
                                 "step": step, "frame": frame_path})
                continue

            logger.info("SCAN %d/%d: vis=%s turn=%+d° dist=%.1f loc=%s | %s",
                        step + 1, SCAN_STEPS,
                        decision.target_visible, decision.turn_angle_deg,
                        decision.drive_distance_m, decision.target_location,
                        decision.reason[:80])

            self._emit_decision(decision, latency_ms, frame_path, prompt)
            self._log_entry({
                "event": "scan_decision", "step": step,
                "vis": decision.target_visible,
                "turn_angle_deg": decision.turn_angle_deg,
                "dist": decision.drive_distance_m,
                "loc": decision.target_location,
                "reason": decision.reason,
                "vlm_latency_ms": latency_ms,
                "frame": frame_path,
            })

            # Verify if target spotted
            if decision.target_visible and decision.target_location:
                verify = await loop.run_in_executor(
                    None, self._vlm_client.verify_candidate,
                    img, self.config.target, decision.target_location)
                if verify:
                    v_latency_ms = 0  # approx
                    self._emit_verifier(verify, v_latency_ms)
                    self._log_entry({
                        "event": "verifier",
                        "confirmed": verify.confirmed,
                        "matches": verify.matches,
                        "mismatches": verify.mismatches,
                        "reason": verify.reason,
                        "scan_step": step,
                    })
                    if verify.confirmed:
                        self._confirmed_count += 1
                        logger.info("SCAN: target confirmed at step %d!", step)
                        self._log_entry({"event": "scan_end",
                                         "reason": "target_found",
                                         "step": step})
                        self._scanning = False
                        return None  # caller proceeds with the decision

            last_decision = {
                "decision": decision,
                "frame_path": frame_path,
                "prompt": prompt,
            }

            # Rotate to next position (skip after last step)
            if step < SCAN_STEPS - 1:
                self._apply_heading_delta(SCAN_STEP_DEG)
                await self.transport.rotate(SCAN_STEP_DEG,
                                            timeout_s=SCAN_ROTATE_TIMEOUT_S)

        self._consecutive_turns = 0
        self._scanning = False
        self._log_entry({"event": "scan_end", "reason": "complete"})
        logger.info("SCAN: complete (no target found)")
        return last_decision

    # ── Autonomous loop ──────────────────────────────────────────────────

    async def _loop(self) -> None:
        while self._running:
            await self._poll_battery()

            if self._teleop_active:
                await self._teleop_tick()
                await asyncio.sleep(0.05)
                continue

            # Initial 360° scan sweep on first autonomous start — gives the
            # VLM full environmental context before committing to a direction.
            if not self._initial_scan_done:
                self._initial_scan_done = True
                result = await self._scan_sweep(reason="initial")
                if result is not None:
                    # No target found during scan — force a forward drive
                    # using the last scan frame's decision.
                    d = result["decision"]
                    if d.drive_distance_m > 0:
                        logger.info("SCAN: forcing drive %.1fm after scan",
                                    d.drive_distance_m)
                        await self._execute_drive(d.drive_distance_m, 0)
                continue

            # Wait for the robot to stop moving before capturing (avoids
            # blurry frames during/after turns — phone IMU gyro based).
            await self.transport.wait_standstill()
            img = await self.transport.capture_frame()
            if img is None:
                logger.warning("Frame capture failed; retrying")
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            self._latest_frame = img
            self._emit_frame(img)
            frame_path = self._save_frame(img)

            loop = asyncio.get_event_loop()
            prompt = self._build_prompt()
            t0 = time.time()
            decision = await loop.run_in_executor(None, self._vlm_client.query, img, prompt)
            latency_ms = (time.time() - t0) * 1000
            if decision is None:
                logger.warning("VLM returned None; stopping for a cycle")
                self._log_entry({"event": "vlm_error", "stage": "decision", "frame": frame_path})
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            logger.info("VLM: vis=%s hdg=%s turn=%+d° dist=%.2f loc=%s frame=%s | %s",
                        decision.target_visible, decision.heading,
                        decision.turn_angle_deg,
                        decision.drive_distance_m, decision.target_location,
                        frame_path, decision.reason[:80])

            # Record in history for anti-loop awareness in future prompts.
            self._decision_history.append({
                "vis": decision.target_visible,
                "turn": decision.turn_angle_deg,
                "dist": decision.drive_distance_m,
                "reason": decision.reason[:60],
            })
            if len(self._decision_history) > self._decision_history_len:
                self._decision_history.pop(0)

            self._emit_decision(decision, latency_ms, frame_path, prompt)

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
                    self._log_entry({
                        "event": "verifier",
                        "confirmed": verify.confirmed,
                        "matches": verify.matches,
                        "mismatches": verify.mismatches,
                        "reason": verify.reason,
                        "latency_ms": v_latency_ms,
                    })
                    if confirmed:
                        self._confirmed_count += 1
                        if decision.drive_distance_m <= self.config.arrive_dist_m:
                            # Ask user to confirm before declaring arrival.
                            user_ok = await self._wait_for_user_confirmation(
                                self.config.target, frame_path,
                                verify.reason)
                            if user_ok:
                                await self._on_arrived()
                                continue
                            else:
                                logger.info("User rejected confirmation — continuing search")
                                await self.transport.halt()
                                self._consecutive_turns = 0
                else:
                    logger.warning("Verifier returned None; treating as unconfirmed")
                    self._log_entry({"event": "vlm_error", "stage": "verifier"})

            self._log_entry({
                "event": "decision",
                "vis": decision.target_visible,
                "heading": decision.heading,
                "turn_angle_deg": decision.turn_angle_deg,
                "dist": decision.drive_distance_m,
                "loc": decision.target_location,
                "confirmed": confirmed,
                "reason": decision.reason,
                "prompt": prompt,
                "vlm_latency_ms": latency_ms,
                "frame": frame_path,
            })

            # Loop detection: count consecutive turns without vis=True.
            # After LOOP_TURNS_TRIGGER, do a scan sweep + forced drive.
            if decision.turn_angle_deg != 0 and not confirmed:
                self._consecutive_turns += 1
            elif decision.drive_distance_m > 0 or confirmed:
                self._consecutive_turns = 0

            if self._consecutive_turns >= LOOP_TURNS_TRIGGER:
                logger.info("Loop detected: %d consecutive turns — triggering scan",
                            self._consecutive_turns)
                result = await self._scan_sweep(reason="loop")
                if result is not None:
                    d = result["decision"]
                    if d.drive_distance_m > 0:
                        logger.info("SCAN: forcing drive %.1fm after loop-break scan",
                                    d.drive_distance_m)
                        self._log_entry({"event": "forced_drive",
                                         "dist": d.drive_distance_m,
                                         "reason": "post_scan"})
                        await self._execute_drive(d.drive_distance_m, 0)
                await asyncio.sleep(self.config.vlm_interval_s)
                continue

            self._apply_heading_delta(decision.turn_angle_deg)
            self._emit_state()

            await self._execute_drive(decision.drive_distance_m, decision.turn_angle_deg)

            await asyncio.sleep(self.config.vlm_interval_s)

    async def _on_arrived(self) -> None:
        """Hook called when a confirmed detection is within arrive_dist."""
        await self.beep("found")
        await self.transport.halt()
        logger.info("ARRIVED at '%s' (confirmed) — switching to teleop",
                     self.config.target)
        self._log_entry({"event": "arrived", "target": self.config.target})
        self.set_teleop(True)

    async def _execute_drive(self, distance_m: float, turn_angle_deg: int = 0) -> None:
        """Execute a drive commitment via the transport.

        Mutually exclusive: turn OR drive, never both in one step.  The VLM
        cannot reason about combined arc motions effectively, so we force
        it to pick one: if turn ≠ 0, rotate in place and discard distance;
        if turn == 0 and distance > 0, drive straight.
        """
        if turn_angle_deg != 0:
            await self.transport.rotate(turn_angle_deg, timeout_s=10.0)
            return

        if distance_m > 0.0:
            await self.transport.move_linear(distance_m, timeout_s=8.0)
        else:
            await self.transport.halt()

    def _apply_heading_delta(self, turn_angle_deg: int) -> None:
        """Update the heading counter when the transport has no real
        heading.  When the transport has real heading (Create 3 /imu),
        this is a no-op — the transport tracks yaw itself."""
        if self._desired_heading is not None and not self.transport.has_real_heading:
            self._desired_heading = self._norm_heading(
                self._desired_heading + turn_angle_deg
            )

    @staticmethod
    def _norm_heading(h: int) -> int:
        return (h % 360 + 360) % 360

    # ── Teleop ───────────────────────────────────────────────────────────

    def set_teleop(self, active: bool) -> None:
        if active and not self._teleop_active:
            self._teleop_lin = 0.0
            self._teleop_turn = 0.0
        self._teleop_active = active
        self._log_entry({"event": "mode", "teleop": active})
        if not active:
            # Switching to autonomous: reset loop counter so detection
            # starts fresh (don't inherit teleop-era turn count).
            self._consecutive_turns = 0
            asyncio.ensure_future(self.transport.halt())
        self._emit_state()

    def teleop_drive(self, lin: float, turn: float) -> None:
        self._teleop_lin = max(-1.0, min(1.0, lin))
        self._teleop_turn = max(-1.0, min(1.0, turn))
        if not self._teleop_active:
            self.set_teleop(True)

    async def _teleop_frame_loop(self) -> None:
        """Stream camera frames to the panel while teleop is active (#24).

        The autonomous loop only requests a frame once per VLM cycle
        (~0.3-1s), too sparse to drive by.  Runs as its own task so the
        frame round-trip never blocks the 20 Hz drive-command tick.
        """
        while self._running:
            if self._teleop_active:
                img = await self.transport.capture_frame()
                if img is not None:
                    self._latest_frame = img
                    self._emit_frame(img)
            await asyncio.sleep(TELEOP_FRAME_MIN_GAP_S)

    async def _teleop_tick(self) -> None:
        """Called every loop iteration while in teleop; sends drive commands
        via the transport.  Bump detector stays armed."""
        if self._hazard_event is not None:
            await self._handle_bump_recovery()
            self._teleop_lin = 0.0
            self._teleop_turn = 0.0
            return

        lin = self._teleop_lin
        turn = self._teleop_turn
        await self.transport.teleop_step(lin, turn)

    async def _handle_bump_recovery(self) -> None:
        """Emergency stop + reverse + turn, shared by teleop and autonomous."""
        logger.info("Bump — emergency stop + recovery")
        await self.beep("bump")
        await self.transport.halt()
        # Reverse via transport (linear negative)
        await self.transport.teleop_step(-0.3, 0.0)
        await asyncio.sleep(BUMP_REVERSE_MS / 1000.0)
        await self.transport.halt()
        self._apply_heading_delta(BUMP_TURN_DEG)
        self._hazard_event = None

    # ── Hazard / sensor callbacks ────────────────────────────────────────

    def _on_hazard(self, event: HazardEvent) -> None:
        """Hazard sink — called by the transport (Create 3 /hazard_detection)
        or by an IMU bump detector (RVR).  Sets the pending event so the
        drive loop and teleop tick trigger recovery."""
        logger.info("Hazard detected: kind=%s mag=%.1f", event.kind, event.magnitude)
        self._log_entry({"event": "hazard", "kind": event.kind, "magnitude": event.magnitude})
        self._hazard_event = event
        if self.on_bump_event:
            self.on_bump_event({
                "kind": event.kind,
                "magnitude": event.magnitude,
                "timestamp": event.ts,
            })

    async def _poll_battery(self) -> None:
        """Subclasses override to poll backend-specific battery sources.
        Default: no-op (transport pushes battery events)."""
        pass

    # ── Panel command handlers ───────────────────────────────────────────

    async def manual_stop(self) -> None:
        """Emergency stop — halt motors immediately. Stays in teleop mode."""
        self._teleop_lin = 0.0
        self._teleop_turn = 0.0
        await self.transport.halt()
        self._log_entry({"event": "estop"})
        self._emit_state()

    async def manual_query(self) -> None:
        """Stop, capture frame, run VLM decision + verifier, emit results.
        Does NOT drive."""
        if self._vlm_client is None:
            logger.warning("VLM client not ready; cannot run manual query")
            return
        self._log_entry({"event": "manual_query"})

        await self.transport.wait_standstill()
        img = await self.transport.capture_frame()
        if img is None:
            logger.warning("Manual query: frame capture failed")
            return
        self._latest_frame = img
        self._emit_frame(img)
        frame_path = self._save_frame(img, tag="manual")

        loop = asyncio.get_event_loop()
        prompt = self._build_prompt()
        t0 = time.time()
        decision = await loop.run_in_executor(None, self._vlm_client.query, img, prompt)
        latency_ms = (time.time() - t0) * 1000
        if decision is None:
            logger.warning("Manual query: VLM returned None")
            self._log_entry({"event": "vlm_error", "stage": "decision", "manual": True})
            return

        logger.info("MANUAL VLM: vis=%s hdg=%s turn=%+d° dist=%.2f loc=%s | %s",
                    decision.target_visible, decision.heading,
                    decision.turn_angle_deg,
                    decision.drive_distance_m, decision.target_location,
                    decision.reason[:80])
        self._emit_decision(decision, latency_ms, "", prompt)
        self._log_entry({
            "event": "decision",
            "vis": decision.target_visible,
            "heading": decision.heading,
            "turn_angle_deg": decision.turn_angle_deg,
            "dist": decision.drive_distance_m,
            "loc": decision.target_location,
            "reason": decision.reason,
            "prompt": prompt,
            "vlm_latency_ms": latency_ms,
            "frame": frame_path,
            "manual": True,
        })

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
                self._log_entry({
                    "event": "verifier",
                    "confirmed": verify.confirmed,
                    "matches": verify.matches,
                    "mismatches": verify.mismatches,
                    "reason": verify.reason,
                    "latency_ms": v_latency_ms,
                    "manual": True,
                })

    def set_target(self, target: str, description: str = "") -> None:
        old = self.config.target
        self.config.target = target
        self.config.target_description = description
        self._confirmed_count = 0
        self._emit_state()
        self._log_entry({"event": "set_target", "old": old, "new": target,
                         "description": description})

    async def set_status(self, kind: str, **kw) -> None:
        await self.transport.set_status(kind, **kw)

    async def beep(self, beep_type: str = "found", **kw) -> None:
        """Emit an audio signal.  Default: delegate to transport.set_status.
        Subclasses may override (RVR sends a phone beep message)."""
        await self.transport.set_status("audio", beep_type=beep_type, **kw)

    def toggle(self, which: str, value: Optional[bool] = None) -> bool:
        if which == "auto":
            self._auto_mode = not self._auto_mode if value is None else value
            new = self._auto_mode
        elif which == "bump":
            self._bump_enabled = not self._bump_enabled if value is None else value
            new = self._bump_enabled
            self._log_entry({"event": "toggle", "which": which, "value": new})
        elif which == "teleop":
            self.set_teleop(not self._teleop_active if value is None else value)
            new = self._teleop_active
        else:
            return False
        self._emit_state()
        return new

    # ── Prompt ───────────────────────────────────────────────────────────

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
            "  - Pick ONE action: turn in place OR drive straight — never both.",
            "    turn_angle_deg: -90/-60/-30/0/30/60/90 (+=right). drive_distance_m: 0.0-2.0 m.",
            "    To turn: set turn_angle_deg ≠ 0, drive_distance_m = 0.0.",
            "    To drive: set turn_angle_deg = 0, drive_distance_m > 0.0.",
            "    Use a turn when facing a wall or needing to search a new area.",
            "    Use drive when the target is visible and ahead, or to explore forward.",
        ]
        # Include recent action history so the VLM can detect and break
        # out of loops (e.g. turning left ↔ right in a dead end).
        if self._decision_history:
            lines.append("")
            lines.append("Recent actions (oldest → newest):")
            for d in self._decision_history:
                vis = "saw target" if d["vis"] else "no target"
                action = (f"turn={d['turn']:+d}°" if d["turn"] != 0
                          else f"drive={d['dist']:.1f}m")
                # Truncate reason to first sentence, max 50 chars
                reason = d["reason"]
                first_sentence = reason.split(".")[0].strip()
                if len(first_sentence) > 50:
                    first_sentence = first_sentence[:47] + "..."
                lines.append(f"  {action} ({vis}) — {first_sentence}")
            lines.append("AVOID back and forth turn patterns — you may be stuck in a")
            lines.append("dead end. Stick to turning in one direction when stuck, or try a different drive distance to escape.")
        lines.append("Reply JSON only.")
        return "\n".join(lines)

    # ── Panel event emitters ─────────────────────────────────────────────

    def _emit_frame(self, img: Image.Image) -> None:
        if self.on_frame:
            self.on_frame(img)

    def _save_frame(self, img: Image.Image, tag: str = "") -> str:
        """Save a frame to <run_dir>/frames/ for debugging.

        Returns the path so it can be logged alongside the VLM decision.
        """
        self._frame_seq += 1
        _FRAME_DIR.mkdir(parents=True, exist_ok=True)
        suffix = f"_{tag}" if tag else ""
        path = _FRAME_DIR / f"frame_{self._frame_seq:05d}{suffix}.jpg"
        try:
            img.save(str(path), format="JPEG", quality=90)
        except Exception as e:
            logger.warning("Failed to save frame: %s", e)
            return ""
        logger.info("Frame saved: %s (%dx%d)", path, img.size[0], img.size[1])
        return str(path)

    def _emit_decision(self, decision, latency_ms: float, frame_path: str = "",
                       prompt: str = "") -> None:
        if self.on_decision:
            self.on_decision({
                "target_visible": decision.target_visible,
                "heading": decision.heading,
                "turn_angle_deg": decision.turn_angle_deg,
                "drive_distance_m": decision.drive_distance_m,
                "target_location": decision.target_location,
                "reason": decision.reason,
                "latency_ms": latency_ms,
                "frame": frame_path,
                "prompt": prompt,
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
            "connection_state": self.transport.connection_state,
            "teleop_active": self._teleop_active,
            "auto_mode": self._auto_mode,
            "bump_enabled": self._bump_enabled,
            "confirmed_count": self._confirmed_count,
            "running": self._running,
            "scanning": self._scanning,
            "consecutive_turns": self._consecutive_turns,
            "vlm_ready": self._vlm_client is not None,
            "backend": self.transport.backend_name,
        }

    def _log_entry(self, entry: dict) -> None:
        if self._log_fh:
            entry["t"] = time.time()
            self._log_fh.write(json.dumps(entry) + "\n")
            self._log_fh.flush()

    # ── User confirmation ──────────────────────────────────────────────

    def _emit_confirm_request(self, target: str, frame_path: str,
                               reason: str) -> None:
        """Ask the panel user to confirm the target is correct."""
        if self.on_confirm_request:
            self.on_confirm_request({
                "target": target,
                "frame": frame_path,
                "reason": reason,
            })

    async def _wait_for_user_confirmation(self, target: str, frame_path: str,
                                            reason: str) -> bool:
        """Emit a confirmation request and wait for the user's response.

        Returns True if the user confirmed, False if rejected or timed out.
        Times out after 120s (auto-reject).
        """
        if not self.config.confirm_with_user:
            return True

        loop = asyncio.get_event_loop()
        self._confirm_future = loop.create_future()
        self._emit_confirm_request(target, frame_path, reason)
        self._log_entry({"event": "confirm_request", "target": target})
        logger.info("Waiting for user confirmation of '%s'...", target)

        try:
            result = await asyncio.wait_for(self._confirm_future, timeout=120.0)
            self._log_entry({"event": "confirm_result", "confirmed": result})
            return result
        except asyncio.TimeoutError:
            logger.warning("User confirmation timed out — auto-rejecting")
            self._log_entry({"event": "confirm_result", "confirmed": False,
                             "reason": "timeout"})
            return False
        finally:
            self._confirm_future = None

    def confirm_ack(self, confirmed: bool) -> None:
        """Called by the debug bus when the user responds to the confirmation."""
        if self._confirm_future and not self._confirm_future.done():
            self._confirm_future.set_result(confirmed)