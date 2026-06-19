"""Active scan state machine for the VLM agent (#3, #14, #15).

Step-stop-shoot rotation sweep: the robot stops, settles, captures a VLM
query, then rotates by ``SCAN_STEP_RAD`` and repeats. Stationary frames are
required for accurate depth back-projection.

Extracted from ``agent_node.py`` to reduce file size. The scan controller
holds its own state; the agent passes a ``ScanContext`` with the shared
dependencies each tick.

Rotation uses the same closed-loop cumulative-yaw tracker as
``cmd_rotate`` (accumulate odom deltas, proportional ramp, 1.5° stop
threshold). The robot never rotates while a VLM query is in flight.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

SCAN_STEPS = 6                          # 6 × 60° tiles the circle (90° HFOV → 30° overlap)
SCAN_STEP_RAD = 2.0 * math.pi / SCAN_STEPS
SCAN_SETTLE_S = 0.4                     # sim-seconds to stand still before a shot
SCAN_ROTATE_TIMEOUT_S = 4.0            # give up rotating a step (fully wedged) and shoot anyway
SCAN_ANG_SPEED = 0.7                    # rad/s sweep speed
SCAN_AFTER_NO_DETECT_S = 30.0          # min sim-seconds since last confirmed detection → scan
SCAN_MIN_ROT_CLEARANCE_M = 0.20        # need ≥ this rotation clearance to bother sweeping


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a - b, result in (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass
class ScanContext:
    """Dependencies the scan controller needs from the agent each tick.

    All fields are callable/value references — the scan controller never
    touches the agent object directly, keeping the coupling minimal.
    """
    get_odom: Callable[[], tuple[float, float, float]]        # (x, y, yaw)
    sim_now: Callable[[], float]                               # current sim time (s)
    safety_command: Callable[[float, float, str], None]        # (lin, ang, src)
    is_blocked: Callable[[], bool]                             # bumper back-off active?
    rotation_clearance_m: Callable[[], float]                  # rotation clearance
    memory_mark: Callable[[float, float], None]               # mark visited cell
    submit_vlm_query: Callable[[], None]                      # submit a VLM query
    get_vlm_future: Callable[[], Any]                          # current VLM future or None
    clear_vlm_future: Callable[[], None]                       # set VLM future to None
    evaluate_candidate: Callable[[Any, float], tuple]          # (result, sim_now) → (proj, publish_ok)
    on_sighting: Callable[[Any, float, float, tuple, bool], None]  # (result, x, y, proj, publish_ok)
    end_scan: Callable[[float, str], None]                    # (sim_now, reason)
    debug_pause: Callable[[], None]


class ScanController:
    """Manages the step-stop-shoot rotation sweep lifecycle.

    Phases per step: settle → query → (evaluate) → rotate → settle → …

    The controller owns all scan state. The agent calls ``should_start()``
    to check if a scan is due, then ``start()`` and ``tick()`` each loop
    iteration until ``active`` becomes False.
    """

    def __init__(self, ctx: ScanContext):
        self.ctx = ctx
        self.active = False
        self._phase = "settle"
        self._shots = 0
        self._dir = 1.0
        self._settle_until = 0.0
        self._rotate_deadline = 0.0
        self._prev_yaw = 0.0
        self._cumulative_rad = 0.0
        self._target_rad = 0.0
        self._was_blocked = False
        self.last_end_sim = -1e9

    def should_start(self, sim_now: float, last_confirmed_sim: float,
                     in_approach: bool, is_blocked: bool) -> bool:
        """Check if a scan should be triggered (scan-on-no-detection)."""
        return (not in_approach
                and not is_blocked
                and (sim_now - last_confirmed_sim) >= SCAN_AFTER_NO_DETECT_S
                and self.ctx.rotation_clearance_m() >= SCAN_MIN_ROT_CLEARANCE_M)

    def start(self, sim_now: float) -> None:
        _, _, yaw = self.ctx.get_odom()
        self.active = True
        self._phase = "settle"          # shoot the current heading first
        self._shots = 0
        self._dir = 1.0
        self._prev_yaw = yaw
        self._cumulative_rad = 0.0
        self._target_rad = 0.0          # first shot: no rotation needed
        self._settle_until = sim_now + SCAN_SETTLE_S
        self._rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S
        self.ctx.safety_command(0.0, 0.0, "scan-stop")
        logger.info("SCAN: start (%d shots × %.0f°)",
                    SCAN_STEPS, math.degrees(SCAN_STEP_RAD))

    def end(self, sim_now: float, reason: str = "complete") -> None:
        self.active = False
        self.last_end_sim = sim_now
        self.ctx.safety_command(0.0, 0.0, "scan-stop")
        logger.info("SCAN: end (%s)", reason)

    def _prepare_next_step(self, sim_now: float) -> None:
        self._shots += 1
        if self._shots >= SCAN_STEPS:
            self.end(sim_now, reason="swept full circle")
            return
        _, _, current_yaw = self.ctx.get_odom()
        self._prev_yaw = current_yaw
        self._cumulative_rad = 0.0
        self._target_rad = self._dir * SCAN_STEP_RAD
        self._phase = "rotate"
        self._rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S

    def tick(self, sim_now: float) -> None:
        """One iteration of the scan state machine."""
        x, y, yaw = self.ctx.get_odom()
        self.ctx.memory_mark(x, y)

        # Bumper back-off owns motion; pause scan timing until it clears.
        if self.ctx.is_blocked():
            if not self._was_blocked:
                self._was_blocked = True
            self._rotate_deadline = sim_now + SCAN_ROTATE_TIMEOUT_S
            return
        if self._was_blocked:
            _, _, yaw = self.ctx.get_odom()
            self._prev_yaw = yaw
            self._was_blocked = False

        if self._phase == "rotate":
            _, _, yaw = self.ctx.get_odom()
            delta = angle_diff(yaw, self._prev_yaw)
            self._prev_yaw = yaw
            self._cumulative_rad += delta
            remaining = self._target_rad - self._cumulative_rad
            if abs(remaining) < math.radians(1.5):
                logger.info("SCAN: step rotated %.0f°/%.0f°, settling",
                            math.degrees(self._cumulative_rad),
                            math.degrees(self._target_rad))
                self.ctx.safety_command(0.0, 0.0, "scan-settle")
                self._phase = "settle"
                self._settle_until = sim_now + SCAN_SETTLE_S
            elif sim_now >= self._rotate_deadline:
                logger.info("SCAN: rotate timeout (rotated %.0f°/%.0f°) — settling anyway",
                            math.degrees(self._cumulative_rad),
                            math.degrees(self._target_rad))
                self.ctx.safety_command(0.0, 0.0, "scan-settle")
                self._phase = "settle"
                self._settle_until = sim_now + SCAN_SETTLE_S
            else:
                SCAN_ROTATE_KP = 2.0
                ang_speed = max(-SCAN_ANG_SPEED, min(SCAN_ANG_SPEED, remaining * SCAN_ROTATE_KP))
                self.ctx.safety_command(0.0, ang_speed, "scan-rotate")
            return

        if self._phase == "settle":
            self.ctx.safety_command(0.0, 0.0, "scan-settle")
            vlm_future = self.ctx.get_vlm_future()
            if sim_now >= self._settle_until and vlm_future is None:
                self.ctx.submit_vlm_query()
                vlm_future = self.ctx.get_vlm_future()
                if vlm_future is not None:
                    self._phase = "query"
                else:
                    logger.warning("SCAN: VLM submission failed (no image?) — ending scan")
                    self.end(sim_now, reason="vlm_submit_failed")
            return

        if self._phase == "query":
            self.ctx.safety_command(0.0, 0.0, "scan-query")
            vlm_future = self.ctx.get_vlm_future()
            if vlm_future is None:
                logger.warning("SCAN: VLM future is None in query phase — ending scan")
                self.end(sim_now, reason="vlm_future_none")
                return
            if not vlm_future.done():
                return
            try:
                result = vlm_future.result()
            except Exception as e:
                logger.error("SCAN VLM error: %s", e)
                result = None
            self.ctx.clear_vlm_future()
            if result is not None:
                logger.info("SCAN shot %d/%d: vis=%s | %s",
                            self._shots + 1, SCAN_STEPS,
                            result.target_visible, result.reason[:80])
                proj, publish_ok = self.ctx.evaluate_candidate(result, sim_now)
                sim_now = self.ctx.sim_now()
                x, y, yaw = self.ctx.get_odom()
                if result.target_visible:
                    self.ctx.on_sighting(result, x, y, proj, publish_ok)
                    self.end(sim_now, reason="approaching sighting")
                    self.ctx.debug_pause()
                    return
            sim_now = self.ctx.sim_now()
            self._prepare_next_step(sim_now)
            return