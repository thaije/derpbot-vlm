"""Planner — Layer 2 of #9.

Converts VLM NavigationDecisions into low-level (heading, distance) commitments
and computes a desired (linear, angular) at each tick. Hands the result to the
safety layer.

Commitment lifecycle:
  - accept_decision() seeds yaw_target = current_yaw + heading_offset and
    distance_target = clamp(drive_distance_m, 0, 2).
  - compute_command(x, y, yaw, sim_now) returns:
      * (0, ω) while not aligned (|yaw_err| > YAW_TOLERANCE_RAD)
      * (DRIVE_SPEED, small ω correction) while aligned and below distance_target
      * (0, 0) when distance reached, deadline elapsed, or no commitment
  - Commitment ends when distance achieved OR deadline OR cancel().
"""

import logging
import math
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

HEADING_OFFSETS_RAD = {
    "left": math.radians(30.0),
    "center": 0.0,
    "right": math.radians(-30.0),
}

DRIVE_SPEED = 0.4
ROTATE_SPEED_MAX = 0.8
YAW_TOLERANCE_RAD = math.radians(8.0)
FORWARD_YAW_CORRECTION_GAIN = 1.0
FORWARD_YAW_CORRECTION_MAX = 0.3

DEFAULT_COMMIT_TIMEOUT_S = 10.0
APPROACH_COMMIT_TIMEOUT_S = 6.0
APPROACH_DIST_M = 1.5
MAX_DISTANCE_M = 2.0



@dataclass
class Commitment:
    yaw_target: float
    distance_target_m: float
    start_x: float
    start_y: float
    deadline_sim_s: float
    issued_sim_s: float
    is_approach: bool
    target_visible: bool
    heading: str
    drive_started: bool = False


class Planner:
    def __init__(self):
        self._commitment: Optional[Commitment] = None
        self._last_completion_reason: str = "init"

    def is_idle(self) -> bool:
        return self._commitment is None

    def in_approach(self) -> bool:
        return self._commitment is not None and self._commitment.is_approach

    @property
    def commitment(self) -> Optional[Commitment]:
        return self._commitment

    @property
    def last_completion_reason(self) -> str:
        return self._last_completion_reason

    def cancel(self, reason: str = "cancelled") -> None:
        if self._commitment is not None:
            self._last_completion_reason = reason
            self._commitment = None

    def accept_decision(self, decision, current_yaw: float, x: float, y: float, sim_now: float) -> Commitment:
        offset = HEADING_OFFSETS_RAD.get(decision.heading, 0.0)
        yaw_target = self._normalize(current_yaw + offset)

        dist = max(0.0, min(MAX_DISTANCE_M, float(decision.drive_distance_m)))
        is_approach = bool(decision.target_visible) and dist <= APPROACH_DIST_M
        timeout = APPROACH_COMMIT_TIMEOUT_S if is_approach else DEFAULT_COMMIT_TIMEOUT_S

        self._commitment = Commitment(
            yaw_target=yaw_target,
            distance_target_m=dist,
            start_x=x,
            start_y=y,
            deadline_sim_s=sim_now + timeout,
            issued_sim_s=sim_now,
            is_approach=is_approach,
            target_visible=bool(decision.target_visible),
            heading=str(decision.heading),
        )
        logger.info("PLANNER: commit hdg=%s dist=%.2fm yaw_tgt=%.2frad approach=%s",
                    decision.heading, dist, yaw_target, is_approach)
        return self._commitment

    def compute_command(self, x: float, y: float, yaw: float, sim_now: float) -> tuple[float, float]:
        c = self._commitment
        if c is None:
            return 0.0, 0.0

        traveled = math.hypot(x - c.start_x, y - c.start_y)
        if c.distance_target_m <= 0.0:
            self._complete("zero_distance")
            return 0.0, 0.0
        if traveled >= c.distance_target_m:
            self._complete("distance_reached")
            return 0.0, 0.0
        if sim_now >= c.deadline_sim_s:
            self._complete("deadline")
            return 0.0, 0.0

        err = self._normalize(c.yaw_target - yaw)
        if abs(err) > YAW_TOLERANCE_RAD:
            ang = max(-ROTATE_SPEED_MAX, min(ROTATE_SPEED_MAX, err * 1.5))
            return 0.0, ang

        c.drive_started = True
        ang_correction = max(
            -FORWARD_YAW_CORRECTION_MAX,
            min(FORWARD_YAW_CORRECTION_MAX, err * FORWARD_YAW_CORRECTION_GAIN),
        )
        return DRIVE_SPEED, ang_correction

    def _complete(self, reason: str) -> None:
        c = self._commitment
        if c is not None:
            logger.info("PLANNER: commitment ended (%s) heading=%s dist_target=%.2fm",
                        reason, c.heading, c.distance_target_m)
        self._last_completion_reason = reason
        self._commitment = None

    @staticmethod
    def _normalize(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
