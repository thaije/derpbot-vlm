"""IMU-based bump detection for real-robot safety (#21).

Replaces the sim's bumper contact topic with an accelerometer peak
detector that streams from the phone over WebSocket. Triggers when
the acceleration magnitude exceeds a threshold for a sustained
window, signalling a collision.
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field

from .protocol import ImuMessage

logger = logging.getLogger(__name__)


@dataclass
class BumpEvent:
    timestamp: float
    magnitude: float


class BumpDetector:
    """Sliding-window accelerometer peak detector.

    Computes |accel| over a rolling window. A bump is detected when
    the short-term RMS exceeds the long-term baseline by a configurable
    factor (like a noise gate / squelch).
    """

    def __init__(
        self,
        window_size: int = 25,          # ~0.5s at 50 Hz
        threshold_factor: float = 2.5,   # short/long ratio to trigger
        min_interval_s: float = 1.0,     # suppress repeated triggers
        gravity: float = 9.81,           # expected resting |accel|
    ):
        self.window_size = window_size
        self.threshold_factor = threshold_factor
        self.min_interval_s = min_interval_s
        self.gravity = gravity

        self._magnitudes: deque[float] = deque(maxlen=window_size)
        self._last_bump_time: float = 0.0
        self._baseline: deque[float] = deque(maxlen=200)

    def feed(self, msg: ImuMessage) -> BumpEvent | None:
        mag = math.sqrt(msg.accel[0] ** 2 + msg.accel[1] ** 2 + msg.accel[2] ** 2)
        self._magnitudes.append(mag)
        self._baseline.append(mag)

        if len(self._magnitudes) < 5:
            return None

        short_rms = self._rms(list(self._magnitudes)[-10:])
        long_mean = self._mean(list(self._baseline))
        if long_mean < 1.0:
            long_mean = self.gravity

        ratio = short_rms / long_mean
        now = msg.ts if msg.ts > 0 else time.time()

        if ratio > self.threshold_factor and (now - self._last_bump_time) > self.min_interval_s:
            self._last_bump_time = now
            logger.info("BUMP detected: ratio=%.2f short_rms=%.2f long_mean=%.2f",
                        ratio, short_rms, long_mean)
            return BumpEvent(timestamp=now, magnitude=mag)

        return None

    def reset(self) -> None:
        self._magnitudes.clear()
        self._baseline.clear()
        self._last_bump_time = 0.0

    @staticmethod
    def _rms(values: list[float]) -> float:
        if not values:
            return 0.0
        return math.sqrt(sum(v * v for v in values) / len(values))

    @staticmethod
    def _mean(values: list[float]) -> float:
        if not values:
            return 0.0
        return sum(values) / len(values)