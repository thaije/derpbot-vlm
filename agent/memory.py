"""Visited-cells memory — Phase 3 of #9.

Tracks robot odom positions on a coarse grid. Renders a compact text summary
for the VLM prompt so the model can prefer unexplored headings.

No SLAM, no map publishing — just a visited-set keyed by integer cell index.
"""

import math
from threading import Lock
from typing import Iterable


CELL_SIZE_M = 0.5
LOOKAHEAD_M = 2.0
SAMPLES_PER_RAY = 4  # rays sampled at 0.5, 1.0, 1.5, 2.0 m


class VisitedCells:
    def __init__(self, cell_size_m: float = CELL_SIZE_M):
        self.cell_size_m = cell_size_m
        self._cells: set[tuple[int, int]] = set()
        self._lock = Lock()

    def mark(self, x: float, y: float) -> None:
        c = self._to_cell(x, y)
        with self._lock:
            self._cells.add(c)

    def count(self) -> int:
        with self._lock:
            return len(self._cells)

    def is_visited(self, x: float, y: float) -> bool:
        c = self._to_cell(x, y)
        with self._lock:
            return c in self._cells

    def _to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(math.floor(x / self.cell_size_m)),
            int(math.floor(y / self.cell_size_m)),
        )

    def heading_summary(self, x: float, y: float, yaw: float) -> dict:
        """For each named heading offset, count how many sample points along
        the ray (out to LOOKAHEAD_M) fall on already-visited cells.

        Returns {heading_name: visited_ratio_0_to_1}.
        """
        offsets = {
            "left": math.radians(30.0),
            "center": 0.0,
            "right": math.radians(-30.0),
        }
        out = {}
        with self._lock:
            cells_snapshot = self._cells.copy()
        for name, offset in offsets.items():
            theta = yaw + offset
            visited = 0
            for i in range(1, SAMPLES_PER_RAY + 1):
                d = i * (LOOKAHEAD_M / SAMPLES_PER_RAY)
                px = x + d * math.cos(theta)
                py = y + d * math.sin(theta)
                if self._to_cell(px, py) in cells_snapshot:
                    visited += 1
            out[name] = visited / SAMPLES_PER_RAY
        return out

    def render_prompt_summary(self, x: float, y: float, yaw: float) -> str:
        """Short text describing whether each heading leads into visited area."""
        ratios = self.heading_summary(x, y, yaw)
        parts = []
        for name in ("left", "center", "right"):
            r = ratios.get(name, 0.0)
            if r >= 0.75:
                tag = "mostly previously visited"
            elif r >= 0.25:
                tag = "partially visited"
            else:
                tag = "unexplored"
            parts.append(f"{name}: {tag}")
        return "; ".join(parts)
