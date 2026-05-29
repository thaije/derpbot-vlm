"""Unit tests for the geometry-aware safety layer veto (#12).

We import the static methods + class constants directly so we don't have to
spin up a ROS node. The clearance / cap helpers are pure functions of
(scan_points, robot dimensions, cushion).
"""

import math

import pytest

from agent.safety_layer import ReactiveSafetyLayer


# Class-level shorthand
F = ReactiveSafetyLayer.ROBOT_FRONT_M     # 0.15
S = ReactiveSafetyLayer.ROBOT_SIDE_M      # 0.13
C = ReactiveSafetyLayer.ROBOT_CORNER_M    # ≈0.198
CUSHION = ReactiveSafetyLayer.SAFETY_CUSHION_M  # 0.10
DECEL = ReactiveSafetyLayer.MAX_LINEAR_DECEL_M_S2  # 2.0


def _stub_layer():
    """Construct a ReactiveSafetyLayer without going through __init__/ROS.

    We only need the geometry helpers + cushion attribute; the methods are
    all data-in / data-out and never touch self.node.
    """
    layer = ReactiveSafetyLayer.__new__(ReactiveSafetyLayer)
    layer.cushion_m = CUSHION
    return layer


# ---- _directional_clearance_m -------------------------------------------


class TestDirectionalClearance:
    def setup_method(self):
        self.s = _stub_layer()

    def test_clear_corridor_returns_inf(self):
        # Single obstacle off to the side at angle 90°, 2 m away — not in the
        # forward corridor at all.
        points = [(math.pi / 2, 2.0)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == math.inf

    def test_obstacle_directly_in_front(self):
        # Obstacle at +x = 1.0 m. Clearance should be 1.0 - FRONT = 0.85.
        points = [(0.0, 1.0)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == pytest.approx(1.0 - F, abs=1e-6)

    def test_obstacle_at_lateral_limit_still_counts(self):
        # Obstacle 1 m forward and 0.20 m to the side — inside SIDE+cushion=0.23
        # so it counts.
        theta = math.atan2(0.20, 1.0)
        r = math.hypot(1.0, 0.20)
        points = [(theta, r)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == pytest.approx(1.0 - F, abs=1e-3)

    def test_obstacle_outside_lateral_limit_ignored(self):
        # 1 m forward, 0.40 m to the side — outside the corridor.
        theta = math.atan2(0.40, 1.0)
        r = math.hypot(1.0, 0.40)
        points = [(theta, r)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == math.inf

    def test_obstacle_behind_ignored_for_forward(self):
        points = [(math.pi, 0.5)]  # straight behind
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == math.inf

    def test_obstacle_behind_counts_for_rearward(self):
        # 0.50 m behind, dead center → clearance = 0.50 - FRONT = 0.35
        # (rear face is at same |x| as front face)
        points = [(math.pi, 0.50)]
        c = self.s._directional_clearance_m(points, direction_is_forward=False)
        assert c == pytest.approx(0.50 - F, abs=1e-6)

    def test_picks_minimum_among_multiple(self):
        points = [(0.0, 2.0), (0.0, 0.30), (math.radians(20), 0.50)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        # Closest obstacle is 0.30 m straight ahead → 0.30 - 0.15 = 0.15
        assert c == pytest.approx(0.15, abs=1e-6)


# ---- _rotation_clearance_m ----------------------------------------------


class TestRotationClearance:
    def setup_method(self):
        self.s = _stub_layer()

    def test_clear_room_returns_large_clearance(self):
        points = [(0.0, 5.0), (math.pi / 2, 5.0)]
        c = self.s._rotation_clearance_m(points)
        assert c == pytest.approx(5.0 - C, abs=1e-6)

    def test_obstacle_inside_corner_radius_is_negative(self):
        # Something inside our perimeter would be a contact already.
        points = [(math.pi / 4, C - 0.05)]
        c = self.s._rotation_clearance_m(points)
        assert c < 0

    def test_takes_minimum_over_all_angles(self):
        points = [(0.0, 3.0), (math.pi, 0.40), (math.pi / 2, 5.0)]
        c = self.s._rotation_clearance_m(points)
        assert c == pytest.approx(0.40 - C, abs=1e-6)


# ---- _safe_linear_cap ----------------------------------------------------


class TestSafeLinearCap:
    def setup_method(self):
        self.s = _stub_layer()

    def test_zero_clearance_means_zero_velocity(self):
        assert self.s._safe_linear_cap(CUSHION) == 0.0
        assert self.s._safe_linear_cap(0.0) == 0.0
        assert self.s._safe_linear_cap(-1.0) == 0.0

    def test_clearance_above_cushion_scales_as_sqrt(self):
        # Clearance = cushion + 0.18 m → v_max = sqrt(2·2·0.18) = sqrt(0.72) ≈ 0.85 m/s
        cap = self.s._safe_linear_cap(CUSHION + 0.18)
        assert cap == pytest.approx(math.sqrt(2 * DECEL * 0.18), abs=1e-6)

    def test_caps_strictly_increases_with_clearance(self):
        a = self.s._safe_linear_cap(CUSHION + 0.05)
        b = self.s._safe_linear_cap(CUSHION + 0.10)
        c = self.s._safe_linear_cap(CUSHION + 0.20)
        assert 0 < a < b < c


# ---- end-to-end: clearance after the LiDAR blind-zone fix ----------------


class TestBlindZoneSemantics:
    """The _scan_cb wrapper converts r < range_min into a synthetic point at
    range_min. We just confirm that the geometry helpers behave correctly
    given such a synthetic point."""

    def setup_method(self):
        self.s = _stub_layer()

    def test_blind_zone_point_in_front_blocks_forward(self):
        # Robot pressed against a wall: the wall is closer than range_min
        # (0.15 m). _scan_cb would record (theta=0, r=0.15). Forward
        # clearance = 0.15 - 0.15 = 0 → cap = 0 → forward fully vetoed.
        points = [(0.0, 0.15)]
        c = self.s._directional_clearance_m(points, direction_is_forward=True)
        assert c == 0.0
        assert self.s._safe_linear_cap(c) == 0.0
