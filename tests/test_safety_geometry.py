"""Unit tests for the safety layer clearance/cap helpers (#12, #14).

The geometry veto (rotation_allowed, wedge_reverse) was removed in #14.
These tests cover the remaining live helpers: directional clearance,
rotation clearance, and the linear velocity cap — all used by the bumper
back-off and the agent's scan gate.
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
REACT = ReactiveSafetyLayer.REACTION_TIME_S  # 0.15


def _expected_cap(clearance, react=REACT):
    """Reference impl of the reaction-then-brake cap: positive root of
    v·t + v²/(2a) = (clearance - cushion)."""
    d = clearance - CUSHION
    if d <= 0.0:
        return 0.0
    return math.sqrt(DECEL * DECEL * react * react + 2.0 * DECEL * d) - DECEL * react


def _stub_layer(react=REACT):
    """Construct a ReactiveSafetyLayer without going through __init__/ROS.

    We only need the geometry helpers + cushion/reaction attributes; the
    methods are all data-in / data-out and never touch self.node.
    """
    layer = ReactiveSafetyLayer.__new__(ReactiveSafetyLayer)
    layer.cushion_m = CUSHION
    layer.reaction_time_s = react
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

    def test_cap_matches_reaction_then_brake_formula(self):
        cap = self.s._safe_linear_cap(CUSHION + 0.18)
        assert cap == pytest.approx(_expected_cap(CUSHION + 0.18), abs=1e-9)

    def test_cap_stops_within_clearance(self):
        # The whole point: at the capped speed, reaction travel + braking
        # distance must fit inside (clearance - cushion). Check it lands on
        # the boundary (the cap is the largest such speed).
        for extra in (0.02, 0.08, 0.25, 0.60):
            d = extra  # clearance - cushion
            v = self.s._safe_linear_cap(CUSHION + d)
            stop_dist = v * REACT + v * v / (2.0 * DECEL)
            assert stop_dist == pytest.approx(d, abs=1e-9)

    def test_reaction_term_is_more_conservative_than_instant_braking(self):
        # With reaction latency the cap must be strictly below the old
        # instantaneous-braking value √(2·a·d) at any positive clearance.
        d = 0.18
        instant = math.sqrt(2 * DECEL * d)
        assert self.s._safe_linear_cap(CUSHION + d) < instant
        # And a zero-reaction layer reproduces the old formula exactly.
        s0 = _stub_layer(react=0.0)
        assert s0._safe_linear_cap(CUSHION + d) == pytest.approx(instant, abs=1e-9)

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