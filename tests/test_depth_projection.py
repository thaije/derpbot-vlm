import math
import numpy as np
import pytest

from agent.depth_projection import (
    back_project_from_location, location_to_bearing, stable_track_id,
    CAMERA_HFOV_RAD, DEPTH_MIN_M, DEPTH_MAX_M,
)

K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]


def _depth_image(w=640, h=480, fill=2.0):
    return np.full((h, w), fill, dtype=np.float32)


def test_location_to_bearing():
    assert location_to_bearing("center") == pytest.approx(0.0)
    assert location_to_bearing("left") < 0.0
    assert location_to_bearing("right") > 0.0
    assert location_to_bearing("far left") < location_to_bearing("left")
    assert location_to_bearing("far right") > location_to_bearing("right")
    assert location_to_bearing("center-left") < 0.0
    assert location_to_bearing("center-right") > 0.0
    assert location_to_bearing("unknown_location") is None
    assert location_to_bearing(None) is None


def test_back_project_center_at_robot_origin():
    """Target dead-centre, 2 m away, robot at origin looking +x — should give ~2 m ahead."""
    img = _depth_image(fill=2.0)
    res = back_project_from_location("center", img, K, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert res is not None
    x_map, y_map, depth = res
    assert depth == pytest.approx(2.0, abs=0.3)
    assert x_map == pytest.approx(2.10, abs=0.2)
    assert y_map == pytest.approx(0.0, abs=0.3)


def test_back_project_left_of_center_yields_positive_y_in_map():
    """Target to the left of centre should yield +y in map frame (robot-facing +x)."""
    img = _depth_image(fill=2.5)
    res = back_project_from_location("left", img, K, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert res is not None
    x_map, y_map, _ = res
    assert y_map > 0.0


def test_back_project_right_of_center_yields_negative_y_in_base():
    """Target to the right of centre should yield -y in map frame (yaw=0)."""
    img = _depth_image(fill=3.0)
    res = back_project_from_location("right", img, K, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert res is not None
    x_map, y_map, _ = res
    assert y_map < 0.0


def test_back_project_robot_rotated_90_left():
    """Robot at origin yawed +90° (facing +y). Centre target at 2 m → target at (0, 2.1)."""
    img = _depth_image(fill=2.0)
    res = back_project_from_location("center", img, K, 0.0, 0.0, math.pi / 2)
    assert res is not None
    x_map, y_map, _ = res
    assert x_map == pytest.approx(0.0, abs=0.2)
    assert y_map == pytest.approx(2.10, abs=0.2)


def test_back_project_invalid_depth_returns_none():
    img = np.zeros((480, 640), dtype=np.float32)
    res = back_project_from_location("center", img, K, 0.0, 0.0, 0.0)
    assert res is None


def test_back_project_unknown_location_returns_none():
    img = _depth_image()
    res = back_project_from_location("above", img, K, 0.0, 0.0, 0.0)
    assert res is None


def test_back_project_none_depth_returns_none():
    res = back_project_from_location("center", None, K, 0.0, 0.0, 0.0)
    assert res is None


def test_stable_track_id_collapses_nearby_detections():
    a = stable_track_id("fire_extinguisher", 2.0, -0.4)
    b = stable_track_id("fire_extinguisher", 2.1, -0.3)
    assert a == b


def test_stable_track_id_distinct_when_far():
    a = stable_track_id("fire_extinguisher", 2.0, -0.4)
    b = stable_track_id("fire_extinguisher", 5.0, 3.0)
    assert a != b