import math
import numpy as np
import pytest

from agent.depth_projection import back_project_bbox, stable_track_id


# 640x480, fx=fy=400, cx=320, cy=240 — synthetic pinhole.
K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]


def _depth_image(w=640, h=480, fill=2.0):
    return np.full((h, w), fill, dtype=np.float32)


def test_back_project_center_at_robot_origin():
    """Target dead-centre, 2 m away, robot at origin looking +x.
    Result should be ~2 m ahead in map frame (plus camera fwd offset)."""
    img = _depth_image(fill=2.0)
    bbox = [310, 230, 330, 250]
    res = back_project_bbox(bbox, img, K, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert res is not None
    x_map, y_map, depth = res
    assert depth == pytest.approx(2.0, abs=1e-3)
    assert x_map == pytest.approx(2.10, abs=0.05)  # 2 m + 0.10 cam offset
    assert y_map == pytest.approx(0.0, abs=0.05)


def test_back_project_right_of_center_yields_negative_y_in_base():
    """Target to the right of optical centre → should be on robot's -y (right)."""
    img = _depth_image(fill=3.0)
    # x1=420 means pixel center ~430, well right of cx=320
    bbox = [420, 230, 440, 250]
    res = back_project_bbox(bbox, img, K, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert res is not None
    x_map, y_map, _ = res
    assert y_map < 0.0  # right of robot in robot/map frame (yaw=0)


def test_back_project_robot_rotated_90_left():
    """Robot at origin yawed +90° (facing +y). Centered bbox 2 m → target at (0, 2.1)."""
    img = _depth_image(fill=2.0)
    bbox = [310, 230, 330, 250]
    res = back_project_bbox(bbox, img, K, 0.0, 0.0, math.pi / 2)
    assert res is not None
    x_map, y_map, _ = res
    assert x_map == pytest.approx(0.0, abs=0.05)
    assert y_map == pytest.approx(2.10, abs=0.05)


def test_back_project_invalid_depth_returns_none():
    img = np.zeros((480, 640), dtype=np.float32)  # all zeros — below DEPTH_MIN_M
    bbox = [310, 230, 330, 250]
    assert back_project_bbox(bbox, img, K, 0.0, 0.0, 0.0) is None


def test_back_project_empty_bbox_returns_none():
    img = _depth_image()
    assert back_project_bbox([10, 10, 10, 10], img, K, 0.0, 0.0, 0.0) is None


def test_stable_track_id_collapses_nearby_detections():
    a = stable_track_id("fire_extinguisher", 2.0, -0.4)
    b = stable_track_id("fire_extinguisher", 2.1, -0.3)
    assert a == b


def test_stable_track_id_distinct_when_far():
    a = stable_track_id("fire_extinguisher", 2.0, -0.4)
    b = stable_track_id("fire_extinguisher", 5.0, 3.0)
    assert a != b
