"""Depth back-projection for detection positioning (#9, reworked #14).

Given a target location description (left/center/etc.) plus a depth image and
camera intrinsics, compute the target's estimated position in the map (odom)
frame using a bearing-from-location + depth-column-median approach.

Camera frame assumed to be REP-105 optical: x=right, y=down, z=forward.
Camera mounted on robot base with a configurable static offset (x_fwd_m,
y_left_m, z_up_m), no roll/pitch.
"""

import math
import logging
from typing import Optional

logger = logging.getLogger(__name__)

DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 6.0
MIN_VALID_DEPTH_SAMPLES = 5
DEPTH_COL_HALF_WIDTH = 15

CAMERA_OFFSET_FORWARD_M = 0.10
CAMERA_OFFSET_LEFT_M = 0.0

CAMERA_HFOV_RAD = 1.5708

LOCATION_BEARINGS = {
    "far left":      -0.75,
    "left":          -0.42,
    "center-left":   -0.21,
    "center":         0.0,
    "center-right":   0.21,
    "right":          0.42,
    "far right":      0.75,
}


def location_to_bearing(location: str) -> Optional[float]:
    """Convert a VLM location string to a horizontal bearing (rad, +left)."""
    return LOCATION_BEARINGS.get(location)


def back_project_from_location(
    location: str,
    depth_image,
    K: list[float],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> Optional[tuple[float, float, float]]:
    """Return (x_map, y_map, depth_m) for a location-bearing target, or None.

    Maps the location to a horizontal bearing fraction of the camera HFOV,
    then reads a column patch from the depth image at that bearing to get
    a robust median depth. Back-projects using the camera intrinsics +
    robot pose.
    """
    import numpy as np

    if depth_image is None or K is None:
        return None

    bearing = location_to_bearing(location)
    if bearing is None:
        return None

    h, w = depth_image.shape[:2]

    fx = float(K[0])
    fy = float(K[4])
    cx_k = float(K[2])
    cy_k = float(K[5])
    if fx <= 0 or fy <= 0:
        return None

    cx_pix = cx_k + fx * math.tan(bearing)
    cx_pix = max(0.0, min(float(w - 1), cx_pix))

    col_lo = max(0, int(cx_pix) - DEPTH_COL_HALF_WIDTH)
    col_hi = min(w, int(cx_pix) + DEPTH_COL_HALF_WIDTH + 1)
    row_lo = int(h * 0.25)
    row_hi = int(h * 0.85)

    patch = depth_image[row_lo:row_hi, col_lo:col_hi]
    valid_mask = np.isfinite(patch) & (patch > DEPTH_MIN_M) & (patch < DEPTH_MAX_M)
    valid = patch[valid_mask]
    if valid.size < MIN_VALID_DEPTH_SAMPLES:
        return None
    depth = float(np.median(valid))

    cy_pix = 0.5 * (row_lo + row_hi)

    x_cam = (cx_pix - cx_k) * depth / fx
    y_cam = (cy_pix - cy_k) * depth / fy
    z_cam = depth

    x_base = z_cam + CAMERA_OFFSET_FORWARD_M
    y_base = -x_cam + CAMERA_OFFSET_LEFT_M
    _ = y_cam

    x_map = robot_x + math.cos(robot_yaw) * x_base - math.sin(robot_yaw) * y_base
    y_map = robot_y + math.sin(robot_yaw) * x_base + math.cos(robot_yaw) * y_base
    return x_map, y_map, depth


def stable_track_id(class_id: str, x_map: float, y_map: float, grid_m: float = 0.5) -> str:
    """Hash (class, rounded position) so repeated sightings of the same physical
    object share an id. grid_m of 0.5 means two detections within ~0.5 m of each
    other on the same axis collapse."""
    gx = round(x_map / grid_m) * grid_m
    gy = round(y_map / grid_m) * grid_m
    return f"{class_id}_{gx:+.1f}_{gy:+.1f}"
