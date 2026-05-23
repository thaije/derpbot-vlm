"""Depth back-projection for detection positioning — Phase 4 of #9.

Given a target bbox in pixel coordinates plus a depth image and camera
intrinsics, compute the target's estimated position in the map (odom) frame.

Camera frame assumed to be REP-105 optical: x=right, y=down, z=forward.
Camera mounted on robot base with a configurable static offset (x_fwd_m,
y_left_m, z_up_m), no roll/pitch. Sufficient for Gazebo's forward-mounted
RGBD camera; TF would be the correct path if mounting becomes non-trivial.
"""

import math
import logging
from typing import Optional

logger = logging.getLogger(__name__)

DEPTH_MIN_M = 0.15
DEPTH_MAX_M = 6.0
MIN_VALID_DEPTH_SAMPLES = 5

# DerpBot RGBD camera is mounted ~10 cm forward of base_footprint, level.
# (Approximate — see robot-sandbox SDF; refine if numbers look off.)
CAMERA_OFFSET_FORWARD_M = 0.10
CAMERA_OFFSET_LEFT_M = 0.0


def back_project_bbox(
    bbox: list[int],
    depth_image,
    K: list[float],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> Optional[tuple[float, float, float]]:
    """Return (x_map, y_map, depth_m) of the target, or None if depth invalid.

    bbox: [x1, y1, x2, y2] in pixels (top-left origin).
    depth_image: 2-D numpy array of depths in metres (float32).
    K: row-major 3x3 camera intrinsics (length-9 list, [fx,0,cx, 0,fy,cy, 0,0,1]).
    """
    import numpy as np

    if depth_image is None or bbox is None or len(bbox) != 4:
        return None

    h, w = depth_image.shape[:2]
    x1 = max(0, min(w - 1, int(bbox[0])))
    y1 = max(0, min(h - 1, int(bbox[1])))
    x2 = max(0, min(w - 1, int(bbox[2])))
    y2 = max(0, min(h - 1, int(bbox[3])))
    if x1 >= x2 or y1 >= y2:
        return None

    patch = depth_image[y1:y2, x1:x2]
    valid_mask = np.isfinite(patch) & (patch > DEPTH_MIN_M) & (patch < DEPTH_MAX_M)
    valid = patch[valid_mask]
    if valid.size < MIN_VALID_DEPTH_SAMPLES:
        return None
    depth = float(np.median(valid))

    cx_pix = 0.5 * (x1 + x2)
    cy_pix = 0.5 * (y1 + y2)

    fx = float(K[0])
    fy = float(K[4])
    cx_k = float(K[2])
    cy_k = float(K[5])
    if fx <= 0 or fy <= 0:
        return None

    x_cam = (cx_pix - cx_k) * depth / fx
    y_cam = (cy_pix - cy_k) * depth / fy
    z_cam = depth

    # Optical → base: base_x = z_cam (+ forward offset), base_y = -x_cam (+ left offset)
    x_base = z_cam + CAMERA_OFFSET_FORWARD_M
    y_base = -x_cam + CAMERA_OFFSET_LEFT_M
    # z dimension dropped — ground-plane target assumption is good enough here.
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
