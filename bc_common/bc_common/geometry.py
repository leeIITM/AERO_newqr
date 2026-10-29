"""Geometry, maths and coordinate helpers (section 1 of banner_corridor_qr_full_mission.py).

Conventions used by every node:
  local frame   NED, x north, y east, z down (m), from the autopilot's EKF origin
  heading/yaw   rad, 0 = north, clockwise positive
  image         u right, v down; image top = body forward, image right = body right
                (gimbal pointing straight down)
"""
import math
from typing import List, Sequence, Tuple

import numpy as np


def wrap_pi(rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def order_corners(pts) -> np.ndarray:
    """Order quadrilateral points: Top-Left, Top-Right, Bottom-Right, Bottom-Left."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
    centre = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centre[1], pts[:, 0] - centre[0])
    pts = pts[np.argsort(angles)]
    start = int(np.argmin(pts.sum(axis=1)))
    return np.roll(pts, -start, axis=0).astype(np.float32)


def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float = 0.0) -> List[float]:
    """Euler roll, pitch, yaw (rad, ZYX) -> unit quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def metres_per_pixel(alt_m: float, img_w: int, hfov_rad: float = 1.22) -> float:
    """Ground metres per camera pixel at `alt_m` for a downward camera with horizontal FOV `hfov_rad`."""
    h = max(0.5, alt_m)
    return 2.0 * h * math.tan(hfov_rad / 2.0) / float(img_w)


def pixel_to_ned_offset(u: float, v: float, img_w: int, img_h: int,
                        alt_m: float, yaw_rad: float, hfov_rad: float = 1.22) -> Tuple[float, float]:
    """Pinhole reprojection of pixel (u, v) to a (north, east) offset in metres from the drone."""
    mpp = metres_per_pixel(alt_m, img_w, hfov_rad)
    dx_body = -(v - img_h / 2.0) * mpp  # +forward
    dy_body = (u - img_w / 2.0) * mpp   # +right
    dn = dx_body * math.cos(yaw_rad) - dy_body * math.sin(yaw_rad)
    de = dx_body * math.sin(yaw_rad) + dy_body * math.cos(yaw_rad)
    return dn, de


def get_body_axes(yaw_rad: float):
    """(forward, left, right) unit vectors (north, east) for a heading."""
    fwd = (math.cos(yaw_rad), math.sin(yaw_rad))
    left = (math.sin(yaw_rad), -math.cos(yaw_rad))
    right = (-math.sin(yaw_rad), math.cos(yaw_rad))
    return fwd, left, right


def offset_ne(point: Sequence[float], *terms) -> List[float]:
    """point (n, e[, d]) + sum(vector * distance) for (vector, distance) terms; keeps d unchanged."""
    out = list(point)
    for vec, dist in terms:
        out[0] += vec[0] * dist
        out[1] += vec[1] * dist
    return out


def dot_ne(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def image_line_heading(drone_yaw: float, image_angle: float) -> float:
    """World heading of a line whose angle in the image is `image_angle` (rad from image +u
    towards image +v). Image right = body right, image down = body backward, so an image
    direction (cos a, sin a) is body (forward = -sin a, right = cos a): heading = yaw + pi/2 + a."""
    return wrap_pi(drone_yaw + math.pi / 2.0 + image_angle)


def corridor_heading_from_banner(drone_yaw: float, image_angle: float, expected_heading: float,
                                 max_correction: float = math.radians(25.0)):
    """Corridor heading perpendicular to a banner seen at `image_angle`.

    The two perpendiculars of the banner's long axis are candidates; the one closer to
    `expected_heading` is kept (180 deg ambiguity). Returns (heading, deviation, accepted):
    accepted is False when the deviation from expected exceeds `max_correction`.

    Note: the original script used yaw + angle +/- 90 deg, which is the banner's own axis
    (it measures the angle from image x = body right, not from body forward)."""
    axis = image_line_heading(drone_yaw, image_angle)
    c1 = wrap_pi(axis + math.pi / 2.0)
    c2 = wrap_pi(axis - math.pi / 2.0)
    d1 = abs(wrap_pi(c1 - expected_heading))
    d2 = abs(wrap_pi(c2 - expected_heading))
    heading, dev = (c1, d1) if d1 < d2 else (c2, d2)
    return heading, dev, dev <= max_correction
