import math

import numpy as np

from bc_common.geometry import (
    corridor_heading_from_banner, get_body_axes, image_line_heading, metres_per_pixel, order_corners,
    pixel_to_ned_offset, wrap_pi,
)


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_wrap_pi():
    assert close(wrap_pi(3 * math.pi), -math.pi)
    assert close(wrap_pi(-math.pi / 2), -math.pi / 2)
    assert close(wrap_pi(2 * math.pi + 0.1), 0.1)


def test_order_corners():
    pts = np.array([[10, 10], [0, 10], [0, 0], [10, 0]], dtype=np.float32)
    assert order_corners(pts).tolist() == [[0, 0], [10, 0], [10, 10], [0, 10]]


def test_body_axes_north_and_east():
    fwd, left, right = get_body_axes(0.0)
    assert np.allclose(fwd, (1, 0)) and np.allclose(left, (0, -1)) and np.allclose(right, (0, 1))
    fwd, left, right = get_body_axes(math.pi / 2)
    assert np.allclose(fwd, (0, 1)) and np.allclose(left, (1, 0)) and np.allclose(right, (-1, 0))


def test_pixel_to_ned_offset():
    w, h, alt = 1280, 720, 5.0
    mpp = metres_per_pixel(alt, w, 1.22)
    # 100 px above the centre = 100 mpp forward; heading north -> north
    dn, de = pixel_to_ned_offset(w / 2, h / 2 - 100, w, h, alt, 0.0)
    assert close(dn, 100 * mpp) and close(de, 0.0)
    # heading east: forward = east, image right = south
    dn, de = pixel_to_ned_offset(w / 2 + 100, h / 2, w, h, alt, math.pi / 2)
    assert close(dn, -100 * mpp) and close(de, 0.0)


def test_image_line_heading():
    # heading north, horizontal line in the image (angle 0) points east
    assert close(image_line_heading(0.0, 0.0), math.pi / 2)
    # vertical line in the image (angle -90 deg = towards image top) points forward
    assert close(image_line_heading(0.3, -math.pi / 2), 0.3)


def test_corridor_heading_perpendicular_to_banner():
    # banner across a north-going corridor, seen heading north: horizontal in the image
    h, dev, ok = corridor_heading_from_banner(0.0, 0.0, expected_heading=0.0)
    assert ok and close(h, 0.0) and close(dev, 0.0)
    # banner rotated 10 deg clockwise in the image -> corridor heading 10 deg
    h, dev, ok = corridor_heading_from_banner(0.0, math.radians(10), 0.0)
    assert ok and close(math.degrees(h), 10.0, 1e-4)
    # same banner flying the return lap (expected south): the other perpendicular is chosen
    h, dev, ok = corridor_heading_from_banner(math.pi, 0.0, expected_heading=math.pi)
    assert ok and close(abs(h), math.pi, 1e-6)
    # correction beyond 25 deg is rejected
    _, dev, ok = corridor_heading_from_banner(0.0, math.radians(40), 0.0)
    assert not ok and close(math.degrees(dev), 40.0, 1e-4)


def test_original_script_formula_was_the_banner_axis():
    """Documents the fix: the script's candidates yaw + angle +/- 90 deg are the banner's own axis,
    90 deg off the corridor for a banner lying across it."""
    yaw, angle = 0.0, 0.0
    script_candidates = [wrap_pi(yaw + angle + math.pi / 2), wrap_pi(yaw + angle - math.pi / 2)]
    assert all(abs(abs(c) - math.pi / 2) < 1e-9 for c in script_candidates)
