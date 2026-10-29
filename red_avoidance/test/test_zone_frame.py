import math

import numpy as np

from red_avoidance.avoidance import RedMap, footprint, project_pixel
from red_avoidance.zone_frame import ZoneFrame, redmap_from_grid, redmap_to_grid

W, H, HFOV = 1280, 720, 1.22


def pixel_to_ned(u, v, north, east, alt, heading):
    """Reference nadir pinhole model in NED (image up = nose), as bc_common.geometry.pixel_to_ned_offset."""
    mpp = 2.0 * alt * math.tan(HFOV / 2.0) / W
    fwd, right = -(v - H / 2.0) * mpp, (u - W / 2.0) * mpp
    return (north + fwd * math.cos(heading) - right * math.sin(heading),
            east + fwd * math.sin(heading) + right * math.cos(heading))


def test_round_trip_and_axes():
    for axis in (0.0, 0.5, math.pi / 2, -2.0):
        f = ZoneFrame(10.0, -4.0, axis)
        n, e = f.to_ned(*f.to_zone(3.0, 7.0))
        assert abs(n - 3.0) < 1e-9 and abs(e - 7.0) < 1e-9
        # +x_z is the axis heading, +y_z 90 deg to its left
        n, e = f.to_ned(1.0, 0.0)
        assert abs(math.atan2(e + 4.0, n - 10.0) - axis) < 1e-9 or abs(abs(axis) - math.pi) < 1e-9
        n, e = f.to_ned(0.0, 1.0)
        left = axis - math.pi / 2
        assert abs(n - 10.0 - math.cos(left)) < 1e-9 and abs(e + 4.0 - math.sin(left)) < 1e-9


def test_repo_projection_in_zone_frame_matches_ned_pinhole():
    """avoidance.project_pixel fed zone-frame poses must land where the NED pinhole model says."""
    for axis in (0.0, 0.7, -1.9):
        f = ZoneFrame(20.0, -10.0, axis)
        for heading in (0.0, 1.0, -2.5):
            drone = f.drone(23.0, -8.0, 5.0, heading)
            for u, v in ((640, 360), (100, 50), (1200, 700), (300, 600)):
                x, y = project_pixel(u, v, drone, HFOV, W, H)
                n, e = f.to_ned(x, y)
                rn, re = pixel_to_ned(u, v, 23.0, -8.0, 5.0, heading)
                assert math.hypot(n - rn, e - re) < 1e-6


def test_footprint_contains_the_image_centre_ground():
    f = ZoneFrame(0.0, 0.0, 0.6)
    box = footprint(f.drone(5.0, 5.0, 5.0, 1.1), HFOV, W, H, shrink=0.4)
    x, y = f.to_zone(5.0, 5.0)
    assert box[0] < x < box[2] and box[1] < y < box[3]


def test_red_map_grid_round_trip():
    room = ((-10.0, 10.0), (-8.0, 8.0))
    red = RedMap(room)
    assert red.add_box((1.0, 1.0, 3.0, 3.0))
    copy = redmap_from_grid(*redmap_to_grid(red))
    for p in ((2.0, 2.0), (3.5, 2.0), (0.0, 0.0), (6.0, 6.0)):
        assert copy.blocked(p, 1.0) == red.blocked(p, 1.0)
    empty = redmap_from_grid(red.x0, red.y0, red.cell, np.zeros_like(red.grid))
    assert not empty.blocked((2.0, 2.0), 1.0)
