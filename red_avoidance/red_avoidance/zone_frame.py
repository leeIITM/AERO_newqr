"""Zone frame: where avoidance.py runs inside the banner-corridor workspace.

avoidance.py works in a "world" frame with X and Y axes (Gazebo: X east, Y north), yaw measured from
+Y clockwise towards +X, and it flies axis-aligned legs over an axis-aligned room. Our delivery zone
is a geofence rectangle at any bearing in the autopilot's local NED frame, so every point is first
rotated into the zone frame:

    +x_z  along the zone's long side (heading `axis_heading`)
    +y_z  90 deg to the LEFT of +x_z (heading axis_heading - 90 deg)
    origin at the zone centre

That is a proper rotation of avoidance.py's X-east / Y-north frame (x_z plays X, y_z plays Y), so
its maths (projection, footprints, the red grid, detours) carries over unchanged, and its
axis-aligned rows and legs run along the zone's sides.
"""
import math

import numpy as np

from red_avoidance.avoidance import RedMap


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class ZoneFrame:
    def __init__(self, origin_north, origin_east, axis_heading):
        self.on, self.oe = float(origin_north), float(origin_east)
        self.axis_heading = float(axis_heading)
        self.ex = (math.cos(axis_heading), math.sin(axis_heading))   # +x_z in (north, east)
        self.ey = (self.ex[1], -self.ex[0])                          # +y_z: 90 deg left of +x_z
        self.y_heading = math.atan2(self.ey[1], self.ey[0])

    def to_zone(self, north, east):
        dn, de = north - self.on, east - self.oe
        return (dn * self.ex[0] + de * self.ex[1], dn * self.ey[0] + de * self.ey[1])

    def to_ned(self, x, y):
        return (self.on + x * self.ex[0] + y * self.ey[0], self.oe + x * self.ex[1] + y * self.ey[1])

    def yaw_to_zone(self, heading):
        """avoidance.py yaw (from +y_z, clockwise) of a NED heading."""
        return wrap_pi(heading - self.y_heading)

    def yaw_to_ned(self, yaw_z):
        return wrap_pi(yaw_z + self.y_heading)

    def drone(self, north, east, alt, heading, roll=None, pitch=None):
        """The `drone` tuple avoidance.project_pixel / footprint expect, in the zone frame."""
        x, y = self.to_zone(north, east)
        base = (x, y, alt, self.yaw_to_zone(heading))
        return base if roll is None else base + (roll, pitch)


# ---------------------------------------------------------------- red map <-> plain grid


def redmap_to_grid(red):
    """(x0, y0, cell, grid uint8 array) of a RedMap (the red seen, not the filled hull)."""
    return red.x0, red.y0, red.cell, red.grid


def redmap_from_grid(x0, y0, cell, grid, min_size=0.3, join=1.5):
    """A RedMap holding `grid` (e.g. rebuilt from a mission_interfaces/RedMap message)."""
    red = RedMap.__new__(RedMap)
    red.cell, red.min_size, red.join = float(cell), min_size, join
    red.x0, red.y0 = float(x0), float(y0)
    red.grid = np.ascontiguousarray(np.asarray(grid, dtype=np.uint8))
    red.version = 1 if red.grid.any() else 0
    red._cache, red._cache_version = {}, -1
    red._filled = None
    red.looked = None
    return red
