"""Geometry for the delivery-zone search, no ROS. From suas_aer target_search/geometry.py: the geofence
parser, the minimum-area rectangle and the camera footprint helpers are unchanged.

Change: lat/lon -> local NED uses ArduPilot's own flat-earth scaling (Location::LOCATION_SCALING_FACTOR,
as flight_controller.ardupilot_utils.latlon_offset) instead of PX4's azimuthal projection with
R = 6371 km, which was 0.11 % off ArduPilot's local frame (suas_aer CONVERSION_NOTES).
The sweep planning around a single red rectangle (plan_path, RedZone) is replaced by
red_avoidance (the Shiva007-RAndom navigator).
Points are (north, east) tuples.
"""
import math
from dataclasses import dataclass

METRES_PER_DEG = 111318.84502145034  # ArduPilot LOCATION_SCALING_FACTOR: 1 degree of latitude


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------- lat/lon -> local NED ----------------

def latlon_to_ned(lat, lon, ref_lat, ref_lon):
    """North/east metres of (lat, lon) from the local frame origin (ref_lat, ref_lon), ArduPilot scaling."""
    return ((lat - ref_lat) * METRES_PER_DEG,
            (lon - ref_lon) * METRES_PER_DEG * math.cos(math.radians(ref_lat)))


def ned_to_latlon(north, east, ref_lat, ref_lon):
    lat = ref_lat + north / METRES_PER_DEG
    return lat, ref_lon + east / (METRES_PER_DEG * math.cos(math.radians(ref_lat)))


def parse_geofence(text):
    """[(lat, lon), ...] from a text file with one 'lat,lon' corner per line.

    Blank lines and lines starting with '#' are skipped; ',', ';', tabs and spaces all separate the
    two numbers. Anything else is an error (with the line number), never silently dropped.
    """
    points = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(";", ",").replace("\t", ",").replace(" ", ",").split(",")
        parts = [p for p in parts if p]
        try:
            if len(parts) != 2:
                raise ValueError
            lat, lon = float(parts[0]), float(parts[1])
        except ValueError:
            raise ValueError(f"geofence line {number} is not 'lat,lon': {raw!r}") from None
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f"geofence line {number} is out of range: {raw!r}")
        points.append((lat, lon))
    if len(points) < 3:
        raise ValueError(f"geofence needs at least 3 corners, found {len(points)}")
    return points


# ---------------- rectangles ----------------

@dataclass(frozen=True)
class Rect:
    """A rectangle: centre, two unit axes u and v (north, east) and the half-lengths along them."""
    cx: float
    cy: float
    ux: float
    uy: float
    vx: float
    vy: float
    hu: float
    hv: float

    @property
    def centre(self):
        return (self.cx, self.cy)

    def corners(self):
        return [(self.cx + a * self.ux + b * self.vx, self.cy + a * self.uy + b * self.vy)
                for a, b in ((-self.hu, -self.hv), (self.hu, -self.hv), (self.hu, self.hv), (-self.hu, self.hv))]


def convex_hull(points):
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def fit_rect(points, min_side=1e-3):
    """Minimum-area rectangle around the points; u is its long axis. ValueError if degenerate."""
    hull = convex_hull(points)
    if len(hull) < 3:
        raise ValueError("need at least 3 non-collinear points")
    best = None
    for i in range(len(hull)):
        p, q = hull[i], hull[(i + 1) % len(hull)]
        length = dist(p, q)
        if length < 1e-9:
            continue
        ex, ey = (q[0] - p[0]) / length, (q[1] - p[1]) / length
        nx, ny = -ey, ex
        a_vals = [x * ex + y * ey for x, y in hull]
        b_vals = [x * nx + y * ny for x, y in hull]
        a_min, a_max, b_min, b_max = min(a_vals), max(a_vals), min(b_vals), max(b_vals)
        area = (a_max - a_min) * (b_max - b_min)
        if best is None or area < best[0]:
            best = (area, ex, ey, nx, ny, a_min, a_max, b_min, b_max)
    if best is None:
        raise ValueError("degenerate point set")
    _, ex, ey, nx, ny, a_min, a_max, b_min, b_max = best
    width, height = a_max - a_min, b_max - b_min
    if min(width, height) < min_side:
        raise ValueError("points are (nearly) collinear")
    a_mid, b_mid = (a_min + a_max) / 2.0, (b_min + b_max) / 2.0
    cx, cy = a_mid * ex + b_mid * nx, a_mid * ey + b_mid * ny
    if width >= height:
        return Rect(cx, cy, ex, ey, nx, ny, width / 2.0, height / 2.0)
    return Rect(cx, cy, nx, ny, ex, ey, height / 2.0, width / 2.0)


# ---------------- camera ----------------

def footprint(altitude, hfov, image_width, image_height):
    """Ground size (m) of one frame from `altitude` m: (across image u, along image v)."""
    across = 2.0 * altitude * math.tan(hfov / 2.0)
    return across, across * image_height / image_width


def footprint_extents(across, along, row_angle):
    """(cross-track, along-track) extent (m) of a frame footprint for rows flown at `row_angle` rad
    from the camera's nose direction (0 = rows along the nose)."""
    c, s = abs(math.cos(row_angle)), abs(math.sin(row_angle))
    return across * c + along * s, across * s + along * c


def rows_needed(zone_width, cross_extent, spacing):
    """Rows (>= 1) whose footprints, `spacing` apart, cover `zone_width` across the rows."""
    if cross_extent >= zone_width or spacing <= 0.0:
        return 1
    return int(math.ceil((zone_width - cross_extent) / spacing - 1e-9)) + 1
