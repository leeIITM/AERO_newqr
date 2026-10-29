"""Geographic waypoint <-> local NED (ArduPilot's flat-earth scaling, as target_search's geometry)."""
import math

METRES_PER_DEG = 111318.84502145034


def latlon_to_ned(lat, lon, ref_lat, ref_lon):
    return ((lat - ref_lat) * METRES_PER_DEG,
            (lon - ref_lon) * METRES_PER_DEG * math.cos(math.radians(ref_lat)))


def ned_to_latlon(north, east, ref_lat, ref_lon):
    return (ref_lat + north / METRES_PER_DEG,
            ref_lon + east / (METRES_PER_DEG * math.cos(math.radians(ref_lat))))
