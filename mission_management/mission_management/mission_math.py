"""Mission geometry of blue_green_qr_fixed.py's run() (stages 2-12), ROS-free so it is unit-tested.

Points are [north, east, down] in the local NED frame. Altitudes are metres above the take-off pad;
`ground_z` is the NED z of the pad recorded before take-off, so a height h is z = ground_z - h.
"""
import math
from typing import List, Sequence, Tuple

from bc_common.geometry import dot_ne, get_body_axes, offset_ne, wrap_pi


def height_z(ground_z: float, height: float) -> float:
    return ground_z - height


def surge_target(takeoff: Sequence[float], takeoff_yaw: float, distance: float, z: float) -> List[float]:
    """Stages 2-3: `distance` forward of the take-off point along the take-off heading."""
    fwd, _, _ = get_body_axes(takeoff_yaw)
    return offset_ne([takeoff[0], takeoff[1], z], (fwd, distance))


def sway_end(start: Sequence[float], heading: float, max_sway: float) -> List[float]:
    """Stage 4 / 11: end of a left sway of `max_sway` m, left of `heading`."""
    _, left, _ = get_body_axes(heading)
    return offset_ne(start, (left, max_sway))


def sway_distance(banner: Sequence[float], takeoff: Sequence[float], takeoff_yaw: float) -> float:
    """Stage 5: sway x = banner position from the take-off point along the take-off left axis."""
    _, left, _ = get_body_axes(takeoff_yaw)
    return dot_ne([banner[0] - takeoff[0], banner[1] - takeoff[1]], left)


def wall_target(exit_pos: Sequence[float], corridor_heading: float, wall_offset: float, sway_x: float,
                forward: float, z: float) -> List[float]:
    """Stage 8: left by (wall_offset - x) along the corridor's left axis and `forward` m on, at z
    (1 m from the 20 m wall)."""
    fwd, left, _ = get_body_axes(corridor_heading)
    return offset_ne([exit_pos[0], exit_pos[1], z], (left, wall_offset - sway_x), (fwd, forward))


def blue_search_start(green: Sequence[float], corridor_heading: float, distance: float,
                      z: float) -> Tuple[List[float], float]:
    """Stage 11: the blue search starts `distance` m (corridor length + 1 m = 11 m) from the green banner
    along the corridor axis, i.e. 1 m past the corridor exit, facing back (corridor heading + 180 deg).
    Returns (start point, return heading)."""
    fwd, _, _ = get_body_axes(corridor_heading)
    return offset_ne([green[0], green[1], z], (fwd, distance)), wrap_pi(corridor_heading + math.pi)


def square_offsets(step, max_half):
    """(forward, left) offsets of growing squares round a point: for half-size r = step, 2*step, ... up to
    max_half, the four edge midpoints then the four corners; ends at the centre. Repeated forever."""
    out = []
    k = 1
    while True:
        r = k * step
        if r > max_half + 1e-9 or step <= 0:
            break
        out += [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r), (r, r), (r, -r), (-r, -r), (-r, r)]
        k += 1
    return out + [(0.0, 0.0)] if out else [(0.0, 0.0)]
