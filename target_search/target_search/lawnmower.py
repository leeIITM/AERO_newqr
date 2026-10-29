"""Stage 9 lawnmower waypoints (script's serpentine), ROS-free."""
from typing import List, Sequence

from bc_common.geometry import get_body_axes, offset_ne


def lawnmower_waypoints(start: Sequence[float], heading: float, length: float, row_step: float,
                        rows: int) -> List[List[float]]:
    """Row r: from start + right * r * row_step, `length` m along `heading`; every other row reversed."""
    fwd, _, right = get_body_axes(heading)
    wps = []
    for r in range(rows):
        p0 = offset_ne(list(start), (right, r * row_step))
        p1 = offset_ne(p0, (fwd, length))
        wps += [p0, p1] if r % 2 == 0 else [p1, p0]
    return wps
