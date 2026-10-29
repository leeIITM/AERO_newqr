"""Stage 2 reference-QR search pattern (growing squares round the surge-1 point)."""
from mission_management.mission_math import square_offsets


def test_growing_squares_then_back_to_centre():
    offs = square_offsets(1.0, 3.0)
    assert offs[:8] == [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, -1), (-1, 1)]
    assert offs[8:12] == [(2, 0), (0, 2), (-2, 0), (0, -2)]
    assert offs[20:24] == [(3, 3), (3, -3), (-3, -3), (-3, 3)]
    assert offs[-1] == (0.0, 0.0) and len(offs) == 25


def test_degenerate_settings_hover_only():
    assert square_offsets(0.0, 3.0) == [(0.0, 0.0)]
    assert square_offsets(2.0, 1.0) == [(0.0, 0.0)]
