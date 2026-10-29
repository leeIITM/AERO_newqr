import math

from mission_management import mission_math as mm


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_heights_relative_to_pad():
    # the script recorded take-off z AFTER the climb, so takeoff_z - alt doubled the altitude;
    # here every height is from the pad
    assert mm.height_z(-0.2, 5.0) == -5.2


def test_surges_and_sway():
    t = [0.0, 0.0, -5.0]
    assert close(mm.surge_target(t, 0.0, 4.0, -5.0), [4.0, 0.0, -5.0])
    assert close(mm.sway_end([4.0, 0.0, -5.0], 0.0, 20.0), [4.0, -20.0, -5.0])  # left of north = west
    assert abs(mm.sway_distance([4.3, -7.5, -5.0], t, 0.0) - 7.5) < 1e-9


def test_wall_target():
    p = mm.wall_target([14.3, -7.5, -3.0], 0.0, 19.0, 7.5, 1.0, -5.0)
    assert close(p, [15.3, -19.0, -5.0])


def test_blue_search_start_and_sway():
    # 11 m from the green banner along the corridor (north), facing back south
    start, ret = mm.blue_search_start([4.3, -7.5], 0.0, 11.0, -5.0)
    assert close(start, [15.3, -7.5, -5.0]) and abs(abs(ret) - math.pi) < 1e-9
    # facing south, LEFT is east: the sway goes towards the adjacent return corridor
    assert close(mm.sway_end(start, ret, 20.0), [15.3, 12.5, -5.0])
    # corridor heading east: 11 m east of the banner, return heading west
    start_e, ret_e = mm.blue_search_start([0.0, 0.0], math.pi / 2, 11.0, -5.0)
    assert close(start_e, [0.0, 11.0, -5.0]) and abs(ret_e + math.pi / 2) < 1e-9
