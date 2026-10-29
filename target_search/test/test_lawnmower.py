from target_search.lawnmower import lawnmower_waypoints


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_lawnmower_waypoints():
    # script defaults: 5 rows x 18 m along the corridor (north), 4 m apart stepping right (east)
    wps = lawnmower_waypoints([15.3, -19.0, -5.0], 0.0, 18.0, 4.0, 5)
    assert len(wps) == 10
    assert close(wps[0], [15.3, -19.0, -5.0]) and close(wps[1], [33.3, -19.0, -5.0])
    assert close(wps[2], [33.3, -15.0, -5.0]) and close(wps[3], [15.3, -15.0, -5.0])  # second row reversed
    assert close(wps[-1], [33.3, -3.0, -5.0])
    assert all(w[2] == -5.0 for w in wps)  # whole search at the start's altitude
