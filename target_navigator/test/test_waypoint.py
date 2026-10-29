from target_navigator.waypoint import latlon_to_ned, ned_to_latlon


def test_round_trip_matches_target_search():
    from target_search import geometry as geo
    ref = (13.08288, 80.27070)
    for n, e in ((24.0, -11.3), (-12.5, 33.0), (0.0, 0.0)):
        lat, lon = geo.ned_to_latlon(n, e, *ref)        # what target_search reports
        nn, ee = latlon_to_ned(lat, lon, *ref)
        assert abs(nn - n) < 1e-6 and abs(ee - e) < 1e-6
        assert all(abs(a - b) < 1e-12 for a, b in zip(ned_to_latlon(n, e, *ref), (lat, lon)))
