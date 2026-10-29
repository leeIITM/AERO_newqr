"""Geotag EXIF (same tags as airdrop_survey), capture spacing and the two-PWM drop sequence."""
import math
import os
import tempfile

import cv2
import numpy as np

from payload_drop.geotag import (
    SpacingTrigger, decimal_to_dms, dms_to_decimal, drop_sequence, read_geotag, wrap_deg, write_geotag,
)


def test_dms_round_trip():
    for value in (13.0827, 80.27118880, 0.0, 12.999999, 45.5):
        assert abs(dms_to_decimal(decimal_to_dms(value)) - value) < 3e-6   # 0.01 arcsec = 0.3 m
    d, m, s = decimal_to_dms(12.9999999)
    assert m[0] < 60 and s[0] < 6000


def test_geotag_written_and_read_back():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "frame000000.jpg")
        cv2.imwrite(path, np.full((72, 128, 3), 120, np.uint8))
        write_geotag(path, 128, 72, 13.0830001, 80.2711234, 15.25, wrap_deg(math.radians(-90)),
                     wrap_deg(0.02), wrap_deg(-0.03))
        lat, lon, alt, yaw, comment = read_geotag(path)
        assert abs(lat - 13.0830001) < 3e-6 and abs(lon - 80.2711234) < 3e-6   # 0.01 arcsec (airdrop_survey)
        assert abs(alt - 15.25) < 0.01 and abs(yaw - 270.0) < 0.01
        assert set(comment) == {"Pitch", "Roll", "Yaw"} and abs(comment["Yaw"] - 270.0) < 0.01
        assert cv2.imread(path).shape == (72, 128, 3)


def test_spacing_trigger():
    t = SpacingTrigger(3.0)
    shots = [x for x in np.arange(0.0, 10.01, 0.1) if t(x, 0.0)]
    assert [round(s, 1) for s in shots] == [0.0, 3.0, 6.0, 9.0]
    t.reset()
    assert t(9.5, 0.0)


def test_drop_sequence_two_pwms_with_interval():
    sent, slept = [], []
    err = drop_sequence(lambda ch, pwm: sent.append((ch, pwm)) or True, lambda s: slept.append(s) or True,
                        9, 1900, 1100, 2.0, log=lambda *_: None)
    assert err is None and sent == [(9, 1900), (9, 1100)] and slept == [2.0]


def test_drop_sequence_closes_even_if_cancelled():
    sent = []
    err = drop_sequence(lambda ch, pwm: sent.append(pwm) or True, lambda s: False, 9, 1900, 1100, 2.0,
                        log=lambda *_: None)
    assert err == "canceled" and sent == [1900, 1100]
    err = drop_sequence(lambda ch, pwm: False, lambda s: True, 9, 1900, 1100, 2.0, log=lambda *_: None)
    assert err and "1900" in err
