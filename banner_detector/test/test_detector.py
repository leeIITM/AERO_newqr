import math

import cv2
import numpy as np

from banner_detector.detector import BannerDetector, BannerParams
from banner_detector.ocr import canon, is_banner_text, partial_match_distance

GREEN = (64, 177, 0)
BLUE = (255, 0, 0)


def scene(color, center=(640, 360), size=(640, 56), angle=0.0, w=1280, h=720):
    img = np.full((h, w, 3), (90, 100, 110), dtype=np.uint8)
    box = cv2.boxPoints(((center[0], center[1]), size, angle))
    cv2.fillPoly(img, [np.int32(box)], color)
    return img


def detector(**kw):
    return BannerDetector(BannerParams(use_ocr=False, **kw), log=lambda *_: None)


def test_green_banner_needs_persistence():
    d = detector()
    img = scene(GREEN)
    results = [d.detect(img, "green", alt_rel=5.0)[0] for _ in range(3)]
    assert results[0] is None and results[1] is None
    b = results[2]
    assert b is not None and b["source"] == "SHAPE" and not b["partial"]
    assert abs(b["u"] - 640) < 3 and abs(b["v"] - 360) < 3


def test_angle_reported():
    d = detector(check_physical_size=False)
    img = scene(GREEN, angle=20.0)
    for _ in range(3):
        b, _, _ = d.detect(img, "green", alt_rel=5.0)
    assert b is not None and abs(math.degrees(b["angle"]) - 20.0) < 2.0


def test_rejections():
    d = detector()
    _, diags, _ = d.detect(scene(GREEN, center=(60, 360)), "green", 5.0)
    assert diags and diags[0]["reason"].startswith("Border")
    _, diags, _ = d.detect(scene(GREEN, size=(200, 100)), "green", 5.0)
    assert diags and diags[0]["reason"].startswith("Ratio")
    # 640 px at 5 m with hfov 1.22 is ~3.5 m: at 15 m the same pixels are ~10.5 m -> too long
    _, diags, _ = d.detect(scene(GREEN), "green", 15.0)
    assert diags and diags[0]["reason"].startswith("Physical")


def test_colour_selectivity():
    d = detector()
    img = scene(BLUE)
    assert all(d.detect(img, "green", 5.0)[0] is None for _ in range(4))
    d.reset()
    assert [d.detect(img, "blue", 5.0)[0] is not None for _ in range(3)][-1]


def test_filmed_blue_hue_detected():
    # a printed/filmed #0000FF reads ~102-118 in the camera: the window centred on 112 must keep it,
    # the old window centred on 120 ([105, 135]) cut it off
    filmed_blue = (200, 130, 20)   # H = 102
    d = detector()
    assert [d.detect(scene(filmed_blue), "blue", 5.0)[0] is not None for _ in range(3)][-1]
    old = detector(blue_h_center=-1)
    assert all(old.detect(scene(filmed_blue), "blue", 5.0)[0] is None for _ in range(4))


def test_ocr_text_rules():
    assert canon("AeroTHON 2026") == canon("aer0th0n2026")
    assert partial_match_distance(canon("xx AER0THN 2026"), canon("aerothon")) <= 2
    assert is_banner_text("AeroTHON 2026")
    assert is_banner_text("AER0TH0N   2O26")
    assert not is_banner_text("AeroTHON")          # year required
    assert not is_banner_text("2026 CORRIDOR")     # brand required
