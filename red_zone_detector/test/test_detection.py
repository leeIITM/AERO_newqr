import numpy as np
import cv2

from red_zone_detector.detection import find_patches, red_mask, simplify_contour

L1, U1 = np.array([0, 80, 40], np.uint8), np.array([15, 255, 255], np.uint8)
L2, U2 = np.array([165, 80, 40], np.uint8), np.array([180, 255, 255], np.uint8)


def test_one_red_square_one_patch():
    img = np.full((720, 1280, 3), (90, 100, 110), np.uint8)
    cv2.rectangle(img, (400, 200), (600, 400), (30, 20, 215), -1)
    cv2.rectangle(img, (900, 100), (960, 160), (0, 177, 64)[::-1], -1)   # green: not red
    patches = find_patches(red_mask(img, L1, U1, L2, U2), 100)
    assert len(patches) == 1
    cx, cy = patches[0]["centroid"]
    assert abs(cx - 500) < 2 and abs(cy - 300) < 2
    outline = simplify_contour(patches[0]["contour"])
    xs, ys = [p[0] for p in outline], [p[1] for p in outline]
    assert 4 <= len(outline) <= 8 and min(xs) < 405 and max(xs) > 595 and min(ys) < 205 and max(ys) > 395


def test_specks_ignored_and_hue_wrap():
    img = np.full((480, 640, 3), 120, np.uint8)
    cv2.rectangle(img, (10, 10), (14, 14), (0, 0, 255), -1)                 # 25 px^2 speck
    cv2.rectangle(img, (300, 200), (380, 280), (60, 0, 230), -1)           # magenta-ish red (hue ~174)
    patches = find_patches(red_mask(img, L1, U1, L2, U2), 100)
    assert len(patches) == 1 and patches[0]["area"] > 5000
