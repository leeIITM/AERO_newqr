"""Red / restricted-zone detection, no ROS. red_mask, find_patches and annotate are ported verbatim from
red_zone/red_zone_node.py of https://github.com/Shiva007-RAndom/Qr-and-red-zone-detection
(commit 5356a90), which ports the "red_zone_detection" notebook: HSV with two red hue ranges (red wraps
the hue wheel) -> open/close -> external contours with area >= min_area -> centroids from moments.
simplify_contour is the node's _contour_points without the ROS message type.
"""
from typing import List

import cv2
import numpy as np


def red_mask(bgr, lower1, upper1, lower2, upper2, kernel_size: int = 7) -> np.ndarray:
    """Binary mask of red pixels, cleaned with open + close."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                          cv2.inRange(hsv, lower2, upper2))
    k = max(3, int(kernel_size) | 1)  # force odd
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def find_patches(mask, min_area: float) -> List[dict]:
    """Contours of the mask with area >= min_area, largest first."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    patches = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        patches.append({
            "contour": cnt,
            "area": float(area),
            "centroid": (m["m10"] / m["m00"], m["m01"] / m["m00"]),
            "bbox": cv2.boundingRect(cnt),
        })
    patches.sort(key=lambda p: p["area"], reverse=True)
    return patches


def annotate(bgr, patches, elapsed_ms: float) -> np.ndarray:
    """Draw filled translucent contours, centroids and a summary line."""
    out = bgr.copy()
    for p in patches:
        overlay = out.copy()
        cv2.drawContours(overlay, [p["contour"]], -1, (0, 255, 0), cv2.FILLED)
        cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)
        cv2.drawContours(out, [p["contour"]], -1, (0, 255, 0), 2)
        cx, cy = int(p["centroid"][0]), int(p["centroid"][1])
        cv2.circle(out, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(out, f"Red ({int(p['area'])}px2)", (cx + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    cv2.putText(out, f"Red patches: {len(patches)}  |  {elapsed_ms:.1f} ms",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return out


def simplify_contour(cnt, epsilon: float = 0.01):
    """Outline simplified with approxPolyDP (epsilon = fraction of the perimeter), as [(x, y), ...]."""
    eps = epsilon * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, eps, True).reshape(-1, 2)
    return [(float(x), float(y)) for x, y in approx]
