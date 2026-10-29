"""Colour-first, strict-shape banner detector (section 4 of blue_green_qr_fixed.py), ROS-free.

  1. Colour segmentation around the competition colours in HSV. Hue window centres:
     green #00B140 -> H 71; blue 112 (pure #0000FF is H 120 on paper, but a printed/filmed blue banner
     reads ~104-118, so a window centred on 120 cut the real banner at its edge and fragmented the mask).
  2. Morphology: open 3x3 (speckle), close ellipse ~1.2 % of the width (bridges text and border).
  3. Rotated-rectangle classification: not > 35 % of the frame, not spanning the frame, fully in view,
     ratio >= min_banner_ratio, fill >= min_banner_fill, physical length 1.8-5.5 m.
  4. Persistence: the best candidate must pass in `banner_persistence` consecutive frames.
  5. Optional non-blocking RapidOCR: 'AeroTHON' + '2026' read near a region classifies it at once.
     One crop per frame, round-robin over the 3 most banner-like elongated regions.
  6. Partial OCR views: the centre is reconstructed along the rotated long axis.
Green and blue go through exactly the same steps.
"""
import math
import time
from dataclasses import dataclass, fields
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from bc_common.geometry import metres_per_pixel, order_corners
from banner_detector.ocr import AsyncOCRWorker

# AeroTHON banner: 11.5 ft x 1 ft
BANNER_LENGTH_M = 3.5052
BANNER_WIDTH_M = 0.3048

GREEN_BGR = (64, 177, 0)   # #00B140
BLUE_BGR = (255, 0, 0)     # #0000FF
WORK_WIDTH = 640


@dataclass
class BannerParams:
    """Same names and defaults as the script's command-line options."""
    green_h_center: int = -1        # OpenCV hue 0-179; < 0 = computed from #00B140 (71)
    green_h_tol: int = 15
    green_s_min: int = 100
    green_v_min: int = 70
    green_min_area: float = 0.0004
    blue_h_center: int = 112        # < 0 = computed from #0000FF (120)
    blue_h_tol: int = 15
    blue_s_min: int = 100
    blue_v_min: int = 70
    blue_min_area: float = 0.0004
    min_banner_ratio: float = 8.0
    min_banner_fill: float = 0.60
    banner_persistence: int = 3
    check_physical_size: bool = True
    min_length_m: float = 1.8
    max_length_m: float = 5.5
    ocr_threads: int = 2
    ocr_min_ratio: float = 3.0      # only regions at least this elongated are sent to OCR
    use_ocr: bool = True

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


def hsv_target(bgr) -> np.ndarray:
    return cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]


def _banner_likeness(d) -> float:
    """Distance to the ideal banner shape (ratio 11.5, fill 0.85); smaller is better."""
    return abs(d["ratio"] - 11.5) + 5.0 * abs(d["fill"] - 0.85)


class BannerDetector:
    def __init__(self, params: BannerParams = None, verbose: bool = False, log=print):
        self.p = params or BannerParams()
        self.verbose = verbose
        self.log = log
        self.ocr_worker = AsyncOCRWorker(threads=self.p.ocr_threads, enabled=self.p.use_ocr, log=log)
        self.green_h_center = int(self.p.green_h_center) if self.p.green_h_center >= 0 \
            else int(hsv_target(GREEN_BGR)[0])
        self.blue_h_center = int(self.p.blue_h_center) if self.p.blue_h_center >= 0 \
            else int(hsv_target(BLUE_BGR)[0])
        self.reset()

    def reset(self):
        """Clears persistence + OCR state. Called before every banner search (green and blue alike)."""
        self.tracked_center: Optional[Tuple[float, float]] = None
        self.consecutive_hits = 0
        self.last_color = ""
        self._ocr_rr = 0
        self.ocr_worker.clear_match()

    # ---------------------------------------------------------------- colour

    def color_params(self, color: str) -> Tuple[int, int, int, int]:
        """(hue_centre, h_tol, s_min, v_min) for a colour."""
        p = self.p
        if color.lower() == "blue":
            return self.blue_h_center, p.blue_h_tol, p.blue_s_min, p.blue_v_min
        return self.green_h_center, p.green_h_tol, p.green_s_min, p.green_v_min

    def get_hsv_ranges(self, color: str):
        """OpenCV HSV lower/upper bounds around the hue centre, with hue wrap-around."""
        target_h, h_tol, s_min, v_min = self.color_params(color)
        h_low, h_high = target_h - h_tol, target_h + h_tol
        lo = lambda h: np.array([h, s_min, v_min], dtype=np.uint8)  # noqa: E731
        hi = lambda h: np.array([h, 255, 255], dtype=np.uint8)  # noqa: E731
        if h_low < 0:
            return [(lo(0), hi(h_high)), (lo(180 + h_low), hi(179))]
        if h_high > 179:
            return [(lo(h_low), hi(179)), (lo(0), hi(h_high - 180))]
        return [(lo(h_low), hi(h_high))]

    def build_color_mask(self, img_bgr: np.ndarray, color: str, hsv: Optional[np.ndarray] = None) -> np.ndarray:
        if hsv is None:
            hsv = cv2.cvtColor(cv2.GaussianBlur(img_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = None
        for low, high in self.get_hsv_ranges(color):
            m = cv2.inRange(hsv, low, high)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        k = max(3, int(round(0.012 * img_bgr.shape[1])))
        k = k if k % 2 == 1 else k + 1
        return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

    # ---------------------------------------------------------------- detection

    def detect(self, frame: np.ndarray, color: str = "green", alt_rel: float = 5.0, hfov_rad: float = 1.22
               ) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]], np.ndarray]:
        """Returns (banner or None, per-region diagnostics, closed colour mask at work size)."""
        if frame is None:
            return None, [], np.zeros((100, 100), dtype=np.uint8)
        p = self.p
        full_h, full_w = frame.shape[:2]
        scale = float(WORK_WIDTH) / float(full_w)
        work_w, work_h = WORK_WIDTH, int(round(full_h * scale))
        work_img = cv2.resize(frame, (work_w, work_h), interpolation=cv2.INTER_AREA)
        work_hsv = cv2.cvtColor(cv2.GaussianBlur(work_img, (5, 5), 0), cv2.COLOR_BGR2HSV)
        color_mask = self.build_color_mask(work_img, color, hsv=work_hsv)
        _, _, s_min_c, v_min_c = self.color_params(color)

        min_area_frac = p.green_min_area if color.lower() == "green" else p.blue_min_area
        min_area_px = min_area_frac * float(work_w * work_h)
        cnts, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = sorted(((cv2.contourArea(c), c) for c in cnts if cv2.contourArea(c) >= min_area_px),
                         key=lambda x: x[0], reverse=True)

        diagnostics, passing = [], []
        margin_px = max(8.0, 0.02 * min(work_w, work_h))
        ocr_match = self.ocr_worker.get_latest_match()
        ocr_quad, ocr_text = None, ""
        mpp = metres_per_pixel(alt_rel, full_w, hfov_rad)

        for area, c in regions:
            rect = cv2.minAreaRect(c)
            (cx_w, cy_w), (rw_w, rh_w), _ = rect
            long_w = max(rw_w, rh_w)
            short_w = max(1e-3, min(rw_w, rh_w))
            ratio = long_w / short_w
            fill = area / max(1e-3, long_w * short_w)
            box_w = order_corners(cv2.boxPoints(rect))
            touching_edge = any(px < margin_px or px > work_w - margin_px or py < margin_px or py > work_h - margin_px
                                for px, py in box_w)
            box_full = box_w / scale
            center_full = (cx_w / scale, cy_w / scale)
            length_m = (long_w / scale) * mpp
            e1, e2 = box_full[1] - box_full[0], box_full[3] - box_full[0]
            long_edge = e1 if np.hypot(*e1) >= np.hypot(*e2) else e2
            angle = _fold_angle(math.atan2(float(long_edge[1]), float(long_edge[0])))
            diag = {"quad": box_full, "center": center_full, "contour": c, "area_frac": area / float(work_w * work_h),
                    "ratio": ratio, "fill": fill, "touching_edge": touching_edge, "length_m": length_m,
                    "angle": angle, "verdict": "REJECTED", "reason": "", "hue_med": None}

            if self.verbose:
                # median hue of saturated pixels in the box, measured WITHOUT the hue window (tuning aid)
                poly_mask = np.zeros((work_h, work_w), dtype=np.uint8)
                cv2.fillPoly(poly_mask, [np.int32(box_w)], 255)
                sel = (poly_mask > 0) & (work_hsv[:, :, 1] >= s_min_c) & (work_hsv[:, :, 2] >= v_min_c)
                if np.count_nonzero(sel) > 20:
                    diag["hue_med"] = float(np.median(work_hsv[:, :, 0][sel]))

            if ocr_match is not None and (time.time() - ocr_match["time"]) < 2.0:
                meta = ocr_match["meta"]
                if meta.get("color") == color and math.dist(meta["center"], center_full) < (80.0 / scale):
                    ocr_quad, ocr_text = box_full, ocr_match["text"]
                    diag["verdict"], diag["reason"] = "BANNER (OCR)", f"OCR verified: '{ocr_text}'"
                    diagnostics.append(diag)
                    continue

            if diag["area_frac"] > 0.35:
                diag["reason"] = "Covers >35% of frame"
            elif long_w > 0.92 * max(work_w, work_h):
                diag["reason"] = "Spans full image dimension"
            elif touching_edge:
                diag["reason"] = "Border cut / touches margin"
            elif ratio < p.min_banner_ratio:
                diag["reason"] = f"Ratio {ratio:.1f} < {p.min_banner_ratio:.1f}"
            elif fill < p.min_banner_fill:
                diag["reason"] = f"Fill {fill:.2f} < {p.min_banner_fill:.2f}"
            elif (p.check_physical_size and alt_rel > 1.5
                  and not (p.min_length_m <= length_m <= p.max_length_m)):
                diag["reason"] = f"Physical length {length_m:.1f}m outside [{p.min_length_m}m, {p.max_length_m}m]"
            else:
                diag["verdict"], diag["reason"] = "BANNER (SHAPE)", f"Shape PASS: ratio={ratio:.1f}, fill={fill:.2f}"
                passing.append(diag)
            diagnostics.append(diag)

        self._submit_ocr(frame, diagnostics, color)

        if ocr_quad is not None:
            if self.verbose:
                self.log(f"[BANNER DETECTED via OCR] Color={color.upper()} | Text='{ocr_text}'")
            return (self._build_output(ocr_quad, frame.shape, "OCR", ocr_text, alt_rel, hfov_rad),
                    diagnostics, color_mask)

        if not passing:
            self.consecutive_hits = 0
            self.tracked_center = None
            return None, diagnostics, color_mask

        passing.sort(key=_banner_likeness)
        best = passing[0]
        if (self.tracked_center is not None and self.last_color == color
                and math.dist(best["center"], self.tracked_center) < 100.0):
            self.consecutive_hits += 1
        else:
            self.consecutive_hits = 1
        self.tracked_center = best["center"]
        self.last_color = color

        if self.consecutive_hits >= p.banner_persistence:
            if self.verbose:
                self.log(f"[BANNER CLASSIFIED via SHAPE] Color={color.upper()} | Ratio={best['ratio']:.1f} | "
                         f"Fill={best['fill']:.2f} | Consecutive={self.consecutive_hits}")
            return (self._build_output(best["quad"], frame.shape, "SHAPE", f"{color.upper()} BANNER",
                                       alt_rel, hfov_rad), diagnostics, color_mask)
        return None, diagnostics, color_mask

    def _submit_ocr(self, frame, diagnostics, color):
        """One crop per frame to the OCR worker. Its queue holds a single job, so submitting every region
        would leave only the last (smallest) one read; with any other same-coloured blob in view the
        banner itself was starved of OCR. Rank elongated, non-frame-filling regions by banner-likeness and
        round-robin over the top 3."""
        pool = [d for d in diagnostics if d["ratio"] >= self.p.ocr_min_ratio and d["area_frac"] <= 0.35]
        if not pool:
            return
        ranked = sorted(pool, key=_banner_likeness)[:3]
        pick = ranked[self._ocr_rr % len(ranked)]
        self._ocr_rr += 1
        full_h, full_w = frame.shape[:2]
        bx, by, bw, bh = cv2.boundingRect(np.int32(pick["quad"]))
        pad = int(0.1 * max(bw, bh))
        x0, y0 = max(0, bx - pad), max(0, by - pad)
        x1, y1 = min(full_w, bx + bw + pad), min(full_h, by + bh + pad)
        if x1 > x0 + 10 and y1 > y0 + 10:
            self.ocr_worker.submit_job(frame[y0:y1, x0:x1], {"center": pick["center"], "color": color})

    def _build_output(self, quad_full, frame_shape, source, text, alt_rel, hfov_rad) -> Dict[str, Any]:
        full_h, full_w = frame_shape[:2]
        pts = order_corners(quad_full)
        e1, e2 = pts[1] - pts[0], pts[3] - pts[0]
        v_long = e1 if np.hypot(*e1) >= np.hypot(*e2) else e2
        angle = _fold_angle(math.atan2(float(v_long[1]), float(v_long[0])))
        m = 8.0
        cut = (float(pts[:, 0].min()) <= m or float(pts[:, 0].max()) >= full_w - 1 - m
               or float(pts[:, 1].min()) <= m or float(pts[:, 1].max()) >= full_h - 1 - m)
        u, v = float(pts[:, 0].mean()), float(pts[:, 1].mean())

        if cut and source == "OCR":
            # Reconstruct the true centre along the rotated long axis from the end that is in view.
            l_exp = BANNER_LENGTH_M / metres_per_pixel(alt_rel, full_w, hfov_rad)
            end_a, end_b = (pts[0] + pts[3]) * 0.5, (pts[1] + pts[2]) * 0.5
            if e1 is not v_long:  # long axis is TL->BL: ends are the top and bottom edges
                end_a, end_b = (pts[0] + pts[1]) * 0.5, (pts[3] + pts[2]) * 0.5
            d_a = min(end_a[0], full_w - end_a[0], end_a[1], full_h - end_a[1])
            d_b = min(end_b[0], full_w - end_b[0], end_b[1], full_h - end_b[1])
            inner, outer = (end_b, end_a) if d_a < d_b else (end_a, end_b)
            direction = (outer - inner) / max(1e-4, float(np.hypot(*(outer - inner))))
            true_c = inner + direction * (l_exp * 0.5)
            u, v = float(true_c[0]), float(true_c[1])

        return {"u": u, "v": v, "quad": pts, "angle": angle, "partial": cut, "source": source,
                "text": text, "size": (full_w, full_h)}

    def stop(self):
        self.ocr_worker.stop()


def _fold_angle(a):
    """Fold a line angle into [-pi/2, pi/2)."""
    if a >= math.pi / 2.0:
        a -= math.pi
    elif a < -math.pi / 2.0:
        a += math.pi
    return a
