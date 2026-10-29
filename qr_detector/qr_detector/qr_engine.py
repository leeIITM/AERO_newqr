"""Rotation-invariant QR engine (section 5 of the original script, algorithm unchanged), ROS-free.

detect             first QR: OpenCV QRCodeDetector, pyzbar fallback (optional), decoded text if readable
detect_all         every QR in view (detectAndDecodeMulti, then pyzbar, then detect)
extract_qr_features warp the QR quad to 210x210, Otsu, sample a 21x21 module grid
compute_similarity  fraction of agreeing modules, best of the 4 rotations
"""
from typing import List, Optional, Tuple

import cv2
import numpy as np

from bc_common.geometry import order_corners

try:
    from pyzbar.pyzbar import decode as _pyzbar_decode
except Exception:  # noqa: BLE001 - pyzbar (and libzbar) are optional
    _pyzbar_decode = None


class QrFinder:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        self.pyzbar_decode = _pyzbar_decode

    def detect(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], np.ndarray, str]:
        """Returns (found, quad points (1, 4, 2) float32, gray image, decoded text or '')."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        ok, pts = self.detector.detect(gray)
        text = ""
        if (not ok or pts is None) and self.pyzbar_decode is not None:
            decoded = self.pyzbar_decode(gray)
            if decoded:
                poly = decoded[0].polygon
                if len(poly) == 4:
                    pts = np.array([[p.x, p.y] for p in poly], dtype=np.float32).reshape(1, 4, 2)
                    ok = True
                    text = decoded[0].data.decode("utf-8", errors="ignore")
        if ok and pts is not None and not text:
            try:
                txt, _, _ = self.detector.detectAndDecode(gray)
                if txt:
                    text = txt
            except cv2.error:
                pass
        if not ok or pts is None or len(pts) == 0:
            return False, None, gray, ""
        return True, np.asarray(pts, dtype=np.float32).reshape(1, 4, 2), gray, text

    def detect_all(self, frame: np.ndarray) -> Tuple[np.ndarray, List[Tuple[np.ndarray, str]]]:
        """Every QR in view as ((4, 2) corners, decoded text or ''), each text paired with its own corners."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        found: List[Tuple[np.ndarray, str]] = []
        try:
            ok, texts, pts_all, _ = self.detector.detectAndDecodeMulti(gray)
            if ok and pts_all is not None:
                for txt, pts in zip(texts, pts_all):
                    found.append((np.asarray(pts, dtype=np.float32).reshape(-1, 2)[:4], txt or ""))
        except cv2.error:
            found = []
        if not found and self.pyzbar_decode is not None:
            for d in self.pyzbar_decode(gray):
                if len(d.polygon) == 4:
                    pts = np.array([[p.x, p.y] for p in d.polygon], dtype=np.float32)
                    found.append((pts, d.data.decode("utf-8", errors="ignore")))
        if not found:
            ok, pts, _, text = self.detect(gray)
            if ok:
                found.append((pts.reshape(-1, 2)[:4], text))
        return gray, found


def extract_qr_features(gray: np.ndarray, quad_pts, grid_size: int = 21) -> Tuple[np.ndarray, np.ndarray]:
    quad = order_corners(quad_pts)
    side = grid_size * 10
    dst = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32)
    warped = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(quad, dst), (side, side))
    _, binary = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    for r in range(grid_size):
        for c in range(grid_size):
            grid[r, c] = 1 if np.mean(binary[r * 10 + 2:r * 10 + 8, c * 10 + 2:c * 10 + 8]) < 128 else 0
    return grid, warped


def compute_similarity(grid_ref: Optional[np.ndarray], grid_cand: Optional[np.ndarray]) -> float:
    if grid_ref is None or grid_cand is None:
        return 0.0
    best = 0.0
    cand = grid_cand.copy()
    for _ in range(4):
        best = max(best, float(np.sum(grid_ref == cand)) / float(grid_ref.size))
        cand = np.rot90(cand)
    return best


def match(ref_grid, ref_text, cand_grid, cand_text, threshold, reject_text_mismatch=True) -> Tuple[float, bool]:
    """(score, matched).

    Exact decoded-text match: score 1.0, matched. Otherwise the 4-rotation grid similarity is compared
    with `threshold`, as in the original script. With reject_text_mismatch (default), two codes that
    both decode to DIFFERENT texts never match: QR codes of the same version share their finder and
    timing patterns, so unrelated codes can score > 0.9 on the grid alone."""
    if cand_text and ref_text and cand_text == ref_text:
        return 1.0, True
    score = compute_similarity(ref_grid, cand_grid)
    if reject_text_mismatch and cand_text and ref_text:
        return score, False
    return score, score >= threshold
