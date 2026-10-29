"""Asynchronous RapidOCR worker (section 3 of the original script).

Runs in its own thread so it never blocks detection. A crop counts as a banner only when BOTH
'AeroTHON' (edit distance <= 2 after confusable-character folding) AND '2026' are read.
RapidOCR is optional: without it the detector runs shape-only classification.
"""
import logging
import queue
import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

TARGET_BRAND = "aerothon"
TARGET_YEAR = "2026"
_CONFUSION = str.maketrans({
    "o": "0", "q": "0", "d": "0",
    "i": "1", "l": "1", "|": "1",
    "z": "2", "s": "6", "5": "6",
    "g": "6", "b": "8",
})


def canon(text: str) -> str:
    text = re.sub(r"[^a-z0-9|]", "", text.lower())
    return text.translate(_CONFUSION)


BRAND_C = canon(TARGET_BRAND)
YEAR_C = canon(TARGET_YEAR)


def partial_match_distance(text: str, target: str) -> int:
    """Levenshtein distance between `target` and the best-matching substring of `text`."""
    if len(target) == 0:
        return 0
    if len(text) == 0:
        return len(target)
    prev = [0] * (len(text) + 1)
    for i, c_t in enumerate(target):
        curr = [i + 1]
        for j, c_s in enumerate(text):
            curr.append(min(prev[j + 1] + 1, curr[j] + 1, prev[j] + (c_t != c_s)))
        prev = curr
    return min(prev)


def is_banner_text(text: str) -> bool:
    c_txt = canon(text)
    return partial_match_distance(c_txt, BRAND_C) <= 2 and YEAR_C in c_txt


def extract_ocr_items(ocr_output: Any) -> List[Tuple[np.ndarray, str, float]]:
    items = []
    if ocr_output is None:
        return items
    if hasattr(ocr_output, "boxes") and hasattr(ocr_output, "txts") and hasattr(ocr_output, "scores"):
        if ocr_output.boxes is not None and ocr_output.txts is not None:
            for b, t, s in zip(ocr_output.boxes, ocr_output.txts, ocr_output.scores):
                items.append((np.asarray(b, dtype=np.float32), str(t), float(s)))
        return items
    if isinstance(ocr_output, (list, tuple)):
        target = ocr_output[0] if (len(ocr_output) == 2 and isinstance(ocr_output[0], list)) else ocr_output
        for item in target:
            if len(item) == 3:
                items.append((np.asarray(item[0], dtype=np.float32), str(item[1]), float(item[2])))
    return items


class AsyncOCRWorker:
    def __init__(self, threads: int = 2, enabled: bool = True, log=print):
        self.lock = threading.Lock()
        self.job_queue = queue.Queue(maxsize=1)
        self.latest_match: Optional[Dict[str, Any]] = None
        self.running = True
        self.engine = None
        self.log = log
        if enabled:
            self._init_ocr(threads)
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def _init_ocr(self, threads: int):
        try:
            from rapidocr import RapidOCR
            try:
                self.engine = RapidOCR(params={
                    "Global.use_cls": False,
                    "Det.limit_type": "max",
                    "Det.limit_side_len": 640,
                    "EngineConfig.onnxruntime.intra_op_num_threads": threads,
                    "EngineConfig.onnxruntime.inter_op_num_threads": 1,
                })
            except Exception:  # noqa: BLE001 - older RapidOCR without params
                self.engine = RapidOCR()
            # "The text detection result is empty" is a normal outcome for a crop without text
            logging.getLogger("RapidOCR").setLevel(logging.ERROR)
            self.log("[VISION] RapidOCR background engine initialized.")
        except Exception as e:  # noqa: BLE001
            self.log(f"[VISION WARN] RapidOCR unavailable ({e}). Running shape-only classification.")
            self.engine = None

    def _worker_loop(self):
        while self.running:
            try:
                crop, meta = self.job_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.engine is None or crop is None or crop.size == 0:
                continue
            # use_cls is off, so upside-down text is never read as-is: if text WAS found but did not
            # match, retry the crop rotated 180 deg (a crop without any text is not retried).
            try:
                matched, had_text = self._try_match(crop, meta)
                if not matched and had_text:
                    self._try_match(cv2.rotate(crop, cv2.ROTATE_180), meta)
            except Exception:  # noqa: BLE001 - OCR failures must never stop detection
                pass

    def _try_match(self, crop, meta) -> Tuple[bool, bool]:
        """(matched, had_text)."""
        items = extract_ocr_items(self.engine(crop))
        if not items:
            return False, False
        combined_text = " ".join(txt for _, txt, _ in items)
        if not is_banner_text(combined_text):
            return False, True
        with self.lock:
            self.latest_match = {
                "meta": meta, "text": combined_text,
                "score": float(np.mean([s for _, _, s in items])), "time": time.time(),
            }
        return True, True

    def submit_job(self, crop: np.ndarray, meta: Dict[str, Any]):
        if self.engine is None or crop is None:
            return
        try:
            try:
                self.job_queue.get_nowait()
            except queue.Empty:
                pass
            self.job_queue.put_nowait((crop.copy(), meta))
        except queue.Full:
            pass

    def get_latest_match(self) -> Optional[Dict[str, Any]]:
        with self.lock:
            return self.latest_match

    def clear_match(self):
        with self.lock:
            self.latest_match = None

    def stop(self):
        self.running = False
