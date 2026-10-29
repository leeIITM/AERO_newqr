"""Zero-latency RTSP camera worker for the SIYI A8 mini (section 2 of the original script).

A background thread keeps only the newest frame so a slow consumer never builds up latency.
Used by siyi_camera's rtsp_camera_node and by banner_detector's vision_test tool.
"""
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np

SIYI_DEFAULT_IP = "192.168.144.25"

# FFmpeg log levels (libavutil/log.h)
FFMPEG_LOG_LEVELS = {"quiet": -8, "panic": 0, "fatal": 8, "error": 16, "warning": 24, "info": 32}


def set_ffmpeg_log_level(level="quiet"):
    """Silence FFmpeg's own decoder messages (e.g. HEVC "Could not find ref with POC ..." / "Error
    constructing the frame RPS" after a lost UDP packet). OpenCV logs FFmpeg at AV_LOG_ERROR by default;
    with OPENCV_FFMPEG_DEBUG set it uses OPENCV_FFMPEG_LOGLEVEL instead. Read when OpenCV first opens an
    FFmpeg capture in the process, so this must run before that (an existing env value is kept)."""
    value = FFMPEG_LOG_LEVELS.get(str(level).lower())
    if value is None:
        raise ValueError(f"ffmpeg log level must be one of {sorted(FFMPEG_LOG_LEVELS)}")
    os.environ.setdefault("OPENCV_FFMPEG_DEBUG", "1")
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", str(value))


def default_rtsp_url(stream: str = "main", ip: str = SIYI_DEFAULT_IP) -> str:
    return f"rtsp://{ip}:8554/{stream}.264"


def parse_source(source):
    """'' -> SIYI main stream; '0', '1', ... -> local device index; anything else -> URL/path."""
    if source is None or source == "":
        return default_rtsp_url()
    if isinstance(source, int):
        return source
    s = str(source).strip()
    return int(s) if s.isdigit() else s


class SiyiStreamWorker:
    """Threaded grabber: cv2.CAP_FFMPEG over UDP, newest frame only, auto-reconnect."""

    def __init__(self, capture_source, log=print, transport="udp", ffmpeg_log_level="quiet"):
        self.capture_source = parse_source(capture_source)
        self.log = log
        if transport not in ("udp", "tcp"):
            raise ValueError("RTSP transport must be udp or tcp")
        self.transport = transport
        set_ffmpeg_log_level(ffmpeg_log_level)
        self.cap = None
        self.frame = None
        self.frame_id = 0
        self.lock = threading.Lock()
        self.running = True
        self.fps = 0.0
        self.last_frame_time = 0.0
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _open(self) -> bool:
        self.log(f"[CAMERA] Connecting to video source: {self.capture_source}")
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{self.transport}"
        if isinstance(self.capture_source, int):
            self.cap = cv2.VideoCapture(self.capture_source)
        else:
            self.cap = cv2.VideoCapture(self.capture_source, cv2.CAP_FFMPEG)
        return self.cap.isOpened()

    def _update(self):
        fps_timer = time.time()
        fps_counter = 0
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                if not self._open():
                    time.sleep(1.0)
                    continue
            ret, img = self.cap.read()
            now = time.time()
            if ret and img is not None:
                with self.lock:
                    self.frame = img
                    self.frame_id += 1
                self.last_frame_time = now
                fps_counter += 1
                if now - fps_timer >= 1.0:
                    self.fps = fps_counter / (now - fps_timer)
                    fps_counter = 0
                    fps_timer = now
            else:
                time.sleep(0.01)

    def get_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def get_frame_with_id(self):
        with self.lock:
            return (None, self.frame_id) if self.frame is None else (self.frame.copy(), self.frame_id)

    @property
    def is_connected(self) -> bool:
        return (time.time() - self.last_frame_time) < 2.0

    def stop(self):
        self.running = False
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
