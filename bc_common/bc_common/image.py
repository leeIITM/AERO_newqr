"""sensor_msgs/Image <-> OpenCV numpy without cv_bridge (avoids cv_bridge / numpy ABI mismatches)."""
import cv2
import numpy as np
from sensor_msgs.msg import Image


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Image message -> BGR (or mono) numpy array; copies the buffer."""
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = data.reshape((msg.height, msg.step // 3 if msg.step else msg.width, 3))[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    if enc in ("bgra8", "rgba8"):
        img = data.reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR)
    if enc in ("mono8", "8uc1"):
        return data.reshape((msg.height, msg.width)).copy()
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


def bgr_to_imgmsg(img: np.ndarray, header=None) -> Image:
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    if img.ndim == 2:
        msg.encoding = "mono8"
        msg.step = msg.width
    else:
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
    msg.is_bigendian = 0
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg
