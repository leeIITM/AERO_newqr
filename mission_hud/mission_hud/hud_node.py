"""Mission HUD: camera image + detections + mission state, published as /hud/image.

Subscribes /camera/image_raw, /perception/banner, /perception/qr, /flight_controller/pose,
/mission/status, /qr_detector/reference_image, /flight_controller/motors.
Publishes /hud/image (bgr8). View with rqt_image_view, or set show_window:=true on a machine with
a display (an OpenCV window, as in the original script).

Parameters: show_window (false), match_thresh (0.88), max_rate_hz (15), scale (0.75 output size)
"""
import math
import os
import time

import cv2
from mission_interfaces.msg import BannerDetection, MissionStatus, MotorStatus, QrDetection, RedZoneResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from bc_common.image import bgr_to_imgmsg, imgmsg_to_bgr
from bc_common.ros_utils import (
    BANNER_TOPIC, DETECTION_QOS, IMAGE_QOS, IMAGE_TOPIC, MISSION_STATUS_TOPIC, MOTOR_TOPIC, QR_TOPIC,
    REFERENCE_IMAGE_TOPIC, VehicleListener, spin,
)
from mission_hud.overlay import HudOverlay

FRESH = 0.5  # s a detection is drawn for (the overlay then retains it 1.2 s more)


class HudNode(Node):
    def __init__(self):
        super().__init__("mission_hud")
        self.show = bool(self.declare_parameter("show_window", False).value) and \
            ("DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ)
        self.max_rate = float(self.declare_parameter("max_rate_hz", 15.0).value)
        self.scale = float(self.declare_parameter("scale", 0.75).value)
        self.overlay = HudOverlay(float(self.declare_parameter("match_thresh", 0.88).value))
        self.vehicle = VehicleListener(self)
        self.banner = self.qr = self.status = self.ref = self.motors = None
        self.banner_t = self.qr_t = self.red_t = 0.0
        self.red = None
        self.last = 0.0
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(BannerDetection, BANNER_TOPIC, self.banner_cb, DETECTION_QOS)
        self.create_subscription(QrDetection, QR_TOPIC, self.qr_cb, DETECTION_QOS)
        self.create_subscription(RedZoneResult, "/perception/red_zone", self.red_cb, DETECTION_QOS)
        self.create_subscription(MissionStatus, MISSION_STATUS_TOPIC, lambda m: setattr(self, "status", m),
                                 DETECTION_QOS)
        self.create_subscription(MotorStatus, MOTOR_TOPIC, lambda m: setattr(self, "motors", m), 10)
        self.create_subscription(Image, REFERENCE_IMAGE_TOPIC, self.ref_cb, latched)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, IMAGE_QOS)
        self.pub = self.create_publisher(Image, "/hud/image", IMAGE_QOS)
        if self.show:
            try:
                cv2.namedWindow("AeroTHON Real-World Mission HUD", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("AeroTHON Real-World Mission HUD", 960, 540)
            except cv2.error as exc:
                # e.g. a pip opencv-python-headless (no GUI) shadowing apt's python3-opencv
                self.show = False
                self.get_logger().warn(
                    f"No OpenCV window support in {cv2.__file__} (OpenCV {cv2.__version__}): HUD window "
                    f"disabled, still publishing /hud/image (rqt_image_view /hud/image). ({str(exc).strip()[:80]})")

    def banner_cb(self, msg):
        if msg.detected:
            self.banner, self.banner_t = msg, time.monotonic()

    def qr_cb(self, msg):
        if msg.detected:
            self.qr, self.qr_t = msg, time.monotonic()

    def red_cb(self, msg):
        self.red, self.red_t = msg, time.monotonic()

    def ref_cb(self, msg):
        self.ref = imgmsg_to_bgr(msg)

    def image_cb(self, msg):
        now = time.monotonic()
        if self.max_rate > 0 and now - self.last < 1.0 / self.max_rate:
            return
        self.last = now
        frame = imgmsg_to_bgr(msg)
        pts, kind, score, sub = None, None, 0.0, ""
        if self.qr is not None and now - self.qr_t < FRESH and (self.banner is None or self.qr_t >= self.banner_t):
            pts, kind = list(zip(self.qr.corners[0::2], self.qr.corners[1::2])), "QR"
            score, sub = self.qr.score, self.qr.decoded_text
        elif self.banner is not None and now - self.banner_t < FRESH:
            pts = list(zip(self.banner.quad[0::2], self.banner.quad[1::2]))
            kind, sub = f"{self.banner.color.upper()}_BANNER", self.banner.source
        p = self.vehicle.latest
        pose = None
        if p is not None:
            alt = p.alt_rel if math.isfinite(p.alt_rel) else -p.z
            pose = {"alt": alt, "x": p.x, "y": p.y, "heading_deg": math.degrees(p.heading), "mode": p.mode}
        motors = None
        if self.motors is not None:
            motors = {"pwm": self.motors.servo_pwm, "roll": self.motors.cmd_roll_deg,
                      "pitch": self.motors.cmd_pitch_deg, "thrust": self.motors.thrust}
        st = self.status
        vis = self.overlay.draw(frame, pts, kind, score, sub, st.state_text if st else "WAITING FOR MISSION",
                                pose, self.ref, st.countdown if st else 0.0, motors, self.red_outlines(now, frame))
        if self.scale > 0 and self.scale != 1.0:
            vis = cv2.resize(vis, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        out = bgr_to_imgmsg(vis)
        out.header = msg.header
        self.pub.publish(out)
        if self.show:
            try:
                cv2.imshow("AeroTHON Real-World Mission HUD", vis)
                cv2.waitKey(1)
            except cv2.error:
                self.show = False

    def red_outlines(self, now, frame):
        r = self.red
        if r is None or now - self.red_t > FRESH or not r.detected or r.image_width == 0:
            return None
        sx, sy = frame.shape[1] / r.image_width, frame.shape[0] / r.image_height
        return [[(pt.x * sx, pt.y * sy) for pt in p.contour] for p in r.patches]

    def shutdown(self):
        if self.show:
            cv2.destroyAllWindows()


def main(args=None):
    spin(HudNode, args)


if __name__ == "__main__":
    main()
