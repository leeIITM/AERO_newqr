"""Banner detector node.

Subscribes
    /camera/image_raw          sensor_msgs/Image (downward SIYI A8 mini)
    /flight_controller/pose    mission_interfaces/VehicleState (altitude for the size check,
                               pose for the position estimate)
Publishes
    /perception/banner         mission_interfaces/BannerDetection, one per processed frame while a
                               colour is active
Services
    ~/set_color                mission_interfaces/SetBannerColor: "green" | "blue" | "" (idle).
                               Changing colour resets the persistence tracker.
Parameters
    all BannerParams fields (same names/defaults as the original script's options),
    hfov_rad, image_topic, color (start-up colour, default idle), max_rate_hz
"""
import math
import threading
import time

from mission_interfaces.msg import BannerDetection
from mission_interfaces.srv import SetBannerColor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image

from banner_detector.detector import BannerDetector, BannerParams
from bc_common.geometry import pixel_to_ned_offset
from bc_common.image import imgmsg_to_bgr
from bc_common.ros_utils import BANNER_TOPIC, DETECTION_QOS, IMAGE_QOS, IMAGE_TOPIC, VehicleListener, spin

COLORS = ("green", "blue", "")


class BannerDetectorNode(Node):
    def __init__(self):
        super().__init__("banner_detector")
        defaults = BannerParams()
        values = {name: self.declare_parameter(name, getattr(defaults, name)).value for name in BannerParams.names()}
        self.params = BannerParams(**values)
        self.hfov = float(self.declare_parameter("hfov_rad", 1.22).value)
        self.max_rate = float(self.declare_parameter("max_rate_hz", 30.0).value)
        topic = self.declare_parameter("image_topic", IMAGE_TOPIC).value
        color = str(self.declare_parameter("color", "").value).lower()

        self.detector = BannerDetector(self.params, verbose=bool(self.declare_parameter("verbose", False).value),
                                       log=self.get_logger().info)
        self.lock = threading.Lock()
        self.color = color if color in COLORS else ""
        self.last_processed = 0.0

        image_group = MutuallyExclusiveCallbackGroup()
        other = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, other)
        self.pub = self.create_publisher(BannerDetection, BANNER_TOPIC, DETECTION_QOS)
        self.create_subscription(Image, topic, self.on_image, IMAGE_QOS, callback_group=image_group)
        self.create_service(SetBannerColor, "~/set_color", self.set_color_cb, callback_group=other)
        self.get_logger().info(f"Banner detector on {topic}; colour: {self.color or 'idle'}")

    def set_color_cb(self, request, response):
        color = request.color.strip().lower()
        if color not in COLORS:
            response.success, response.message = False, f"unknown colour {request.color!r}"
            return response
        with self.lock:
            self.color = color
            self.detector.reset()
        self.get_logger().info(f"Detecting: {color.upper() or 'IDLE'}")
        response.success, response.message = True, "ok"
        return response

    def on_image(self, msg: Image):
        with self.lock:
            color = self.color
        if not color:
            return
        now = time.monotonic()
        if self.max_rate > 0 and now - self.last_processed < 1.0 / self.max_rate:
            return
        self.last_processed = now
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        pose = self.vehicle.latest
        pose_ok = self.vehicle.fresh()
        alt = self.vehicle.alt_rel() if pose_ok else 5.0
        with self.lock:
            if color != self.color:
                return
            banner, diags, _ = self.detector.detect(frame, color=color, alt_rel=alt, hfov_rad=self.hfov)

        out = BannerDetection()
        out.header = msg.header
        out.color = color
        out.image_width, out.image_height = int(frame.shape[1]), int(frame.shape[0])
        out.candidates = len(diags)
        out.consecutive_hits = int(self.detector.consecutive_hits)
        out.pose_valid = bool(pose_ok)
        out.alt_rel = float(alt)
        if pose_ok:
            out.drone_x, out.drone_y, out.drone_z = float(pose.x), float(pose.y), float(pose.z)
            out.drone_heading = float(pose.heading)
        if banner is not None:
            out.detected = True
            out.source, out.text = banner["source"], banner["text"]
            out.u, out.v = float(banner["u"]), float(banner["v"])
            out.quad = [float(c) for c in banner["quad"].reshape(-1)]
            out.angle = float(banner["angle"])
            out.partial = bool(banner["partial"])
            if pose_ok:
                dn, de = pixel_to_ned_offset(out.u, out.v, out.image_width, out.image_height, alt,
                                             pose.heading, self.hfov)
                out.position_estimate.x = float(pose.x + dn)
                out.position_estimate.y = float(pose.y + de)
                out.position_estimate.z = float(pose.z)
            else:
                out.position_estimate.x = out.position_estimate.y = out.position_estimate.z = math.nan
        self.pub.publish(out)

    def shutdown(self):
        self.detector.stop()


def main(args=None):
    spin(BannerDetectorNode, args)


if __name__ == "__main__":
    main()
