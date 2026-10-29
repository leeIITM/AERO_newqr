"""Red / restricted-zone detector node (red_zone_node of Shiva007-RAndom/Qr-and-red-zone-detection,
on this workspace's camera topic and messages).

Subscribes
    /camera/image_raw          sensor_msgs/Image
Publishes
    /perception/red_zone       mission_interfaces/RedZoneResult for EVERY processed frame (detected =
                               false too: target_search uses those frames for its coverage map).
                               Not latched: a stale result would be paired with the wrong pose.
    ~/debug_image              sensor_msgs/Image annotated frame (publish_debug_image)
Services
    ~/set_active               std_srvs/SetBool. IDLE by default: no frame is processed (no CPU) until the
                               mission tree switches it on in the delivery zone (stage 9 target search ->
                               payload drop) and off again when leaving; the only red zone is in that area.
Parameters (same names and defaults as the repo's red_zone.yaml)
    image_topic, min_area (100 px^2), hsv_lower1/upper1/lower2/upper2, kernel_size (7),
    contour_epsilon (0.01), process_every_n (1), publish_debug_image (false), log_every_frame (false),
    active (false: start idle)
"""
import time

import numpy as np
from geometry_msgs.msg import Point32
from mission_interfaces.msg import RedPatch, RedZoneResult
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from bc_common.image import bgr_to_imgmsg, imgmsg_to_bgr
from bc_common.ros_utils import DETECTION_QOS, IMAGE_QOS, IMAGE_TOPIC, spin
from red_zone_detector.detection import annotate, find_patches, red_mask, simplify_contour

RED_ZONE_TOPIC = "/perception/red_zone"


class RedZoneDetector(Node):
    def __init__(self):
        super().__init__("red_zone_detector")
        p = self.declare_parameter
        image_topic = str(p("image_topic", IMAGE_TOPIC).value)
        self.min_area = float(p("min_area", 100.0).value)
        self.lower1 = np.array(p("hsv_lower1", [0, 80, 40]).value, dtype=np.uint8)
        self.upper1 = np.array(p("hsv_upper1", [15, 255, 255]).value, dtype=np.uint8)
        self.lower2 = np.array(p("hsv_lower2", [165, 80, 40]).value, dtype=np.uint8)
        self.upper2 = np.array(p("hsv_upper2", [180, 255, 255]).value, dtype=np.uint8)
        self.kernel_size = int(p("kernel_size", 7).value)
        self.epsilon = float(p("contour_epsilon", 0.01).value)
        self.every_n = max(1, int(p("process_every_n", 1).value))
        self.debug = bool(p("publish_debug_image", False).value)
        self.log_every = bool(p("log_every_frame", False).value)
        self.active = bool(p("active", False).value)
        self.frame_count = 0
        self.pub = self.create_publisher(RedZoneResult, RED_ZONE_TOPIC, DETECTION_QOS)
        self.pub_debug = self.create_publisher(Image, "~/debug_image", IMAGE_QOS) if self.debug else None
        self.create_subscription(Image, image_topic, self.on_image, IMAGE_QOS)
        self.create_service(SetBool, "~/set_active", self.set_active)
        self.get_logger().info(f"Ready. Listening on '{image_topic}', min_area {self.min_area:.0f} px2, "
                               f"1 in {self.every_n} frame(s); {'ACTIVE' if self.active else 'idle until ~/set_active'}.")

    def set_active(self, request, response):
        if bool(request.data) != self.active:
            self.active = bool(request.data)
            self.frame_count = 0
            self.get_logger().info("Red-zone detection ON (delivery zone)" if self.active
                                   else "Red-zone detection OFF (left the delivery zone)")
        response.success, response.message = True, "active" if self.active else "idle"
        return response

    def on_image(self, msg):
        if not self.active:
            return
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        if frame is None or frame.size == 0 or frame.ndim != 3:
            return
        start = time.perf_counter()
        try:
            mask = red_mask(frame, self.lower1, self.upper1, self.lower2, self.upper2, self.kernel_size)
            patches = find_patches(mask, self.min_area)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.get_logger().error(f"Processing failed on this frame: {exc}")
            return
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        height, width = frame.shape[:2]
        image_area = float(width * height)

        result = RedZoneResult()
        result.header = msg.header
        result.image_width, result.image_height = int(width), int(height)
        result.detected = len(patches) > 0
        result.num_patches = len(patches)
        result.total_area_fraction = float(sum(q["area"] for q in patches) / image_area)
        result.processing_time_ms = float(elapsed_ms)
        for q in patches:
            cx, cy = q["centroid"]
            rp = RedPatch()
            rp.centroid_x, rp.centroid_y = float(cx), float(cy)
            rp.offset_x = float((cx - width / 2.0) / (width / 2.0))
            rp.offset_y = float((cy - height / 2.0) / (height / 2.0))
            rp.area_px = float(q["area"])
            rp.area_fraction = float(q["area"] / image_area)
            rp.bbox_xywh = [int(v) for v in q["bbox"]]
            rp.contour = [Point32(x=x, y=y, z=0.0) for x, y in simplify_contour(q["contour"], self.epsilon)]
            result.patches.append(rp)
        self.pub.publish(result)

        if self.pub_debug is not None:
            dbg = bgr_to_imgmsg(annotate(frame, patches, elapsed_ms))
            dbg.header = msg.header
            self.pub_debug.publish(dbg)
        if self.log_every or result.detected:
            if result.detected:
                top = result.patches[0]
                self.get_logger().info(f"RED: {len(patches)} patch(es), largest {top.area_px:.0f} px2 at "
                                       f"({top.centroid_x:.0f}, {top.centroid_y:.0f}) ({elapsed_ms:.0f} ms)",
                                       throttle_duration_sec=1.0)
            else:
                self.get_logger().info(f"no red ({elapsed_ms:.0f} ms)")


def main(args=None):
    spin(RedZoneDetector, args)


if __name__ == "__main__":
    main()
