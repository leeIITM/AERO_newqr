"""QR detector node.

The reference QR comes ONLY from the camera (stage 2): ~/capture_reference memorises the first QR seen.
A copy of the warp is written to `reference_save_path` for logging and is never read back.

Subscribes
    /camera/image_raw          sensor_msgs/Image
    /flight_controller/pose    mission_interfaces/VehicleState
Publishes
    /perception/qr             mission_interfaces/QrDetection, one per processed frame: the QR in view that
                               best matches the reference (every QR in the frame is compared), else the first
    ~/reference_image          sensor_msgs/Image (mono8 210x210 warp, latched) once captured
Actions
    ~/capture_reference        mission_interfaces/CaptureReference
Services
    ~/set_active               std_srvs/SetBool: search every frame for QRs (lawnmower, centring) or not.
                               Off by default: matching every QR in view costs 200-400 ms a frame on the Pi,
                               which starved the banner detector of CPU. Capturing the reference works either way.
Parameters
    match_thresh (0.88), reject_text_mismatch (true: both decoded + different text = no match), hfov_rad, grid_size (21), max_rate_hz (15), image_topic,
    reference_save_path ("" = do not save), active (false)
"""
import math
import os
import threading
import time

import cv2
from mission_interfaces.action import CaptureReference
from mission_interfaces.msg import QrDetection
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from bc_common.geometry import pixel_to_ned_offset
from bc_common.image import bgr_to_imgmsg, imgmsg_to_bgr
from bc_common.ros_utils import (
    DETECTION_QOS, IMAGE_QOS, IMAGE_TOPIC, QR_TOPIC, VehicleListener, finish, serve_action, spin,
)
from qr_detector.qr_engine import QrFinder, extract_qr_features, match

REPORT_PERIOD = 10.0  # s between "still waiting for the reference QR" log lines

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class QrDetectorNode(Node):
    def __init__(self):
        super().__init__("qr_detector")
        self.threshold = float(self.declare_parameter("match_thresh", 0.88).value)
        self.reject_text_mismatch = bool(self.declare_parameter("reject_text_mismatch", True).value)
        self.hfov = float(self.declare_parameter("hfov_rad", 1.22).value)
        self.grid_size = int(self.declare_parameter("grid_size", 21).value)
        self.max_rate = float(self.declare_parameter("max_rate_hz", 15.0).value)
        self.save_path = os.path.expanduser(str(self.declare_parameter(
            "reference_save_path", "~/aerothon_ws/reference_qr_memorized.png").value))
        topic = self.declare_parameter("image_topic", IMAGE_TOPIC).value
        self.active = bool(self.declare_parameter("active", False).value)
        # reference capture: only a QR fully in view (not cut by the image border) is memorised
        self.edge_margin = float(self.declare_parameter("capture_edge_margin", 0.03).value)
        # ... and one that does not decode only if its centre is in the central capture_centre_fraction of the
        # image (an undecoded code near the border is often partial / oblique and makes a poor reference grid)
        self.centre_fraction = float(self.declare_parameter("capture_centre_fraction", 0.5).value)

        self.finder = QrFinder()
        self.lock = threading.Lock()
        self.ref_grid = None
        self.ref_warped = None
        self.ref_text = ""
        self.capture_requested = False
        self.captured = threading.Event()
        self.last_processed = 0.0

        other = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, other)
        self.pub = self.create_publisher(QrDetection, QR_TOPIC, DETECTION_QOS)
        self.ref_pub = self.create_publisher(Image, "~/reference_image", LATCHED)
        self.create_subscription(Image, topic, self.on_image, IMAGE_QOS,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        serve_action(self, CaptureReference, "~/capture_reference", self.execute_capture, other)
        self.create_service(SetBool, "~/set_active", self.set_active_cb, callback_group=other)
        if self.finder.pyzbar_decode is None:
            self.get_logger().info("pyzbar not available: OpenCV QR detection only.")

    def set_active_cb(self, request, response):
        self.active = bool(request.data)
        self.get_logger().info(f"QR search {'ON' if self.active else 'OFF'}")
        response.success, response.message = True, "ok"
        return response

    # ---------------------------------------------------------------- frames

    def near_centre(self, pts, shape):
        h, w = shape[:2]
        cx, cy = pts.reshape(-1, 2).mean(axis=0)
        f = self.centre_fraction / 2.0
        return abs(cx - w / 2.0) <= f * w and abs(cy - h / 2.0) <= f * h

    def inside_frame(self, pts, shape):
        """All four QR corners at least capture_edge_margin (fraction of the shorter side) inside the image."""
        h, w = shape[:2]
        m = self.edge_margin * min(w, h)
        p = pts.reshape(-1, 2)
        return bool((p[:, 0] >= m).all() and (p[:, 0] <= w - m).all() and (p[:, 1] >= m).all()
                    and (p[:, 1] <= h - m).all())

    def on_image(self, msg: Image):
        now = time.monotonic()
        if not self.capture_requested and not self.active:
            return
        if not self.capture_requested and self.max_rate > 0 and now - self.last_processed < 1.0 / self.max_rate:
            return
        self.last_processed = now
        frame = imgmsg_to_bgr(msg)
        pose_ok = self.vehicle.fresh()
        pose = self.vehicle.latest

        out = QrDetection()
        out.header = msg.header
        out.image_width, out.image_height = int(frame.shape[1]), int(frame.shape[0])
        out.match_threshold = self.threshold
        out.pose_valid = bool(pose_ok)
        alt = self.vehicle.alt_rel() if pose_ok else 5.0
        out.alt_rel = float(alt)
        if pose_ok:
            out.drone_x, out.drone_y, out.drone_z = float(pose.x), float(pose.y), float(pose.z)
            out.drone_heading = float(pose.heading)

        with self.lock:
            capturing = self.capture_requested and self.ref_grid is None
            ref_grid, ref_text = self.ref_grid, self.ref_text
        best = None  # (pts (4, 2), text, score, matched)
        if capturing:
            # stage 2: the first QR seen becomes the reference
            ok, pts, gray, text = self.finder.detect(frame)
            if ok and not self.inside_frame(pts, frame.shape):
                ok = False  # cut by the image border (e.g. while the search moves): not a usable reference
            if ok and not text and not self.near_centre(pts, frame.shape):
                ok = False  # not decoded and off-centre (partial / oblique): wait for a better view
            if ok:
                grid, warped = extract_qr_features(gray, pts, grid_size=self.grid_size)
                with self.lock:
                    self.store_reference(grid, warped, text)
                best = (pts.reshape(-1, 2), text, 1.0, True)
        else:
            # every QR in view is compared with the reference: a decoy next to the twin cannot hide it
            gray, found = self.finder.detect_all(frame)
            for pts, text in found:
                score, matched = 0.0, False
                if ref_grid is not None:
                    grid, _ = extract_qr_features(gray, pts, grid_size=self.grid_size)
                    score, matched = match(ref_grid, ref_text, grid, text, self.threshold, self.reject_text_mismatch)
                if best is None or (matched, score) > (best[3], best[2]):
                    best = (pts, text, score, matched)

        if best is not None:
            p, text, score, matched = best
            out.detected = True
            out.corners = [float(c) for c in p.reshape(-1)]
            out.u, out.v = float(p[:, 0].mean()), float(p[:, 1].mean())
            out.decoded_text = text
            if ref_grid is not None or capturing:
                out.score, out.matched = float(score), bool(matched)
            if pose_ok:
                dn, de = pixel_to_ned_offset(out.u, out.v, out.image_width, out.image_height, alt,
                                             pose.heading, self.hfov)
                out.position_estimate.x, out.position_estimate.y = float(pose.x + dn), float(pose.y + de)
                out.position_estimate.z = float(pose.z)
            else:
                out.position_estimate.x = out.position_estimate.y = out.position_estimate.z = math.nan
        out.reference_ready = self.ref_grid is not None
        self.pub.publish(out)

    def store_reference(self, grid, warped, text):
        """Caller holds self.lock."""
        self.ref_grid, self.ref_warped, self.ref_text = grid, warped, text
        self.capture_requested = False
        self.captured.set()
        self.get_logger().info(f"REFERENCE QR MEMORIZED FROM CAMERA! Payload: '{text}'")
        self.ref_pub.publish(bgr_to_imgmsg(warped))

    # ---------------------------------------------------------------- action

    def execute_capture(self, goal_handle):
        """No timeout: scan until a QR is seen (or the goal is cancelled), reporting every 10 s."""
        result = CaptureReference.Result()
        with self.lock:
            if self.ref_grid is not None:
                result.decoded_text = self.ref_text
                return finish(goal_handle, result, True, "reference already captured")
            self.captured.clear()
            self.capture_requested = True
        self.get_logger().info("Scanning reference QR from the camera (no timeout: until a QR is seen)...")
        start = time.monotonic()
        next_report = start + REPORT_PERIOD
        feedback = CaptureReference.Feedback()
        while not self.captured.wait(0.1):
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.capture_requested = False
                return finish(goal_handle, result, False, "canceled", self.get_logger())
            if now >= next_report:
                self.get_logger().info(f"Still waiting for the reference QR ({now - start:.0f} s)...")
                next_report += REPORT_PERIOD
            feedback.elapsed = float(now - start)
            goal_handle.publish_feedback(feedback)
        result.decoded_text = self.ref_text
        result.image_path = self.save_reference()
        self.get_logger().info(f"Reference QR captured after {time.monotonic() - start:.1f} s.")
        return finish(goal_handle, result, True, "ok")

    def save_reference(self):
        if not self.save_path:
            return ""
        try:
            os.makedirs(os.path.dirname(self.save_path) or ".", exist_ok=True)
            if cv2.imwrite(self.save_path, self.ref_warped):
                return self.save_path
        except OSError as e:
            self.get_logger().warn(f"Could not save reference snapshot: {e}")
        return ""


def main(args=None):
    spin(QrDetectorNode, args)


if __name__ == "__main__":
    main()
