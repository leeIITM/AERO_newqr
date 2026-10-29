"""Geotagged survey frames during the geofenced target search (capture + geotag of airdrop_survey).

While active (target_search switches it on for the search through ~/set_active), every `capture_spacing` m
of horizontal travel the newest SIYI frame is saved as a JPEG with EXIF GPS + attitude, exactly the tags
of airdrop_survey's add_geotag_metadata. The frame comes from /camera/image_raw, i.e. the SIYI RTSP stream
already held by siyi_camera's rtsp_camera_node (a second RTSP client on the SIYI adds lag or fails).

Position for the geotag: /mavros/global_position/global (fresh fix), else the local pose converted with
the local frame's lat/lon origin (bench / no fix); altitude = fix altitude (AMSL), else height above the pad.
Attitude: /flight_controller/pose heading, pitch, roll (degrees 0-360 in the UserComment, as airdrop_survey).

Service   ~/set_active  std_srvs/SetBool   true = new folder <save_dir>/<date_time>/, start capturing
Inputs    /camera/image_raw, /flight_controller/pose, /mavros/global_position/global
Output    frame000000.jpg ... + index.csv (frame, time, lat, lon, alt, yaw, pitch, roll, local N/E, height)

Parameters: save_dir (~/bc_geotag), capture_spacing (3.0 m), jpeg_quality (92), image_topic, gps_topic,
gps_timeout (2.0 s), min_height (1.0 m: nothing is captured on the ground), frame_timeout (1.0 s).
"""
import math
import os
import threading
import time

import cv2
from mission_interfaces.msg import VehicleState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from std_srvs.srv import SetBool

from bc_common.image import imgmsg_to_bgr
from bc_common.ros_utils import DETECTION_QOS, IMAGE_QOS, IMAGE_TOPIC, POSE_TOPIC, spin
from payload_drop.geotag import SpacingTrigger, wrap_deg, write_geotag

METRES_PER_DEG = 111318.84502145034  # ArduPilot flat-earth scaling (same as target_search)


class SurveyAndImageCapture(Node):
    def __init__(self):
        super().__init__("survey_capture")
        self.callback_group = ReentrantCallbackGroup()
        p = self.declare_parameter
        self.save_root = os.path.expanduser(str(p("save_dir", "~/bc_geotag").value))
        self.spacing = float(p("capture_spacing", 3.0).value)
        self.quality = int(p("jpeg_quality", 92).value)
        self.gps_timeout = float(p("gps_timeout", 2.0).value)
        self.min_height = float(p("min_height", 1.0).value)
        self.frame_timeout = float(p("frame_timeout", 1.0).value)
        image_topic = str(p("image_topic", IMAGE_TOPIC).value)
        gps_topic = str(p("gps_topic", "/mavros/global_position/global").value)

        self.mission_active = False
        self.frame_counter = 0
        self.save_dir = None
        self.trigger = SpacingTrigger(self.spacing)
        self.frame_lock = threading.Lock()
        self.latest_frame = None      # (monotonic time, Image msg): converted only when captured
        self.pose = None
        self.fix = None               # (monotonic time, lat, lon, alt)
        self.backend = None

        self.create_subscription(Image, image_topic, self.image_callback, IMAGE_QOS, callback_group=self.callback_group)
        self.create_subscription(VehicleState, POSE_TOPIC, self.pose_callback, DETECTION_QOS,
                                 callback_group=self.callback_group)
        self.create_subscription(NavSatFix, gps_topic, self.gps_callback, qos_profile_sensor_data,
                                 callback_group=self.callback_group)
        self.create_service(SetBool, "~/set_active", self.set_active_callback, callback_group=self.callback_group)
        self.create_timer(0.1, self.tick, callback_group=self.callback_group)
        self.get_logger().info(f"Survey capture ready: every {self.spacing:.1f} m while active, into {self.save_root}")

    # ------------------------------------------------------------------ inputs

    def image_callback(self, msg):
        with self.frame_lock:
            self.latest_frame = (time.monotonic(), msg)

    def pose_callback(self, msg):
        self.pose = msg

    def gps_callback(self, msg):
        if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.fix = (time.monotonic(), msg.latitude, msg.longitude, msg.altitude)

    # ------------------------------------------------------------------ control

    def set_active_callback(self, request, response):
        if request.data and not self.mission_active:
            self.save_dir = os.path.join(self.save_root, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(self.save_dir, exist_ok=True)
            with open(os.path.join(self.save_dir, "index.csv"), "w") as f:
                f.write("frame,time,lat,lon,alt,yaw_deg,pitch_deg,roll_deg,north,east,height,gps_source\n")
            self.frame_counter = 0
            self.trigger.reset()
            self.mission_active = True
            self.get_logger().info(f"Survey capture STARTED -> {self.save_dir}")
        elif not request.data and self.mission_active:
            self.mission_active = False
            self.get_logger().info(f"Survey capture stopped: {self.frame_counter} geotagged frames in {self.save_dir}")
        response.success = True
        response.message = self.save_dir or ""
        return response

    def tick(self):
        if not self.mission_active:
            return
        p = self.pose
        if p is None or not p.valid:
            return
        height = p.alt_rel if math.isfinite(p.alt_rel) else -p.z
        if height < self.min_height:
            return
        if self.trigger(p.x, p.y):
            self.capture_and_save_image()

    # ------------------------------------------------------------------ capture (airdrop_survey)

    def geoposition(self):
        """(lat, lon, alt, source) for the geotag, or None."""
        p = self.pose
        if self.fix is not None and time.monotonic() - self.fix[0] < self.gps_timeout:
            return self.fix[1], self.fix[2], self.fix[3], "gps"
        if p is not None and p.xy_global:
            lat = p.ref_lat + p.x / METRES_PER_DEG
            lon = p.ref_lon + p.y / (METRES_PER_DEG * math.cos(math.radians(p.ref_lat)))
            height = p.alt_rel if math.isfinite(p.alt_rel) else -p.z
            return lat, lon, height, "local"
        return None

    def capture_and_save_image(self):
        with self.frame_lock:
            latest = self.latest_frame
        if latest is None or time.monotonic() - latest[0] > self.frame_timeout:
            self.get_logger().warning("No camera frame available; skipping capture.", throttle_duration_sec=5.0)
            self.trigger.reset()  # try again on the next tick
            return
        geo = self.geoposition()
        if geo is None:
            self.get_logger().warning("No GPS fix / local origin; skipping capture.", throttle_duration_sec=5.0)
            self.trigger.reset()
            return
        try:
            frame = imgmsg_to_bgr(latest[1])
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        height, width = frame.shape[:2]
        p = self.pose
        lat, lon, alt, source = geo
        yaw, pitch, roll = wrap_deg(p.heading), wrap_deg(p.pitch), wrap_deg(p.roll)
        file_path = os.path.join(self.save_dir, f"frame{self.frame_counter:06d}.jpg")
        if not cv2.imwrite(file_path, frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]):
            self.get_logger().error(f"Could not write {file_path}")
            return
        try:
            self.backend = write_geotag(file_path, width, height, lat, lon, alt, yaw, pitch, roll)
        except Exception as exc:  # noqa: BLE001 - keep the frame even if tagging fails
            self.get_logger().error(f"Geotagging failed: {exc}")
        h = p.alt_rel if math.isfinite(p.alt_rel) else -p.z
        with open(os.path.join(self.save_dir, "index.csv"), "a") as f:
            f.write(f"frame{self.frame_counter:06d}.jpg,{time.time():.3f},{lat:.8f},{lon:.8f},{alt:.2f},"
                    f"{yaw:.2f},{pitch:.2f},{roll:.2f},{p.x:.2f},{p.y:.2f},{h:.2f},{source}\n")
        self.get_logger().info(f"Captured and geotagged {os.path.basename(file_path)}: {lat:.7f}, {lon:.7f}, "
                               f"alt {alt:.1f} m, yaw {yaw:.1f} deg ({source})")
        self.frame_counter += 1


def main(args=None):
    spin(SurveyAndImageCapture, args)


if __name__ == "__main__":
    main()
