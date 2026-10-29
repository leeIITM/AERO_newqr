"""target_search: delivery-zone QR search with red-zone avoidance (stage 9), and red-aware transit legs.

The search itself is in search_logic.py (no ROS); this node connects it to ROS.

Actions
    /search_target   mission_interfaces/SearchTarget   sweep the geofenced zone, stop on the matching QR
    /safe_goto       mission_interfaces/SafeGoto       fly to a point keeping clear of the mapped red
Inputs
    /flight_controller/pose   VehicleState (pose history for pairing frames; local frame lat/lon origin)
    /perception/red_zone      RedZoneResult (red_zone_detector, every frame)
    /perception/qr            QrDetection (qr_detector)
    Each frame is placed with the pose of its own header stamp (minus camera_latency), not the latest pose.
Outputs
    /flight_controller/setpoint  PositionSetpoint, 20 Hz carrot while the navigator flies
    /flight_controller/goto      GoTo goals (straight SafeGoto legs, final approach)
    /flight_controller/survey    Survey goals: the lawnmower as AUTO missions (search_mode auto)
    /perception/red_map          RedMap (latched): the red seen so far, for qr_centering and the HUD
    /qr_detector/set_active      switched on for the search, off after it
    /survey_capture/set_active   geotagged frames every N m during the search (optional)

Parameters
    hfov_rad (1.22), camera_latency (0.0 s), camera_on_gimbal (true: SIYI gimbal keeps the camera straight
    down, roll/pitch not applied), max_tilt (0.12 rad, fixed camera only), geofence_tolerance (1.0 m),
    accel (3.0 m/s^2), max_lead (0 = auto), cluster_radius (1.5 m), no_frames_timeout (10 s),
    transit_pad (10 m),
    search_mode ("auto": the lawnmower is flown as ArduPilot AUTO missions, GUIDED only round the red |
    "guided": the whole search in GUIDED with streamed setpoints)
"""
import math
import os
import threading
import time
import traceback

import numpy as np
from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus
from mission_interfaces.action import SafeGoto, SearchTarget, Survey
from rclpy.action import ActionClient
from mission_interfaces.msg import PositionSetpoint, QrDetection, RedMap, RedZoneResult, VehicleState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import SetBool

from bc_common.ros_utils import (
    DETECTION_QOS, POSE_TOPIC, QR_ACTIVE_SERVICE, QR_TOPIC, FlightClient, GotoOutcome, VehicleListener, finish,
    call_service, serve_action, set_qr_active, spin,
)
from target_search import geometry as geo
from target_search import search_logic as sl

RED_ZONE_TOPIC = "/perception/red_zone"
CAPTURE_ACTIVE_SERVICE = "/survey_capture/set_active"
RED_MAP_TOPIC = "/perception/red_map"
SETPOINT_TOPIC = "/flight_controller/setpoint"
SURVEY_ACTION = "/flight_controller/survey"
LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class RosPlatform:
    def __init__(self, node):
        self.node = node
        self.goal_handle = None
        self.feedback_type = None
        self.survey_handle = None
        self.survey_result = None
        self.survey_index = 0

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds):
        time.sleep(seconds)

    def position(self):
        """Waits for a fresh pose (no timeout, logs every 5 s; the drone holds meanwhile, the setpoint stream
        stops). None only if the goal is cancelled while waiting."""
        v = self.node.vehicle
        if not v.wait_fresh(self.node.get_logger(), "target_search", self.cancel_requested):
            return None
        p = v.latest
        return (float(p.x), float(p.y), float(p.z), float(p.heading), float(v.alt_rel()))

    def position_nowait(self):
        v = self.node.vehicle
        if not v.fresh():
            return None
        p = v.latest
        return (float(p.x), float(p.y), float(p.z), float(p.heading), float(v.alt_rel()))

    # AUTO survey (flight_controller ~/survey)

    def survey_start(self, points, down, yaw, speed):
        client = self.node.survey_client
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=5.0):
            return False, "flight_controller survey server not available"
        time.sleep(0.6)  # let the setpoint stream time out so flight_controller accepts the goal
        goal = Survey.Goal(north=[float(n) for n, _ in points], east=[float(e) for _, e in points],
                           down=float(down), yaw=float(yaw), speed=float(speed), acceptance_radius=0.0)
        self.survey_index, self.survey_result = 0, None
        send = client.send_goal_async(goal, feedback_callback=self._survey_feedback)
        deadline = time.monotonic() + 5.0
        while not send.done():
            if time.monotonic() > deadline:
                return False, "survey goal not answered"
            time.sleep(0.02)
        handle = send.result()
        if not handle.accepted:
            return False, "survey goal rejected (another goal running?)"
        self.survey_handle = handle
        self.survey_result = handle.get_result_async()
        return True, "ok"

    def _survey_feedback(self, msg):
        fb = getattr(msg, "feedback", msg)
        self.survey_index = int(fb.current_index)

    def survey_poll(self):
        fut = self.survey_result
        if fut is None:
            return "failed", self.survey_index, "no survey running"
        if not fut.done():
            return "running", self.survey_index, ""
        res = fut.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success:
            return "done", int(res.result.reached), "ok"
        return "failed", int(res.result.reached), res.result.message or f"status {res.status}"

    def survey_stop(self):
        """Cancel the AUTO mission and wait until flight_controller is back in GUIDED, holding."""
        handle, fut = self.survey_handle, self.survey_result
        if handle is None or fut is None:
            return self.survey_index
        if not fut.done():
            handle.cancel_goal_async()
            deadline = time.monotonic() + 15.0
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        index = self.survey_index
        if fut.done():
            index = int(fut.result().result.reached)
        self.survey_handle = self.survey_result = None
        return index

    def stream(self, north, east, down, yaw):
        msg = PositionSetpoint()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.x, msg.y, msg.z, msg.yaw = float(north), float(east), float(down), float(yaw)
        self.node.setpoint_pub.publish(msg)

    def stop_stream(self):
        pos = self.position()
        if pos is not None:  # last setpoint where the drone is: it brakes and holds there
            self.stream(pos[0], pos[1], pos[2], pos[3])

    def goto(self, north, east, down, yaw, speed):
        cancel = (lambda: self.goal_handle.is_cancel_requested) if self.goal_handle else None
        time.sleep(0.6)  # let the setpoint stream time out so flight_controller accepts the goal
        out = self.node.flight.goto(north, east, down, yaw=yaw, speed=speed, cancel=cancel)
        return out.status == GotoOutcome.REACHED, out.message

    def cancel_requested(self):
        return self.goal_handle is not None and self.goal_handle.is_cancel_requested

    def feedback(self, **fields):
        if self.goal_handle is None or self.feedback_type is None:
            return
        fb = self.feedback_type()
        for k, v in fields.items():
            if hasattr(fb, k) or k in getattr(fb, "_fields", {}):
                try:
                    setattr(fb, k, type(getattr(fb, k))(v))
                except (AttributeError, TypeError, ValueError, AssertionError):
                    pass
        self.goal_handle.publish_feedback(fb)

    def log(self, level, text):
        # one call line per severity: rclpy remembers the severity of every logging call site and raises
        # "Logger severity cannot be changed between calls" if the same line logs at another level
        logger = self.node.get_logger()
        if level == "error":
            logger.error(text)
        elif level in ("warn", "warning"):
            logger.warn(text)
        elif level == "debug":
            logger.debug(text)
        else:
            logger.info(text)


class TargetSearch(Node):
    def __init__(self):
        super().__init__("target_search")
        p = self.declare_parameter
        cfg = sl.Config(
            hfov=float(p("hfov_rad", 1.22).value),
            camera_latency=float(p("camera_latency", 0.0).value),
            camera_on_gimbal=bool(p("camera_on_gimbal", True).value),
            max_tilt=float(p("max_tilt", 0.12).value),
            geofence_tolerance=float(p("geofence_tolerance", 1.0).value),
            accel=float(p("accel", 3.0).value),
            max_lead=float(p("max_lead", 0.0).value),
            cluster_radius=float(p("cluster_radius", 1.5).value),
            no_frames_timeout=float(p("no_frames_timeout", 10.0).value),
            transit_pad=float(p("transit_pad", 10.0).value),
            search_mode=str(p("search_mode", "auto").value).lower(),
        )
        if cfg.search_mode not in ("auto", "guided"):
            raise ValueError(f"search_mode must be 'auto' or 'guided', not {cfg.search_mode!r}")
        self.platform = RosPlatform(self)
        self.mission = sl.SearchMission(self.platform, cfg)
        self.busy = threading.Lock()
        self.published_version = None

        group = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, group)
        self.create_subscription(VehicleState, POSE_TOPIC, self.pose_cb, DETECTION_QOS, callback_group=group)
        self.create_subscription(RedZoneResult, RED_ZONE_TOPIC, self.red_cb, DETECTION_QOS, callback_group=group)
        self.create_subscription(QrDetection, QR_TOPIC, self.qr_cb, DETECTION_QOS, callback_group=group)
        self.setpoint_pub = self.create_publisher(PositionSetpoint, SETPOINT_TOPIC, 10)
        self.map_pub = self.create_publisher(RedMap, RED_MAP_TOPIC, LATCHED)
        self.flight = FlightClient(self, group)
        self.survey_client = ActionClient(self, Survey, SURVEY_ACTION, callback_group=group)
        self.qr_client = self.create_client(SetBool, QR_ACTIVE_SERVICE, callback_group=group)
        self.capture_client = self.create_client(SetBool, CAPTURE_ACTIVE_SERVICE, callback_group=group)
        self.ref = None
        serve_action(self, SearchTarget, "/search_target", self.execute_search, group)
        serve_action(self, SafeGoto, "/safe_goto", self.execute_safe_goto, group)
        self.create_timer(1.0, self.publish_map, callback_group=group)
        self.get_logger().info(f"target_search ready (hfov {cfg.hfov:.2f} rad, camera latency "
                               f"{cfg.camera_latency:.2f} s, {'gimbal' if cfg.camera_on_gimbal else 'fixed'} camera, "
                               f"search_mode {cfg.search_mode}).")

    # ---------------------------------------------------------------- inputs

    def pose_cb(self, m):
        if m.valid:
            alt = m.alt_rel if math.isfinite(m.alt_rel) else -m.z
            self.mission.on_pose(stamp_s(m.header), m.x, m.y, alt, m.heading, m.roll, m.pitch)

    def red_cb(self, m):
        patches = [[(pt.x, pt.y) for pt in patch.contour] for patch in m.patches]
        self.mission.on_red(stamp_s(m.header), m.image_width, m.image_height, patches)

    def qr_cb(self, m):
        if m.detected:
            self.mission.on_qr(stamp_s(m.header), m.image_width, m.image_height, m.u, m.v, m.matched, m.score,
                               m.decoded_text)

    # ---------------------------------------------------------------- red map out

    def publish_map(self):
        snap = self.mission.red_snapshot()
        if snap is None:
            return
        frame, red, margin = snap
        with self.mission.lock:
            if red.version == self.published_version:
                return
            self.published_version = red.version
            grid = np.ascontiguousarray(red.grid, dtype=np.uint8)
            msg = RedMap()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "local_ned"
            msg.origin_north, msg.origin_east = float(frame.on), float(frame.oe)
            msg.axis_heading = float(frame.axis_heading)
            msg.cell = float(red.cell)
            msg.grid_x0, msg.grid_y0 = float(red.x0), float(red.y0)
            msg.height, msg.width = int(grid.shape[0]), int(grid.shape[1])
            msg.data = grid.tobytes()
            msg.version = int(red.version)
            msg.margin = float(margin)
        self.map_pub.publish(msg)

    # ---------------------------------------------------------------- survey capture

    def set_capture(self, active):
        """Geotagged survey frames (payload_drop's survey_capture) while searching. Optional: the search
        runs without it (e.g. survey_capture not launched)."""
        if not self.capture_client.wait_for_service(timeout_sec=1.0 if active else 0.2):
            if active:
                self.get_logger().warn("survey_capture not available: no geotagged frames for this search")
            return False
        response = call_service(self.capture_client, SetBool.Request(data=bool(active)))
        return response is not None and response.success

    # ---------------------------------------------------------------- actions

    def execute_search(self, goal_handle):
        req = goal_handle.request
        result = SearchTarget.Result()
        log = self.get_logger()
        if not self.busy.acquire(blocking=False):
            return finish(goal_handle, result, False, "a search or SafeGoto is running", log)
        self.platform.goal_handle, self.platform.feedback_type = goal_handle, SearchTarget.Feedback
        qr_on = False
        self.ref = None
        try:
            qr_on = set_qr_active(self, self.qr_client, True)  # qr_detector only decodes while asked to
            self.set_capture(True)                             # geotagged survey frames during the search
            outcome = self.search(req) if qr_on else sl.Outcome("failed", "qr_detector set_active unavailable")
        except Exception as exc:  # noqa: BLE001 - report instead of leaving the mission hanging
            log.error(f"SearchTarget crashed: {traceback.format_exc()}")
            outcome = sl.Outcome("failed", f"internal error: {exc}")
        finally:
            if qr_on:
                set_qr_active(self, self.qr_client, False)
            self.set_capture(False)
            self.platform.goal_handle = None
            self.busy.release()
        self.publish_map()
        result.matched, result.score, result.decoded_text = outcome.matched, float(outcome.score), outcome.text
        result.target = Point(x=float(outcome.target[0]), y=float(outcome.target[1]), z=float("nan"))
        result.latitude = result.longitude = float("nan")
        if outcome.status == "success":
            pos = self.platform.position()
            result.target.z = float(pos[2]) if pos else float("nan")
            if self.ref is not None:
                result.latitude, result.longitude = geo.ned_to_latlon(outcome.target[0], outcome.target[1],
                                                                      *self.ref)
        result.red_zones = int(outcome.red_zones)
        result.coverage = float(outcome.coverage)
        if outcome.status == "success":
            log.info(f"SearchTarget done: {outcome.message} (red zones {outcome.red_zones}, "
                     f"coverage {100 * outcome.coverage:.0f} %)")
        return finish(goal_handle, result, outcome.status == "success", outcome.message, log)

    def search(self, req):
        if not req.geofence_file:
            return sl.Outcome("failed", "SearchTarget.geofence_file is empty")
        path = os.path.expanduser(req.geofence_file)
        try:
            with open(path) as f:
                corners = geo.parse_geofence(f.read())
        except (OSError, ValueError) as exc:
            return sl.Outcome("failed", f"geofence file {path}: {exc}")
        v = self.vehicle.latest
        if v is None or not v.xy_global:
            return sl.Outcome("failed", "no lat/lon origin for the local frame (EKF origin / GPS fix)")
        if not self.flight.wait_for_server(5.0):
            return sl.Outcome("failed", "flight_controller GoTo server not available")
        request = sl.Request(altitude=req.altitude, yaw=req.yaw, wall_margin=req.wall_margin, overlap=req.overlap,
                             rows=int(req.rows), speed=req.speed if req.speed > 0 else 1.2,
                             margin=req.red_zone_margin, confirm=max(1, int(req.confirm_matches)),
                             max_revisits=int(req.max_revisits), revisit_time=req.revisit_time,
                             skip_fly=bool(req.skip_fly_to_target))
        self.ref = (v.ref_lat, v.ref_lon)
        self.get_logger().info(f"Searching the zone of {len(corners)} geofence corners from {path} "
                               f"(local origin {v.ref_lat:.7f}, {v.ref_lon:.7f}).")
        return self.mission.run(request, corners, (v.ref_lat, v.ref_lon))

    def execute_safe_goto(self, goal_handle):
        req = goal_handle.request
        result = SafeGoto.Result()
        if not self.busy.acquire(blocking=False):
            return finish(goal_handle, result, False, "a search or SafeGoto is running", self.get_logger())
        self.platform.goal_handle, self.platform.feedback_type = goal_handle, SafeGoto.Feedback
        try:
            pos = self.platform.position()
            t = req.target
            if pos is None:
                ok, msg, detoured = False, "no vehicle pose", False
            else:
                target = [t.x if math.isfinite(t.x) else pos[0], t.y if math.isfinite(t.y) else pos[1],
                          t.z if math.isfinite(t.z) else pos[2]]
                ok, msg, detoured = self.mission.safe_goto(*target, req.yaw, req.speed if req.speed > 0 else 1.2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SafeGoto crashed: {traceback.format_exc()}")
            ok, msg, detoured = False, f"internal error: {exc}", False
        finally:
            self.platform.goal_handle = None
            self.busy.release()
        result.detoured = bool(detoured)
        return finish(goal_handle, result, ok, msg, self.get_logger())


def main(args=None):
    spin(TargetSearch, args)


if __name__ == "__main__":
    main()
