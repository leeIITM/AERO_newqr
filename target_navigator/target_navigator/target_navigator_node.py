"""Target navigator: flies to the target QR's coordinates (passed as a waypoint) after the geofenced search.

target_search finds the QR (matched, or the best candidate) and reports its lat/lon without flying there
(SearchTarget.skip_fly_to_target). The mission tree passes those coordinates here as the waypoint:
  1. lat/lon -> local NED with the local frame's origin (VehicleState.ref_lat/ref_lon, the EKF origin)
  2. fly there at the requested height and heading through target_search's /safe_goto (straight when the
     line is clear, detour round the red mapped during the search otherwise); use_safe_goto: false = plain
     flight_controller GoTo
  3. check the vehicle arrived within `tolerance`; then qr_centering takes over

Action  /navigate_to_target  mission_interfaces/NavigateToTarget
Uses    /safe_goto (target_search) or /flight_controller/goto, /flight_controller/pose
Parameters: use_safe_goto (true), arrival_tolerance (1.0 m), navigator_speed (1.2 m/s, when the goal has none),
safe_goto_timeout (5 s)
"""
import math
import time

from action_msgs.msg import GoalStatus
from mission_interfaces.action import NavigateToTarget, SafeGoto
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bc_common.ros_utils import FlightClient, VehicleListener, finish, serve_action, spin, to_point
from target_navigator.waypoint import latlon_to_ned


class TargetNavigator(Node):
    def __init__(self):
        super().__init__("target_navigator")
        self.use_safe_goto = bool(self.declare_parameter("use_safe_goto", True).value)
        self.tolerance = float(self.declare_parameter("arrival_tolerance", 1.0).value)
        self.default_speed = float(self.declare_parameter("navigator_speed", 1.2).value)
        self.server_timeout = float(self.declare_parameter("safe_goto_timeout", 5.0).value)
        group = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, group)
        self.flight = FlightClient(self, group)
        self.safe_goto = ActionClient(self, SafeGoto, "/safe_goto", callback_group=group)
        serve_action(self, NavigateToTarget, "/navigate_to_target", self.execute, group)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = NavigateToTarget.Result()
        feedback = NavigateToTarget.Feedback()
        log = self.get_logger()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        if not self.vehicle.wait_fresh(log, "NavigateToTarget", cancel):  # rides out a MAVROS data gap
            return finish(goal_handle, result, False, "canceled")
        v = self.vehicle.latest
        if not v.xy_global:
            return finish(goal_handle, result, False, "no lat/lon origin for the local frame (EKF origin)", log)
        if not (math.isfinite(req.latitude) and math.isfinite(req.longitude)):
            return finish(goal_handle, result, False, "waypoint lat/lon is not set", log)
        north, east = latlon_to_ned(req.latitude, req.longitude, v.ref_lat, v.ref_lon)
        z = req.z if math.isfinite(req.z) else v.z
        speed = req.speed if req.speed > 0 else self.default_speed
        tolerance = req.tolerance if req.tolerance > 0 else self.tolerance
        result.target = to_point((north, east, z))
        dist = math.hypot(north - v.x, east - v.y)
        log.info(f"[NAVIGATOR] waypoint {req.latitude:.7f}, {req.longitude:.7f} -> N={north:+.2f} E={east:+.2f} "
                 f"z={z:+.2f} ({dist:.1f} m away, {'red-aware SafeGoto' if self.use_safe_goto else 'GoTo'})")
        feedback.state, feedback.distance = "flying to the target", float(dist)
        goal_handle.publish_feedback(feedback)

        if self.use_safe_goto:
            ok, message, detoured = self.fly_safe(north, east, z, req.yaw, speed, cancel, goal_handle, feedback)
        else:
            out = self.flight.goto(north, east, z, yaw=req.yaw, speed=speed, cancel=cancel)
            ok, message, detoured = out.ok, out.message, False
        result.detoured = bool(detoured)
        p = self.vehicle.latest
        result.position = to_point((p.x, p.y, p.z))
        if not ok:
            return finish(goal_handle, result, False, f"navigation failed: {message}", log)
        miss = math.hypot(p.x - north, p.y - east)
        if miss > tolerance:
            return finish(goal_handle, result, False, f"stopped {miss:.2f} m from the target (> {tolerance:.2f} m)", log)
        log.info(f"[NAVIGATOR] at the target QR ({miss:.2f} m){' after a detour round the red' if detoured else ''}")
        return finish(goal_handle, result, True, "at the target")

    def fly_safe(self, north, east, z, yaw, speed, cancel, goal_handle, feedback):
        """(ok, message, detoured) through target_search's /safe_goto."""
        if not self.safe_goto.server_is_ready() and not self.safe_goto.wait_for_server(timeout_sec=self.server_timeout):
            return False, "/safe_goto (target_search) not available", False
        send = self.safe_goto.send_goal_async(
            SafeGoto.Goal(target=to_point((north, east, z)), yaw=float(yaw), speed=float(speed)),
            feedback_callback=lambda fb: self.relay(fb, north, east, goal_handle, feedback))
        if not self.wait(send, 5.0):
            return False, "SafeGoto goal not answered", False
        handle = send.result()
        if not handle.accepted:
            return False, "SafeGoto goal rejected", False
        res = handle.get_result_async()
        while not res.done():
            if cancel():
                handle.cancel_goal_async()
                self.wait(res, 5.0)
                return False, "canceled", False
            time.sleep(0.05)
        r = res.result()
        ok = r.status == GoalStatus.STATUS_SUCCEEDED and r.result.success
        return ok, r.result.message, r.result.detoured

    def relay(self, fb, north, east, goal_handle, feedback):
        p = self.vehicle.latest
        feedback.state = f"steering: {fb.feedback.steering}"
        feedback.distance = float(math.hypot(north - p.x, east - p.y)) if p else float("nan")
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True


def main(args=None):
    spin(TargetNavigator, args)


if __name__ == "__main__":
    main()
