"""Banner search: fly a path until a banner of the requested colour is classified.

Stage 4  (green): sway left along the take-off left axis up to max_sway, detecting while moving,
                  then hover at max sway to complete the persistence check.
Stage 11 (blue):  look at the predicted return-corridor entrance, then fly the local search pattern,
                  detecting only while dwelling at each point.
On classification the GoTo is cancelled (flight_controller holds = the script's BRAKE) and the
node waits `brake_time` s for the lateral momentum to die out.

Action: /search_banner (mission_interfaces/SearchBanner)
Input:  /perception/banner, /flight_controller/pose
Uses:   /flight_controller/goto, /banner_detector/set_color
"""
import math

from mission_interfaces.action import SearchBanner
from mission_interfaces.msg import BannerDetection
from mission_interfaces.srv import SetBannerColor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bc_common.ros_utils import (
    BANNER_COLOR_SERVICE, BANNER_TOPIC, FlightClient, GotoOutcome, MessageCache, VehicleListener, finish,
    serve_action, set_banner_color, sleep_checked, spin,
)


class BannerSearch(Node):
    def __init__(self):
        super().__init__("banner_search")
        self.brake_time = float(self.declare_parameter("brake_time", 1.2).value)
        group = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, group)
        self.detections = MessageCache(self, BannerDetection, BANNER_TOPIC, callback_group=group)
        self.color_client = self.create_client(SetBannerColor, BANNER_COLOR_SERVICE, callback_group=group)
        self.flight = FlightClient(self, group)
        serve_action(self, SearchBanner, "/search_banner", self.execute, group)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = SearchBanner.Result()
        log = self.get_logger()
        color = req.color.lower()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        if not req.waypoints:
            return finish(goal_handle, result, False, "no waypoints", log)
        if not set_banner_color(self, self.color_client, color):
            return finish(goal_handle, result, False, "banner_detector unavailable", log)

        def found(msg):
            return msg.detected and msg.color == color and msg.pose_valid

        feedback = SearchBanner.Feedback()
        hit = None
        for i, wp in enumerate(req.waypoints):
            feedback.waypoint = i + 1
            feedback.state = f"flying to waypoint {i + 1}/{len(req.waypoints)}"
            goal_handle.publish_feedback(feedback)
            mark = self.detections.mark()
            stop_when = (lambda: self.detections.find_since(mark, found) is not None) \
                if req.detect_while_moving else None
            outcome = self.flight.goto(wp.x, wp.y, wp.z, yaw=req.yaw, speed=req.speed,
                                       stop_when=stop_when, cancel=cancel)
            if outcome.status == GotoOutcome.CANCELED:
                return finish(goal_handle, result, False, "canceled")
            if outcome.status == GotoOutcome.FAILED:
                return finish(goal_handle, result, False, f"GoTo failed: {outcome.message}", log)
            hit = self.detections.find_since(mark, found) if req.detect_while_moving else None
            dwell = req.dwell[i] if i < len(req.dwell) else 0.0
            if hit is None and dwell > 0.0:
                feedback.state = f"looking at waypoint {i + 1} for {dwell:.1f} s"
                goal_handle.publish_feedback(feedback)
                look = self.detections.mark()
                hit = self.detections.wait_for(look, found, timeout=dwell, cancel=cancel)
                if cancel():
                    return finish(goal_handle, result, False, "canceled")
            if hit is not None:
                result.waypoint_index = i
                break

        if hit is None:
            return finish(goal_handle, result, False,
                          f"{color.upper()} banner not classified along the search path", log)

        log.info(f"[BANNER CLASSIFIED] {color.upper()} via {hit.source} at waypoint {result.waypoint_index + 1}; "
                 f"estimate N={hit.position_estimate.x:+.2f} E={hit.position_estimate.y:+.2f}")
        if req.detect_while_moving and self.brake_time > 0:
            sleep_checked(self.brake_time, cancel)  # flight_controller is already holding position
        result.banner_estimate.x = hit.position_estimate.x
        result.banner_estimate.y = hit.position_estimate.y
        result.banner_estimate.z = hit.drone_z
        result.drone_position.x, result.drone_position.y, result.drone_position.z = \
            hit.drone_x, hit.drone_y, hit.drone_z
        result.source = hit.source
        if not all(math.isfinite(v) for v in (hit.position_estimate.x, hit.position_estimate.y)):
            return finish(goal_handle, result, False, "banner seen but no position estimate", log)
        return finish(goal_handle, result, True, "ok")


def main(args=None):
    spin(BannerSearch, args)


if __name__ == "__main__":
    main()
