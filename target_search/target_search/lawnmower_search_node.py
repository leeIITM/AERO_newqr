"""Lawnmower QR search (stage 9 of blue_green_qr_fixed.py).

Flies the serpentine from the wall point at the search altitude, holding the corridor heading, and stops
at the first frame whose QR matches the reference (qr_detector compares every QR in view: same decoded
text, or grid similarity >= match_thresh when the text can't be read). The QR is placed on the ground
with the drone pose of that frame; the GoTo is cancelled (flight_controller holds = the script's BRAKE)
and the node waits `brake_time` s. No match after the last row: failure (stage 10 is then skipped).

Action: /lawnmower_search (mission_interfaces/LawnmowerSearch)
Input:  /perception/qr
Uses:   /flight_controller/goto, /qr_detector/set_active (on for the search only)
"""
import math

from mission_interfaces.action import LawnmowerSearch
from mission_interfaces.msg import QrDetection
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool

from bc_common.ros_utils import QR_ACTIVE_SERVICE, QR_TOPIC, FlightClient, GotoOutcome, MessageCache, finish, \
    serve_action, set_qr_active, sleep_checked, spin
from target_search.lawnmower import lawnmower_waypoints


def is_match(m):
    return m.detected and m.matched and m.pose_valid and math.isfinite(m.position_estimate.x)


class LawnmowerSearchNode(Node):
    def __init__(self):
        super().__init__("lawnmower_search")
        self.brake_time = float(self.declare_parameter("brake_time", 1.2).value)
        group = ReentrantCallbackGroup()
        self.detections = MessageCache(self, QrDetection, QR_TOPIC, callback_group=group)
        self.flight = FlightClient(self, group)
        self.qr_client = self.create_client(SetBool, QR_ACTIVE_SERVICE, callback_group=group)
        serve_action(self, LawnmowerSearch, "/lawnmower_search", self.execute_with_qr, group)

    def execute_with_qr(self, goal_handle):
        if not set_qr_active(self, self.qr_client, True):
            return finish(goal_handle, LawnmowerSearch.Result(), False, "qr_detector unavailable", self.get_logger())
        try:
            return self.execute(goal_handle)
        finally:
            set_qr_active(self, self.qr_client, False)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = LawnmowerSearch.Result()
        log = self.get_logger()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        start = [req.start.x, req.start.y, req.start.z]
        wps = lawnmower_waypoints(start, req.heading, req.length, req.row_step, int(req.rows))
        if not wps:
            return finish(goal_handle, result, False, "no rows", log)

        feedback = LawnmowerSearch.Feedback()
        mark = self.detections.mark()
        hit = None
        for i, wp in enumerate(wps):
            feedback.waypoint = i + 1
            feedback.state = f"waypoint {i + 1}/{len(wps)}"
            goal_handle.publish_feedback(feedback)
            log.info(f"[WAYPOINT {i + 1}/{len(wps)}] Target: N={wp[0]:+.1f} E={wp[1]:+.1f}")
            outcome = self.flight.goto(*wp, yaw=req.heading, speed=req.speed, tolerance=req.waypoint_tolerance,
                                       stop_when=lambda: self.detections.find_since(mark, is_match) is not None,
                                       cancel=cancel)
            if outcome.status == GotoOutcome.CANCELED:
                return finish(goal_handle, result, False, "canceled")
            if outcome.status == GotoOutcome.FAILED:
                return finish(goal_handle, result, False, f"GoTo failed: {outcome.message}", log)
            hit = self.detections.find_since(mark, is_match)
            if hit is not None:
                break

        if hit is None:
            return finish(goal_handle, result, False, "no QR matching the reference along the lawnmower", log)

        result.target.x, result.target.y, result.target.z = \
            float(hit.position_estimate.x), float(hit.position_estimate.y), float(req.start.z)
        result.decoded_text, result.score = hit.decoded_text, float(hit.score)
        log.info(f"[MATCH FOUND!] Similarity {100 * hit.score:.1f}% | Payload '{hit.decoded_text}' | "
                 f"target N={result.target.x:+.2f} E={result.target.y:+.2f}")
        if self.brake_time > 0 and not sleep_checked(self.brake_time, cancel):
            return finish(goal_handle, result, False, "canceled")
        return finish(goal_handle, result, True, "matched")


def main(args=None):
    spin(LawnmowerSearchNode, args)


if __name__ == "__main__":
    main()
