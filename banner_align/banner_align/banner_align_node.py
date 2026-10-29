"""Banner alignment: position-based visual servoing over a classified banner (stage 5 green,
stage 11 blue), re-evaluating the full classification every step.

  1. Fly to the estimate from SearchBanner holding `expected_heading`.
  2. Up to max_iters times: take a fresh classified frame;
       - corridor heading = perpendicular to the banner's long axis, 180 deg ambiguity resolved
         towards expected_heading, corrections > max_heading_correction rejected;
       - locked if centred within tol_px AND the banner is fully in view;
       - otherwise fly to (drone position at that frame + gain x measured offset), arriving within
         servo_tolerance (0.1 m), and settle before the next measurement.
     A frame without a classified banner holds position for one iteration.

Action: /align_banner (mission_interfaces/AlignBanner)
Input:  /perception/banner
Uses:   /flight_controller/goto, /banner_detector/set_color
"""
import math

from mission_interfaces.action import AlignBanner
from mission_interfaces.msg import BannerDetection
from mission_interfaces.srv import SetBannerColor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bc_common.geometry import corridor_heading_from_banner
from bc_common.ros_utils import (
    BANNER_COLOR_SERVICE, BANNER_TOPIC, FlightClient, GotoOutcome, MessageCache, VehicleListener, finish,
    serve_action, set_banner_color, sleep_checked, spin,
)


class BannerAlign(Node):
    def __init__(self):
        super().__init__("banner_align")
        self.frame_timeout = float(self.declare_parameter("frame_timeout", 1.0).value)
        self.servo_tolerance = float(self.declare_parameter("servo_tolerance", 0.1).value)  # m
        group = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, group)
        self.detections = MessageCache(self, BannerDetection, BANNER_TOPIC, callback_group=group)
        self.color_client = self.create_client(SetBannerColor, BANNER_COLOR_SERVICE, callback_group=group)
        self.flight = FlightClient(self, group)
        serve_action(self, AlignBanner, "/align_banner", self.execute, group)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = AlignBanner.Result()
        log = self.get_logger()
        color = req.color.lower()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        max_corr = req.max_heading_correction if req.max_heading_correction > 0 else math.radians(25.0)
        tol = req.tol_px if req.tol_px > 0 else 25.0
        gain = req.gain if req.gain > 0 else 0.7
        iters = req.max_iters if req.max_iters > 0 else 6

        if not self.vehicle.wait_fresh(log, "AlignBanner", cancel):  # rides out a MAVROS data gap
            return finish(goal_handle, result, False, "canceled")
        if not set_banner_color(self, self.color_client, color):
            return finish(goal_handle, result, False, "banner_detector unavailable", log)

        cur = self.vehicle.ned()
        t = req.initial_target
        target = [t.x if math.isfinite(t.x) else cur[0], t.y if math.isfinite(t.y) else cur[1],
                  t.z if math.isfinite(t.z) else cur[2]]
        yaw_lock = req.expected_heading
        corridor_heading = req.expected_heading
        measured = locked = False

        outcome = self.flight.goto(*target, yaw=yaw_lock, tolerance=self.servo_tolerance, cancel=cancel)
        if not outcome.ok:
            return self.fail(goal_handle, result, outcome)
        if req.settle_time > 0 and not sleep_checked(req.settle_time, cancel):
            return finish(goal_handle, result, False, "canceled")

        feedback = AlignBanner.Feedback()
        for it in range(1, iters + 1):
            mark = self.detections.mark()
            m = self.detections.wait_for(mark, lambda d: d.color == color and d.pose_valid and d.detected,
                                         timeout=self.frame_timeout, cancel=cancel)
            if cancel():
                return finish(goal_handle, result, False, "canceled")
            feedback.iteration = it
            if m is None:
                log.info(f"[CENTERING] Iter {it}/{iters}: {color.upper()} banner not classified, holding position...")
                goal_handle.publish_feedback(feedback)
                continue

            err_u, err_v = m.u - m.image_width / 2.0, m.v - m.image_height / 2.0
            err_px = math.hypot(err_u, err_v)
            feedback.error_px, feedback.partial = float(err_px), bool(m.partial)
            goal_handle.publish_feedback(feedback)
            log.info(f"[CENTERING] Iter {it}/{iters}: centroid ({m.u:.1f}, {m.v:.1f})px | Offset: {err_px:.1f}px | "
                     f"{'PARTIAL' if m.partial else 'FULL-VIEW'}")

            heading, dev, accepted = corridor_heading_from_banner(m.drone_heading, m.angle,
                                                                  req.expected_heading, max_corr)
            if accepted:
                yaw_lock = corridor_heading = heading
                measured = True
                log.info(f"  [HEADING ALIGN] corridor axis {math.degrees(heading):+.1f} deg "
                         f"(correction {math.degrees(dev):.1f} deg <= {math.degrees(max_corr):.0f})")
            else:
                log.warn(f"  [HEADING WARN] correction {math.degrees(dev):.1f} deg > {math.degrees(max_corr):.0f}; "
                         f"keeping {math.degrees(yaw_lock):+.1f} deg")

            if err_px <= tol and not m.partial:
                log.info(f"[CENTERING] Lock acquired: {err_px:.1f}px <= {tol:.1f}px, full view.")
                locked = True
                break

            # correct from where the drone WAS when the frame was taken (not from the old setpoint)
            target[0] = m.drone_x + gain * (m.position_estimate.x - m.drone_x)
            target[1] = m.drone_y + gain * (m.position_estimate.y - m.drone_y)
            outcome = self.flight.goto(*target, yaw=yaw_lock, tolerance=self.servo_tolerance, cancel=cancel)
            if not outcome.ok:
                return self.fail(goal_handle, result, outcome)
            if req.settle_time > 0 and not sleep_checked(req.settle_time, cancel):
                return finish(goal_handle, result, False, "canceled")

        if not locked:
            log.warn(f"[CENTERING] No lock after {iters} iterations; continuing with the last target.")
        result.locked = locked
        result.heading_measured = measured
        result.corridor_heading = float(corridor_heading)
        result.banner_position.x, result.banner_position.y, result.banner_position.z = map(float, target)
        return finish(goal_handle, result, True, "locked" if locked else "not locked")

    def fail(self, goal_handle, result, outcome):
        if outcome.status == GotoOutcome.CANCELED:
            return finish(goal_handle, result, False, "canceled")
        return finish(goal_handle, result, False, f"GoTo failed: {outcome.message}", self.get_logger())


def main(args=None):
    spin(BannerAlign, args)


if __name__ == "__main__":
    main()
