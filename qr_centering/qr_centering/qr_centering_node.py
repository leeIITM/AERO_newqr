"""Precision QR centring + hover hold (stage 10 of blue_green_qr_fixed.py).

  1. Fly to the matched QR estimate (approach_tolerance, default 0.25 m).
  2. Up to max_iters times: settle, take a fresh frame with the MATCHED QR in view (a decoy is never
     centred on); done if within tol_px of the image centre, else fly to (drone position at that
     frame + gain x measured offset), arriving within servo_tolerance.
  3. Hover hold_time s (15 s) over the QR. While the matched QR is in view and more than
     hold_deadband_px off centre, nudge back over it (hold_gain x offset) at most once per
     hold_nudge_period s, so wind / EKF drift is corrected during the hold.
`any_qr` (target_search found no match and flew to the best candidate): centre on any QR in view.
Never moves into red (repo lawnmower_mission.py): a centring step or hold nudge that would end within
`centre_clearance` m of the red mapped by target_search (/perception/red_map, latched) is halved up to
4 times; if still too close, centring stops / the nudge is skipped and the drone holds where it is.

Action: /center_on_qr (mission_interfaces/CenterOnQr)
Input:  /perception/qr, /perception/red_map (optional)
Uses:   /flight_controller/goto, /qr_detector/set_active (on while centring/holding)
"""
import math
import time

from mission_interfaces.action import CenterOnQr
from mission_interfaces.msg import QrDetection, RedMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import SetBool

import numpy as np
from red_avoidance.zone_frame import ZoneFrame, redmap_from_grid

from bc_common.ros_utils import (
    QR_ACTIVE_SERVICE, QR_TOPIC, FlightClient, GotoOutcome, MessageCache, finish, serve_action, set_qr_active,
    sleep_checked, spin,
)


def matched_qr(d):
    return d.detected and d.matched and d.pose_valid


def any_qr(d):
    return d.detected and d.pose_valid


class QrCentering(Node):
    def __init__(self):
        super().__init__("qr_centering")
        self.frame_timeout = float(self.declare_parameter("frame_timeout", 1.5).value)
        self.servo_tolerance = float(self.declare_parameter("servo_tolerance", 0.1).value)  # m
        self.settle_time = float(self.declare_parameter("settle_time", 0.5).value)  # s before measuring
        self.nudge_period = float(self.declare_parameter("hold_nudge_period", 1.0).value)  # s
        self.deadband_px = float(self.declare_parameter("hold_deadband_px", 10.0).value)
        self.clearance = float(self.declare_parameter("centre_clearance", 1.0).value)  # m from red
        self.red = None  # (ZoneFrame, RedMap)
        group = ReentrantCallbackGroup()
        self.detections = MessageCache(self, QrDetection, QR_TOPIC, callback_group=group)
        self.flight = FlightClient(self, group)
        self.qr_client = self.create_client(SetBool, QR_ACTIVE_SERVICE, callback_group=group)
        serve_action(self, CenterOnQr, "/center_on_qr", self.execute_with_qr, group)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RedMap, "/perception/red_map", self.red_map_cb, latched, callback_group=group)

    def red_map_cb(self, m):
        grid = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape((m.height, m.width))
        self.red = (ZoneFrame(m.origin_north, m.origin_east, m.axis_heading),
                    redmap_from_grid(m.grid_x0, m.grid_y0, m.cell, grid))

    def too_close_to_red(self, north, east):
        if self.red is None:
            return False
        frame, red = self.red
        return red.blocked(frame.to_zone(north, east), self.clearance)

    def safe_step(self, m, gain):
        """Correction toward the QR seen in `m`, from the drone position at that frame, shrunk so it does
        not end within centre_clearance of red. Returns (north, east) or None (too close even when small)."""
        step_n = gain * (m.position_estimate.x - m.drone_x)
        step_e = gain * (m.position_estimate.y - m.drone_y)
        for _ in range(4):
            if not self.too_close_to_red(m.drone_x + step_n, m.drone_y + step_e):
                return m.drone_x + step_n, m.drone_y + step_e
            step_n, step_e = step_n / 2.0, step_e / 2.0
        return None

    def execute_with_qr(self, goal_handle):
        if not set_qr_active(self, self.qr_client, True):
            return finish(goal_handle, CenterOnQr.Result(), False, "qr_detector unavailable", self.get_logger())
        try:
            return self.execute(goal_handle)
        finally:
            set_qr_active(self, self.qr_client, False)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = CenterOnQr.Result()
        log = self.get_logger()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        target = [req.target.x, req.target.y, req.target.z]
        tol = req.tol_px if req.tol_px > 0 else 25.0
        gain = req.gain if req.gain > 0 else 0.7
        iters = req.max_iters if req.max_iters > 0 else 6
        approach = req.approach_tolerance if req.approach_tolerance > 0 else 0.25
        feedback = CenterOnQr.Feedback()
        self.want = any_qr if req.any_qr else matched_qr

        outcome = self.flight.goto(*target, yaw=req.yaw, tolerance=approach, cancel=cancel)
        if not outcome.ok:
            return self.fail(goal_handle, result, outcome)

        locked = False
        for it in range(1, iters + 1):
            if not sleep_checked(self.settle_time, cancel):  # measure only once the drone has stopped
                return finish(goal_handle, result, False, "canceled")
            m = self.detections.wait_for(self.detections.mark(), self.want, self.frame_timeout, cancel)
            if cancel():
                return finish(goal_handle, result, False, "canceled")
            if m is None:
                log.info(f"[QR CENTERING] Iter {it}/{iters}: {'QR' if req.any_qr else 'matched QR'} not in view, "
                         "holding position...")
                continue
            err_px = math.hypot(m.u - m.image_width / 2.0, m.v - m.image_height / 2.0)
            feedback.iteration, feedback.error_px = it, float(err_px)
            goal_handle.publish_feedback(feedback)
            if err_px <= tol:
                log.info(f"[QR CENTERING] Lock acquired! (Offset {err_px:.1f}px <= tol {tol:.1f}px)")
                locked = True
                break
            # correct from where the drone WAS when the frame was taken (not from the old setpoint)
            step = self.safe_step(m, gain)
            if step is None:
                log.warn(f"[QR CENTERING] the QR is within {self.clearance:.1f} m of red: "
                         "stopping here instead of moving closer")
                break
            target[0], target[1] = step
            log.info(f"[QR CENTERING] Iter {it}/{iters}: offset {err_px:.1f}px -> "
                     f"N={target[0]:+.2f} E={target[1]:+.2f}")
            outcome = self.flight.goto(*target, yaw=req.yaw, tolerance=self.servo_tolerance, cancel=cancel)
            if not outcome.ok:
                return self.fail(goal_handle, result, outcome)
        if not locked:
            log.warn(f"[QR CENTERING] not centred after {iters} iterations; holding where it is.")

        if req.hold_time > 0:
            error = self.hold(goal_handle, req, target, feedback, cancel)
            if error is not None:
                return error
        result.locked = locked
        result.position.x, result.position.y, result.position.z = map(float, target)
        return finish(goal_handle, result, True, "centred" if locked else "not centred")

    def hold(self, goal_handle, req, target, feedback, cancel):
        """Hover hold with vision nudges. Returns a finished result on failure, else None."""
        log = self.get_logger()
        hold_gain = req.hold_gain if req.hold_gain > 0 else 0.5
        log.info(f"[ALIGN] Hover hold for {req.hold_time:.1f} s over the QR...")
        end = time.monotonic() + req.hold_time
        last_nudge = time.monotonic()
        mark = self.detections.mark()
        while (left := end - time.monotonic()) > 0:
            if cancel():
                return finish(goal_handle, CenterOnQr.Result(), False, "canceled")
            feedback.hold_remaining = float(left)
            goal_handle.publish_feedback(feedback)
            m = self.detections.wait_for(mark, self.want, timeout=min(0.2, left), cancel=cancel)
            if m is None:
                continue
            mark = self.detections.mark()
            err_px = math.hypot(m.u - m.image_width / 2.0, m.v - m.image_height / 2.0)
            if time.monotonic() - last_nudge < self.nudge_period or err_px <= self.deadband_px:
                continue
            step = self.safe_step(m, hold_gain)
            if step is None:  # the QR is next to red: hold where it is
                last_nudge = time.monotonic()
                continue
            target[0], target[1] = step
            outcome = self.flight.goto(*target, yaw=req.yaw, tolerance=self.servo_tolerance, cancel=cancel)
            if not outcome.ok:
                return self.fail(goal_handle, CenterOnQr.Result(), outcome)
            last_nudge = time.monotonic()
        feedback.hold_remaining = 0.0
        goal_handle.publish_feedback(feedback)
        log.info(f"[ALIGN HOVER] {req.hold_time:.0f}-second hold complete!")
        return None

    def fail(self, goal_handle, result, outcome):
        if outcome.status == GotoOutcome.CANCELED:
            return finish(goal_handle, result, False, "canceled")
        return finish(goal_handle, result, False, f"GoTo failed: {outcome.message}", self.get_logger())


def main(args=None):
    spin(QrCentering, args)


if __name__ == "__main__":
    main()
