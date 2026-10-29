"""py_trees behaviours used by mission_tree.

MissionContext  config + mission state shared by the steps (the script's self.* variables)
ContextSetup    first step: declares the ROS parameters and creates the pose listener
ActionStep      sends an action goal built at tick time from the context, stores the result
Compute         runs a function on the context (SUCCESS unless it returns False / raises)
Check           condition on the context
Hold            timed hover (the vehicle holds by itself), counts down for the HUD
WaitForVehicle  RUNNING until /flight_controller/pose is fresh and ready
ReferenceCapture stage 2: reference QR capture (no timeout) while hovering, then a growing-square search
"""
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

import py_trees
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from bc_common.ros_utils import VehicleListener
from mission_management import mission_math as mm
from mission_management.config import MissionConfig


@dataclass
class MissionState:
    ground_z: float = math.nan                  # NED z of the pad, recorded before take-off
    takeoff: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # NED at transit altitude
    takeoff_yaw: float = 0.0
    ref_ok: bool = False
    ref_text: str = ""
    sp_surge1: Optional[List[float]] = None
    sp_surge2: Optional[List[float]] = None
    green_estimate: Optional[List[float]] = None
    green_banner: Optional[List[float]] = None
    corridor_heading: float = 0.0
    sway_x: float = 0.0
    corridor_exit: Optional[List[float]] = None  # after the vertical climb at the exit
    sp_wall: Optional[List[float]] = None
    target_qr: Optional[List[float]] = None
    target_latlon: Optional[List[float]] = None  # target_search's chosen QR, the navigator's waypoint
    qr_matched: bool = False             # False = target_search's best candidate (centre on any QR)
    blue_start: Optional[List[float]] = None
    return_heading: float = 0.0                 # corridor heading + 180 deg (expected)
    blue_estimate: Optional[List[float]] = None
    blue_banner: Optional[List[float]] = None
    return_yaw: float = 0.0                     # measured from the blue banner


class MissionContext:
    def __init__(self, cfg: MissionConfig = None):
        self.cfg = cfg or MissionConfig()
        self.state = MissionState()
        self.node = None
        self.vehicle: Optional[VehicleListener] = None
        self.countdown = 0.0
        self.abort_reason = ""  # set when a fallback "return home and land" branch runs

    def log(self):
        return self.node.get_logger()

    def z(self, height):
        """NED z for `height` m above the take-off pad."""
        return self.state.ground_z - height

    def pose(self):
        return self.vehicle.latest


class ContextSetup(py_trees.behaviour.Behaviour):
    def __init__(self, ctx: MissionContext, name="Load mission parameters"):
        super().__init__(name)
        self.ctx = ctx

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.ctx.node = node
        self.ctx.cfg.load(node)
        self.ctx.vehicle = VehicleListener(node)

    def update(self):
        return py_trees.common.Status.SUCCESS


class WaitForVehicle(py_trees.behaviour.Behaviour):
    def __init__(self, ctx, name="Wait for flight controller ready"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        v = self.ctx.vehicle
        if v is not None and v.fresh() and v.latest.ready:
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "waiting for /flight_controller/pose (valid + ready)"
        return py_trees.common.Status.RUNNING


class Compute(py_trees.behaviour.Behaviour):
    """fn(ctx) -> None/True = SUCCESS, False = FAILURE; exceptions are logged as FAILURE."""

    def __init__(self, name, ctx, fn):
        super().__init__(name)
        self.ctx, self.fn = ctx, fn

    def update(self):
        try:
            ok = self.fn(self.ctx)
        except Exception as e:  # noqa: BLE001 - a mission step must fail visibly, not crash the tree
            self.ctx.log().error(f"'{self.name}' failed: {e!r}")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE if ok is False else py_trees.common.Status.SUCCESS


class Check(py_trees.behaviour.Behaviour):
    def __init__(self, name, ctx, predicate):
        super().__init__(name)
        self.ctx, self.predicate = ctx, predicate

    def update(self):
        return py_trees.common.Status.SUCCESS if self.predicate(self.ctx) else py_trees.common.Status.FAILURE


class Hold(py_trees.behaviour.Behaviour):
    """Hover for `seconds` (a number or fn(ctx)). flight_controller keeps holding the last target."""

    def __init__(self, name, ctx, seconds, countdown=False):
        super().__init__(name)
        self.ctx, self.seconds, self.countdown = ctx, seconds, countdown
        self.end = 0.0

    def initialise(self):
        s = self.seconds(self.ctx) if callable(self.seconds) else self.seconds
        self.end = time.monotonic() + float(s)

    def update(self):
        left = self.end - time.monotonic()
        if self.countdown:
            self.ctx.countdown = max(0.0, left)
        if left > 0:
            self.feedback_message = f"{left:.1f} s left"
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self.countdown:
            self.ctx.countdown = 0.0


class SetActive(py_trees.behaviour.Behaviour):
    """Call a std_srvs/SetBool service (e.g. /red_zone_detector/set_active). SUCCESS when accepted."""

    def __init__(self, name, ctx, service, value, timeout=3.0):
        super().__init__(name)
        self.ctx, self.service, self.value, self.timeout = ctx, service, bool(value), timeout
        self.client = None
        self.future = None
        self.started = 0.0

    def setup(self, **kwargs):
        from std_srvs.srv import SetBool
        self.srv = SetBool
        self.client = kwargs["node"].create_client(SetBool, self.service)
        timeout = self.ctx.cfg.server_timeout
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {self.service} not available after {timeout:.0f} s")

    def initialise(self):
        self.future = self.client.call_async(self.srv.Request(data=self.value))
        self.started = time.monotonic()

    def update(self):
        S = py_trees.common.Status
        if not self.future.done():
            if time.monotonic() - self.started > self.timeout:
                self.ctx.log().error(f"'{self.name}': {self.service} did not answer")
                return S.FAILURE
            return S.RUNNING
        resp = self.future.result()
        if resp is None or not resp.success:
            self.ctx.log().error(f"'{self.name}': {self.service} refused set_active={self.value}")
            return S.FAILURE
        return S.SUCCESS


class ActionStep(py_trees.behaviour.Behaviour):
    """Send goal_fn(ctx) to an action server; on_result(ctx, result, ok) stores what is needed,
    on_feedback(ctx, feedback) sees every feedback message.

    SUCCESS when the goal succeeded and result.success is true. Interrupting the step cancels the goal."""

    def __init__(self, name, ctx, action_type, action_name, goal_fn, on_result=None, on_feedback=None):
        super().__init__(name)
        self.ctx = ctx
        self.action_type, self.action_name = action_type, action_name
        self.goal_fn, self.on_result, self.on_feedback = goal_fn, on_result, on_feedback
        self.client = None
        self.reset()

    def reset(self):
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.failed = False

    def setup(self, **kwargs):
        node = kwargs["node"]
        self.client = ActionClient(node, self.action_type, self.action_name)
        timeout = self.ctx.cfg.server_timeout
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"action server {self.action_name} not available after {timeout:.0f} s")

    def initialise(self):
        self.reset()
        try:
            goal = self.goal_fn(self.ctx)
        except Exception as e:  # noqa: BLE001
            self.ctx.log().error(f"'{self.name}': cannot build the goal: {e!r}")
            self.failed = True
            return
        feedback_cb = None
        if self.on_feedback is not None:
            feedback_cb = lambda msg: self.on_feedback(self.ctx, msg.feedback)  # noqa: E731
        self.send_future = self.client.send_goal_async(goal, feedback_callback=feedback_cb)

    def update(self):
        S = py_trees.common.Status
        if self.failed:
            return S.FAILURE
        if self.goal_handle is None:
            if not self.send_future.done():
                return S.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.ctx.log().error(f"'{self.name}': goal rejected by {self.action_name}")
                return S.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done():
            return S.RUNNING
        response = self.result_future.result()
        result = response.result
        ok = response.status == GoalStatus.STATUS_SUCCEEDED and getattr(result, "success", True)
        if self.on_result is not None:
            try:
                self.on_result(self.ctx, result, ok)
            except Exception as e:  # noqa: BLE001
                self.ctx.log().error(f"'{self.name}': result handling failed: {e!r}")
                return S.FAILURE
        if not ok:
            self.ctx.log().warn(f"'{self.name}' failed: {getattr(result, 'message', response.status)}")
        return S.SUCCESS if ok else S.FAILURE

    def terminate(self, new_status):
        if self.on_feedback is not None:
            self.ctx.countdown = 0.0
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None \
                and self.result_future is not None and not self.result_future.done():
            self.goal_handle.cancel_goal_async()


class ReferenceCapture(py_trees.behaviour.Behaviour):
    """Stage 2 without a timeout: the CaptureReference goal runs until qr_detector memorises a QR. Meanwhile
    the drone hovers over centre_fn(ctx) for cfg.ref_search_hover s, then flies growing squares round it
    (cfg.ref_search_step .. ref_search_max m, cfg.ref_search_dwell s at each corner, heading yaw_fn(ctx)),
    repeating until the QR is seen. SUCCESS when captured (the current GoTo is cancelled and the vehicle
    holds where it is); FAILURE only if qr_detector aborts. on_result(ctx, result, ok) as ActionStep."""

    REPORT_PERIOD = 10.0

    def __init__(self, name, ctx, centre_fn, yaw_fn, on_result=None):
        super().__init__(name)
        self.ctx, self.centre_fn, self.yaw_fn, self.on_result = ctx, centre_fn, yaw_fn, on_result
        self.capture_client = self.goto_client = None

    def setup(self, **kwargs):
        from mission_interfaces.action import CaptureReference, GoTo
        from bc_common.ros_utils import GOTO_ACTION
        self.CaptureReference, self.GoTo = CaptureReference, GoTo
        node = kwargs["node"]
        self.capture_client = ActionClient(node, CaptureReference, "/qr_detector/capture_reference")
        self.goto_client = ActionClient(node, GoTo, GOTO_ACTION)
        timeout = self.ctx.cfg.server_timeout
        for client, name in ((self.capture_client, "/qr_detector/capture_reference"), (self.goto_client, GOTO_ACTION)):
            if not client.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f"action server {name} not available after {timeout:.0f} s")

    def initialise(self):
        cfg = self.ctx.cfg
        self.start = time.monotonic()
        self.next_report = self.start + self.REPORT_PERIOD
        self.next_move = self.start + cfg.ref_search_hover
        self.offsets = mm.square_offsets(cfg.ref_search_step, cfg.ref_search_max)
        self.index = 0
        self.goto_send = self.goto_handle = self.goto_result = None
        self.capture_handle = self.capture_result = None
        self.stop_requested = None
        self.cancel_sent = False
        self.capture_send = self.capture_client.send_goal_async(self.CaptureReference.Goal())
        self.ctx.log().info(f"[STAGE 2] Scanning for the reference QR (no timeout): hovering "
                            f"{cfg.ref_search_hover:.0f} s, then squares up to {cfg.ref_search_max:.1f} m round it")

    def target(self, offset):
        n, e, d = self.centre_fn(self.ctx)
        yaw = self.yaw_fn(self.ctx)
        fwd, left = offset
        return (n + fwd * math.cos(yaw) + left * math.sin(yaw),
                e + fwd * math.sin(yaw) - left * math.cos(yaw), d)

    def update(self):
        S = py_trees.common.Status
        now = time.monotonic()
        # ---- the capture
        if self.capture_handle is None:
            if not self.capture_send.done():
                return S.RUNNING
            self.capture_handle = self.capture_send.result()
            if not self.capture_handle.accepted:
                self.ctx.log().error(f"'{self.name}': capture goal rejected by qr_detector")
                return S.FAILURE
            self.capture_result = self.capture_handle.get_result_async()
        if self.capture_result.done():
            # stop the search move and wait until flight_controller has really ended it, otherwise the next
            # step's GoTo is rejected ("goto is running")
            if not self.goto_stopped(now):
                return S.RUNNING
            response = self.capture_result.result()
            result = response.result
            ok = response.status == GoalStatus.STATUS_SUCCEEDED and result.success
            if self.on_result is not None:
                self.on_result(self.ctx, result, ok)
            if ok:
                self.ctx.log().info(f"[STAGE 2] Reference QR captured after {now - self.start:.1f} s")
            return S.SUCCESS if ok else S.FAILURE
        if now >= self.next_report:
            self.ctx.log().info(f"[STAGE 2] Still searching for the reference QR ({now - self.start:.0f} s)")
            self.next_report += self.REPORT_PERIOD
        # ---- the search pattern
        if self.goto_send is not None:
            if self.goto_handle is None:
                if not self.goto_send.done():
                    return S.RUNNING
                self.goto_handle = self.goto_send.result()
                if not self.goto_handle.accepted:      # e.g. flight_controller busy: retry after the dwell
                    self.goto_send = self.goto_handle = None
                    self.next_move = now + self.ctx.cfg.ref_search_dwell
                    return S.RUNNING
                self.goto_result = self.goto_handle.get_result_async()
            if not self.goto_result.done():
                return S.RUNNING
            self.goto_send = self.goto_handle = self.goto_result = None
            self.next_move = now + self.ctx.cfg.ref_search_dwell
        elif now >= self.next_move:
            offset = self.offsets[self.index % len(self.offsets)]
            self.index += 1
            n, e, d = self.target(offset)
            cfg = self.ctx.cfg
            self.ctx.log().info(f"[STAGE 2] reference search: {offset[0]:+.1f} m forward, {offset[1]:+.1f} m left "
                                f"-> N={n:+.2f} E={e:+.2f}")
            self.goto_send = self.goto_client.send_goal_async(self.GoTo.Goal(
                frame=self.GoTo.Goal.FRAME_LOCAL_NED, x=float(n), y=float(e), z=float(d),
                yaw=float(self.yaw_fn(self.ctx)), speed=float(cfg.ref_search_speed), acceptance_radius=0.0))
        return S.RUNNING

    def goto_stopped(self, now):
        """Cancel the search GoTo (once) and report True once it has finished (or after 5 s)."""
        if self.goto_send is None:
            return True
        if self.stop_requested is None:
            self.stop_requested = now
        if now - self.stop_requested > 5.0:
            return True
        if self.goto_handle is None:
            if not self.goto_send.done():
                return False
            self.goto_handle = self.goto_send.result()
            if not self.goto_handle.accepted:
                return True
            self.goto_result = self.goto_handle.get_result_async()
        if self.goto_result.done():
            return True
        if not self.cancel_sent:
            self.goto_handle.cancel_goal_async()
            self.cancel_sent = True
        return False

    def cancel_goto(self):
        if self.goto_handle is not None and self.goto_result is not None and not self.goto_result.done():
            self.goto_handle.cancel_goal_async()
        self.goto_send = self.goto_handle = self.goto_result = None

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.cancel_goto()
            if self.capture_handle is not None and self.capture_result is not None \
                    and not self.capture_result.done():
                self.capture_handle.cancel_goal_async()
