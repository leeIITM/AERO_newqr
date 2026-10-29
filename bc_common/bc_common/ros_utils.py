"""ROS plumbing shared by the behaviour nodes.

  VehicleListener   latest /flight_controller/pose (VehicleState)
  MessageCache      latest message of a topic + "wait for a message newer than X"
  FlightClient      blocking GoTo client for action threads (cancel on demand)
  serve_action      ActionServer that accepts one goal at a time and accepts cancels
  spin              run a node on a MultiThreadedExecutor
"""
import math
import threading
import time
from collections import deque

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from mission_interfaces.action import GoTo
from mission_interfaces.msg import VehicleState
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

# ------------------------------------------------------------------ topic names

POSE_TOPIC = "/flight_controller/pose"
GOTO_ACTION = "/flight_controller/goto"
TAKEOFF_ACTION = "/flight_controller/takeoff"
LAND_ACTION = "/flight_controller/land"
IMAGE_TOPIC = "/camera/image_raw"
BANNER_TOPIC = "/perception/banner"
QR_TOPIC = "/perception/qr"
BANNER_COLOR_SERVICE = "/banner_detector/set_color"
QR_ACTIVE_SERVICE = "/qr_detector/set_active"
RED_ACTIVE_SERVICE = "/red_zone_detector/set_active"
MISSION_STATUS_TOPIC = "/mission/status"
MOTOR_TOPIC = "/flight_controller/motors"
REFERENCE_IMAGE_TOPIC = "/qr_detector/reference_image"

IMAGE_QOS = qos_profile_sensor_data
DETECTION_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=10)

STALE_POSE = 0.5  # s
POSE_WAIT_REPORT = 5.0  # s between "waiting for the vehicle pose" log lines


def nan_point():
    return Point(x=math.nan, y=math.nan, z=math.nan)


def to_point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def from_point(p):
    return [float(p.x), float(p.y), float(p.z)]


class VehicleListener:
    def __init__(self, node, callback_group=None):
        self.latest = None
        self.stamp = 0.0
        node.create_subscription(VehicleState, POSE_TOPIC, self._cb, DETECTION_QOS,
                                 callback_group=callback_group)

    def _cb(self, msg):
        self.latest = msg
        self.stamp = time.monotonic()

    def fresh(self, max_age=STALE_POSE):
        return self.latest is not None and self.latest.valid and time.monotonic() - self.stamp < max_age

    def why_stale(self):
        if self.latest is None:
            return "none received yet"
        if not self.latest.valid:
            return "pose not valid (no EKF position from ArduPilot)"
        return f"last one {time.monotonic() - self.stamp:.1f} s old"

    def wait_fresh(self, logger=None, what="", cancel=None, report=POSE_WAIT_REPORT, max_age=STALE_POSE):
        """Block until a fresh pose arrives: no timeout, a warning every `report` s. Returns True, or False
        if cancel() became true. Only from an action thread of a MultiThreadedExecutor (the subscription
        must keep running): a gap in MAVROS data on a busy computer then delays a step instead of failing it."""
        if self.fresh(max_age):
            return True
        start = time.monotonic()
        next_report = start + report
        while not self.fresh(max_age):
            if cancel is not None and cancel():
                return False
            now = time.monotonic()
            if logger is not None and now >= next_report:
                logger.warn(f"{what}: waiting for the vehicle pose ({POSE_TOPIC}) for {now - start:.0f} s: "
                            f"{self.why_stale()}")
                next_report += report
            time.sleep(0.05)
        waited = time.monotonic() - start
        if logger is not None and waited > 1.0:
            logger.info(f"{what}: vehicle pose back after {waited:.1f} s")
        return True

    def ned(self):
        p = self.latest
        return [p.x, p.y, p.z]

    def alt_rel(self):
        p = self.latest
        if p is None:
            return math.nan
        return p.alt_rel if math.isfinite(p.alt_rel) else -p.z


class MessageCache:
    """Recent messages of a topic, numbered. `mark()` returns the current number; `find_since(mark,
    pred)` / `wait_for(mark, pred, timeout)` return the newest message after the mark that satisfies
    `pred`, so a thread polling slower than the topic rate never misses a matching frame."""

    def __init__(self, node, msg_type, topic, qos=DETECTION_QOS, callback_group=None, history=60):
        self.cond = threading.Condition()
        self.history = deque(maxlen=history)  # (number, msg)
        self.count = 0
        node.create_subscription(msg_type, topic, self._cb, qos, callback_group=callback_group)

    def _cb(self, msg):
        with self.cond:
            self.count += 1
            self.history.append((self.count, msg))
            self.cond.notify_all()

    @property
    def msg(self):
        with self.cond:
            return self.history[-1][1] if self.history else None

    def mark(self):
        with self.cond:
            return self.count

    def find_since(self, mark, pred=None):
        with self.cond:
            for number, msg in reversed(self.history):
                if number <= mark:
                    break
                if pred is None or pred(msg):
                    return msg
        return None

    def messages_since(self, mark):
        with self.cond:
            return [m for n, m in self.history if n > mark]

    def wait_for(self, mark, pred=None, timeout=1.0, cancel=None):
        """Newest 11message after `mark` with pred(msg) true, waiting up to `timeout` s; else None."""
        deadline = time.monotonic() + timeout
        while True:
            msg = self.find_since(mark, pred)
            if msg is not None:
                return msg
            left = deadline - time.monotonic()
            if left <= 0 or (cancel is not None and cancel()):
                return None
            with self.cond:
                self.cond.wait(min(left, 0.05))


class GotoOutcome:
    REACHED = "reached"
    STOPPED = "stopped"      # stop_when() became true and the GoTo was cancelled (vehicle holds)
    CANCELED = "canceled"    # the calling goal was cancelled
    FAILED = "failed"

    def __init__(self, status, message="", result=None):
        self.status = status
        self.message = message
        self.result = result

    @property
    def ok(self):
        return self.status in (self.REACHED, self.STOPPED)

    def __repr__(self):
        return f"GotoOutcome({self.status}, {self.message!r})"


class FlightClient:
    """GoTo client for use inside an action execute thread (MultiThreadedExecutor)."""

    POLL = 0.02

    def __init__(self, node, callback_group=None):
        self.node = node
        self.client = ActionClient(node, GoTo, GOTO_ACTION, callback_group=callback_group)

    def wait_for_server(self, timeout=10.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def goto(self, n, e, d, yaw=math.nan, speed=0.0, tolerance=0.0, stop_when=None, cancel=None, on_tick=None):
        """Fly to local NED (n, e, d) holding `yaw` (NaN components keep the current value)."""
        goal = GoTo.Goal(frame=GoTo.Goal.FRAME_LOCAL_NED, x=float(n), y=float(e), z=float(d),
                         yaw=float(yaw), speed=float(speed), acceptance_radius=float(tolerance))
        if not self.client.server_is_ready() and not self.wait_for_server(5.0):
            return GotoOutcome(GotoOutcome.FAILED, "flight_controller GoTo server not available")
        send = self.client.send_goal_async(goal)
        if not self._wait(send, 5.0):
            return GotoOutcome(GotoOutcome.FAILED, "GoTo goal not answered")
        handle = send.result()
        if not handle.accepted:
            return GotoOutcome(GotoOutcome.FAILED, "GoTo goal rejected (another goal running?)")
        result_future = handle.get_result_async()
        reason = None
        while not result_future.done():
            if on_tick is not None:
                on_tick()
            if cancel is not None and cancel():
                reason = GotoOutcome.CANCELED
            elif stop_when is not None and stop_when():
                reason = GotoOutcome.STOPPED
            if reason is not None:
                handle.cancel_goal_async()
                self._wait(result_future, 5.0)
                return GotoOutcome(reason, reason)
            time.sleep(self.POLL)
        res = result_future.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success:
            return GotoOutcome(GotoOutcome.REACHED, "ok", res.result)
        return GotoOutcome(GotoOutcome.FAILED, res.result.message or f"status {res.status}", res.result)

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(self.POLL)
        return True


def sleep_checked(seconds, cancel=None, on_tick=None, period=0.05):
    """Sleep `seconds`; returns False early if cancel() becomes true."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cancel is not None and cancel():
            return False
        if on_tick is not None:
            on_tick()
        time.sleep(min(period, max(0.0, end - time.monotonic())))
    return True


def serve_action(node, action_type, name, execute, callback_group):
    """ActionServer accepting one goal at a time; cancels are always accepted."""
    busy = threading.Lock()

    def goal_cb(_goal):
        if busy.locked():
            node.get_logger().warn(f"Rejecting {name}: a goal is already running.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def wrapped(goal_handle):
        with busy:
            return execute(goal_handle)

    return ActionServer(node, action_type, name, execute_callback=wrapped, goal_callback=goal_cb,
                        cancel_callback=lambda _h: CancelResponse.ACCEPT, callback_group=callback_group)


def finish(goal_handle, result, ok, message, logger=None):
    """Complete an action goal: succeed / canceled / abort, with result.success + message filled."""
    result.success = bool(ok)
    result.message = message
    if ok:
        goal_handle.succeed()
    elif goal_handle.is_cancel_requested:
        goal_handle.canceled()
    else:
        goal_handle.abort()
        if logger is not None:
            logger.warn(message)
    return result


def spin(node_factory, args=None):
    import rclpy
    rclpy.init(args=args)
    node = node_factory()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "shutdown"):
            node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def call_service(client, request, timeout=3.0):
    """Blocking service call from an action thread. Returns the response or None."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.01)
    return future.result()


def set_banner_color(node, client, color):
    """Ask banner_detector to classify `color` ("" = idle). Returns True if accepted."""
    from mission_interfaces.srv import SetBannerColor
    response = call_service(client, SetBannerColor.Request(color=color))
    if response is None or not response.success:
        node.get_logger().error(f"banner_detector did not accept colour {color!r}")
        return False
    return True


def set_qr_active(node, client, active):
    """Switch qr_detector's per-frame QR search on/off (std_srvs/SetBool). Returns True if accepted."""
    from std_srvs.srv import SetBool
    response = call_service(client, SetBool.Request(data=bool(active)))
    if response is None or not response.success:
        node.get_logger().error(f"qr_detector did not accept set_active={active}")
        return False
    return True
