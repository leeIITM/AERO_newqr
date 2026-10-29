"""ArduPilot <-> ROS 2 helpers (through MAVROS) shared by every node that talks to the autopilot.

Replaces px4_utils.py. The nodes keep the conventions they had with PX4, so their navigation
logic is unchanged: local frame NED (x north, y east, z down, metres), heading in radians with
0 = north and clockwise positive, body velocities FLU. MAVROS publishes ENU / FLU, so every
conversion lives in this file and nowhere else.

  LocalPositionFeed   /mavros/local_position/pose + velocity + global fix  ->  LocalPose (NED)
  VehicleMonitor      /mavros/state + /mavros/extended_state  (armed, mode, connected, landed)
  CommandRunner       send MAVROS services from a timer callback without blocking the executor
  call_blocking       send a MAVROS service from an action thread
  make_setpoint       NED position / velocity setpoint -> mavros_msgs/PositionTarget
  make_waypoint       mavros_msgs/Waypoint item for a mission pushed with /mavros/mission/push
"""
import math
import time

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State, Waypoint
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

# ArduCopter flight mode names as MAVROS /mavros/set_mode wants them (custom_mode, base_mode = 0).
MODE_GUIDED = "GUIDED"
MODE_AUTO = "AUTO"
MODE_LAND = "LAND"
MODE_LOITER = "LOITER"

# MAVROS publishes most telemetry best-effort; a best-effort subscriber also connects to reliable
# publishers (/mavros/state is reliable + transient-local), so one profile fits every subscription.
MAVROS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# ArduPilot's lat/lon <-> metres factor (Location::LOCATION_SCALING_FACTOR): 1 degree of latitude.
METRES_PER_DEG = 111318.84502145034

# MAV_FRAME values for mission items
MAV_FRAME_GLOBAL_REL_ALT = 3
MAV_FRAME_MISSION = 2
# MAV_CMD values for mission items
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_CONDITION_YAW = 115
MAV_CMD_DO_CHANGE_SPEED = 178

# MAV_STATE (State.system_status)
MAV_STATE_STANDBY = 3
MAV_STATE_ACTIVE = 4

POSE_FRESH = 1.0  # s; a fix or velocity older than this is not merged into a LocalPose
ORIGIN_MAX_SPEED = 0.5  # m/s; the local-frame origin is estimated while (almost) standing still


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------- frame conversions


def heading_from_enu_quaternion(x, y, z, w):
    """Heading (rad, 0 = north, clockwise) from a body-FLU -> ENU quaternion (what MAVROS publishes)."""
    yaw_enu = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def enu_yaw_from_heading(heading):
    """ENU yaw (rad, 0 = east, counter-clockwise) for a heading (0 = north, clockwise)."""
    return wrap_pi(math.pi / 2.0 - heading)


def enu_flu_quaternion_to_rpy(x, y, z, w):
    """Aircraft roll, pitch, yaw (rad; NED earth, FRD body, yaw = heading) from MAVROS' ENU/FLU quaternion."""
    # Rotation matrix body FLU -> ENU
    r = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    # body FRD -> NED  =  T_ned_enu * R * T_flu_frd,  T_ned_enu swaps x/y and flips z, T_flu_frd = diag(1, -1, -1)
    n = (
        (r[1][0], -r[1][1], -r[1][2]),
        (r[0][0], -r[0][1], -r[0][2]),
        (-r[2][0], r[2][1], r[2][2]),
    )
    roll = math.atan2(n[2][1], n[2][2])
    pitch = -math.asin(max(-1.0, min(1.0, n[2][0])))
    yaw = math.atan2(n[1][0], n[0][0])
    return roll, pitch, yaw


def offset_latlon(lat, lon, north, east):
    """lat/lon (deg) displaced by north/east metres, with ArduPilot's own flat-earth scaling."""
    new_lat = lat + north / METRES_PER_DEG
    new_lon = lon + east / (METRES_PER_DEG * math.cos(math.radians(lat)))
    return new_lat, new_lon


def latlon_offset(lat, lon, ref_lat, ref_lon):
    """North/east metres of (lat, lon) from (ref_lat, ref_lon); inverse of offset_latlon."""
    north = (lat - ref_lat) * METRES_PER_DEG
    east = (lon - ref_lon) * METRES_PER_DEG * math.cos(math.radians(ref_lat))
    return north, east


# ---------------------------------------------------------------- local position


class LocalPose:
    """Drop-in for PX4's VehicleLocalPosition: same field names, NED, heading 0 = north clockwise."""

    def __init__(self):
        nan = float("nan")
        self.x = self.y = self.z = 0.0  # north, east, down (m) from the EKF origin
        self.heading = 0.0  # rad
        self.vx = self.vy = self.vz = 0.0  # m/s NED
        self.xy_valid = False
        self.z_valid = False
        self.heading_good_for_control = False
        self.xy_global = False  # ref_lat / ref_lon are known
        self.ref_lat = self.ref_lon = nan  # deg, geographic position of the local origin
        self.global_valid = False  # lat / lon below are a valid GPS fix
        self.lat = self.lon = nan  # deg
        self.alt = nan  # m, as reported by /mavros/global_position/global (WGS-84 ellipsoid)
        self.rel_alt = nan  # m above home (ArduPilot's relative altitude)
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w quaternion, body FLU -> ENU
        self.stamp = 0.0  # time.monotonic() when received


class LocalPositionFeed:
    """Subscribes to MAVROS local position, velocity, global fix and EKF origin, and calls
    `callback(LocalPose)` for every pose message. `self.latest` is the newest LocalPose (or None)."""

    def __init__(self, node, callback, callback_group=None):
        self.callback = callback
        self.latest = None
        self._velocity = (0.0, 0.0, 0.0)
        self._velocity_time = 0.0
        self._fix = None
        self._fix_time = 0.0
        self._rel_alt = float("nan")
        self._origin = None  # (lat, lon) from MAVROS gp_origin
        self._estimated_origin = None  # (lat, lon) worked out from the fix and the local position

        def subscribe(msg_type, topic, handler):
            node.create_subscription(msg_type, topic, handler, MAVROS_QOS, callback_group=callback_group)

        subscribe(PoseStamped, "/mavros/local_position/pose", self._on_pose)
        subscribe(TwistStamped, "/mavros/local_position/velocity_local", self._on_velocity)
        subscribe(NavSatFix, "/mavros/global_position/global", self._on_fix)
        subscribe(Float64, "/mavros/global_position/rel_alt", self._on_rel_alt)
        subscribe(GeoPointStamped, "/mavros/global_position/gp_origin", self._on_origin)

    def _on_velocity(self, msg):
        v = msg.twist.linear  # ENU -> NED
        self._velocity = (v.y, v.x, -v.z)
        self._velocity_time = time.monotonic()

    def _on_fix(self, msg):
        self._fix = msg
        self._fix_time = time.monotonic()

    def _on_rel_alt(self, msg):
        self._rel_alt = float(msg.data)

    def _on_origin(self, msg):
        self._origin = (msg.position.latitude, msg.position.longitude)

    def _on_pose(self, msg):
        now = time.monotonic()
        p, q = msg.pose.position, msg.pose.orientation
        pose = LocalPose()
        pose.x, pose.y, pose.z = p.y, p.x, -p.z  # ENU -> NED
        pose.heading = heading_from_enu_quaternion(q.x, q.y, q.z, q.w)
        pose.orientation = (q.x, q.y, q.z, q.w)
        pose.xy_valid = pose.z_valid = pose.heading_good_for_control = all(
            math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.heading)
        )
        if now - self._velocity_time < POSE_FRESH:
            pose.vx, pose.vy, pose.vz = self._velocity
        fix = self._fix
        if fix is not None and now - self._fix_time < POSE_FRESH and fix.status.status >= NavSatStatus.STATUS_FIX:
            pose.global_valid = math.isfinite(fix.latitude) and math.isfinite(fix.longitude)
            pose.lat, pose.lon, pose.alt = fix.latitude, fix.longitude, fix.altitude
        pose.rel_alt = self._rel_alt
        origin = self._origin
        if origin is None:
            if (self._estimated_origin is None and pose.global_valid
                    and math.hypot(pose.vx, pose.vy) < ORIGIN_MAX_SPEED):
                # No gp_origin from the autopilot: back the origin out of the GPS fix and the local
                # position while standing still (on the ground), and keep it.
                ref_lat = pose.lat - pose.x / METRES_PER_DEG
                ref_lon = pose.lon - pose.y / (METRES_PER_DEG * math.cos(math.radians(ref_lat)))
                self._estimated_origin = (ref_lat, ref_lon)
            origin = self._estimated_origin
        if origin is not None:
            pose.xy_global = True
            pose.ref_lat, pose.ref_lon = origin
        pose.stamp = now
        self.latest = pose
        self.callback(pose)


# ---------------------------------------------------------------- vehicle state


class VehicleMonitor:
    """/mavros/state and /mavros/extended_state."""

    def __init__(self, node, callback_group=None, on_state=None):
        self.state = None  # latest mavros_msgs/State
        self.extended = None  # latest mavros_msgs/ExtendedState
        self._on_state = on_state
        node.create_subscription(State, "/mavros/state", self._state_cb, MAVROS_QOS, callback_group=callback_group)
        node.create_subscription(
            ExtendedState, "/mavros/extended_state", self._extended_cb, MAVROS_QOS, callback_group=callback_group
        )

    def _state_cb(self, msg):
        self.state = msg
        if self._on_state is not None:
            self._on_state(msg)

    def _extended_cb(self, msg):
        self.extended = msg

    def connected(self):
        return self.state is not None and self.state.connected

    def armed(self):
        return self.state is not None and self.state.armed

    def mode(self):
        return self.state.mode if self.state is not None else ""

    def ready(self):
        """Autopilot booted and not in a critical state (it can still refuse to arm)."""
        return self.connected() and self.state.system_status in (MAV_STATE_STANDBY, MAV_STATE_ACTIVE)

    def on_ground(self):
        """True if the autopilot says it is landed (EXTENDED_SYS_STATE), False otherwise or if unknown."""
        return self.extended is not None and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def in_air(self):
        return self.extended is not None and self.extended.landed_state in (
            ExtendedState.LANDED_STATE_IN_AIR, ExtendedState.LANDED_STATE_TAKEOFF, ExtendedState.LANDED_STATE_LANDING
        )


def gps_ok(fix):
    """A /mavros/global_position/global NavSatFix that has a position fix."""
    return fix is not None and fix.status.status >= NavSatStatus.STATUS_FIX and math.isfinite(fix.latitude)


# ---------------------------------------------------------------- services


def call_blocking(client, request, timeout=5.0):
    """Call a service from an action thread and wait for the answer. Returns the response or None.

    Never call it from a timer or subscription callback of a single-threaded executor."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.01)
    try:
        return future.result()
    except Exception:  # noqa: BLE001 - the caller treats "no answer" as a failure
        return None


class CommandRunner:
    """Sends one MAVROS service request at a time from a timer callback and looks at the answer on
    later ticks (never blocks). Used by the state-machine nodes (takeoff, move_forward).

        status = runner.run("arm", arm_client, lambda: CommandBool.Request(value=True), "success")

    returns PENDING until the autopilot accepted the request, then OK (and keeps returning OK for
    the same key until reset()). A refused or unanswered request is sent again after `retry_after`
    seconds; the caller's step timeout ends the retries.
    """

    PENDING = "pending"
    OK = "ok"

    def __init__(self, node, retry_after=1.0):
        self.node = node
        self.retry_after = retry_after
        self.reset()

    def reset(self):
        self.key = None
        self.future = None
        self.sent_at = 0.0
        self.accepted = False
        self.last_refusal = ""

    def run(self, key, client, make_request, ok_field):
        if key != self.key:
            self.reset()
            self.key = key
        if self.accepted:
            return self.OK
        now = time.monotonic()
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
            except Exception as exc:  # noqa: BLE001
                response = None
                self.last_refusal = f"service call failed: {exc}"
            if response is not None:
                if getattr(response, ok_field):
                    self.accepted = True
                    return self.OK
                self.last_refusal = f"{key} refused by the autopilot ({response})"
            self.future = None
        if self.future is None and (now - self.sent_at >= self.retry_after):
            if client.service_is_ready():
                self.future = client.call_async(make_request())
                self.sent_at = now
        elif self.future is not None and now - self.sent_at > 3.0 * self.retry_after:
            self.future = None  # no answer: send again
        return self.PENDING


# ---------------------------------------------------------------- setpoints and mission items


def make_setpoint(stamp, mode, position=None, heading=float("nan"), velocity=None, yaw_rate=float("nan")):
    """mavros_msgs/PositionTarget for /mavros/setpoint_raw/local from NED values.

    mode "position": position (north, east, down) and `heading` (rad, 0 = north, clockwise; NaN = leave yaw alone)
    mode "velocity": velocity (north, east, down m/s) and `yaw_rate` (rad/s, clockwise positive; NaN = none)

    MAVROS takes the values of a local-frame setpoint in ENU and converts them to NED itself, so this
    is the one place that does the NED -> ENU swap.
    """
    msg = PositionTarget()
    msg.header.stamp = stamp
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    if mode == "position":
        mask |= (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
                 | PositionTarget.IGNORE_YAW_RATE)
        north, east, down = position
        msg.position.x, msg.position.y, msg.position.z = float(east), float(north), float(-down)
        if math.isfinite(heading):
            msg.yaw = float(enu_yaw_from_heading(heading))
        else:
            mask |= PositionTarget.IGNORE_YAW
    elif mode == "velocity":
        mask |= (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                 | PositionTarget.IGNORE_YAW)
        north, east, down = velocity
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(east), float(north), float(-down)
        if math.isfinite(yaw_rate):
            msg.yaw_rate = float(-yaw_rate)  # NED clockwise -> ENU counter-clockwise
        else:
            mask |= PositionTarget.IGNORE_YAW_RATE
    else:
        raise ValueError(f"unknown setpoint mode {mode!r}")
    msg.type_mask = mask
    return msg


def make_waypoint(command, frame=MAV_FRAME_GLOBAL_REL_ALT, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                  lat=0.0, lon=0.0, alt=0.0, current=False):
    """One mission item, built like suas_ardupilot's create_waypoint()."""
    wp = Waypoint()
    wp.frame = frame
    wp.command = command
    wp.is_current = current
    wp.autocontinue = True
    wp.param1, wp.param2, wp.param3, wp.param4 = float(p1), float(p2), float(p3), float(p4)
    wp.x_lat, wp.y_long, wp.z_alt = float(lat), float(lon), float(alt)
    return wp
