"""Bench stand-in for flight_controller (the original script's --bench-sim / --spin-motors).

Runs under the SAME node name (flight_controller), so it serves the same actions and topic and no
other node can tell the difference:
    /flight_controller/takeoff  /flight_controller/goto  /flight_controller/land  (mission_interfaces)
    /flight_controller/pose     mission_interfaces/VehicleState (simulated kinematics)
    /flight_controller/motors   mission_interfaces/MotorStatus  (spin_motors only)
    /flight_controller/cmd_vel_body  geometry_msgs/Twist (body FLU velocity, while no goal runs)
    /flight_controller/setpoint      mission_interfaces/PositionSetpoint (streamed position, while no goal runs)

The vehicle is simulated: it chases the position setpoint with limited speed/acceleration, like the
script's bench kinematics. GoTo moves the setpoint along the line at the requested speed (same as the
real controller's GUIDED GoTo) and arrives when within the acceptance radius and settled.

spin_motors:=true  (PROPELLERS MUST BE OFF)  real Cube Orange+ through MAVROS, as the script's --spin-motors:
    DISARM_DELAY 0, GUIDED_NOGPS, arm, then /mavros/setpoint_raw/attitude at 25 Hz with the bench
    thrust and a roll/pitch tilt following the simulated motion (max +/-6 deg). Re-arms if the autopilot
    disarms during the mission. Landing ramps the thrust to 0 over 2 s, disarms, restores
    DISARM_DELAY 10 and switches to STABILIZE. Servo outputs from /mavros/rc/out go to ~/motors.

Parameters: spin_motors (false), thrust (0.20), sim_max_speed (2.0 m/s), sim_max_climb (0.8 m/s),
sim_max_accel (1.5 m/s^2), yaw_rate_deg (90), initial_heading_deg (0), origin_lat / origin_lon (the
simulated lat/lon of the local origin, for geofence files), default_speed (1.0),
default_acceptance_radius (0.3), land_speed (0.5), rate_hz (50)
"""
import math
import threading
import time

from geometry_msgs.msg import Point, Twist
from mission_interfaces.action import GoTo, Land, Survey, Takeoff
from mission_interfaces.msg import MotorStatus, PositionSetpoint, VehicleState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bc_common.geometry import euler_to_quaternion, wrap_pi
from bc_common.ros_utils import spin
from bench_flight_controller.sim_vehicle import SimVehicle

SETTLED_SPEED = 0.3
GOTO_EXTRA_TIME = 30.0
ALTITUDE_TOLERANCE = 0.5
CMD_VEL_TIMEOUT = 0.5


class BenchFlightController(Node):
    def __init__(self):
        super().__init__("flight_controller")
        p = self.declare_parameter
        self.spin_motors = bool(p("spin_motors", False).value)
        self.thrust_setting = float(p("thrust", 0.20).value)
        self.default_speed = float(p("default_speed", 1.0).value)
        self.default_radius = float(p("default_acceptance_radius", 0.3).value)
        self.land_speed = float(p("land_speed", 0.5).value)
        rate = float(p("rate_hz", 50.0).value)
        self.sim = SimVehicle(float(p("sim_max_speed", 2.0).value), float(p("sim_max_climb", 0.8).value),
                              float(p("sim_max_accel", 1.5).value), math.radians(float(p("yaw_rate_deg", 90.0).value)),
                              math.radians(float(p("initial_heading_deg", 0.0).value)))
        self.lock = threading.Lock()
        self.armed = False
        self.active_goal = None
        self.cmd_vel_stamp = 0.0
        self.setpoint_stamp = 0.0
        self.streaming = False
        self.origin = (float(p("origin_lat", 13.0827).value), float(p("origin_lon", 80.2707).value))
        self.thrust = 0.0
        self.cmd_roll = self.cmd_pitch = 0.0
        self.mode = "GUIDED_NOGPS" if self.spin_motors else "BENCH_SIM"

        group = ReentrantCallbackGroup()
        self.pose_pub = self.create_publisher(VehicleState, "~/pose", 10)
        self.create_subscription(Twist, "~/cmd_vel_body", self.cmd_vel_cb, 10, callback_group=group)
        self.create_subscription(PositionSetpoint, "~/setpoint", self.setpoint_cb, 10, callback_group=group)
        for action_type, name, execute in ((Takeoff, "~/takeoff", self.execute_takeoff),
                                           (GoTo, "~/goto", self.execute_goto),
                                           (Land, "~/land", self.execute_land),
                                           (Survey, "~/survey", self.execute_survey)):
            ActionServer(self, action_type, name, execute_callback=execute,
                         goal_callback=lambda g, n=name: self.on_goal(n),
                         cancel_callback=lambda h, n=name: CancelResponse.REJECT if n == "~/land"
                         else CancelResponse.ACCEPT, callback_group=group)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.physics, callback_group=group)

        self.mavros = None
        if self.spin_motors:
            self.mavros = MavrosBench(self, group)
            self.get_logger().warn("BENCH ROTOR ACTIVATION ENABLED: real motors at "
                                   f"{self.thrust_setting * 100:.0f}% thrust. PROPELLERS MUST BE OFF!")
        self.get_logger().info(f"Bench flight controller ready ({'spin motors' if self.spin_motors else 'simulation only'}).")

    # ---------------------------------------------------------------- simulation

    def physics(self):
        with self.lock:
            if self.streaming and time.monotonic() - self.setpoint_stamp > CMD_VEL_TIMEOUT:
                self.streaming = False
                if self.active_goal is None:
                    self.sim.sp = list(self.sim.pos)
            if self.sim.sp_vel is not None and time.monotonic() - self.cmd_vel_stamp > CMD_VEL_TIMEOUT:
                self.sim.sp_vel = None
                self.sim.sp = list(self.sim.pos)
            self.sim.step(self.dt)
            # bench tilt follows the simulated motion in the body frame (max +/- 6 deg)
            h = self.sim.heading
            v_fwd = self.sim.vel[0] * math.cos(h) + self.sim.vel[1] * math.sin(h)
            v_right = -self.sim.vel[0] * math.sin(h) + self.sim.vel[1] * math.cos(h)
            scale = 6.0 / max(0.1, self.sim.max_speed)
            self.cmd_pitch = max(-6.0, min(6.0, -v_fwd * scale))
            self.cmd_roll = max(-6.0, min(6.0, v_right * scale))
            pos, vel, heading = list(self.sim.pos), list(self.sim.vel), self.sim.heading
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.valid = True
        msg.ready = self.mavros.ready() if self.mavros else True
        msg.armed = self.armed
        msg.airborne = self.armed and -pos[2] > 0.3
        msg.mode = self.mode
        msg.controller = "bench"
        msg.x, msg.y, msg.z = map(float, pos)
        msg.heading = float(heading)
        msg.vx, msg.vy, msg.vz = map(float, vel)
        msg.alt_rel = float(-pos[2])
        msg.roll, msg.pitch = math.radians(self.cmd_roll), math.radians(self.cmd_pitch)
        msg.xy_global = True
        msg.ref_lat, msg.ref_lon = self.origin
        self.pose_pub.publish(msg)
        if self.mavros:
            self.mavros.tick(msg)

    def setpoint_cb(self, msg):
        with self.lock:
            if not self.armed or self.active_goal is not None:
                return
            self.sim.sp_vel = None
            self.sim.sp = [float(msg.x), float(msg.y), float(msg.z)]
            if math.isfinite(msg.yaw):
                self.sim.sp_yaw = float(msg.yaw)
            self.setpoint_stamp = time.monotonic()
            self.streaming = True

    def cmd_vel_cb(self, msg):
        with self.lock:
            if not self.armed or self.active_goal is not None:
                return
            h = self.sim.heading
            fwd, left = msg.linear.x, msg.linear.y
            self.sim.sp_vel = [fwd * math.cos(h) + left * math.sin(h), fwd * math.sin(h) - left * math.cos(h),
                               -msg.linear.z]
            self.cmd_vel_stamp = time.monotonic()

    def hold_here(self):
        with self.lock:
            self.sim.sp_vel = None
            self.sim.sp = list(self.sim.pos)

    # ---------------------------------------------------------------- actions

    def on_goal(self, name):
        with self.lock:
            if self.active_goal is not None:
                self.get_logger().warn(f"Rejecting {name}: {self.active_goal} is running.")
                return GoalResponse.REJECT
            self.active_goal = name
            self.streaming = False
        return GoalResponse.ACCEPT

    def finish(self, goal_handle, result, error=None):
        result.success = error is None
        result.message = error or "ok"
        if error is None:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
            self.get_logger().error(f"{self.active_goal} failed: {error}")
        with self.lock:
            self.active_goal = None
        return result

    def wait_until(self, cond, timeout, goal_handle, on_tick=None):
        deadline = time.monotonic() + timeout
        while not cond():
            if goal_handle.is_cancel_requested:
                return "canceled"
            if time.monotonic() > deadline:
                return "timeout"
            if on_tick:
                on_tick()
            time.sleep(0.02)
        return None

    def execute_takeoff(self, goal_handle):
        result = Takeoff.Result()
        alt = goal_handle.request.altitude
        if alt <= ALTITUDE_TOLERANCE:
            return self.finish(goal_handle, result, f"altitude must be > {ALTITUDE_TOLERANCE} m")
        if self.armed and -self.sim.pos[2] > 0.3:
            return self.finish(goal_handle, result, "already airborne")
        self.get_logger().info(f"[BENCH] Takeoff to {alt:.1f} m")
        if self.mavros:
            error = self.mavros.start()
            if error:
                return self.finish(goal_handle, result, error)
        target_z = self.sim.pos[2] - alt
        with self.lock:
            self.armed = True
            self.thrust = self.thrust_setting
            self.sim.sp = [self.sim.pos[0], self.sim.pos[1], target_z]
        feedback = Takeoff.Feedback()

        def tick():
            feedback.altitude_remaining = float(max(0.0, self.sim.pos[2] - target_z))
            goal_handle.publish_feedback(feedback)

        error = self.wait_until(lambda: self.sim.pos[2] <= target_z + ALTITUDE_TOLERANCE, 60.0, goal_handle, tick)
        self.hold_here()
        if error:
            return self.finish(goal_handle, result, f"climb: {error}")
        self.get_logger().info(f"[BENCH-SIM] Target altitude reached: {-self.sim.pos[2]:.1f} m")
        return self.finish(goal_handle, result)

    def execute_goto(self, goal_handle):
        req = goal_handle.request
        result = GoTo.Result()
        if not self.armed:
            return self.finish(goal_handle, result, "not armed (run Takeoff first)")
        start = list(self.sim.pos)
        heading = self.sim.heading
        if req.frame == GoTo.Goal.FRAME_LOCAL_NED:
            target = [t if math.isfinite(t) else s for t, s in zip((req.x, req.y, req.z), start)]
            yaw = req.yaw if math.isfinite(req.yaw) else heading
        elif req.frame == GoTo.Goal.FRAME_BODY_FLU:
            target = [start[0] + req.x * math.cos(heading) + req.y * math.sin(heading),
                      start[1] + req.x * math.sin(heading) - req.y * math.cos(heading), start[2] - req.z]
            yaw = wrap_pi(heading - req.yaw) if math.isfinite(req.yaw) else heading
        else:
            return self.finish(goal_handle, result, f"unknown frame {req.frame}")
        speed = req.speed if req.speed > 0 else self.default_speed
        radius = req.acceptance_radius if req.acceptance_radius > 0 else self.default_radius
        delta = [t - s for t, s in zip(target, start)]
        total = math.sqrt(sum(d * d for d in delta))
        with self.lock:
            self.sim.sp_vel = None
            self.sim.sp_yaw = yaw
        started = time.monotonic()
        feedback = GoTo.Feedback()

        def dist():
            return math.sqrt(sum((t - p) ** 2 for t, p in zip(target, self.sim.pos)))

        def advance():
            frac = min(1.0, speed * (time.monotonic() - started) / total) if total > 0 else 1.0
            with self.lock:
                self.sim.sp = [s + d * frac for s, d in zip(start, delta)]
            feedback.distance_remaining = float(dist())
            goal_handle.publish_feedback(feedback)

        def arrived():
            return (time.monotonic() - started >= total / speed and dist() < radius
                    and self.sim.speed() < SETTLED_SPEED)

        error = self.wait_until(arrived, total / speed + GOTO_EXTRA_TIME, goal_handle, advance)
        if error:
            self.hold_here()
            return self.finish(goal_handle, result, error)
        result.final_position = Point(x=float(self.sim.pos[0]), y=float(self.sim.pos[1]), z=float(self.sim.pos[2]))
        return self.finish(goal_handle, result)

    def execute_survey(self, goal_handle):
        """Simulated AUTO mission: the setpoint walks the waypoint polyline at `speed`; cancel = hold here."""
        req = goal_handle.request
        result = Survey.Result()
        if not self.armed:
            return self.finish(goal_handle, result, "not armed (run Takeoff first)")
        north, east = list(req.north), list(req.east)
        if not north or len(north) != len(east):
            return self.finish(goal_handle, result, "survey needs the same number (> 0) of north and east values")
        down = float(req.down) if math.isfinite(req.down) else self.sim.pos[2]
        pts = [list(self.sim.pos)] + [[float(n), float(e), down] for n, e in zip(north, east)]
        legs = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
        total = sum(legs)
        speed = req.speed if req.speed > 0 else self.default_speed
        radius = req.acceptance_radius if req.acceptance_radius > 0 else self.default_radius
        self.get_logger().info(f"[BENCH] Survey: {len(north)} waypoints, {total:.1f} m at {speed:.1f} m/s (AUTO sim)")
        with self.lock:
            self.sim.sp_vel = None
            if math.isfinite(req.yaw):
                self.sim.sp_yaw = float(req.yaw)
        started = time.monotonic()
        feedback = Survey.Feedback()
        state = {"index": 0}

        def advance():
            s = min(total, speed * (time.monotonic() - started))
            k, acc = 0, 0.0
            while k < len(legs) - 1 and acc + legs[k] < s:
                acc += legs[k]
                k += 1
            frac = (s - acc) / legs[k] if legs[k] > 0 else 1.0
            a, b = pts[k], pts[k + 1]
            with self.lock:
                self.sim.sp = [x + (y - x) * min(1.0, frac) for x, y in zip(a, b)]
            state["index"] = k
            feedback.current_index = int(k)
            feedback.distance_remaining = float(math.dist(self.sim.pos, b))
            goal_handle.publish_feedback(feedback)

        def arrived():
            return (time.monotonic() - started >= total / speed and math.dist(self.sim.pos, pts[-1]) < radius
                    and self.sim.speed() < SETTLED_SPEED)

        error = self.wait_until(arrived, total / speed + GOTO_EXTRA_TIME, goal_handle, advance)
        result.final_position = Point(x=float(self.sim.pos[0]), y=float(self.sim.pos[1]), z=float(self.sim.pos[2]))
        if error:
            self.hold_here()
            result.reached = int(state["index"])
            return self.finish(goal_handle, result, error)
        result.reached = len(north)
        return self.finish(goal_handle, result)

    def execute_land(self, goal_handle):
        result = Land.Result()
        if not self.armed:
            return self.finish(goal_handle, result)
        self.get_logger().info("[BENCH] Landing...")
        feedback = Land.Feedback()

        def descend():
            with self.lock:
                self.sim.sp = [self.sim.pos[0], self.sim.pos[1], min(0.5, self.sim.pos[2] + self.land_speed)]
            feedback.altitude = float(-self.sim.pos[2])
            goal_handle.publish_feedback(feedback)

        error = self.wait_until(lambda: self.sim.pos[2] >= -0.02 and self.sim.speed() < 0.05, 120.0,
                                goal_handle, descend)
        if error:
            self.hold_here()
            return self.finish(goal_handle, result, error)
        if self.mavros:
            self.mavros.stop()
        with self.lock:
            self.sim.sp = None
            self.armed = False
            self.thrust = 0.0
        self.get_logger().info("[COMPLETE] Landed and disarmed.")
        return self.finish(goal_handle, result)


class MavrosBench:
    """Real Cube Orange+ motor spin through MAVROS (GUIDED_NOGPS + attitude setpoints)."""

    ATTITUDE_RATE = 25.0
    RC_CHANNELS_STREAM = 3

    def __init__(self, node, group):
        from mavros_msgs.msg import AttitudeTarget, RCOut, State
        from mavros_msgs.srv import CommandBool, SetMode, StreamRate
        from rcl_interfaces.srv import SetParameters
        self.node = node
        self.AttitudeTarget, self.CommandBool, self.SetMode, self.StreamRate = \
            AttitudeTarget, CommandBool, SetMode, StreamRate
        self.SetParameters = SetParameters
        self.state = None
        self.servos = [1000, 1000, 1000, 1000]
        self.running = False
        self.last_rearm = 0.0
        from bc_common.ros_utils import IMAGE_QOS  # best effort, as MAVROS publishes
        node.create_subscription(State, "/mavros/state", self._state_cb, IMAGE_QOS, callback_group=group)
        node.create_subscription(RCOut, "/mavros/rc/out", self._rc_cb, IMAGE_QOS, callback_group=group)
        self.att_pub = node.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)
        self.motor_pub = node.create_publisher(MotorStatus, "~/motors", 10)
        self.mode_client = node.create_client(SetMode, "/mavros/set_mode", callback_group=group)
        self.arm_client = node.create_client(CommandBool, "/mavros/cmd/arming", callback_group=group)
        self.stream_client = node.create_client(StreamRate, "/mavros/set_stream_rate", callback_group=group)
        self.param_client = node.create_client(SetParameters, "/mavros/param/set_parameters", callback_group=group)
        node.create_timer(1.0 / self.ATTITUDE_RATE, self._stream, callback_group=group)

    def _state_cb(self, msg):
        self.state = msg

    def _rc_cb(self, msg):
        ch = list(msg.channels)[:4]
        self.servos = ch + [1000] * (4 - len(ch))

    def ready(self):
        return self.state is not None and self.state.connected

    def _call(self, client, request, timeout=3.0):
        from bc_common.ros_utils import call_service
        return call_service(client, request, timeout)

    def _set_disarm_delay(self, value):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        req = self.SetParameters.Request(parameters=[Parameter(
            name="DISARM_DELAY", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))])
        if self._call(self.param_client, req) is None:
            self.node.get_logger().warn("Could not set DISARM_DELAY through /mavros/param (param plugin loaded?)")

    def _set_mode(self, mode):
        for _ in range(4):
            r = self._call(self.mode_client, self.SetMode.Request(base_mode=0, custom_mode=mode))
            if r is not None and r.mode_sent:
                return True
            time.sleep(0.3)
        return False

    def _arm(self, value):
        for _ in range(8):
            r = self._call(self.arm_client, self.CommandBool.Request(value=bool(value)))
            if r is not None and r.success:
                return True
            time.sleep(0.25)
        return False

    def start(self):
        log = self.node.get_logger()
        if not self.ready():
            return "MAVROS not connected to the autopilot"
        self._call(self.stream_client, self.StreamRate.Request(stream_id=self.RC_CHANNELS_STREAM,
                                                               message_rate=20, on_off=True))
        self._set_disarm_delay(0.0)
        if not self._set_mode("GUIDED_NOGPS"):
            return "could not switch to GUIDED_NOGPS"
        self.running = True  # stream attitude before/while arming
        if not self._arm(True):
            self.running = False
            return "arming refused"
        log.info(f"[DRONE] Motors ARMED in GUIDED_NOGPS. ESC PWM: {self.servos}")
        return None

    def stop(self):
        log = self.node.get_logger()
        log.info("[DRONE] Ramping down motor throttle safely...")
        start, initial = time.monotonic(), self.node.thrust
        while time.monotonic() - start < 2.0:
            self.node.thrust = initial * max(0.0, 1.0 - (time.monotonic() - start) / 2.0)
            time.sleep(0.05)
        self.node.thrust = 0.0
        self.running = False
        self._arm(False)
        self._set_disarm_delay(10.0)
        self._set_mode("STABILIZE")
        log.info("[COMPLETE] Multirotor safely disarmed!")

    def tick(self, pose):
        """Called at the physics rate: re-arm watchdog and motor telemetry."""
        if self.running and self.state is not None and not self.state.armed \
                and time.monotonic() - self.last_rearm > 2.0:
            self.last_rearm = time.monotonic()
            self.node.get_logger().warn("[BENCH] Autopilot disarmed: re-arming.")
            threading.Thread(target=lambda: (self._set_mode("GUIDED_NOGPS"), self._arm(True)), daemon=True).start()
        m = MotorStatus()
        m.header.stamp = pose.header.stamp
        m.servo_pwm = [int(s) for s in self.servos]
        m.thrust = float(self.node.thrust)
        m.cmd_roll_deg, m.cmd_pitch_deg = float(self.node.cmd_roll), float(self.node.cmd_pitch)
        self.motor_pub.publish(m)

    def _stream(self):
        if not self.running:
            return
        n = self.node
        # NED/FRD roll, pitch, heading -> MAVROS ENU/FLU: roll same, pitch negated, yaw_enu = pi/2 - heading
        w, x, y, z = euler_to_quaternion(math.radians(n.cmd_roll), -math.radians(n.cmd_pitch),
                                         wrap_pi(math.pi / 2.0 - n.sim.heading))
        msg = self.AttitudeTarget()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.type_mask = (self.AttitudeTarget.IGNORE_ROLL_RATE | self.AttitudeTarget.IGNORE_PITCH_RATE
                         | self.AttitudeTarget.IGNORE_YAW_RATE)
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.thrust = float(n.thrust)
        self.att_pub.publish(msg)


def main(args=None):
    spin(BenchFlightController, args)


if __name__ == "__main__":
    main()
