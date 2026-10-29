"""Payload drop controller (structure of airdrop_controller_node), for the geofenced target search.

After target_navigator has flown to the target QR and qr_centering has centred (and held) over it, the
mission tree asks for the drop. Unlike airdrop_controller (mission upload + AUTO), the drone stays in GUIDED
holding its position, and the two PWMs go out as MAV_CMD_DO_SET_SERVO commands through /mavros/cmd/command:
    SERVO<drop_channel> = open_pwm  ->  wait drop_interval s  ->  SERVO<drop_channel> = close_pwm
Each command is checked against /mavros/rc/out (when MAVROS publishes it).

Action    /payload_drop        mission_interfaces/PayloadDrop (zero goal fields = the parameters below)
Topic in  /task_command        std_msgs/String "payload_drop" also triggers a drop (manual test, as "airdrop")
          /mavros/rc/out       mavros_msgs/RCOut (check of the commanded PWM)
Topics    /drop_status         std_msgs/String JSON {"action": "drop_complete", "payloads_delivered": n} (latched)
          /task_status         std_msgs/Bool (true after a drop, false after a failed one)
Service   /mavros/cmd/command  mavros_msgs/CommandLong (MAV_CMD_DO_SET_SERVO = 183)

Parameters: drop_channel (9 = Cube AUX1), open_pwm (1900), close_pwm (1100), drop_interval (2.0 s),
pre_drop_settle (0.5 s hover before the release), simulate (false: log only, no MAVROS; bench runs),
verify_rc_out (true), command_timeout (5.0 s).
ArduPilot: the output used must have SERVOn_FUNCTION = 0 (Disabled) or DO_SET_SERVO is ignored.
"""
import json
import threading
import time

import rclpy
from mavros_msgs.msg import RCOut
from mavros_msgs.srv import CommandLong
from mission_interfaces.action import PayloadDrop
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String

from bc_common.ros_utils import finish, serve_action, sleep_checked, spin
from payload_drop.geotag import drop_sequence

MAV_CMD_DO_SET_SERVO = 183


class PayloadDropController(Node):
    def __init__(self):
        super().__init__("payload_drop")

        self.callback_group = ReentrantCallbackGroup()

        self.command_service = self.create_client(CommandLong, "/mavros/cmd/command",
                                                  callback_group=self.callback_group)

        qos = QoSProfile(depth=1)
        self.task_status_pub = self.create_publisher(Bool, "/task_status", qos)
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.drop_status_pub = self.create_publisher(String, "/drop_status", status_qos)
        task_command_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd = ""
        self.drop_active = False
        self.payloads_delivered = 0
        self.lock = threading.Lock()
        self.rc_out = None  # (time, channels)

        self.drop_channel = int(self.declare_parameter("drop_channel", 9).value)
        self.open_pwm = int(self.declare_parameter("open_pwm", 1900).value)
        self.close_pwm = int(self.declare_parameter("close_pwm", 1100).value)
        self.drop_interval = float(self.declare_parameter("drop_interval", 2.0).value)
        self.pre_drop_settle = float(self.declare_parameter("pre_drop_settle", 0.5).value)
        self.simulate = bool(self.declare_parameter("simulate", False).value)
        self.verify_rc_out = bool(self.declare_parameter("verify_rc_out", True).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 5.0).value)

        self.create_subscription(String, "/task_command", self.task_command_callback, task_command_qos,
                                 callback_group=self.callback_group)
        self.create_subscription(RCOut, "/mavros/rc/out", self.rc_out_callback, qos_profile_sensor_data,
                                 callback_group=self.callback_group)
        serve_action(self, PayloadDrop, "/payload_drop", self.execute, self.callback_group)
        self.get_logger().info(
            f"Payload drop ready: SERVO{self.drop_channel} {self.open_pwm} -> {self.drop_interval:.1f} s -> "
            f"{self.close_pwm} us{' (SIMULATED: no autopilot commands)' if self.simulate else ''}")

    # ------------------------------------------------------------------ plumbing (as airdrop_controller)

    def _call_service(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service not available: {client.srv_name}")
            return None

        future = client.call_async(request)
        start_time = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - start_time) > timeout:
                self.get_logger().error(f"Service call timed out: {client.srv_name}")
                return None
            time.sleep(0.01)

        if future.done():
            try:
                return future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Service call failed on {client.srv_name}: {exc}")
                return None
        return None

    def publish_task_status(self, completed: bool):
        msg = Bool()
        msg.data = completed
        self.task_status_pub.publish(msg)

    def publish_drop_status(self, action: str, **extra):
        msg = String()
        msg.data = json.dumps({"action": action, "payloads_delivered": self.payloads_delivered, **extra})
        self.drop_status_pub.publish(msg)

    def rc_out_callback(self, msg):
        self.rc_out = (time.monotonic(), list(msg.channels))

    # ------------------------------------------------------------------ servo commands

    def create_servo_command(self, pwm, channel):
        """MAV_CMD_DO_SET_SERVO as an immediate command (airdrop_controller put it in a mission item)."""
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(channel)
        req.param2 = float(pwm)
        return req

    def send_servo(self, channel, pwm):
        if self.simulate:
            self.get_logger().info(f"[PAYLOAD][SIM] SERVO{channel} = {pwm} us")
            return True
        stamp = time.monotonic()
        resp = self._call_service(self.command_service, self.create_servo_command(pwm, channel),
                                  self.command_timeout)
        if resp is None or not resp.success:
            self.get_logger().error(f"DO_SET_SERVO {channel} {pwm} rejected "
                                    f"(result {getattr(resp, 'result', '?')}): is SERVO{channel}_FUNCTION = 0?")
            return False
        if self.verify_rc_out:
            self.check_rc_out(channel, pwm, stamp)
        return True

    def check_rc_out(self, channel, pwm, since, timeout=1.0):
        """Warn if /mavros/rc/out does not show the commanded PWM (it is not always streamed)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rc = self.rc_out
            if rc is not None and rc[0] > since and len(rc[1]) >= channel:
                if abs(rc[1][channel - 1] - pwm) <= 5:
                    self.get_logger().info(f"[PAYLOAD] rc/out confirms SERVO{channel} = {rc[1][channel - 1]} us")
                    return True
            time.sleep(0.05)
        rc = self.rc_out
        if rc is None:
            self.get_logger().info("[PAYLOAD] /mavros/rc/out not streamed: output not verified")
        else:
            got = rc[1][channel - 1] if len(rc[1]) >= channel else "n/a"
            self.get_logger().warn(f"[PAYLOAD] rc/out shows SERVO{channel} = {got}, commanded {pwm}")
        return False

    # ------------------------------------------------------------------ drop

    def run_drop(self, channel=0, open_pwm=0, close_pwm=0, interval=0.0, cancel=None, on_state=None):
        """Returns None on success, else an error message."""
        channel = int(channel) or self.drop_channel
        open_pwm = int(open_pwm) or self.open_pwm
        close_pwm = int(close_pwm) or self.close_pwm
        interval = float(interval) if interval > 0 else self.drop_interval
        with self.lock:
            if self.drop_active:
                return "a drop is already running"
            self.drop_active = True
        try:
            if on_state:
                on_state("settling over the target")
            if not sleep_checked(self.pre_drop_settle, cancel):
                return "canceled"
            if on_state:
                on_state(f"releasing: SERVO{channel} {open_pwm} us")
            error = drop_sequence(self.send_servo, lambda s: sleep_checked(s, cancel), channel, open_pwm,
                                  close_pwm, interval, log=self.get_logger().info)
            if error is None:
                self.payloads_delivered += 1
                self.complete_task()
            else:
                self.publish_drop_status("drop_failed", error=error)
                self.publish_task_status(False)
            return error
        finally:
            with self.lock:
                self.drop_active = False

    def complete_task(self):
        self.get_logger().info(f"Payload drop complete ({self.payloads_delivered} delivered).")
        self.cmd = ""
        self.publish_drop_status("drop_complete")
        self.publish_task_status(True)

    def task_command_callback(self, msg):
        cmd = msg.data.strip()
        if cmd != "payload_drop":
            return
        self.cmd = cmd
        self.get_logger().info("payload_drop command received.")
        threading.Thread(target=self.run_drop, daemon=True).start()

    def execute(self, goal_handle):
        req = goal_handle.request
        result = PayloadDrop.Result()
        feedback = PayloadDrop.Feedback()

        def on_state(text):
            feedback.state = text
            goal_handle.publish_feedback(feedback)

        error = self.run_drop(req.channel, req.open_pwm, req.close_pwm, req.interval,
                              cancel=lambda: goal_handle.is_cancel_requested, on_state=on_state)
        result.payloads_delivered = int(self.payloads_delivered)
        if error is None:
            return finish(goal_handle, result, True, "payload dropped")
        return finish(goal_handle, result, False, error, self.get_logger())


def main(args=None):
    spin(PayloadDropController, args)


if __name__ == "__main__":
    main()
