"""Corridor traverse (script's surge_corridor: stages 6-7 forward, stage 11 return), same for both corridors.

  1. Descend over the entrance (banner) to `altitude_z`, turning to `heading`.
  2. Hover `settle_time` s.
  3. Surge `length` m along `heading` at `speed`, holding height and heading, to within
     `surge_tolerance` (0.10 m, so the vehicle really clears the walls), then hold `exit_hold` s.
  4. If `exit_altitude_z` is set: climb straight up to it at the exit, then hold `climb_hold` s.
     Nothing moves sideways at corridor height next to the wall ends.

Action: /traverse_corridor (mission_interfaces/TraverseCorridor)
Uses:   /flight_controller/goto
"""
import math

from mission_interfaces.action import TraverseCorridor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from bc_common.geometry import get_body_axes, offset_ne
from bc_common.ros_utils import FlightClient, GotoOutcome, VehicleListener, finish, serve_action, sleep_checked, spin


class CorridorNavigator(Node):
    def __init__(self):
        super().__init__("corridor_navigator")
        self.surge_tolerance = float(self.declare_parameter("surge_tolerance", 0.10).value)  # m
        self.exit_hold = float(self.declare_parameter("exit_hold", 1.0).value)  # s
        self.climb_tolerance = float(self.declare_parameter("climb_tolerance", 0.15).value)  # m
        self.climb_hold = float(self.declare_parameter("climb_hold", 0.5).value)  # s
        group = ReentrantCallbackGroup()
        self.vehicle = VehicleListener(self, group)
        self.flight = FlightClient(self, group)
        serve_action(self, TraverseCorridor, "/traverse_corridor", self.execute, group)

    def execute(self, goal_handle):
        req = goal_handle.request
        result = TraverseCorridor.Result()
        log = self.get_logger()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        feedback = TraverseCorridor.Feedback()

        def step(state):
            feedback.state = state
            goal_handle.publish_feedback(feedback)
            log.info(f"[CORRIDOR] {state}")

        step(f"descending to NED z {req.altitude_z:.2f} over the entrance")
        outcome = self.flight.goto(req.entry.x, req.entry.y, req.altitude_z, yaw=req.heading, cancel=cancel)
        if not outcome.ok:
            return self.fail(goal_handle, result, outcome)

        step(f"settling {req.settle_time:.1f} s")
        if not sleep_checked(req.settle_time, cancel):
            return finish(goal_handle, result, False, "canceled")

        fwd, _, _ = get_body_axes(req.heading)
        end = offset_ne([req.entry.x, req.entry.y, req.altitude_z], (fwd, req.length))
        step(f"surging {req.length:.1f} m at {req.speed:.1f} m/s (heading {math.degrees(req.heading):+.1f} deg) "
             f"-> N={end[0]:+.2f} E={end[1]:+.2f}")
        outcome = self.flight.goto(*end, yaw=req.heading, speed=req.speed, tolerance=self.surge_tolerance,
                                   cancel=cancel)
        if not outcome.ok:
            return self.fail(goal_handle, result, outcome)
        if not sleep_checked(self.exit_hold, cancel):
            return finish(goal_handle, result, False, "canceled")
        p = self.vehicle.latest
        log.info(f"[CORRIDOR EXIT] Exited corridor at N={p.x:+.2f} E={p.y:+.2f}")

        if math.isfinite(req.exit_altitude_z):
            step(f"climbing vertically at the exit to NED z {req.exit_altitude_z:.2f}")
            outcome = self.flight.goto(end[0], end[1], req.exit_altitude_z, yaw=req.heading,
                                       tolerance=self.climb_tolerance, cancel=cancel)
            if not outcome.ok:
                return self.fail(goal_handle, result, outcome)
            if not sleep_checked(self.climb_hold, cancel):
                return finish(goal_handle, result, False, "canceled")

        p = self.vehicle.latest
        result.exit_position.x, result.exit_position.y, result.exit_position.z = p.x, p.y, p.z
        return finish(goal_handle, result, True, "ok")

    def fail(self, goal_handle, result, outcome):
        if outcome.status == GotoOutcome.CANCELED:
            return finish(goal_handle, result, False, "canceled")
        return finish(goal_handle, result, False, f"GoTo failed: {outcome.message}", self.get_logger())


def main(args=None):
    spin(CorridorNavigator, args)


if __name__ == "__main__":
    main()
