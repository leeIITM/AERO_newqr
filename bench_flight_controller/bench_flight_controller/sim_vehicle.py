"""Point-mass bench kinematics (ROS-free)."""
import math

from bc_common.geometry import wrap_pi


class SimVehicle:
    """Point-mass kinematics chasing a position setpoint (NED) or a velocity command."""

    def __init__(self, max_speed, max_climb, max_accel, yaw_rate, heading=0.0):
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.heading = heading
        self.sp = None          # position setpoint [n, e, d]
        self.sp_vel = None      # velocity setpoint [vn, ve, vd]
        self.sp_yaw = heading
        self.max_speed, self.max_climb, self.max_accel, self.yaw_rate = max_speed, max_climb, max_accel, yaw_rate
        self.kp = 1.2

    def step(self, dt):
        if self.sp is None and self.sp_vel is None:
            want = [0.0, 0.0, 0.0]
        elif self.sp_vel is not None:
            want = list(self.sp_vel)
        else:
            want = [self.kp * (s - p) for s, p in zip(self.sp, self.pos)]
        h = math.hypot(want[0], want[1])
        if h > self.max_speed:
            want[0], want[1] = want[0] * self.max_speed / h, want[1] * self.max_speed / h
        want[2] = max(-self.max_climb, min(self.max_climb, want[2]))
        dv = [w - v for w, v in zip(want, self.vel)]
        mag = math.sqrt(sum(d * d for d in dv))
        lim = self.max_accel * dt
        if mag > lim:
            dv = [d * lim / mag for d in dv]
        self.vel = [v + d for v, d in zip(self.vel, dv)]
        self.pos = [p + v * dt for p, v in zip(self.pos, self.vel)]
        if self.pos[2] > 0.0:  # ground at z = 0
            self.pos[2] = 0.0
            self.vel[2] = min(0.0, self.vel[2])
            if self.sp is None:
                self.vel = [0.0, 0.0, 0.0]
        err = wrap_pi(self.sp_yaw - self.heading)
        self.heading = wrap_pi(self.heading + max(-self.yaw_rate * dt, min(self.yaw_rate * dt, err)))

    def speed(self):
        return math.sqrt(sum(v * v for v in self.vel))
