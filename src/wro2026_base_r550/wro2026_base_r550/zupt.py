"""
Zero-velocity update (ZUPT) of the gyro bias. Pure logic, no ROS.

Why: the firmware zeroes the gyro with ONE sample at second 10 after power-on,
and the bias keeps changing while the board warms up (2026-10-20: 0.61 deg/min
right after boot vs 0.25 deg/min an hour later). Each minute of that is
heading error for the EKF.

Idea: an Ackermann car whose rear wheels do not turn cannot yaw (no turning on
the spot). So when the wheels have been still for still_time, whatever the gyro
reads is bias: follow it with a low-pass filter (time_constant) and subtract it.
Every stop re-zeroes the gyro, also after temperature changes.

Safety against learning a real rotation as bias (car carried by hand, wheels
lifted): samples further than max_rate from the current bias are not used.
The bias is clamped to max_bias.
"""


class GyroBias:
    def __init__(self, enabled=True, still_time=1.0, time_constant=5.0, max_rate=0.01,
                 vx_eps=0.005, wz_eps=0.01, max_bias=0.02):
        if still_time < 0 or time_constant <= 0 or max_rate <= 0 or max_bias <= 0:
            raise ValueError('bad ZUPT settings')
        self.enabled = enabled
        self.still_time = still_time
        self.time_constant = time_constant
        self.max_rate = max_rate
        self.vx_eps = vx_eps
        self.wz_eps = wz_eps
        self.max_bias = max_bias
        self.bias = [0.0, 0.0, 0.0]
        self.still_since = None
        self.last_t = None
        self.updates = 0
        self.rejected = 0

    @property
    def learning(self):
        return self.still_since is not None and self.last_t is not None and \
            self.last_t - self.still_since >= self.still_time

    def update(self, t, vx, wz, gx, gy, gz):
        """One feedback frame at time t (s). Returns the corrected (gx, gy, gz)."""
        g = (gx, gy, gz)
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
        self.last_t = t
        if not self.enabled:
            return g
        wheels_still = abs(vx) < self.vx_eps and abs(wz) < self.wz_eps
        if not wheels_still:
            self.still_since = None
        else:
            if self.still_since is None:
                self.still_since = t
            if t - self.still_since >= self.still_time and dt > 0:
                if all(abs(g[i] - self.bias[i]) < self.max_rate for i in range(3)):
                    a = dt / (self.time_constant + dt)
                    for i in range(3):
                        b = self.bias[i] + a * (g[i] - self.bias[i])
                        self.bias[i] = max(-self.max_bias, min(self.max_bias, b))
                    self.updates += 1
                else:
                    self.rejected += 1
        return tuple(g[i] - self.bias[i] for i in range(3))
