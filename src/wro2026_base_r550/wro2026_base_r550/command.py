"""
Safety gate between /cmd_vel and the STM32 (no rclpy, no serial).

CommandGate decides, at every send tick, what speed may actually go to the
car. Everything is in real units (m/s, rad/s); base_node converts to the
STM32's units. Rules, checked in this order (first one that fires wins):

  disabled       commands not enabled                      -> 0, 0
  run_time_over  longer than max_run_time (if > 0) since the -> 0, 0 (latched)
                 first /cmd_vel
  feedback_lost  no valid STM32 frame for feedback_timeout  -> 0, 0
  battery_low    filtered voltage below min_battery_voltage -> 0, 0 (until it
                 recovers by battery_hysteresis)
  no_cmd         no /cmd_vel received yet                   -> 0, 0
  cmd_timeout    newest /cmd_vel older than cmd_timeout     -> 0, 0
  ok             clamp speed, ramp vx, keep the curvature   -> vx, wz

Safety stops are immediate (0 is sent at once; the STM32's own PI brakes).
Only speeding up / slowing down on a valid command is ramped by max_accel.

Ackermann: the car cannot turn on the spot. wz is derived from the
commanded curvature k = wz / vx times the ramped vx, so the path keeps its
shape while accelerating, and wz is 0 when vx is (nearly) 0.
"""
import math

DISABLED = 'disabled'
RUN_TIME_OVER = 'run_time_over'
FEEDBACK_LOST = 'feedback_lost'
BATTERY_LOW = 'battery_low'
NO_CMD = 'no_cmd'
CMD_TIMEOUT = 'cmd_timeout'
OK = 'ok'


def clamp(value, low, high):
    """Limit value to [low, high]."""
    return max(low, min(high, value))


class CommandGate:
    """See the module docstring. Times are seconds from one monotonic clock."""

    def __init__(self, *, enabled=False, cmd_timeout=0.3, max_vx=0.3, max_reverse_vx=0.15,
                 max_wz=1.0, max_accel=0.5, min_turn_speed=0.02, feedback_timeout=0.5,
                 min_battery_voltage=10.5, battery_hysteresis=0.2, battery_filter_time=2.0,
                 max_run_time=0.0):
        if cmd_timeout <= 0 or max_vx < 0 or max_reverse_vx < 0 or max_wz < 0 or max_accel <= 0:
            raise ValueError('cmd_timeout and max_accel must be > 0, limits must be >= 0')
        self.enabled = enabled
        self.cmd_timeout = cmd_timeout
        self.max_vx = max_vx
        self.max_reverse_vx = max_reverse_vx
        self.max_wz = max_wz
        self.max_accel = max_accel
        self.min_turn_speed = min_turn_speed
        self.feedback_timeout = feedback_timeout
        self.min_battery_voltage = min_battery_voltage
        self.battery_hysteresis = battery_hysteresis
        self.battery_filter_time = battery_filter_time
        self.max_run_time = max_run_time

        self.start_time = None
        self.cmd_time = None
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.rejected_cmds = 0
        self.voltage = None
        self.voltage_time = None
        self.battery_low = False
        self.run_time_over = False
        self.out_vx = 0.0
        self.out_wz = 0.0
        self.last_tick = None
        self.reason = DISABLED if not enabled else NO_CMD

    # ------------------------------------------------------------ inputs
    def set_cmd(self, t, vx, wz):
        """Store a /cmd_vel. Non-finite values are rejected (and count as no command)."""
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self.rejected_cmds += 1
            return False
        if self.start_time is None:  # max_run_time counts from the first command
            self.start_time = t
        self.cmd_time, self.cmd_vx, self.cmd_wz = t, vx, wz
        return True

    def set_voltage(self, t, volts):
        """Feed a battery reading; filtered so short sags under load do not trip it."""
        if self.voltage is None or self.battery_filter_time <= 0:
            self.voltage = volts
        else:
            dt = max(0.0, t - self.voltage_time)
            a = min(1.0, dt / self.battery_filter_time)
            self.voltage += a * (volts - self.voltage)
        self.voltage_time = t
        if self.voltage < self.min_battery_voltage:
            self.battery_low = True
        elif self.voltage >= self.min_battery_voltage + self.battery_hysteresis:
            self.battery_low = False

    # ------------------------------------------------------------ output
    def update(self, now, last_feedback_time):
        """Return (vx, wz, reason) to send now."""
        dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        reason = self._stop_reason(now, last_feedback_time)
        if reason != OK:
            self.out_vx = self.out_wz = 0.0
        else:
            target_vx = clamp(self.cmd_vx, -self.max_reverse_vx, self.max_vx)
            step = self.max_accel * dt
            self.out_vx = clamp(target_vx, self.out_vx - step, self.out_vx + step)
            if abs(self.cmd_vx) < self.min_turn_speed or abs(self.out_vx) < self.min_turn_speed:
                self.out_wz = 0.0  # Ackermann: no turning on the spot
            else:
                curvature = self.cmd_wz / self.cmd_vx
                self.out_wz = clamp(curvature * self.out_vx, -self.max_wz, self.max_wz)
        self.reason = reason
        return self.out_vx, self.out_wz, reason

    def _stop_reason(self, now, last_feedback_time):
        if not self.enabled:
            return DISABLED
        if (self.max_run_time > 0 and self.start_time is not None
                and now - self.start_time > self.max_run_time):
            self.run_time_over = True
        if self.run_time_over:
            return RUN_TIME_OVER
        if last_feedback_time is None or now - last_feedback_time > self.feedback_timeout:
            return FEEDBACK_LOST
        if self.voltage is None or self.battery_low:
            return BATTERY_LOW
        if self.cmd_time is None:
            return NO_CMD
        if now - self.cmd_time > self.cmd_timeout:
            return CMD_TIMEOUT
        return OK
