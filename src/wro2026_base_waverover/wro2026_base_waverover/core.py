"""
Pure-Python core of the WAVE ROVER base driver (no rclpy).

Everything here can be tested on any machine with plain pytest:

* differential kinematics and the speed cap,
* the Waveshare JSON protocol (encoding commands, parsing feedback),
* ``BaseController``: decides which wheel command may be sent right now
  (``/cmd_vel`` timeout, feedback watchdog, battery limits),
* ``DriverLoop``: the single I/O loop that sends wheel commands and polls
  feedback through a replaceable transport (see ``transport.py``),
* dead-reckoning odometry (``odom_twist`` + ``OdomIntegrator``).

Units: the ESP32 takes left/right wheel commands as PWM fractions
(firmware PWM = L * 512, so 0.5 = 100 %), not m/s. ``/cmd_vel`` values are
passed through the kinematics unchanged, exactly like the original driver,
until the PWM -> m/s factor has been measured.
"""
import json
import math
import threading
import time

MAX_WHEEL = 0.5  # ESP32 accepts -0.5 .. 0.5 (= 100% PWM)
G = 9.80665
STOP_CMD = {'T': 1, 'L': 0, 'R': 0}
POLL_BASE_CMD = {'T': 130}
POLL_IMU_CMD = {'T': 126}
REPLY_BASE = 1001  # reply to T:130 (battery voltage 'v', applied L/R, ...)
REPLY_IMU = 1002  # reply to T:126 (r/p/y deg, ax/ay/az mg, gx/gy/gz deg/s)

FEEDBACK_OK = 'ok'
FEEDBACK_WAITING = 'waiting'
FEEDBACK_LOST = 'lost'
FEEDBACK_DISABLED = 'disabled'

LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2


# ------------------------------------------------------------- kinematics
def clamp(value, limit):
    """Clamp value to [-limit, limit]."""
    return max(-limit, min(limit, value))


def speed_limit(max_speed):
    """Return the effective wheel cap: |max_speed| but never above MAX_WHEEL."""
    return min(abs(max_speed), MAX_WHEEL)


def twist_to_wheels(v, w, track_width, max_speed):
    """
    Convert a body twist to clamped (left, right) wheel commands.

    Same math as the tested driver: each wheel is clamped on its own, so a
    command beyond the cap changes the turn radius rather than being scaled.
    """
    limit = speed_limit(max_speed)
    left = clamp(v - w * track_width / 2.0, limit)
    right = clamp(v + w * track_width / 2.0, limit)
    return left, right


def wire_number(value):
    """Round to 3 decimals like the original driver; any zero becomes int 0."""
    value = round(float(value), 3)
    if value == 0:
        return 0
    return value


# --------------------------------------------------------------- protocol
def encode_command(cmd):
    """Return the compact JSON text sent to the ESP32."""
    return json.dumps(cmd, separators=(',', ':'))


def wheel_command(left, right):
    """Build a T:1 command; a stop is exactly {"T":1,"L":0,"R":0}."""
    return {'T': 1, 'L': wire_number(left), 'R': wire_number(right)}


def parse_feedback(line):
    """Parse one reply line; return a dict or None when it is not a JSON object."""
    try:
        msg = json.loads(line)
    except (TypeError, ValueError):
        return None
    return msg if isinstance(msg, dict) else None


def battery_voltage(msg):
    """Return the voltage of a T:1001 reply, or None."""
    if not msg or msg.get('T') != REPLY_BASE:
        return None
    try:
        return float(msg['v'])
    except (KeyError, TypeError, ValueError):
        return None


def imu_values(msg):
    """
    Convert a T:1002 reply to SI units, or return None.

    Firmware units (ugv_base_general, QMI8658_UINT_MG_DPS): r/p/y degrees,
    ax/ay/az mg, gx/gy/gz deg/s.
    """
    if not msg or msg.get('T') != REPLY_IMU:
        return None
    try:
        out = {k: math.radians(float(msg[k])) for k in ('r', 'p', 'y', 'gx', 'gy', 'gz')}
        out.update({k: float(msg[k]) * G / 1000.0 for k in ('ax', 'ay', 'az')})
    except (KeyError, TypeError, ValueError):
        return None
    return out


def quaternion_from_rpy(roll, pitch, yaw):
    """Return (x, y, z, w) for roll/pitch/yaw in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    angle = math.fmod(angle + math.pi, 2.0 * math.pi)
    if angle <= 0.0:
        angle += 2.0 * math.pi
    return angle - math.pi


# ------------------------------------------------------------- controller
class BaseController:
    """
    Decide which wheel command may be sent right now (thread-safe).

    Gates, in order:
      1. ``/cmd_vel`` older than ``cmd_timeout`` (or never received) -> stop.
      2. Feedback watchdog (``feedback_timeout`` > 0): no T:130 reply for
         ``feedback_timeout`` seconds -> stop and report ERROR. Before the
         first reply, non-zero commands are refused as well.
      3. Battery below ``critical_battery_voltage`` -> refuse non-zero
         commands. Below ``low_battery_voltage`` -> WARN only.
    """

    def __init__(self, track_width, max_speed, cmd_timeout, feedback_timeout=2.0,
                 low_battery_voltage=11.0, critical_battery_voltage=10.5,
                 clock=time.monotonic):
        self.track_width = track_width
        self.max_speed = speed_limit(max_speed)
        self.cmd_timeout = cmd_timeout
        self.feedback_timeout = feedback_timeout
        self.low_battery_voltage = low_battery_voltage
        self.critical_battery_voltage = critical_battery_voltage
        self.clock = clock
        self.lock = threading.Lock()
        self.start_time = clock()
        self.left = 0.0
        self.right = 0.0
        self.last_cmd = None
        self.last_feedback = None
        self.battery_v = None

    # inputs
    def set_cmd_vel(self, v, w):
        """Store a /cmd_vel request (already clamped to max_speed)."""
        left, right = twist_to_wheels(v, w, self.track_width, self.max_speed)
        with self.lock:
            self.left, self.right = left, right
            self.last_cmd = self.clock()

    def on_base_feedback(self, voltage=None):
        """Record a T:1001 reply (voltage may be None if the field is missing)."""
        with self.lock:
            self.last_feedback = self.clock()
            if voltage is not None:
                self.battery_v = voltage

    # state
    def feedback_state(self, now=None):
        """Return FEEDBACK_OK / WAITING / LOST / DISABLED."""
        now = self.clock() if now is None else now
        if self.feedback_timeout <= 0:
            return FEEDBACK_DISABLED
        with self.lock:
            last = self.last_feedback
        if last is None:
            if now - self.start_time < self.feedback_timeout:
                return FEEDBACK_WAITING
            return FEEDBACK_LOST
        if now - last > self.feedback_timeout:
            return FEEDBACK_LOST
        return FEEDBACK_OK

    def feedback_age(self, now=None):
        """Seconds since the last T:1001 reply, or None."""
        now = self.clock() if now is None else now
        with self.lock:
            last = self.last_feedback
        return None if last is None else now - last

    def battery_state(self):
        """Return 'unknown', 'ok', 'low' or 'critical'."""
        with self.lock:
            v = self.battery_v
        if v is None:
            return 'unknown'
        if v < self.critical_battery_voltage:
            return 'critical'
        if v < self.low_battery_voltage:
            return 'low'
        return 'ok'

    def requested(self, now=None):
        """Return the /cmd_vel request after the timeout gate, rounded for the wire."""
        now = self.clock() if now is None else now
        with self.lock:
            if self.last_cmd is None or now - self.last_cmd > self.cmd_timeout:
                return 0, 0
            return wire_number(self.left), wire_number(self.right)

    def decide(self, now=None):
        """
        Return (left, right, reason) for the command to send now.

        reason is 'cmd' when the request passes, otherwise why it is zero:
        'idle' (no fresh /cmd_vel), 'feedback_waiting', 'feedback_lost' or
        'battery_critical'.
        """
        now = self.clock() if now is None else now
        left, right = self.requested(now)
        if left == 0 and right == 0:
            return 0, 0, 'idle'
        fb = self.feedback_state(now)
        if fb == FEEDBACK_WAITING:
            return 0, 0, 'feedback_waiting'
        if fb == FEEDBACK_LOST:
            return 0, 0, 'feedback_lost'
        if self.battery_state() == 'critical':
            return 0, 0, 'battery_critical'
        return left, right, 'cmd'

    def status(self, now=None):
        """Return (level, message) for /diagnostics."""
        now = self.clock() if now is None else now
        fb = self.feedback_state(now)
        bat = self.battery_state()
        with self.lock:
            v = self.battery_v
        problems = []
        level = LEVEL_OK
        if fb == FEEDBACK_LOST:
            level = LEVEL_ERROR
            problems.append(f'no T:130 reply for > {self.feedback_timeout:.1f} s, wheels stopped')
        elif fb == FEEDBACK_WAITING:
            level = max(level, LEVEL_WARN)
            problems.append('waiting for the first T:130 reply')
        if bat == 'critical':
            level = LEVEL_ERROR
            problems.append(f'battery {v:.2f} V < {self.critical_battery_voltage:.2f} V, '
                            'non-zero commands refused')
        elif bat == 'low':
            level = max(level, LEVEL_WARN)
            problems.append(f'battery low {v:.2f} V < {self.low_battery_voltage:.2f} V')
        return level, '; '.join(problems) if problems else 'ok'


# -------------------------------------------------------------- I/O loop
class DriverLoop:
    """
    Single I/O loop: wheel commands first, then T:130 / T:126 polling.

    The transport does the actual I/O. ``step()`` does at most one send and
    returns True when it did something, so tests can drive it with a fake
    clock. Callbacks (all optional):
      on_sent(record)        every T:1 actually attempted
      on_line(line, msg)     every reply line (msg = parsed dict or None)
      log(level, text)       level is 'info', 'warn' or 'error'
    """

    def __init__(self, controller, transport, cmd_period, feedback_period=0.2,
                 poll_base=True, poll_imu=True, clock=time.monotonic,
                 wall_clock=time.time, on_sent=None, on_line=None, log=None):
        self.controller = controller
        self.transport = transport
        self.cmd_period = cmd_period
        self.feedback_period = feedback_period
        self.clock = clock
        self.wall_clock = wall_clock
        self.on_sent = on_sent
        self.on_line = on_line
        self.log = log or (lambda level, text: None)
        self.polls = []
        if poll_base:
            self.polls.append(POLL_BASE_CMD)
        if poll_imu:
            self.polls.append(POLL_IMU_CMD)
        self.next_poll = [0.0] * len(self.polls)
        self.last_sent = None
        self.last_sent_time = -math.inf
        self.last_reason = 'idle'
        self.prev_feedback = None
        self.send_failures = 0
        self.lock = threading.Lock()
        self.sent_ok_wheels = (0, 0)  # last T:1 that the transport accepted

    def handle_line(self, line):
        """Process one reply line from the ESP32 (any thread)."""
        line = line.strip()
        if not line:
            return
        msg = parse_feedback(line)
        if msg is not None and msg.get('T') == REPLY_BASE:
            self.controller.on_base_feedback(battery_voltage(msg))
        if self.on_line:
            self.on_line(line, msg)

    def _send(self, cmd):
        """Send through the transport; return the reply text, None, or False on failure."""
        try:
            return self.transport.send(cmd)
        except Exception as e:  # keep running, just report
            self.send_failures += 1
            self.log('warn', f'send {encode_command(cmd)} failed: {e}')
            return False

    def send_wheels(self, left, right, reason='cmd'):
        """Send one T:1 command and report it through on_sent."""
        cmd = wheel_command(left, right)
        t0 = self.wall_clock()
        reply = self._send(cmd)
        ok = reply is not False
        if ok:
            with self.lock:
                self.sent_ok_wheels = (cmd['L'], cmd['R'])
        if self.on_sent:
            self.on_sent({'t': t0, 'dt': round(self.wall_clock() - t0, 4), 'ok': ok,
                          'cmd': cmd, 'gate': reason})
        return ok

    def poll(self, cmd):
        """Send a feedback request; HTTP replies are handled inline."""
        reply = self._send(cmd)
        if isinstance(reply, str) and reply:
            for line in reply.splitlines():
                self.handle_line(line)

    def step(self):
        """Do at most one I/O action; return True if something was sent."""
        now = self.clock()
        fb = self.controller.feedback_state(now)
        force = False
        if fb == FEEDBACK_LOST and self.prev_feedback != FEEDBACK_LOST:
            self.log('error', f'feedback lost: no T:130 reply for '
                              f'{self.controller.feedback_timeout:.1f} s, sending stop')
            force = True
        elif fb == FEEDBACK_OK and self.prev_feedback == FEEDBACK_LOST:
            self.log('info', 'feedback recovered')
        self.prev_feedback = fb

        left, right, reason = self.controller.decide(now)
        if force:
            reason = 'feedback_lost'
        if reason == 'battery_critical' and self.last_reason != reason:
            self.log('error', 'battery critical, refusing non-zero wheel commands')
        self.last_reason = reason
        desired = (left, right)
        if force or desired != self.last_sent or now - self.last_sent_time >= self.cmd_period:
            self.send_wheels(left, right, reason)
            self.last_sent, self.last_sent_time = desired, now
            return True
        due = [i for i, t in enumerate(self.next_poll) if t <= now]
        if due:
            i = min(due, key=lambda k: self.next_poll[k])
            self.next_poll[i] = now + self.feedback_period
            self.poll(self.polls[i])
            return True
        return False

    def run(self, keep_running, idle_sleep=0.005):
        """Loop until keep_running() is False."""
        while keep_running():
            if not self.step():
                time.sleep(idle_sleep)

    def send_stop(self, attempts=3, pause=0.1, sleep=time.sleep):
        """Send {"T":1,"L":0,"R":0} until the transport accepts it; return success."""
        for i in range(attempts):
            if self.send_wheels(0, 0, 'shutdown'):
                return True
            if i + 1 < attempts:
                sleep(pause)
        return False


# ------------------------------------------------------------------ odom
def odom_twist(left, right, gz, pwm_to_mps, imu_fresh=True, hold_when_stopped=True,
               yaw_rate_sign=1.0):
    """
    Return (v, wz) for dead reckoning.

    v  = mean of the wheel commands actually sent x pwm_to_mps (m/s).
    wz = IMU gz (rad/s) x yaw_rate_sign; 0 when the IMU sample is stale.
    With hold_when_stopped, a zero wheel command means the car is not
    driven, so both are 0 (gyro bias does not make the pose drift).
    """
    if hold_when_stopped and left == 0 and right == 0:
        return 0.0, 0.0
    v = (left + right) / 2.0 * pwm_to_mps
    wz = gz * yaw_rate_sign if (imu_fresh and gz is not None) else 0.0
    return v, wz


class OdomIntegrator:
    """Planar pose integration (midpoint rule)."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Go back to the origin."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.wz = 0.0

    def update(self, dt, v, wz):
        """Advance the pose by dt seconds at body speed v (m/s) and yaw rate wz (rad/s)."""
        self.v, self.wz = v, wz
        if dt <= 0.0:
            return
        mid = self.yaw + wz * dt / 2.0
        self.x += v * math.cos(mid) * dt
        self.y += v * math.sin(mid) * dt
        self.yaw = normalize_angle(self.yaw + wz * dt)
