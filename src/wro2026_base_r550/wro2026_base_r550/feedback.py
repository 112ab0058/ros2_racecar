"""
Read-only feedback handling for the R550 (no rclpy, no serial writes).

Turns decoded STM32 feedback frames (protocol.Feedback) into planar
odometry and keeps reception statistics. Used by feedback_node (ROS) and
listen (command line). Nothing in this module can send to the STM32.
"""
import math
import time


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) for a rotation of yaw radians about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomIntegrator:
    """
    Planar pose from the STM32's body speeds (encoder based).

    update(t, vx, wz) integrates from the previous frame's time to t with
    the midpoint rule. A gap longer than max_gap seconds (lost frames,
    unplugged cable) is not integrated: the pose holds and integration
    restarts from t, because guessing the motion across a gap is worse than
    missing it.
    """

    def __init__(self, max_gap=0.5):
        self.max_gap = max_gap
        self.gaps = 0
        self.reset()

    def reset(self):
        """Go back to the origin."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_t = None

    def update(self, t, vx, wz):
        """Advance the pose to time t (s) with body speed vx (m/s), yaw rate wz (rad/s)."""
        self.vx, self.wz = vx, wz
        last, self.last_t = self.last_t, t
        if last is None:
            return
        dt = t - last
        if dt <= 0.0:
            return
        if dt > self.max_gap:
            self.gaps += 1
            return
        mid = self.yaw + wz * dt / 2.0
        self.x += vx * math.cos(mid) * dt
        self.y += vx * math.sin(mid) * dt
        self.yaw = normalize_angle(self.yaw + wz * dt)


class RateMeter:
    """Frames per second over a sliding window."""

    def __init__(self, window=2.0):
        self.window = window
        self.times = []

    def tick(self, t):
        """Record one frame at time t."""
        self.times.append(t)
        self._trim(t)

    def rate(self, now):
        """Frames per second over the last window seconds."""
        self._trim(now)
        return len(self.times) / self.window

    def _trim(self, now):
        cutoff = now - self.window
        while self.times and self.times[0] < cutoff:
            self.times.pop(0)


def motors_enabled(feedback):
    """flag_stop byte: 0 = motors enabled (official doc), anything else = disabled."""
    return feedback.stop_flag == 0


def feedback_summary(fb):
    """One-line human-readable summary of a Feedback (for logs and listen)."""
    return (f'motor={"ON " if motors_enabled(fb) else "OFF"}(flag={fb.stop_flag}) '
            f'vx={fb.vx:+.3f} m/s wz={fb.wz:+.3f} rad/s '
            f'acc=({fb.accel_x:+.2f},{fb.accel_y:+.2f},{fb.accel_z:+.2f}) m/s2 '
            f'gyro_z={fb.gyro_z:+.4f} rad/s battery={fb.voltage:.2f} V')


def monotonic():
    """Clock used by default (separate function so tests can replace it)."""
    return time.monotonic()
