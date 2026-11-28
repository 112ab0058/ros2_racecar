"""
Keyboard teleop logic (pure: no ROS, no terminal). teleop_node.py wraps it.

A terminal has no "key released" event. While a key is held, the terminal
repeats it: first after the repeat delay (Windows default ~0.5 s), then about
30 times a second. So "released" = no key for a while:
  - before the first repeat arrives: first_hold s (longer than the delay)
  - once repeats arrive:             release s (short, so the car stops fast)
After that the command is 0, and the node keeps PUBLISHING 0 at its rate
(never stops publishing: the base node's 0.3 s timeout is the second layer).

Keys (one key at a time; terminals cannot report two held keys):
    q  w  e      forward-left   forward   forward-right
    a  s  d      (nothing)      STOP      (nothing)
    z  x  c      back-left      back      back-right
    space = STOP, + / - = speed step.  a / d do nothing: no turning on the spot.
"Left" means the front wheels steer left, also when reversing (the curvature
is kept, so wz = vx * curvature changes sign with vx, like base_node does).
"""

# key -> (direction of vx, steering: +1 left, -1 right)
DRIVE_KEYS = {
    'w': (1, 0), 'q': (1, 1), 'e': (1, -1),
    'x': (-1, 0), 'z': (-1, 1), 'c': (-1, -1),
}
STOP_KEYS = (' ', 's')
MIN_TURN_RADIUS = 0.35   # m, firmware MINI_AKM_MIN_TURN_RADIUS


class Teleop:
    def __init__(self, speed=0.1, max_speed=0.15, speed_step=0.02, min_speed=0.04,
                 turn_radius=0.5, first_hold=0.6, release=0.25, repeat_gap=0.15):
        if turn_radius < MIN_TURN_RADIUS:
            raise ValueError('turn_radius %.2f m is below the firmware minimum %.2f m'
                             % (turn_radius, MIN_TURN_RADIUS))
        if not 0 < min_speed <= speed <= max_speed:
            raise ValueError('need 0 < min_speed <= speed <= max_speed')
        if not 0 < release <= first_hold:
            raise ValueError('need 0 < release <= first_hold')
        self.speed = speed
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.speed_step = speed_step
        self.curvature = 1.0 / turn_radius
        self.first_hold = first_hold
        self.release = release
        self.repeat_gap = repeat_gap
        self.active = None        # key being held, or None
        self.last_key_t = None
        self.repeating = False

    def key(self, t, ch):
        """Handle one key at time t. Returns a short text to show, or None."""
        ch = ch.lower()
        if ch in STOP_KEYS:
            self.active = None
            return 'STOP'
        if ch in ('+', '='):
            self.speed = min(self.max_speed, round(self.speed + self.speed_step, 3))
            return 'speed %.2f m/s' % self.speed
        if ch in ('-', '_'):
            self.speed = max(self.min_speed, round(self.speed - self.speed_step, 3))
            return 'speed %.2f m/s' % self.speed
        if ch not in DRIVE_KEYS:
            return None
        if ch == self.active and self.last_key_t is not None \
                and t - self.last_key_t <= self.repeat_gap:
            self.repeating = True
        elif ch != self.active:
            self.repeating = False
        self.active = ch
        self.last_key_t = t
        return None

    def command(self, t):
        """(vx, wz) to publish at time t."""
        if self.active is None:
            return 0.0, 0.0
        hold = self.release if self.repeating else self.first_hold
        if t - self.last_key_t > hold:
            self.active = None
            self.repeating = False
            return 0.0, 0.0
        direction, steer = DRIVE_KEYS[self.active]
        vx = direction * self.speed
        return vx, vx * steer * self.curvature
