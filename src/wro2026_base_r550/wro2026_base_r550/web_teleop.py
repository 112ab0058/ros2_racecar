"""
Phone teleop: logic and web server (no ROS). web_teleop_node.py wraps it.

For driving without a laptop. The page (web_teleop_page.html) offers what the
vendor's WHEELTEC app offers (APP遥控用户手册 2026.6.30), but every command goes
through ROS and base_node, so its limits and timeouts apply and the bag gets
/cmd_vel_out:
  keys       left / straight / right forward, back: for training data
  joystick   8 directions, Ackermann table 3-1 of the manual: up forward,
             up-left/right turn, left/right full lock, down-left/right back
             turning, down back
  tilt       the app's "gravity" mode, same 8 directions, but only while the
             hold button is pressed (tilting alone never drives); needs https
  speed +/-  steps of 0.05 m/s, at most max_speed (<= 0.15)
  gauges     battery (10-12.6 V = 0-100 %, as the app), odometry speed, gyro z,
             base_node state, latency; waveform of gyro z and speed
  auto       (only when the node has an auto topic) hold the 自動 button and the car
             drives with the commands of the autonomous policy (bc_node); let go and
             it stops. The policy never reaches /cmd_vel by itself: this is its
             dead-man switch. Its commands older than auto_timeout count as stop, and
             they are clamped to 0..max_speed and a radius of at least 0.35 m.
Not offered: the app's parameter / PID page (it writes the STM32 flash).

Hold to drive: while a key, the joystick or the tilt hold button is pressed,
the page sends POST /cmd every 0.1 s. A command stays active only while the
newest request is younger than hold_timeout (0.35 s), so letting go, closing
the page, switching apps or losing WiFi all stop the car within hold_timeout.
The node keeps publishing 0 after that (base_node's 0.3 s timeout is the next
layer, the Motor switch the last one). Every request must carry the token
printed at start (?k=...), so other devices on the same network cannot drive.
With an ssl context the server speaks https only (see web_cert.py).
"""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
from urllib.parse import parse_qs, urlparse

with open(os.path.join(os.path.dirname(__file__), 'web_teleop_page.html'),
          encoding='utf-8') as _f:
    PAGE = _f.read()

# action -> (direction of vx, steering: +1 left / -1 right, full lock)
# "Left" means the front wheels steer left, also when reversing (as teleop.py).
ACTIONS = {
    'straight': (1, 0, False), 'left': (1, 1, False), 'right': (1, -1, False),
    'full_left': (1, 1, True), 'full_right': (1, -1, True),
    'back': (-1, 0, False), 'back_left': (-1, 1, False), 'back_right': (-1, -1, False),
}
AUTO = 'auto'
NAMES = {'straight': '直走', 'left': '左轉', 'right': '右轉', 'full_left': '左滿舵',
         'full_right': '右滿舵', 'back': '後退', 'back_left': '左後退',
         'back_right': '右後退', AUTO: '自動', None: '停'}
MIN_TURN_RADIUS = 0.35   # m, firmware MINI_AKM_MIN_TURN_RADIUS (= full lock)
MAX_SPEED = 0.15         # m/s, same limit as orin_map.sh
SPEED_STEP = 0.05        # m/s per +/- press
MIN_SPEED = 0.05


class WebTeleop:
    """Hold-to-drive state. Times are seconds from one monotonic clock."""

    def __init__(self, speed=0.1, turn_radius=0.5, hold_timeout=0.35, back_speed=0.08,
                 max_speed=None, auto=False, auto_timeout=0.3):
        """auto: accept the 自動 hold button (the node then has an auto topic)."""
        max_speed = speed if max_speed is None else max_speed
        if not (0.0 < speed <= max_speed <= MAX_SPEED and 0.0 < back_speed <= MAX_SPEED):
            raise ValueError('need 0 < speed <= max_speed <= %.2f m/s, 0 < back_speed <= %.2f'
                             % (MAX_SPEED, MAX_SPEED))
        if turn_radius < MIN_TURN_RADIUS:
            raise ValueError('turn_radius must be >= %.2f m' % MIN_TURN_RADIUS)
        if hold_timeout <= 0.0:
            raise ValueError('hold_timeout must be > 0')
        self.speed = speed
        self.max_speed = max_speed
        self.back_speed = back_speed
        self.turn_radius = turn_radius
        self.hold_timeout = hold_timeout
        self.lock = threading.Lock()
        self.action = None
        self.time = None
        self.requests = 0
        self.auto = auto
        self.auto_timeout = auto_timeout
        self.auto_cmd = None             # (time, vx, wz) from the policy

    def press(self, t, action):
        """Store one request from the page; 'stop' or anything unknown means stop."""
        with self.lock:
            self.requests += 1
            known = action in ACTIONS or (self.auto and action == AUTO)
            self.action = action if known else None
            self.time = t

    def set_auto(self, t, vx, wz):
        """Store the newest command of the autonomous policy."""
        with self.lock:
            self.auto_cmd = (t, vx, wz)

    def auto_command(self, t):
        """Policy command clamped to the limits, or (0, 0) if it is too old or missing."""
        with self.lock:
            cmd = self.auto_cmd
        if cmd is None or t - cmd[0] > self.auto_timeout:
            return 0.0, 0.0
        vx = min(max(cmd[1], 0.0), self.max_speed)       # forward only, at most max_speed
        limit = vx / MIN_TURN_RADIUS
        return vx, max(-limit, min(limit, cmd[2]))

    def change_speed(self, steps):
        """Change the forward speed by steps * SPEED_STEP, within MIN_SPEED..max_speed."""
        with self.lock:
            self.speed = round(min(self.max_speed, max(MIN_SPEED, self.speed
                                                       + steps * SPEED_STEP)), 3)
            return self.speed

    def current(self, t):
        """Return the action in force at time t, or None (stopped)."""
        with self.lock:
            if self.action is None or t - self.time > self.hold_timeout:
                return None
            return self.action

    def command(self, t):
        """Return (vx, wz) to publish at time t."""
        action = self.current(t)
        if action is None:
            return 0.0, 0.0
        if action == AUTO:
            return self.auto_command(t)
        direction, steer, full = ACTIONS[action]
        vx = self.speed if direction > 0 else -min(self.back_speed, self.speed)
        radius = MIN_TURN_RADIUS if full else self.turn_radius
        return vx, steer * vx / radius


def make_server(teleop, token, port, clock, status=dict, ssl_context=None, ca_cert=None):
    """
    Return a ThreadingHTTPServer for the page (call serve_forever in a thread).

    GET  /?k=TOKEN              the page
    GET  /state?k=TOKEN         JSON: action, vx, wz, speed, text, plus status()
    POST /cmd?k=TOKEN&a=ACT     press ACT (see ACTIONS, auto if enabled, or stop); JSON reply
    POST /speed?k=TOKEN&d=up    faster (d=down: slower); JSON reply
    GET  /ca.crt                the CA certificate file ca_cert (public, no token)
    ssl_context: an ssl.SSLContext for https (the server then speaks https only).
    """
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def _reply(self, code, body, kind, extra=()):
            data = body if isinstance(body, bytes) else body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            for name, value in extra:
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(data)

        def _parse(self):
            url = urlparse(self.path)
            query = parse_qs(url.query)
            ok = hmac.compare_digest(query.get('k', [''])[0].encode(), token.encode())
            if not ok:
                self._reply(403, 'wrong or missing ?k=', 'text/plain; charset=utf-8')
            return url.path, query, ok

        def _state(self):
            t = clock()
            action = teleop.current(t)
            vx, wz = teleop.command(t)
            body = dict(status(), action=action or 'stop', vx=round(vx, 3), wz=round(wz, 3),
                        speed=teleop.speed, max_speed=teleop.max_speed,
                        requests=teleop.requests, auto=teleop.auto,
                        text='%s  %.2f m/s' % (NAMES[action], vx))
            self._reply(200, json.dumps(body, ensure_ascii=False),
                        'application/json; charset=utf-8')

        def do_GET(self):
            if urlparse(self.path).path == '/ca.crt' and ca_cert:
                with open(ca_cert, 'rb') as f:
                    self._reply(200, f.read(), 'application/x-x509-ca-cert',
                                [('Content-Disposition', 'attachment; filename="r550-ca.crt"')])
                return
            path, _, ok = self._parse()
            if not ok:
                return
            if path == '/':
                self._reply(200, PAGE, 'text/html; charset=utf-8')
            elif path == '/state':
                self._state()
            else:
                self._reply(404, 'not found', 'text/plain; charset=utf-8')

        def do_POST(self):
            length = int(self.headers.get('Content-Length') or 0)
            if length:
                self.rfile.read(length)
            path, query, ok = self._parse()
            if not ok:
                return
            if path == '/cmd':
                teleop.press(clock(), query.get('a', ['stop'])[0])
            elif path == '/speed':
                teleop.change_speed(1 if query.get('d', [''])[0] == 'up' else -1)
            else:
                self._reply(404, 'not found', 'text/plain; charset=utf-8')
                return
            self._state()

    class Server(ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            pass    # e.g. a plain http request on the https port: drop it quietly

    server = Server(('0.0.0.0', port), Handler)
    if ssl_context is not None:
        # The TLS handshake runs in the request's thread, not in accept().
        server.socket = ssl_context.wrap_socket(server.socket, server_side=True,
                                                do_handshake_on_connect=False)
    return server
