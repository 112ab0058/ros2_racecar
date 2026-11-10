"""
Phone teleop: logic and web server (no ROS). web_teleop_node.py wraps it.

For driving without a laptop: the phone's browser shows big buttons. While a
button is held, the page sends POST /cmd every 0.1 s. The command stays active
only while the newest request is younger than hold_timeout (0.35 s), so
letting go, closing the page, switching apps or losing WiFi all stop the car
within hold_timeout. The node keeps publishing 0 after that (base_node's 0.3 s
timeout is the next layer, the Motor switch the last one).

One action at a time (the last one pressed). Forward runs at a fixed speed and
turn radius, so the training labels stay clean; back is straight back, slower.
Every request must carry the token printed at start (?k=...), so other devices
on the same network cannot drive the car.
"""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from urllib.parse import parse_qs, urlparse

# action -> (direction of vx, steering: +1 left, -1 right)
ACTIONS = {'left': (1, 1), 'straight': (1, 0), 'right': (1, -1), 'back': (-1, 0)}
NAMES = {'left': '左前', 'straight': '直走', 'right': '右前', 'back': '後退', None: '停'}
MIN_TURN_RADIUS = 0.35   # m, firmware MINI_AKM_MIN_TURN_RADIUS
MAX_SPEED = 0.15         # m/s, same limit as orin_map.sh


class WebTeleop:
    """Hold-to-drive state. Times are seconds from one monotonic clock."""

    def __init__(self, speed=0.1, turn_radius=0.5, hold_timeout=0.35, back_speed=0.08):
        if not (0.0 < speed <= MAX_SPEED and 0.0 < back_speed <= MAX_SPEED):
            raise ValueError('speed and back_speed must be in (0, %.2f] m/s' % MAX_SPEED)
        if turn_radius < MIN_TURN_RADIUS:
            raise ValueError('turn_radius must be >= %.2f m' % MIN_TURN_RADIUS)
        if hold_timeout <= 0.0:
            raise ValueError('hold_timeout must be > 0')
        self.speed = speed
        self.back_speed = back_speed
        self.turn_radius = turn_radius
        self.hold_timeout = hold_timeout
        self.lock = threading.Lock()
        self.action = None
        self.time = None
        self.requests = 0

    def press(self, t, action):
        """Store one request from the page; 'stop' or anything unknown means stop."""
        with self.lock:
            self.requests += 1
            self.action = action if action in ACTIONS else None
            self.time = t

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
        direction, steer = ACTIONS[action]
        if direction < 0:
            return -self.back_speed, 0.0
        return self.speed, steer * self.speed / self.turn_radius


def make_server(teleop, token, port, clock, status=dict):
    """
    Return a ThreadingHTTPServer for the page (call serve_forever in a thread).

    GET  /?k=TOKEN            the page
    GET  /state?k=TOKEN       JSON: action, vx, wz, text, plus status()
    POST /cmd?k=TOKEN&a=ACT   press ACT (left, straight, right, back, stop); JSON reply
    """
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def _reply(self, code, body, kind):
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
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
                        text='%s  %.2f m/s' % (NAMES[action], vx))
            self._reply(200, json.dumps(body, ensure_ascii=False),
                        'application/json; charset=utf-8')

        def do_GET(self):
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
            if path != '/cmd':
                self._reply(404, 'not found', 'text/plain; charset=utf-8')
                return
            teleop.press(clock(), query.get('a', ['stop'])[0])
            self._state()

    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    server.daemon_threads = True
    return server


PAGE = """<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport"
 content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>R550 遙控</title>
<style>
:root{--bg:#f4f4f2;--fg:#1d1d1b;--btn:#ffffff;--edge:#c9c9c4;--on:#1f6feb;--stop:#c62828}
@media (prefers-color-scheme:dark){:root{--bg:#161616;--fg:#eeeeea;--btn:#262626;
--edge:#3a3a3a;--on:#3b82f6;--stop:#ef5350}}
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
font:16px/1.4 system-ui,sans-serif;touch-action:none;overflow:hidden}
main{height:100%;display:flex;flex-direction:column;padding:16px;gap:12px}
#state{font-size:28px;font-weight:700;text-align:center;padding:12px;border-radius:12px;
border:2px solid var(--edge)}
#state.on{border-color:var(--on);color:var(--on)}
#state.bad{border-color:var(--stop);color:var(--stop)}
#info{text-align:center;opacity:.75;font-size:14px}
.grid{flex:1;display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:2fr 1fr;gap:12px}
button{font:inherit;font-size:22px;font-weight:700;border-radius:16px;border:2px solid var(--edge);
background:var(--btn);color:var(--fg);touch-action:none}
button.on{background:var(--on);border-color:var(--on);color:#fff}
#back{grid-column:2}
</style></head><body><main>
<div id="state">停</div>
<div class="grid">
<button data-a="left">◀ 左前</button><button data-a="straight">▲ 直走</button>
<button data-a="right">右前 ▶</button><button id="back" data-a="back">▼ 後退</button>
</div>
<div id="info">按住才會走，放開就停。緊急：Motor 開關 OFF</div>
</main><script>
const k = new URLSearchParams(location.search).get('k') || '';
const state = document.getElementById('state'), info = document.getElementById('info');
let timer = null, action = null, held = null;
function show(s) {
  state.textContent = s.text;
  state.className = s.action === 'stop' ? '' : 'on';
  info.textContent = '電池 ' + (s.battery ? s.battery.toFixed(2) + ' V' : '?') +
    '　按住才會走，放開就停。緊急：Motor 開關 OFF';
}
function lost() { state.textContent = '連線中斷：車子會自己停'; state.className = 'bad'; }
function send(a) {
  fetch('/cmd?k=' + k + '&a=' + a, {method: 'POST', cache: 'no-store'})
    .then(r => r.json()).then(show).catch(lost);
}
function start(a, btn) {
  stop();
  action = a; held = btn; btn.classList.add('on');
  send(a); timer = setInterval(() => send(action), 100);
}
function stop() {
  if (timer) { clearInterval(timer); timer = null; }
  if (held) { held.classList.remove('on'); held = null; }
  if (action) { action = null; send('stop'); }
}
for (const b of document.querySelectorAll('button')) {
  b.addEventListener('pointerdown', e => {
    e.preventDefault(); start(b.dataset.a, b);
    try { b.setPointerCapture(e.pointerId); } catch (err) { /* stop below still works */ }
  });
  b.addEventListener('lostpointercapture', () => { if (held === b) stop(); });
  b.addEventListener('contextmenu', e => e.preventDefault());
}
// Letting go anywhere stops, also when the finger slid off the button.
for (const ev of ['pointerup', 'pointercancel', 'touchend', 'touchcancel']) {
  document.addEventListener(ev, stop);
}
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
window.addEventListener('blur', stop);
window.addEventListener('pagehide', stop);
setInterval(() => {
  if (timer) return;
  fetch('/state?k=' + k, {cache: 'no-store'}).then(r => r.json()).then(show).catch(lost);
}, 1000);
</script></body></html>
"""
