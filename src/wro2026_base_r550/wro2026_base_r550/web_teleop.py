"""
Phone teleop: logic and web server (no ROS). web_teleop_node.py wraps it.

For driving without a laptop. The page offers what the vendor's WHEELTEC app
offers (APP遥控用户手册 2026.6.30), but every command goes through ROS and
base_node, so its limits and timeouts apply and the bag gets /cmd_vel_out:
  keys       left / straight / right forward, back: for training data
  joystick   8 directions, Ackermann table 3-1 of the manual: up forward,
             up-left/right turn, left/right full lock, down-left/right back
             turning, down back
  speed +/-  steps of 0.05 m/s, at most max_speed (<= 0.15)
  gauges     battery bar (10-12.6 V = 0-100 %, as the app), odometry speed,
             gyro z, base_node state; waveform of gyro z and speed
Not offered: tilt control (iPhone gives motion sensors to https pages only) and
the app's parameter / PID page (it writes the STM32 flash).

Hold to drive: while a key or the joystick is held, the page sends POST /cmd
every 0.1 s. A command stays active only while the newest request is younger
than hold_timeout (0.35 s), so letting go, closing the page, switching apps or
losing WiFi all stop the car within hold_timeout. The node keeps publishing 0
after that (base_node's 0.3 s timeout is the next layer, the Motor switch the
last one). Every request must carry the token printed at start (?k=...), so
other devices on the same network cannot drive the car.
"""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from urllib.parse import parse_qs, urlparse

# action -> (direction of vx, steering: +1 left / -1 right, full lock)
# "Left" means the front wheels steer left, also when reversing (as teleop.py).
ACTIONS = {
    'straight': (1, 0, False), 'left': (1, 1, False), 'right': (1, -1, False),
    'full_left': (1, 1, True), 'full_right': (1, -1, True),
    'back': (-1, 0, False), 'back_left': (-1, 1, False), 'back_right': (-1, -1, False),
}
NAMES = {'straight': '直走', 'left': '左轉', 'right': '右轉', 'full_left': '左滿舵',
         'full_right': '右滿舵', 'back': '後退', 'back_left': '左後退',
         'back_right': '右後退', None: '停'}
MIN_TURN_RADIUS = 0.35   # m, firmware MINI_AKM_MIN_TURN_RADIUS (= full lock)
MAX_SPEED = 0.15         # m/s, same limit as orin_map.sh
SPEED_STEP = 0.05        # m/s per +/- press
MIN_SPEED = 0.05


class WebTeleop:
    """Hold-to-drive state. Times are seconds from one monotonic clock."""

    def __init__(self, speed=0.1, turn_radius=0.5, hold_timeout=0.35, back_speed=0.08,
                 max_speed=None):
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

    def press(self, t, action):
        """Store one request from the page; 'stop' or anything unknown means stop."""
        with self.lock:
            self.requests += 1
            self.action = action if action in ACTIONS else None
            self.time = t

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
        direction, steer, full = ACTIONS[action]
        vx = self.speed if direction > 0 else -min(self.back_speed, self.speed)
        radius = MIN_TURN_RADIUS if full else self.turn_radius
        return vx, steer * vx / radius


def make_server(teleop, token, port, clock, status=dict):
    """
    Return a ThreadingHTTPServer for the page (call serve_forever in a thread).

    GET  /?k=TOKEN              the page
    GET  /state?k=TOKEN         JSON: action, vx, wz, speed, text, plus status()
    POST /cmd?k=TOKEN&a=ACT     press ACT (see ACTIONS, or stop); JSON reply
    POST /speed?k=TOKEN&d=up    faster (d=down: slower); JSON reply
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
                        speed=teleop.speed, max_speed=teleop.max_speed,
                        requests=teleop.requests, text='%s  %.2f m/s' % (NAMES[action], vx))
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
            if path == '/cmd':
                teleop.press(clock(), query.get('a', ['stop'])[0])
            elif path == '/speed':
                teleop.change_speed(1 if query.get('d', [''])[0] == 'up' else -1)
            else:
                self._reply(404, 'not found', 'text/plain; charset=utf-8')
                return
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
:root{--bg:#f4f4f2;--fg:#1d1d1b;--btn:#ffffff;--edge:#c9c9c4;--on:#1f6feb;--stop:#c62828;
--ok:#2e7d32;--warn:#e65100;--dim:#6b6b66}
@media (prefers-color-scheme:dark){:root{--bg:#161616;--fg:#eeeeea;--btn:#262626;
--edge:#3a3a3a;--on:#3b82f6;--stop:#ef5350;--ok:#66bb6a;--warn:#ffa726;--dim:#9a9a94}}
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
font:16px/1.35 system-ui,sans-serif;touch-action:none;overflow:hidden}
main{height:100%;display:flex;flex-direction:column;padding:12px 16px;gap:8px}
#state{font-size:24px;font-weight:700;text-align:center;padding:8px;border-radius:12px;
border:2px solid var(--edge)}
#state.on{border-color:var(--on);color:var(--on)}
#state.bad{border-color:var(--stop);color:var(--stop)}
#bat{position:relative;height:22px;border-radius:11px;border:1px solid var(--edge);overflow:hidden}
#batfill{position:absolute;inset:0 auto 0 0;width:0;background:var(--ok)}
#battxt{position:relative;display:block;text-align:center;font-size:13px;line-height:20px}
nav{display:flex;gap:6px}
nav button{flex:1;padding:8px 0;font-size:16px;border-radius:10px}
nav button.on{border-color:var(--on);color:var(--on)}
section{flex:1;display:none;min-height:0}
section.on{display:grid}
button{font:inherit;font-weight:700;border-radius:14px;border:2px solid var(--edge);
background:var(--btn);color:var(--fg);touch-action:none}
#keys{grid-template-columns:1fr 1fr 1fr;grid-template-rows:2fr 1fr;gap:10px}
#keys button{font-size:21px}
#keys button.held{background:var(--on);border-color:var(--on);color:#fff}
#back{grid-column:2}
#stick{place-items:center}
#pad{position:relative;width:min(78vw,40vh);aspect-ratio:1;border-radius:50%;
border:2px solid var(--edge);background:var(--btn);touch-action:none}
#pad i{position:absolute;font-style:normal;font-size:12px;color:var(--dim)}
#knob{position:absolute;left:50%;top:50%;width:26%;height:26%;margin:-13% 0 0 -13%;
border-radius:50%;background:var(--edge);pointer-events:none}
#knob.on{background:var(--on)}
#wave{grid-template-rows:1fr auto}
canvas{width:100%;height:100%;border:1px solid var(--edge);border-radius:10px}
.legend{font-size:13px;color:var(--dim);text-align:center}
#speedrow{display:grid;grid-template-columns:64px 1fr 64px;gap:8px;align-items:center}
#speedrow button{height:44px;font-size:24px}
#speed{text-align:center;font-size:18px;font-weight:700}
#speed.warn{color:var(--warn)}
#gauges{display:grid;grid-template-columns:1fr 1fr;gap:8px;text-align:center;font-size:13px}
#gauges b{display:block;font-size:18px}
#debug{font:12px/1.3 ui-monospace,monospace;color:var(--dim);text-align:center}
</style></head><body><main>
<div id="state">停</div>
<div id="bat"><div id="batfill"></div><span id="battxt">電池 ?</span></div>
<nav><button data-tab="keys" class="on">按鍵</button><button data-tab="stick">搖桿</button>
<button data-tab="wave">波形</button></nav>
<section id="keys" class="on">
<button class="drive" data-a="left">◀ 左前</button>
<button class="drive" data-a="straight">▲ 直走</button>
<button class="drive" data-a="right">右前 ▶</button>
<button class="drive" id="back" data-a="back">▼ 後退</button>
</section>
<section id="stick"><div id="pad">
<i style="left:46%;top:3%">前</i><i style="left:46%;bottom:3%">後</i>
<i style="left:3%;top:45%">左滿舵</i><i style="right:3%;top:45%">右滿舵</i><div id="knob"></div></div>
</section>
<section id="wave"><canvas id="plot"></canvas>
<div class="legend">上：角速度 Z（±1 rad/s）　下：速度（±0.2 m/s）　最近 10 秒</div></section>
<div id="speedrow"><button id="slower">－</button><div id="speed">0.10 m/s</div>
<button id="faster">＋</button></div>
<div id="gauges"><div>實際速度<b id="g_vx">-</b></div><div>角速度 Z<b id="g_gz">-</b></div></div>
<div id="debug">-</div>
</main><script>
const k = new URLSearchParams(location.search).get('k') || '';
const $ = id => document.getElementById(id);
const BASE = {ok: '正常', disabled: '未啟用（唯讀）', no_cmd: '等待指令', cmd_timeout: '指令逾時（停）',
  run_time_over: '時間到（停）', feedback_lost: '底盤沒回應（停）', battery_low: '電壓低（停）'};
const hist = [];
let timer = null, action = null, pointer = null, heldBtn = null;
const f = (v, d) => (v === null || v === undefined) ? '-' : v.toFixed(d);
function show(s) {
  $('state').textContent = s.text;
  $('state').className = s.action === 'stop' ? '' : 'on';
  const v = s.battery;
  const p = v ? Math.max(0, Math.min(1, (v - 10.0) / 2.6)) : 0;
  $('batfill').style.width = (100 * p) + '%';
  $('batfill').style.background = v && v < 11.0 ? 'var(--stop)' : 'var(--ok)';
  $('battxt').textContent = v ? '電池 ' + Math.round(100 * p) + ' %　' + v.toFixed(2) + ' V' : '電池 ?';
  $('speed').textContent = s.speed.toFixed(2) + ' m/s' + (Math.abs(s.speed - 0.1) > 1e-6 ?
    '（錄資料用 0.10）' : '');
  $('speed').className = Math.abs(s.speed - 0.1) > 1e-6 ? 'warn' : '';
  $('g_vx').textContent = f(s.odom_vx, 3) + ' m/s';
  $('g_gz').textContent = f(s.gyro_z, 2) + ' rad/s';
  $('debug').textContent = '送出 vx ' + s.vx.toFixed(2) + ' wz ' + s.wz.toFixed(2) +
    ' | 底盤 ' + (BASE[s.base] || s.base || '?') + ' | 請求 ' + s.requests;
  hist.push([performance.now() / 1000, s.gyro_z, s.odom_vx]);
  while (hist.length && hist[0][0] < hist[hist.length - 1][0] - 10) hist.shift();
  if ($('wave').classList.contains('on')) plot();
}
function lost() { $('state').textContent = '連線中斷：車子會自己停'; $('state').className = 'bad'; }
function post(path) {
  const url = path + (path.includes('?') ? '&' : '?') + 'k=' + k;
  return fetch(url, {method: 'POST', cache: 'no-store'})
    .then(r => r.json()).then(show).catch(lost);
}
function drive(a) {
  if (a === action) return;
  action = a;
  if (!a) { if (timer) { clearInterval(timer); timer = null; } post('/cmd?a=stop'); return; }
  post('/cmd?a=' + a);
  if (!timer) timer = setInterval(() => { if (action) post('/cmd?a=' + action); }, 100);
}
function release() {
  pointer = null;
  if (heldBtn) { heldBtn.classList.remove('held'); heldBtn = null; }
  $('knob').style.transform = ''; $('knob').classList.remove('on');
  drive(null);
}
for (const b of document.querySelectorAll('button.drive')) {
  b.addEventListener('pointerdown', e => {
    e.preventDefault(); release(); pointer = e.pointerId; heldBtn = b; b.classList.add('held');
    drive(b.dataset.a);
    try { b.setPointerCapture(e.pointerId); } catch (err) { /* release below still works */ }
  });
  b.addEventListener('contextmenu', e => e.preventDefault());
}
// joystick: 8 sectors of 45 deg, as the vendor app's Ackermann table
const SECTORS = ['full_right', 'right', 'straight', 'left', 'full_left', 'back_left', 'back',
  'back_right'];
function stick(e) {
  const r = $('pad').getBoundingClientRect(), R = r.width / 2;
  let dx = e.clientX - r.left - R, dy = r.top + R - e.clientY;
  const d = Math.hypot(dx, dy);
  if (d > R) { dx *= R / d; dy *= R / d; }
  $('knob').style.transform = 'translate(' + dx + 'px,' + (-dy) + 'px)';
  const sector = (Math.round(Math.atan2(dy, dx) / (Math.PI / 4)) + 8) % 8;
  const a = d < 0.25 * R ? null : SECTORS[sector];
  $('knob').classList.toggle('on', !!a);
  drive(a);
}
$('pad').addEventListener('pointerdown', e => {
  e.preventDefault(); release(); pointer = e.pointerId;
  try { $('pad').setPointerCapture(e.pointerId); } catch (err) { /* see release */ }
  stick(e);
});
$('pad').addEventListener('pointermove', e => { if (e.pointerId === pointer) stick(e); });
// Letting go anywhere stops, also when the finger slid off the key or pad.
for (const ev of ['pointerup', 'pointercancel']) {
  document.addEventListener(ev, e => { if (e.pointerId === pointer) release(); });
}
document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
window.addEventListener('blur', release);
window.addEventListener('pagehide', release);
for (const t of document.querySelectorAll('nav button')) {
  t.addEventListener('click', () => {
    release();
    for (const x of document.querySelectorAll('nav button, section')) x.classList.remove('on');
    t.classList.add('on'); $(t.dataset.tab).classList.add('on');
  });
}
$('faster').addEventListener('click', () => post('/speed?d=up'));
$('slower').addEventListener('click', () => post('/speed?d=down'));
function plot() {
  const c = $('plot'), dpr = window.devicePixelRatio || 1;
  c.width = c.clientWidth * dpr; c.height = c.clientHeight * dpr;
  const g = c.getContext('2d'), w = c.width, h = c.height, css = getComputedStyle(document.body);
  g.clearRect(0, 0, w, h);
  const now = hist.length ? hist[hist.length - 1][0] : 0;
  const lane = (i, scale, color) => {
    const mid = h * (i + 0.5) / 2, amp = h / 4 - 4 * dpr;
    g.strokeStyle = css.getPropertyValue('--edge').trim(); g.lineWidth = dpr;
    g.beginPath(); g.moveTo(0, mid); g.lineTo(w, mid); g.stroke();
    g.strokeStyle = css.getPropertyValue(color).trim(); g.lineWidth = 2 * dpr; g.beginPath();
    let first = true;
    for (const p of hist) {
      const v = p[i + 1];
      if (v === null || v === undefined) continue;
      const x = w * (1 - (now - p[0]) / 10), y = mid - amp * Math.max(-1, Math.min(1, v / scale));
      if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y);
    }
    g.stroke();
  };
  lane(0, 1.0, '--on'); lane(1, 0.2, '--warn');
}
setInterval(() => {
  if (timer) return;
  fetch('/state?k=' + k, {cache: 'no-store'}).then(r => r.json()).then(show).catch(lost);
}, 250);
</script></body></html>
"""
