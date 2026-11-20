#!/usr/bin/env python3
"""
Orin: launcher page, a fixed bookmark that starts an autonomous run from the phone.

    https://<orin>:8443/?k=<launcher token>      (token: ~/.r550_web/launcher_token)

Started at boot by tools/real/r550-launcher.service (systemd, see the README in that
file); by hand: python3 ~/racecar_ws/tools/orin_launcher.py (ROS sourced).

The page shows the battery, the state of the run and the last run's summary. Its
buttons start `orin_auto.sh <ccw|cw> 0.1 <seconds> <corner_ahead>`, the stack tested in
the air on 2026-11-20, and stop it. Pressing a start button next to the car IS the
confirmation orin_auto.sh asks for (CONFIRM=YES). The car still moves only while the
drive page's 自動 button is held, and every limit of orin_auto.sh stays (0.10 m/s,
cmd_timeout 0.3 s, run time, the too-close latch). Speed is fixed at 0.10 m/s here.

Both tokens are fixed (~/.r550_web/launcher_token, drive_token, made once), so the
bookmarks keep working; the drive page link uses the host the launcher was opened with.
HTTPS with the Orin's own CA (web_cert.py), like the drive page, so a phone that trusts
the CA trusts this page too; the certificate is made again when the Orin's addresses
change (home WiFi, iPhone hotspot, NetBird).

The battery is read straight from the STM32 (read only, 0.6 s) only while no other
program uses the port; a start is refused below 11.3 V.
"""
import html
import json
import os
import re
import secrets
import ssl
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import serial
from wro2026_base_r550.protocol import FeedbackStream
from wro2026_base_r550.web_cert import ensure_ca, local_addresses, make_server_cert

HOME = os.path.expanduser('~')
CERT_DIR = os.path.join(HOME, '.r550_web')
AUTO = os.path.join(HOME, 'racecar_ws', 'tools', 'orin_auto.sh')
MAP_DIR = os.path.join(HOME, 'r550_map')
STM32 = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'
PORT, DRIVE_PORT = 8443, 8080
DNS_NAMES = ('112ab-jno.netbird.cloud',)
MIN_BATTERY = 11.3
# the same programs orin_auto.sh refuses to start next to
BUSY = ('wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p|'
        'robot_state_publisher|foxglove_bridge|ekf_node|slam_toolbox|bc_node|wro2026_camera_c70')


def read_token(name):
    """Fixed token in ~/.r550_web/<name>, made once (starts with a letter: never a number)."""
    path = os.path.join(CERT_DIR, name)
    if not os.path.exists(path):
        os.makedirs(CERT_DIR, mode=0o700, exist_ok=True)
        with open(path, 'w') as f:
            f.write('r' + secrets.token_hex(4))
        os.chmod(path, 0o600)
    with open(path) as f:
        return f.read().strip()


def busy():
    """True when a ROS program of the car is running (it may hold the serial ports)."""
    return subprocess.run(['pgrep', '-f', BUSY], stdout=subprocess.DEVNULL).returncode == 0


def read_battery(seconds=0.6):
    """Median battery voltage from the STM32 feedback, None if nothing was read."""
    volts = []
    try:
        with serial.Serial(STM32, 115200, timeout=0.05) as port:
            stream = FeedbackStream()
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                volts += [f.voltage for f in stream.feed(port.read(port.in_waiting or 1))]
    except (serial.SerialException, OSError):
        return None
    return sorted(volts)[len(volts) // 2] if volts else None


class Run:
    """One orin_auto.sh started from the page."""

    def __init__(self, direction, seconds, ahead, drive_token):
        os.makedirs(MAP_DIR, exist_ok=True)
        self.log = os.path.join(MAP_DIR, 'launcher_%s.out' % time.strftime('%Y%m%d_%H%M%S'))
        self.args = (direction, seconds, ahead)
        self.started = time.time()
        self.stop_requested = None
        env = dict(os.environ, CONFIRM='YES', DRIVE_TOKEN=drive_token)
        with open(self.log, 'w') as out:
            self.proc = subprocess.Popen(
                ['bash', AUTO, direction, '0.1', str(seconds), '%.3f' % ahead],
                stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env,
                start_new_session=True)

    def text(self):
        try:
            with open(self.log, encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError:
            return ''

    def result_dir(self):
        m = re.search(r'RESULT_DIR=(\S+)', self.text())
        return m.group(1) if m else None

    def alive(self):
        return self.proc.poll() is None

    def state(self):
        if not self.alive():
            return 'ended'
        if self.stop_requested:
            return 'stopping'
        return 'ready' if 'READY:' in self.text() else 'starting'

    def stop(self):
        """Ask orin_auto.sh to stop (its stop file); SIGTERM it if it is still there after 15 s."""
        self.stop_requested = time.time()
        d = self.result_dir()
        if d:
            open(os.path.join(d, 'stop'), 'w').close()
        else:
            self.proc.terminate()

        def later():
            time.sleep(15)
            if self.alive():
                self.proc.terminate()
        threading.Thread(target=later, daemon=True).start()


def last_run_summary():
    """A few lines about the newest ~/r550_map/*_auto run (Chinese, for the page)."""
    dirs = sorted(d for d in os.listdir(MAP_DIR) if d.endswith('_auto')) if os.path.isdir(
        MAP_DIR) else []
    if not dirs:
        return []
    d = os.path.join(MAP_DIR, dirs[-1])

    def read(name):
        try:
            with open(os.path.join(d, name), encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError:
            return ''
    bc, web = read('bc.log'), read('web.log')
    lines = ['%s' % dirs[-1],
             '按住「自動」%d 次，轉彎 %d 次' % (web.count('phone: 自動'), bc.count(' turn at '))]
    states = re.findall(r'state: (\w+)(?: \(([^)]*)\))?', bc)
    if states:
        name, why = states[-1]
        lines.append('最後的狀態：%s' % ('正常' if name == 'ok' else why or name))
    stopped = [w for name, w in states if name == 'too_close']
    if stopped:
        lines.append('太靠近停車：%s' % stopped[-1])
    return lines


class Launcher:
    """State shared by the HTTP handler threads."""

    def __init__(self):
        self.token = read_token('launcher_token')
        self.drive_token = read_token('drive_token')
        self.run = None
        self.lock = threading.Lock()
        self.battery = (None, 0.0)          # (volts, monotonic time read)

    def status(self, host):
        with self.lock:
            run = self.run
            other = (run is None or not run.alive()) and busy()
            if (run is None or not run.alive()) and not other \
                    and time.monotonic() - self.battery[1] > 10:
                self.battery = (read_battery(), time.monotonic())
        st = 'other' if other else (run.state() if run else 'idle')
        tail = [line for line in (run.text() if run else '').splitlines()
                if not line.startswith('PHONE_URL')][-6:]
        return {'state': st, 'battery': self.battery[0],
                'battery_age': round(time.monotonic() - self.battery[1]),
                'args': run.args if run else None,
                'drive_url': 'https://%s:%d/?k=%s' % (host, DRIVE_PORT, self.drive_token),
                'tail': tail, 'last': last_run_summary(), 'min_battery': MIN_BATTERY}

    def start(self, direction, seconds, ahead):
        with self.lock:
            if self.run is not None and self.run.alive():
                return '已經在跑了'
            if busy():
                return 'Orin 上有其他車子的程式在跑，先停掉它們'
            volts = read_battery()
            self.battery = (volts, time.monotonic())
            if volts is None:
                return '讀不到電池電壓（STM32 沒接好或沒開電源）'
            if volts < MIN_BATTERY:
                return '電池 %.2f V，低於 %.1f V，先充電' % (volts, MIN_BATTERY)
            self.run = Run(direction, seconds, ahead, self.drive_token)
            return None

    def stop(self):
        with self.lock:
            if self.run is None or not self.run.alive():
                return '沒有在跑'
            self.run.stop()
            return None


PAGE = """<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>R550 啟動頁</title><style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#f6f5f2;color:#111}
h1{font-size:20px;margin:0 0 12px}.card{background:#fff;border-radius:12px;padding:14px;
margin-bottom:12px;box-shadow:0 1px 3px #0002}.big{font-size:22px;font-weight:600}
button{font-size:18px;padding:14px;border-radius:10px;border:0;width:100%;margin:6px 0}
.go{background:#1f6feb;color:#fff}.stop{background:#d1242f;color:#fff}
button:disabled{background:#bbb}.low{color:#d1242f}a.drive{display:block;text-align:center;
font-size:20px;font-weight:600;background:#1a7f37;color:#fff;padding:16px;border-radius:10px;
text-decoration:none}pre{white-space:pre-wrap;font-size:12px;color:#555;margin:0}
label{display:block;margin:6px 0}input{font-size:16px;width:90px}
</style></head><body><h1>R550 啟動頁</h1>
<div class="card"><div>電池</div><div id="bat" class="big">讀取中…</div></div>
<div class="card"><div>狀態</div><div id="st" class="big">…</div><div id="drive"></div>
<pre id="tail"></pre></div>
<div class="card" id="startcard">
<button class="go" onclick="go('ccw')">逆時針（左轉）開始</button>
<button class="go" onclick="go('cw')">順時針（右轉）開始</button>
<details><summary>設定</summary>
<label>時間（秒，10–600）<input id="secs" type="number" value="120"></label>
<label>corner_ahead（m）<input id="ahead" type="number" step="0.01" value="0.15"></label>
</details></div>
<div class="card"><button class="stop" onclick="stopRun()">停止</button>
<div>停止 = 讓程式正常結束（存地圖、關掉全部）。緊急時直接關 Motor 開關。</div></div>
<div class="card"><div>上一次</div><pre id="last"></pre></div>
<script>
const K = new URLSearchParams(location.search).get('k');
const NAMES = {idle:'待機', starting:'啟動中（約 20 秒，車子不要動）', ready:'可以開了：打開駕駛頁，按住「自動」',
  stopping:'停止中…', ended:'已結束', other:'Orin 上有別的車子程式在跑（不是從這頁開的）'};
async function api(path, opts) {
  const r = await fetch(path + (path.includes('?') ? '&' : '?') + 'k=' + K, opts);
  return r.json();
}
async function tick() {
  try {
    const s = await api('/status');
    const b = document.getElementById('bat');
    b.textContent = s.battery == null ? '讀不到（程式在跑時不讀）' : s.battery.toFixed(2) + ' V';
    b.className = 'big' + (s.battery != null && s.battery < s.min_battery ? ' low' : '');
    document.getElementById('st').textContent = (NAMES[s.state] || s.state) +
      (s.args ? '　' + (s.args[0] == 'ccw' ? '逆時針' : '順時針') + ' ' + s.args[1] + ' 秒' : '');
    document.getElementById('drive').innerHTML = (s.state == 'ready' || s.state == 'starting') ?
      '<a class="drive" href="' + s.drive_url + '">打開駕駛頁</a>' : '';
    document.getElementById('tail').textContent = s.tail.join('\\n');
    document.getElementById('last').textContent = s.last.join('\\n');
    const running = ['starting', 'ready', 'stopping', 'other'].includes(s.state);
    document.querySelectorAll('.go').forEach(x => x.disabled = running);
  } catch (e) { document.getElementById('st').textContent = '連不到 Orin'; }
}
async function go(dir) {
  const secs = document.getElementById('secs').value, ahead = document.getElementById('ahead').value;
  if (!confirm((dir == 'ccw' ? '逆時針' : '順時針') + ' ' + secs + ' 秒，0.10 m/s。\\n' +
      '有人在 Motor 開關旁、周圍淨空？\\n車子只在按住「自動」時會動。')) return;
  const r = await api('/start?dir=' + dir + '&secs=' + secs + '&ahead=' + ahead, {method:'POST'});
  if (r.error) alert(r.error);
  tick();
}
async function stopRun() {
  const r = await api('/stop', {method:'POST'});
  if (r.error) alert(r.error);
  tick();
}
tick(); setInterval(tick, 2000);
</script></body></html>
"""


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype):
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False), 'application/json')

        def _query(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            if not secrets.compare_digest(q.get('k', ''), app.token):
                self._send(403, 'wrong or missing ?k=', 'text/plain; charset=utf-8')
                return None, None
            return url.path, q

        def do_GET(self):
            path, q = self._query()
            if path is None:
                return
            host = (self.headers.get('Host') or 'localhost').rsplit(':', 1)[0]
            if path == '/':
                self._send(200, PAGE, 'text/html; charset=utf-8')
            elif path == '/status':
                self._json(app.status(html.escape(host)))
            else:
                self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            path, q = self._query()
            if path is None:
                return
            if path == '/start':
                try:
                    direction = q['dir']
                    seconds = int(q.get('secs', 120))
                    ahead = float(q.get('ahead', 0.15))
                except (KeyError, ValueError):
                    return self._json({'error': '參數不對'}, 400)
                if direction not in ('ccw', 'cw') or not 10 <= seconds <= 600 \
                        or not -0.10 <= ahead <= 0.25:
                    return self._json({'error': '方向要 ccw/cw，時間 10–600 秒，'
                                       'corner_ahead -0.10–0.25'}, 400)
                self._json({'error': app.start(direction, seconds, ahead)})
            elif path == '/stop':
                self._json({'error': app.stop()})
            else:
                self._send(404, 'not found', 'text/plain')
    return Handler


def tls_context():
    """Context with a certificate for the current addresses; also returns them."""
    ensure_ca(CERT_DIR)
    own = os.path.join(CERT_DIR, 'launcher')       # own server.* (the drive page has its own)
    os.makedirs(own, mode=0o700, exist_ok=True)
    for name in ('ca.crt', 'ca.key'):
        link = os.path.join(own, name)
        if not os.path.lexists(link):
            os.symlink(os.path.join(CERT_DIR, name), link)
    addresses = sorted(local_addresses())
    crt, key = make_server_cert(own, addresses, DNS_NAMES)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(crt, key)
    return ctx, own, addresses


def main():
    app = Launcher()
    ctx, own, addresses = tls_context()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(app))
    server.daemon_threads = True
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    def renew():
        nonlocal addresses
        while True:
            time.sleep(30)
            try:
                now = sorted(local_addresses())
                if now and now != addresses:
                    crt, key = make_server_cert(own, now, DNS_NAMES)
                    ctx.load_cert_chain(crt, key)
                    addresses = now
                    print('certificate renewed for', ' '.join(now), flush=True)
            except (subprocess.CalledProcessError, ValueError, OSError) as e:
                print('certificate renewal failed:', e, flush=True)
    threading.Thread(target=renew, daemon=True).start()
    for a in addresses:
        print('launcher page: https://%s:%d/?k=%s' % (a, PORT, app.token), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
