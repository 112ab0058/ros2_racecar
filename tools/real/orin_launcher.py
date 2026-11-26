#!/usr/bin/env python3
"""
Orin: launcher page, a fixed bookmark that starts an autonomous run from the phone.

    https://<orin>:8443/?k=<launcher token>      (token: ~/.r550_web/launcher_token)

Started at boot by tools/real/r550-launcher.service (systemd, see the README in that
file); by hand: python3 ~/racecar_ws/tools/orin_launcher.py (ROS sourced).

The page shows the battery, the state of the run and the last run's summary. Its
buttons start `orin_auto.sh <auto|ccw|cw> 0.1 <seconds> <corner_ahead> <ahead|predict>
<laps>` (設定: time 180 s, laps 3 = stop at the start after three laps), the stack tested in
the air on 2026-11-20, and stop it. Pressing a start button next to the car IS the
confirmation orin_auto.sh asks for (CONFIRM=YES). The car still moves only while the
drive page's 自動 button is held, and every limit of orin_auto.sh stays (0.10 m/s,
cmd_timeout 0.3 s, run time, the too-close latch). Speed is fixed at 0.10 m/s here.

Two switches pick the modes (2026-11-24, Ray: every new mode must switch back to the tested
one with one tap): 方向 自動判斷 (one start button, direction auto) / 手動選 (逆時針 and
順時針 buttons), and 打滿時機 預測式 (timing predict) / 固定距離 (corner_ahead). The phone
remembers the last choice (localStorage); the first default is the tested pair, 手動選 +
固定距離.

Imitation learning (2026-11-25): a third switch 駕駛 picks 規則 (the corner technique, the
default) or 網路 (the network ~/r550_map/model.npz drives: orin_auto.sh policy model; one
start button, no direction or timing). The 訓練 button runs orin_train.sh (reads the
recorded runs, writes the model; no motors) while nothing else runs. Every run started
here records the data the network learns from: drive with the drive page's keys
(Ray's driving) or hold 自動 (the rule's driving).

Both tokens are fixed (~/.r550_web/launcher_token, drive_token, made once), so the
bookmarks keep working; the drive page link uses the host the launcher was opened with.
HTTPS with the Orin's own CA (web_cert.py), like the drive page, so a phone that trusts
the CA trusts this page too; the certificate is made again when the Orin's addresses
change (home WiFi, iPhone hotspot, NetBird).

The battery is read straight from the STM32 (read only, 0.6 s) only while no other
program uses the port; a start is refused below MIN_BATTERY (11.0 V).
"""
import html
import json
import os
import re
import secrets
import socket
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
TRAIN = os.path.join(HOME, 'racecar_ws', 'tools', 'orin_train.sh')
MAP_DIR = os.path.join(HOME, 'r550_map')
MODEL = os.path.join(MAP_DIR, 'model.npz')
STM32 = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'
PORT, DRIVE_PORT = 8443, 8080
# 112ab-jno.local: the Orin by name on any WiFi (avahi / mDNS; iPhone resolves .local),
# so a bookmark works whatever address the hotspot gives the Orin
DNS_NAMES = ('112ab-jno.netbird.cloud', '112ab-jno.local')
# 11.0 V for these short runs (Ray, 2026-11-20 at the field): 120 s at 0.10 m/s took
# 0.1-0.2 V; the STM32 cuts the motors at 10 V, the pack's own cut-off is 9 V.
# 11.3 V stays the rule for long (soak) tests.
MIN_BATTERY = 11.0
# the same programs orin_auto.sh refuses to start next to
BUSY = ('wro2026_base_r550/(feedback_node|base_node|web_teleop_node)|wro2026_lidar_n10p|'
        'robot_state_publisher|foxglove_bridge|ekf_node|slam_toolbox|bc_node|wro2026_camera_c70')
DIRECTION_NAMES = {'auto': '自動判斷方向', 'ccw': '逆時針', 'cw': '順時針'}
TIMING_NAMES = {'predict': '預測式打滿', 'ahead': '固定距離打滿'}
POLICY_NAMES = {'corner': '規則', 'model': '網路'}


def model_info():
    """Chinese lines about ~/r550_map/model.npz (what policy model drives), [] if none."""
    try:
        import numpy as np
        with np.load(MODEL) as d:
            meta = json.loads(str(d['meta']))
    except (OSError, ValueError, KeyError):
        return []
    full = len(meta.get('labels', ())) == 5
    lines = ['模型：%s 訓練，%s 筆資料（%s）' % (
        meta.get('trained', '?'), meta.get('samples', '?'),
        {'all': '人和規則', 'hand': '只有人開的', 'rule': '只有規則開的'}.get(
            meta.get('source', 'hand'), meta.get('source')))]
    if 'val_accuracy' in meta:
        lines.append('驗證：準確率 %.0f %%，各動作平均 %.0f %%%s' % (
            100.0 * meta['val_accuracy'], 100.0 * meta.get('val_balanced', float('nan')),
            '' if full else '（舊模型，不會打滿）'))
    return lines


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

    def __init__(self, direction, seconds, ahead, timing, laps, drive_token, policy='corner'):
        os.makedirs(MAP_DIR, exist_ok=True)
        self.log = os.path.join(MAP_DIR, 'launcher_%s.out' % time.strftime('%Y%m%d_%H%M%S'))
        self.args = (direction, seconds, ahead, timing, laps, policy)
        self.started = time.time()
        self.stop_requested = None
        env = dict(os.environ, CONFIRM='YES', DRIVE_TOKEN=drive_token)
        with open(self.log, 'w') as out:
            self.proc = subprocess.Popen(
                ['bash', AUTO, direction, '0.1', str(seconds), '%.3f' % ahead, timing,
                 str(laps), policy],
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


class Training:
    """One orin_train.sh started from the page (reads files, no motors)."""

    def __init__(self):
        self.log = os.path.join(MAP_DIR, 'train_%s.out' % time.strftime('%Y%m%d_%H%M%S'))
        with open(self.log, 'w') as out:
            self.proc = subprocess.Popen(['bash', TRAIN], stdout=out, stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL, start_new_session=True)

    def alive(self):
        return self.proc.poll() is None

    def text(self):
        try:
            with open(self.log, encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError:
            return ''


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
    # 'turn at' (timing ahead) or 'turn (room ...) at' (timing predict)
    turns = len(re.findall(r'corner policy: turn\b', bc))
    lines = ['%s' % dirs[-1],
             '按住「自動」%d 次，轉彎 %d 次' % (web.count('phone: 自動'), turns)]
    if re.search(r'policy model \(', bc):
        lines.append('模式：網路（模仿學習）')
    mode = re.search(r'policy corner \((auto|ccw|cw), timing (\w+)', bc)
    if mode:
        lines.append('模式：%s，%s' % (DIRECTION_NAMES[mode.group(1)],
                                     TIMING_NAMES.get(mode.group(2), mode.group(2))))
    picked = re.search(r'corner policy: direction (ccw|cw) at ([\d.]+) m', bc)
    if picked:
        lines.append('自動判斷方向：%s（開出 %s m 時決定）'
                     % (DIRECTION_NAMES[picked.group(1)], picked.group(2)))
    elif mode and mode.group(1) == 'auto':
        lines.append('自動判斷方向：還沒決定')
    if 'corner policy: done' in bc:
        lines.append('跑完設定的圈數，停在起點')
    blocked = len(re.findall(r'corner policy: blocked\b', bc))
    if blocked:
        lines.append('過不了彎、停下來 %d 次（車太靠島或太靠近彎道）：倒車或把車放回走道中間'
                     % blocked)
    states = re.findall(r'state: (\w+)(?: \(([^)]*)\))?', bc)
    if states:
        name, why = states[-1]
        lines.append('最後的狀態：%s' % ('正常' if name == 'ok' else why or name))
    stopped = [w for name, w in states if name == 'too_close']
    if stopped:
        lines.append('太靠近停車：%s' % stopped[-1])
    if lost_mid_run(bc):
        lines.append('中途沒有光達／里程計資料（通訊停住？）：按停止再開一次')
    if not run_alive(d) and not os.path.exists(os.path.join(d, 'map.yaml')):
        lines.append('地圖沒有存到')
    return lines


def lost_mid_run(bc):
    """
    True if scans or odometry stopped after the run had started (2026-11-20 on the hotspot:
    every topic of the launch stopped at once). Every 重置 also passes through no_scan for
    about 0.1 s (bc_node waits for a new scan; 2026-11-26 air test), so only a gap longer
    than 2 s, or one that never ended, counts.
    """
    timed = re.findall(r'\[(\d+\.\d+)\] \[bc_node\]: state: (\w+)', bc)
    started = False
    for i, (t, name) in enumerate(timed):
        if name == 'ok':
            started = True
        elif started and name in ('no_scan', 'no_odom'):
            if i + 1 == len(timed) or float(timed[i + 1][0]) - float(t) > 2.0:
                return True
    return False


def run_alive(d):
    """True while the run's orin_auto.sh (script.pid) is still running."""
    try:
        with open(os.path.join(d, 'script.pid')) as f:
            os.kill(int(f.read().strip()), 0)
        return True
    except (OSError, ValueError):
        return False


class Launcher:
    """State shared by the HTTP handler threads."""

    def __init__(self):
        self.token = read_token('launcher_token')
        self.drive_token = read_token('drive_token')
        self.run = None
        self.training = None
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
        training = self.training
        if training is not None and training.alive() and st in ('idle', 'ended'):
            st = 'training'
        return {'state': st, 'battery': self.battery[0],
                'battery_age': round(time.monotonic() - self.battery[1]),
                'args': run.args if run else None,
                'drive_url': 'https://%s:%d/?k=%s' % (host, DRIVE_PORT, self.drive_token),
                'tail': tail, 'last': last_run_summary(), 'min_battery': MIN_BATTERY,
                'model': model_info(),
                'train_tail': training.text().splitlines()[-8:] if training else []}

    def train(self):
        with self.lock:
            if self.training is not None and self.training.alive():
                return '已經在訓練了'
            if (self.run is not None and self.run.alive()) or busy():
                return '車子的程式在跑，先按停止再訓練'
            self.training = Training()
            return None

    def start(self, direction, seconds, ahead, timing='ahead', laps=3, policy='corner'):
        with self.lock:
            if self.run is not None and self.run.alive():
                return '已經在跑了'
            if self.training is not None and self.training.alive():
                return '正在訓練，等它結束'
            if policy == 'model' and not os.path.exists(MODEL):
                return '還沒有模型：先按「訓練」'
            if busy():
                return 'Orin 上有其他車子的程式在跑，先停掉它們'
            volts = read_battery()
            self.battery = (volts, time.monotonic())
            if volts is None:
                return '讀不到電池電壓（STM32 沒接好或沒開電源）'
            if volts < MIN_BATTERY:
                return '電池 %.2f V，低於 %.1f V，先充電' % (volts, MIN_BATTERY)
            self.run = Run(direction, seconds, ahead, timing, laps, self.drive_token, policy)
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
.train{background:#6639ba;color:#fff}
button:disabled{background:#bbb}.low{color:#d1242f}a.drive{display:block;text-align:center;
font-size:20px;font-weight:600;background:#1a7f37;color:#fff;padding:16px;border-radius:10px;
text-decoration:none}pre{white-space:pre-wrap;font-size:12px;color:#555;margin:0}
label{display:block;margin:6px 0}input{font-size:16px;width:90px}
.mode{margin:4px 0 8px}.seg{display:flex;gap:6px}.seg button{flex:1;font-size:16px;
padding:11px 6px;margin:4px 0;background:#e6e4df;color:#111}.seg button.on{background:#111;
color:#fff}.hint{font-size:13px;color:#8a5a00;margin:2px 0 4px}[hidden]{display:none}
</style></head><body><h1>R550 啟動頁</h1>
<div class="card"><div>電池</div><div id="bat" class="big">讀取中…</div></div>
<div class="card"><div>狀態</div><div id="st" class="big">…</div><div id="drive"></div>
<pre id="tail"></pre></div>
<div class="card" id="startcard">
<div class="mode"><div>駕駛</div><div class="seg">
<button id="pol_corner" onclick="setMode('pol', 'corner')">規則（轉彎技巧）</button>
<button id="pol_model" onclick="setMode('pol', 'model')">網路（模仿學習）</button></div>
<div class="hint" id="pol_hint" hidden>網路駕駛還沒在地上跑過；不對就放開自動、按停止，切回「規則」。網路不會數圈，時間到才停。</div>
<pre id="model"></pre></div>
<div id="rule_modes">
<div class="mode"><div>方向</div><div class="seg">
<button id="dir_auto" onclick="setMode('dir', 'auto')">自動判斷</button>
<button id="dir_hand" onclick="setMode('dir', 'hand')">手動選</button></div>
<div class="hint" id="dir_hint" hidden>自動判斷還沒在場地驗證過；選錯就按停止，切回「手動選」</div></div>
<div class="mode"><div>打滿時機</div><div class="seg">
<button id="timing_predict" onclick="setMode('timing', 'predict')">預測式</button>
<button id="timing_ahead" onclick="setMode('timing', 'ahead')">固定距離</button></div>
<div class="hint" id="timing_hint" hidden>預測式還沒在地上跑過；不對就按停止，切回「固定距離」</div></div>
</div>
<div class="hint">放車：整台車在任一段直道裡、車頭朝要跑的方向（照 WRO 規則）。逆時針時，直道後半段不要貼著島放，也不要車頭朝外牆。</div>
<button class="go" id="go_auto" onclick="go('auto')">開始（自動判斷方向）</button>
<button class="go" id="go_ccw" onclick="go('ccw')">逆時針（左轉）開始</button>
<button class="go" id="go_cw" onclick="go('cw')">順時針（右轉）開始</button>
<button class="go" id="go_model" onclick="go('auto')">網路駕駛開始</button>
<details><summary>設定</summary>
<label>時間（秒，10–600；3 圈約 150 秒）<input id="secs" type="number" value="180"></label>
<label>圈數（跑完停在起點；0＝不停）<input id="laps" type="number" min="0" max="10" value="3"></label>
<label>corner_ahead（m，固定距離才用）<input id="ahead" type="number" step="0.01" value="0.15"></label>
</details></div>
<div class="card"><button class="stop" onclick="stopRun()">停止</button>
<div>停止 = 讓程式正常結束（存地圖、關掉全部）。緊急時直接關 Motor 開關。</div></div>
<div class="card"><div>上一次</div><pre id="last"></pre></div>
<div class="card"><div>模仿學習</div>
<div class="hint">每次按「開始」都會錄資料：在駕駛頁用「按鍵」自己開＝人的示範，按住「自動」＝規則的示範。
架空的紀錄（輪子在轉、光達看到的不動）和撞牆停車前 1.5 秒會自動排除。</div>
<button class="train" id="train" onclick="train()">用全部紀錄訓練網路（不會動馬達，約幾分鐘）</button>
<pre id="train_tail"></pre></div>
<script>
const K = new URLSearchParams(location.search).get('k');
const NAMES = {idle:'待機', starting:'啟動中（約 20 秒，車子不要動）', ready:'可以開了：打開駕駛頁，按住「自動」',
  stopping:'停止中…', ended:'已結束', other:'Orin 上有別的車子程式在跑（不是從這頁開的）',
  training:'訓練網路中（馬達不會動）'};
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
      (s.args && s.args[5] == 'model' ? '　網路駕駛 ' + s.args[1] + ' 秒' :
       s.args ? '　' + DIR_NAMES[s.args[0]] + ' ' + s.args[1] + ' 秒　' +
       (s.args[3] == 'predict' ? '預測式' : '固定距離 ' + s.args[2] + ' m') + '　' +
       (s.args[4] ? s.args[4] + ' 圈' : '不限圈數') : '');
    document.getElementById('model').textContent = s.model.length ? s.model.join('\\n') :
      '還沒有模型：先在下面按「訓練」';
    document.getElementById('train_tail').textContent = s.train_tail.join('\\n');
    document.getElementById('drive').innerHTML = s.state == 'ready' ?
      '<a class="drive" href="' + s.drive_url + '">打開駕駛頁</a>' : '';
    document.getElementById('tail').textContent = s.tail.join('\\n');
    document.getElementById('last').textContent = s.last.join('\\n');
    const running = ['starting', 'ready', 'stopping', 'other', 'training'].includes(s.state);
    document.querySelectorAll('.go').forEach(x => x.disabled = running);
    document.getElementById('train').disabled = running;
    if (!s.model.length) document.getElementById('go_model').disabled = true;
  } catch (e) { document.getElementById('st').textContent = '連不到 Orin'; }
}
// mode switches: the phone remembers them; first default = the tested pair
const DIR_NAMES = {auto: '自動判斷方向', ccw: '逆時針', cw: '順時針'};
const mode = {dir: 'hand', timing: 'ahead', pol: 'corner'};
function load(key, ok) {
  try { const v = localStorage.getItem('r550_' + key); if (ok.includes(v)) mode[key] = v; }
  catch (e) {}
}
function setMode(key, value) {
  mode[key] = value;
  try { localStorage.setItem('r550_' + key, value); } catch (e) {}
  show();
}
function show() {
  for (const v of ['auto', 'hand']) document.getElementById('dir_' + v).className =
    mode.dir == v ? 'on' : '';
  for (const v of ['predict', 'ahead']) document.getElementById('timing_' + v).className =
    mode.timing == v ? 'on' : '';
  for (const v of ['corner', 'model']) document.getElementById('pol_' + v).className =
    mode.pol == v ? 'on' : '';
  const net = mode.pol == 'model';
  document.getElementById('pol_hint').hidden = !net;
  document.getElementById('model').hidden = !net;
  document.getElementById('rule_modes').hidden = net;
  document.getElementById('dir_hint').hidden = mode.dir != 'auto';
  document.getElementById('timing_hint').hidden = mode.timing != 'predict';
  document.getElementById('go_auto').hidden = net || mode.dir != 'auto';
  document.getElementById('go_ccw').hidden = net || mode.dir == 'auto';
  document.getElementById('go_cw').hidden = net || mode.dir == 'auto';
  document.getElementById('go_model').hidden = !net;
}
load('dir', ['auto', 'hand']); load('timing', ['predict', 'ahead']);
load('pol', ['corner', 'model']); show();
async function go(dir) {
  const secs = document.getElementById('secs').value, ahead = document.getElementById('ahead').value;
  const timing = mode.timing, laps = document.getElementById('laps').value, pol = mode.pol;
  const what = pol == 'model' ? '網路駕駛 ' + secs + ' 秒，0.10 m/s，不數圈（時間到才停）' :
    DIR_NAMES[dir] + ' ' + secs + ' 秒，0.10 m/s，' +
    (timing == 'predict' ? '預測式打滿' : '固定距離打滿 ' + ahead + ' m') + '，' +
    (laps > 0 ? laps + ' 圈後停在起點' : '不限圈數');
  if (!confirm(what + '。\\n' +
      '有人在 Motor 開關旁、周圍淨空？\\n車子只在按住「自動」時會動。')) return;
  const r = await api('/start?dir=' + dir + '&secs=' + secs + '&ahead=' + ahead +
                      '&timing=' + timing + '&laps=' + laps + '&policy=' + pol, {method:'POST'});
  if (r.error) alert(r.error);
  tick();
}
async function train() {
  if (!confirm('用 ~/r550_map 裡全部的自動駕駛紀錄訓練網路？\\n只讀檔案，馬達不會動；訓練時不能開車。')) return;
  const r = await api('/train', {method:'POST'});
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
            # the drive page listens on IPv4 only: link to the IPv4 address this phone
            # reached us on (a .local name may resolve to IPv6 on the phone and fail)
            host = self.connection.getsockname()[0]
            if host.startswith('::ffff:'):
                host = host[len('::ffff:'):]
            if ':' in host:
                host = (local_addresses() or ['127.0.0.1'])[0]
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
                    timing = q.get('timing', 'ahead')
                    laps = int(q.get('laps', 3))
                    policy = q.get('policy', 'corner')
                except (KeyError, ValueError):
                    return self._json({'error': '參數不對'}, 400)
                if direction not in DIRECTION_NAMES or not 10 <= seconds <= 600 \
                        or not -0.10 <= ahead <= 0.25 or timing not in TIMING_NAMES \
                        or not 0 <= laps <= 10 or policy not in POLICY_NAMES:
                    return self._json({'error': '方向要 auto/ccw/cw，時間 10–600 秒，'
                                       'corner_ahead -0.10–0.25，打滿時機 ahead/predict，'
                                       '圈數 0–10，駕駛 corner/model'}, 400)
                self._json({'error': app.start(direction, seconds, ahead, timing, laps,
                                               policy)})
            elif path == '/train':
                self._json({'error': app.train()})
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


class Server(ThreadingHTTPServer):
    """IPv4 and IPv6 (a .local name may resolve to IPv6 first); errors dropped quietly."""

    address_family = socket.AF_INET6
    daemon_threads = True

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()

    def handle_error(self, request, client_address):
        pass    # e.g. a plain http request or a failed handshake: drop it


def main():
    app = Launcher()
    ctx, own, addresses = tls_context()
    server = Server(('::', PORT), make_handler(app))
    # the TLS handshake runs in the request's thread, so a slow phone cannot block accept()
    server.socket = ctx.wrap_socket(server.socket, server_side=True,
                                    do_handshake_on_connect=False)

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
