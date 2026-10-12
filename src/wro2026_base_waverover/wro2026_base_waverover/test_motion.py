"""
Automated low-speed motion test. WHEELS MUST BE OFF THE GROUND.

  ros2 run wro2026_base_waverover test_motion --transport http --host 192.168.0.16
  ros2 run wro2026_base_waverover test_motion --transport serial --port /dev/ttyTHS1
  ros2 run wro2026_base_waverover test_motion --startup-only ...   (never moves the wheels)

Starts its own driver, then runs:
  B1 startup (driver must only send stop)
  B2 forward / backward / spin left / spin right, each <= --duration s,
     --pause s between motions, battery checked before every motion
  B3 stop protection: stop publishing /cmd_vel, driver must send stop in 0.5 s
  B4 shutdown: SIGINT the driver, it must send stop on exit
Any failure sends {"T":1,"L":0,"R":0} and skips the remaining motions.

Writes ~/wave_rover_logs/motion_<time>.csv (+ driver log and summary json).
The only commands this script sends directly to the ESP32 are T:1 zero
(emergency/final stop) and T:130 (read back); motion goes through /cmd_vel.
"""
import argparse
import csv
import datetime
import json
import os
import signal
import subprocess
import threading
import time
import urllib.parse
import urllib.request

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .process_guard import find_other_drivers

SPEED_CAP = 0.1


def spin_quietly(ex):
    try:
        ex.spin()
    except ExternalShutdownException:
        pass


STOP = {'T': 1, 'L': 0, 'R': 0}


class Recorder(Node):
    def __init__(self, csv_path):
        super().__init__('wave_rover_test_motion')
        self.lock = threading.Lock()
        self.step = 'init'
        self.sent = []      # (t, L, R, ok, dt)
        self.fb = []        # (t, L, R, v)
        self.battery_v = None
        self.battery_t = 0.0
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['wall_time', 't_unix', 'step', 'event', 'sent_cmd',
                           't130_response', 'battery_v', 'note'])
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'wave_rover/sent_cmd', self._on_sent, 50)
        self.create_subscription(String, 'wave_rover/feedback', self._on_fb, 50)
        self.create_subscription(BatteryState, 'battery', self._on_bat, 10)

    def row(self, event, sent='', fb='', note='', t=None):
        t = time.time() if t is None else t
        with self.lock:
            wall = datetime.datetime.fromtimestamp(t).isoformat(timespec='milliseconds')
            self.csv.writerow([wall,
                               f'{t:.3f}', self.step, event, sent, fb,
                               '' if self.battery_v is None else f'{self.battery_v:.3f}', note])
            self.csv_file.flush()

    def _on_sent(self, msg):
        rec = json.loads(msg.data)
        c = rec['cmd']
        with self.lock:
            self.sent.append((rec['t'], c['L'], c['R'], rec['ok'], rec['dt']))
        self.row('sent_cmd', sent=json.dumps(c), note=f"ok={rec['ok']} dt={rec['dt']}", t=rec['t'])

    def _on_fb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('T') == 1001:
            with self.lock:
                self.fb.append((time.time(), d.get('L'), d.get('R'), d.get('v')))
            self.row('t130_feedback', fb=msg.data)

    def _on_bat(self, msg):
        with self.lock:
            self.battery_v = msg.voltage
            self.battery_t = time.monotonic()

    def publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def sent_between(self, t0, t1):
        with self.lock:
            return [s for s in self.sent if t0 <= s[0] <= t1]

    def fb_between(self, t0, t1):
        with self.lock:
            return [f for f in self.fb if t0 <= f[0] <= t1]


class MotionTest:
    def __init__(self, args):
        self.a = args
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(args.log_dir, exist_ok=True)
        self.csv_path = os.path.join(args.log_dir, f'motion_{stamp}.csv')
        self.driver_log_path = os.path.join(args.log_dir, f'driver_{stamp}.log')
        self.summary_path = os.path.join(args.log_dir, f'motion_{stamp}_summary.json')
        self.results = []
        self.decisions = []
        self.motion_allowed = True
        self.driver = None

    # ------------------------------------------------------------- helpers
    def result(self, step, ok, **data):
        entry = {'step': step, 'result': 'PASS' if ok else 'FAIL', **data}
        self.results.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
        self.n.row('result', note=json.dumps(entry, ensure_ascii=False))
        return ok

    def skip(self, step, reason):
        entry = {'step': step, 'result': 'SKIPPED', 'reason': reason}
        self.results.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
        self.n.row('result', note=json.dumps(entry, ensure_ascii=False))

    def direct(self, cmd):
        """Direct ESP32 access, restricted to stop and T:130."""
        if cmd not in (STOP, {'T': 130}):
            raise ValueError(f'command not allowed: {cmd}')
        data = json.dumps(cmd, separators=(',', ':'))
        if self.a.transport != 'http':
            return None
        try:
            url = f'http://{self.a.host}/js?json=' + urllib.parse.quote(data)
            return urllib.request.urlopen(url, timeout=1.0).read().decode(errors='ignore')
        except Exception as e:
            return f'ERROR {e}'

    def emergency_stop(self, why):
        self.motion_allowed = False
        for _ in range(3):
            self.n.publish(0.0, 0.0)
        reply = self.direct(STOP)
        self.n.row('EMERGENCY_STOP', sent=json.dumps(STOP), note=f'{why}; direct reply={reply}')
        print(f'EMERGENCY STOP: {why}', flush=True)

    def battery_ok(self, step):
        with self.n.lock:
            v, age = self.n.battery_v, time.monotonic() - self.n.battery_t
        if v is None or age > 1.5:
            self.skip(step, f'battery reading missing or stale (age {age:.1f}s)')
            self.emergency_stop('no fresh battery reading')
            return False
        self.n.row('battery_check', note=f'{v:.3f} V')
        if v < self.a.min_battery:
            self.skip(step, f'battery {v:.2f} V < {self.a.min_battery} V')
            self.emergency_stop('battery low')
            return False
        return True

    def driver_alive(self):
        return self.driver is not None and self.driver.poll() is None

    # ---------------------------------------------------------------- steps
    def start_driver(self):
        pkg = 'wro2026_base_waverover'
        exe = os.path.join(get_package_prefix(pkg), 'lib', pkg, 'driver')
        config = os.path.join(get_package_share_directory(pkg), 'config', 'wave_rover.yaml')
        cmd = [exe, '--ros-args', '--params-file', config,
               '-p', f'transport:={self.a.transport}', '-p', f'host:={self.a.host}',
               '-p', f'port:={self.a.port}', '-p', f'max_speed:={self.a.speed}',
               '-p', f'track_width:={self.a.track_width}',
               '-p', f'cmd_timeout:={self.a.cmd_timeout}']
        self.driver_log = open(self.driver_log_path, 'w')
        self.n.step = 'B1_startup'
        self.n.row('driver_start', note=' '.join(cmd))
        self.t_driver_start = time.time()
        self.driver = subprocess.Popen(cmd, stdout=self.driver_log, stderr=subprocess.STDOUT,
                                       env={**os.environ, 'PYTHONUNBUFFERED': '1'})

    def step_startup(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if (self.n.battery_v is not None and self.n.sent
                    and self.n.cmd_pub.get_subscription_count() > 0):
                break
            if not self.driver_alive():
                break
            time.sleep(0.1)
        time.sleep(self.a.idle)
        sent = self.n.sent_between(0, time.time())
        fb = self.n.fb_between(0, time.time())
        nonzero_sent = [s for s in sent if s[1] != 0 or s[2] != 0]
        nonzero_fb = [f for f in fb if f[1] != 0 or f[2] != 0]
        ok = (self.driver_alive() and len(sent) > 0 and len(fb) > 0
              and self.n.battery_v is not None
              and not nonzero_sent and not nonzero_fb and all(s[3] for s in sent))
        self.result('B1 startup only sends stop', ok, sent_count=len(sent),
                    sent_nonzero=len(nonzero_sent),
                    send_failures=sum(1 for s in sent if not s[3]), t130_count=len(fb),
                    t130_nonzero=len(nonzero_fb), battery_v=self.n.battery_v,
                    driver_alive=self.driver_alive())
        if not ok:
            self.emergency_stop('startup check failed')

    def step_motion(self, name, v, w, exp_l, exp_r):
        self.n.step = name
        if not self.motion_allowed:
            self.skip(name, 'motion disabled by an earlier failure')
            return
        if not self.driver_alive():
            self.skip(name, 'driver not running')
            self.emergency_stop('driver not running')
            return
        if not self.battery_ok(name):
            return
        active = self.a.duration - 0.1  # zero is published at active, so command lasts <= duration
        t0 = time.time()
        self.n.row('motion_start', note=f'v={v} w={w:.4f} expected L={exp_l} R={exp_r}')
        end = time.monotonic() + active
        while time.monotonic() < end:
            if not self.driver_alive():
                self.emergency_stop(f'driver died during {name}')
                break
            self.n.publish(v, w)
            time.sleep(0.1)
        t_zero = time.time()
        for _ in range(3):
            self.n.publish(0.0, 0.0)
        self.n.row('cmd_vel_zero', note='published zero twist x3')
        # stop must be sent within 0.5 s; otherwise stop immediately, not after the pause
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            recent = self.n.sent_between(t_zero - 0.05, time.time())
            if any(s[1] == 0 and s[2] == 0 for s in recent):
                break
            time.sleep(0.02)
        else:
            self.emergency_stop(f'no stop sent within 0.5 s after {name}')
        time.sleep(max(0.0, self.a.pause - (time.time() - t_zero)))
        t_end = time.time()

        sent = self.n.sent_between(t0, t_end)
        moving = [s for s in sent if s[0] < t_zero + 0.05
                  and abs(s[1] - exp_l) < 1e-3 and abs(s[2] - exp_r) < 1e-3]
        stops = [s for s in sent if s[0] >= t_zero - 0.05 and s[1] == 0 and s[2] == 0]
        stop_latency = (stops[0][0] - t_zero) if stops else None
        last_motion = max((s[0] for s in moving), default=None)
        on_time = (last_motion - moving[0][0]) if moving else None
        fb = self.n.fb_between(t0, t_end)
        fb_moving = [f for f in fb if f[1] and f[2]
                     and (f[1] > 0) == (exp_l > 0) and (f[2] > 0) == (exp_r > 0)]
        fb_after = self.n.fb_between(t_zero + 0.8, t_end)
        fb_stopped = bool(fb_after) and all(f[1] == 0 and f[2] == 0 for f in fb_after)
        failures = [s for s in sent if not s[3]]
        ok = (bool(moving) and stop_latency is not None and stop_latency <= 0.5
              and fb_stopped and not failures)
        self.result(name, ok, expected={'L': exp_l, 'R': exp_r},
                    motion_cmds_sent=len(moving),
                    motion_cmd_span_s=None if on_time is None else round(on_time, 3),
                    stop_sent_after_zero_s=(None if stop_latency is None
                                            else round(stop_latency, 3)),
                    t130_applied_pwm=sorted({(f[1], f[2]) for f in fb_moving}),
                    t130_samples_in_motion=len(fb_moving), t130_zero_after=fb_stopped,
                    send_failures=len(failures), battery_v=self.n.battery_v)
        if not ok:
            self.emergency_stop(f'{name} failed')

    def step_stop_protection(self):
        name = 'B3 stop protection (cmd_vel cut)'
        self.n.step = 'B3_stop_protection'
        if not self.motion_allowed:
            self.skip(name, 'motion disabled by an earlier failure')
            return
        if not self.driver_alive():
            self.skip(name, 'driver not running')
            return
        if not self.battery_ok(name):
            return
        t0 = time.time()
        active = self.a.duration - 0.1 - self.a.cmd_timeout  # cmd + timeout <= duration
        end = time.monotonic() + max(active, 0.2)
        t_last = t0
        while time.monotonic() < end:
            self.n.publish(self.a.speed, 0.0)
            t_last = time.time()
            time.sleep(0.1)
        self.n.row('cmd_vel_cut', note='stopped publishing without a zero twist')
        deadline = time.monotonic() + 1.0
        stop_t = None
        while time.monotonic() < deadline:
            stops = [s for s in self.n.sent_between(t_last, time.time())
                     if s[1] == 0 and s[2] == 0]
            if stops:
                stop_t = stops[0][0]
                break
            time.sleep(0.02)
        if stop_t is None:
            self.emergency_stop('driver did not send stop after cmd_vel cut')
        time.sleep(self.a.pause)
        latency = None if stop_t is None else stop_t - t_last
        fb_after = self.n.fb_between(t_last + 0.8, time.time())
        fb_stopped = bool(fb_after) and all(f[1] == 0 and f[2] == 0 for f in fb_after)
        moving = [s for s in self.n.sent_between(t0, t_last + 0.05)
                  if s[1] == self.a.speed and s[2] == self.a.speed]
        ok = bool(moving) and latency is not None and latency <= 0.5 and fb_stopped
        self.result(name, ok, motion_cmds_sent=len(moving),
                    stop_sent_after_last_cmd_vel_s=None if latency is None else round(latency, 3),
                    cmd_timeout=self.a.cmd_timeout, t130_zero_after=fb_stopped)
        if not ok and self.motion_allowed:
            self.emergency_stop('stop protection failed')

    def step_shutdown(self):
        name = 'B4 shutdown sends stop'
        self.n.step = 'B4_shutdown'
        if not self.driver_alive():
            self.skip(name, 'driver not running')
            return
        t0 = time.time()
        self.driver.send_signal(signal.SIGINT)
        try:
            rc = self.driver.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.driver.kill()
            rc = 'killed'
        self.driver_log.flush()
        log = open(self.driver_log_path).read()
        stop_logged = 'shutdown stop sent: ok' in log
        time.sleep(0.5)
        reply = self.direct({'T': 130})
        self.n.row('direct_t130_after_shutdown', sent='{"T":130}', fb=reply or '')
        lr_zero = None
        try:
            d = json.loads(reply)
            lr_zero = d.get('L') == 0 and d.get('R') == 0
        except (TypeError, ValueError):
            pass
        ok = stop_logged and lr_zero is not False
        self.result(name, ok, exit_code=rc, shutdown_s=round(time.time() - t0, 3),
                    driver_logged_stop=stop_logged, t130_after=reply, t130_lr_zero=lr_zero)

    # ------------------------------------------------------------------ run
    def run(self):
        rclpy.init()
        self.n = Recorder(self.csv_path)
        ex = SingleThreadedExecutor()
        ex.add_node(self.n)
        spin_thread = threading.Thread(target=spin_quietly, args=(ex,), daemon=True)
        spin_thread.start()
        s, tw = self.a.speed, self.a.track_width
        w_spin = 2.0 * s / tw
        try:
            self.decisions.append('no automatic retry at 0.1: the car has no encoders and '
                                  'T:130 L/R stay 0 while T:1 is applied, so feedback cannot '
                                  'show whether a wheel physically turns; check visually and '
                                  'rerun with --speed 0.1')
            self.start_driver()
            self.step_startup()
            if self.a.startup_only:
                self.motion_allowed = False
                self.decisions.append('--startup-only: no motion commands')
            else:
                self.step_motion('B2a forward', s, 0.0, s, s)
                self.step_motion('B2b backward', -s, 0.0, -s, -s)
                self.step_motion('B2c spin left', 0.0, w_spin, -s, s)
                self.step_motion('B2d spin right', 0.0, -w_spin, s, -s)
                self.step_stop_protection()
            self.step_shutdown()
        except Exception as e:
            self.n.row('exception', note=repr(e))
            self.result('exception', False, error=repr(e))
            self.emergency_stop(f'exception {e!r}')
        finally:
            if self.driver_alive():
                self.driver.send_signal(signal.SIGINT)
                try:
                    self.driver.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.driver.kill()
            reply = self.direct(STOP)
            self.n.step = 'final'
            self.n.row('final_stop', sent=json.dumps(STOP), fb=reply or '')
            summary = {'csv': self.csv_path, 'driver_log': self.driver_log_path,
                       'args': vars(self.a), 'decisions': self.decisions, 'results': self.results}
            with open(self.summary_path, 'w') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
            print(f'csv: {self.csv_path}\nsummary: {self.summary_path}', flush=True)
            # shut the context down first so ex.spin() returns before teardown
            rclpy.try_shutdown()
            spin_thread.join(timeout=2.0)
            ex.shutdown()
            self.n.csv_file.close()
            self.n.destroy_node()
        return 0 if all(r['result'] == 'PASS' for r in self.results) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--transport', choices=['http', 'serial'], default='http')
    ap.add_argument('--host', default='192.168.0.16')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--speed', type=float, default=0.05,
                    help=f'wheel PWM fraction, max {SPEED_CAP}')
    ap.add_argument('--track-width', type=float, default=0.125)
    ap.add_argument('--cmd-timeout', type=float, default=0.3)
    ap.add_argument('--duration', type=float, default=1.0, help='max seconds per motion (<= 1.0)')
    ap.add_argument('--pause', type=float, default=2.0, help='seconds between motions (>= 2.0)')
    ap.add_argument('--idle', type=float, default=3.0, help='startup observation time')
    ap.add_argument('--min-battery', type=float, default=11.0)
    ap.add_argument('--log-dir', default=os.path.expanduser('~/wave_rover_logs'))
    ap.add_argument('--startup-only', action='store_true',
                    help='only B1 startup and B4 shutdown checks; never moves the wheels')
    args = ap.parse_args()
    if not 0 < args.speed <= SPEED_CAP:
        ap.error(f'--speed must be in (0, {SPEED_CAP}]')
    if not 0 < args.duration <= 1.0:
        ap.error('--duration must be in (0, 1.0]')
    if args.pause < 2.0:
        ap.error('--pause must be >= 2.0')
    running = find_other_drivers()
    if running:
        pids = [pid for pid, _ in running]
        ap.error(f'a wave_rover driver is already running (pid {pids}); stop it first')
    return MotionTest(args).run()


if __name__ == '__main__':
    raise SystemExit(main())
