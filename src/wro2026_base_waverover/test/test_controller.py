"""Safety behaviour of BaseController + DriverLoop, recorded through FakeTransport."""
import json

from wro2026_base_waverover.core import (BaseController, DriverLoop, FEEDBACK_LOST, FEEDBACK_OK,
                                         FEEDBACK_WAITING, LEVEL_ERROR, LEVEL_OK, LEVEL_WARN)
from wro2026_base_waverover.transport import FakeTransport

STOP = '{"T":1,"L":0,"R":0}'
ALLOWED_WITHOUT_MOTION = {STOP, '{"T":130}', '{"T":126}'}
IMU_REPLY = json.dumps({'T': 1002, 'r': 0, 'p': 0, 'y': 0, 'ax': 0, 'ay': 0, 'az': 980,
                        'gx': 0, 'gy': 0, 'gz': 0})


class Harness:
    """Driver loop + fake ESP32 on a fake clock."""

    def __init__(self, clock, battery=12.25, feedback=True, cmd_period=0.5):
        self.clock = clock
        self.battery = battery
        self.feedback = feedback
        self.controller = BaseController(track_width=0.125, max_speed=0.05, cmd_timeout=0.3,
                                         feedback_timeout=2.0, low_battery_voltage=11.0,
                                         critical_battery_voltage=10.5, clock=clock)
        self.fake = FakeTransport(reply=self._reply)
        self.records = []
        self.logs = []
        self.last_cmd_t = None
        self.loop = DriverLoop(self.controller, self.fake, cmd_period, feedback_period=0.2,
                               clock=clock, wall_clock=clock, on_sent=self.records.append,
                               log=lambda level, text: self.logs.append((level, text)))

    def _reply(self, cmd):
        if not self.feedback:
            return ''
        if cmd == {'T': 130}:
            return json.dumps({'T': 1001, 'L': 0, 'R': 0, 'v': self.battery})
        if cmd == {'T': 126}:
            return IMU_REPLY
        return ''

    def run(self, seconds, cmd=None, dt=0.01):
        """Run the loop for `seconds`; publish /cmd_vel `cmd` at 10 Hz when given."""
        end = self.clock.t + seconds - 1e-9
        next_cmd = self.clock.t
        while self.clock.t < end:
            if cmd is not None and self.clock.t >= next_cmd:
                self.controller.set_cmd_vel(*cmd)
                self.last_cmd_t = self.clock.t
                next_cmd += 0.1
            while self.loop.step():
                pass
            self.clock.advance(dt)

    def wheels(self, since=None):
        return [(r['t'], r['cmd']['L'], r['cmd']['R']) for r in self.records
                if since is None or r['t'] >= since]


def test_startup_only_sends_stop_and_polls(clock):
    h = Harness(clock)
    h.run(3.0)
    assert set(h.fake.sent) <= ALLOWED_WITHOUT_MOTION
    assert h.fake.sent.count(STOP) == 6  # first stop immediately, then every 0.5 s (http)
    assert '{"T":130}' in h.fake.sent and '{"T":126}' in h.fake.sent
    assert all(r['ok'] for r in h.records)
    assert h.controller.feedback_state() == FEEDBACK_OK
    assert h.controller.status() == (LEVEL_OK, 'ok')


def test_cmd_vel_is_forwarded_when_feedback_is_ok(clock):
    h = Harness(clock)
    h.run(0.5)
    t0 = clock.t
    h.run(0.5, cmd=(0.05, 0.0))
    assert (0.05, 0.05) in [(left, right) for _, left, right in h.wheels(t0)]
    assert h.loop.sent_ok_wheels == (0.05, 0.05)


def test_cmd_timeout_sends_stop(clock):
    h = Harness(clock)
    h.run(0.5)
    h.run(0.5, cmd=(0.05, 0.0))
    t_last_cmd = h.last_cmd_t
    h.run(1.0)
    stops = [t for t, left, right in h.wheels(t_last_cmd) if (left, right) == (0, 0)]
    assert stops, 'no stop after /cmd_vel stopped'
    # cmd_timeout 0.3 s, loop tick 0.01 s
    assert 0.3 <= stops[0] - t_last_cmd <= 0.32
    assert h.loop.sent_ok_wheels == (0, 0)


def test_nonzero_refused_before_first_feedback(clock):
    h = Harness(clock, feedback=False)
    h.run(1.0, cmd=(0.05, 0.0))
    assert set(h.fake.sent) <= ALLOWED_WITHOUT_MOTION
    assert h.controller.feedback_state() == FEEDBACK_WAITING
    assert h.loop.last_reason == 'feedback_waiting'
    assert h.controller.status()[0] == LEVEL_WARN


def test_feedback_loss_stops_logs_error_and_reports_error(clock):
    h = Harness(clock)
    h.run(0.5)
    h.run(1.0, cmd=(0.05, 0.0))
    assert (0.05, 0.05) in [(left, right) for _, left, right in h.wheels()]
    last_reply = h.controller.last_feedback
    h.feedback = False
    t_cut = clock.t
    h.run(3.0, cmd=(0.05, 0.0))
    # watchdog fires 2 s after the last T:130 reply and a stop goes out at once
    stops = [t for t, left, right in h.wheels(t_cut) if (left, right) == (0, 0)]
    lost_at = last_reply + 2.0
    assert any(lost_at <= t <= lost_at + 0.02 for t in stops)
    assert all((left, right) == (0, 0) for t, left, right in h.wheels(lost_at))
    assert any(level == 'error' and 'feedback lost' in text for level, text in h.logs)
    assert h.controller.feedback_state() == FEEDBACK_LOST
    level, message = h.controller.status()
    assert level == LEVEL_ERROR and 'T:130' in message
    # recovery: replies come back -> commands pass again
    h.feedback = True
    t_back = clock.t
    h.run(1.0, cmd=(0.05, 0.0))
    assert (0.05, 0.05) in [(left, right) for _, left, right in h.wheels(t_back)]
    assert any(level == 'info' and 'recovered' in text for level, text in h.logs)


def test_low_battery_warns_but_still_drives(clock):
    h = Harness(clock, battery=10.9)
    h.run(0.5)
    h.run(0.5, cmd=(0.05, 0.0))
    assert (0.05, 0.05) in [(left, right) for _, left, right in h.wheels()]
    level, message = h.controller.status()
    assert level == LEVEL_WARN and 'battery low' in message


def test_critical_battery_refuses_nonzero_commands(clock):
    h = Harness(clock, battery=10.4)
    h.run(0.5)
    h.run(1.0, cmd=(0.05, 0.0))
    assert set(h.fake.sent) <= ALLOWED_WITHOUT_MOTION
    assert h.loop.last_reason == 'battery_critical'
    assert any(level == 'error' and 'battery critical' in text for level, text in h.logs)
    assert h.controller.status()[0] == LEVEL_ERROR


def test_shutdown_sends_stop(clock):
    h = Harness(clock)
    h.run(0.5)
    h.fake.sent.clear()
    assert h.loop.send_stop(sleep=lambda s: None) is True
    assert h.fake.sent == [STOP]


def test_shutdown_stop_retries_three_times_then_reports_failure(clock):
    h = Harness(clock)
    h.fake.fail = True
    sleeps = []
    assert h.loop.send_stop(attempts=3, pause=0.1, sleep=sleeps.append) is False
    assert [r['ok'] for r in h.records] == [False, False, False]
    assert sleeps == [0.1, 0.1]


def test_transport_failures_are_counted_not_fatal(clock):
    h = Harness(clock)
    h.fake.fail = True
    h.run(1.0)
    assert h.loop.send_failures > 0
    assert not any(r['ok'] for r in h.records)
    assert any(level == 'warn' for level, _ in h.logs)
    h.fake.fail = False
    h.run(1.0)
    assert h.records[-1]['ok']


def test_serial_style_async_feedback(clock):
    h = Harness(clock, cmd_period=0.1)
    h.fake.reply = None  # serial: send() returns None, replies arrive on their own
    h.fake.start(h.loop.handle_line)
    h.run(0.3)
    assert h.controller.feedback_state() == FEEDBACK_WAITING
    h.fake.push_line('{"T":1001,"L":0,"R":0,"v":12.0}')
    assert h.controller.feedback_state() == FEEDBACK_OK
    assert h.controller.battery_v == 12.0
    t0 = clock.t
    h.run(0.3, cmd=(0.0, 0.8))
    assert (-0.05, 0.05) in [(left, right) for _, left, right in h.wheels(t0)]
    # serial resends every 0.1 s
    assert sum(1 for _ in h.wheels(t0)) >= 3


def test_feedback_watchdog_can_be_disabled(clock):
    h = Harness(clock, feedback=False)
    h.controller.feedback_timeout = 0.0
    h.run(0.5, cmd=(0.05, 0.0))
    assert (0.05, 0.05) in [(left, right) for _, left, right in h.wheels()]
