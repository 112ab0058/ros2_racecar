"""Phone teleop: hold-to-drive logic and the web server (no ROS, no motors)."""
import json
import threading
import urllib.error
import urllib.request

import pytest
from wro2026_base_r550.web_teleop import make_server, WebTeleop


def test_nothing_pressed_is_stop():
    assert WebTeleop().command(0.0) == (0.0, 0.0)


@pytest.mark.parametrize('action, expected', [
    ('straight', (0.1, 0.0)), ('left', (0.1, 0.2)), ('right', (0.1, -0.2)),
    ('full_left', (0.1, 0.1 / 0.35)), ('full_right', (0.1, -0.1 / 0.35)),
    ('back', (-0.08, 0.0)),
    ('back_left', (-0.08, -0.16)), ('back_right', (-0.08, 0.16)),   # wheels left, reversing
    ('stop', (0.0, 0.0)), ('spin', (0.0, 0.0))])
def test_actions(action, expected):
    # Ackermann table 3-1 of the vendor app manual; never turning on the spot
    t = WebTeleop(speed=0.1, turn_radius=0.5)
    t.press(1.0, action)
    vx, wz = t.command(1.1)
    assert (vx, wz) == pytest.approx(expected)
    assert wz == 0.0 or abs(vx) > 0.0


def test_speed_buttons_stay_within_limits():
    t = WebTeleop(speed=0.1, max_speed=0.15)
    assert t.change_speed(1) == pytest.approx(0.15)
    assert t.change_speed(1) == pytest.approx(0.15)         # max_speed (orin_field max_vx)
    assert [t.change_speed(-1) for _ in range(3)] == pytest.approx([0.1, 0.05, 0.05])
    t.press(0.0, 'back')
    assert t.command(0.1)[0] == pytest.approx(-0.05)        # reverse never faster than forward
    assert WebTeleop(speed=0.1).change_speed(1) == pytest.approx(0.1)   # default max = speed


def test_letting_go_stops_within_the_hold_timeout():
    t = WebTeleop(hold_timeout=0.35)
    t.press(1.0, 'straight')
    assert t.command(1.34)[0] > 0.0
    assert t.command(1.36) == (0.0, 0.0)     # no new request: phone let go or lost WiFi
    t.press(1.4, 'straight')
    t.press(1.45, 'stop')
    assert t.command(1.46) == (0.0, 0.0)


def test_auto_button_does_nothing_unless_enabled():
    t = WebTeleop()
    t.set_auto(1.0, 0.1, 0.0)
    t.press(1.0, 'auto')
    assert t.command(1.05) == (0.0, 0.0) and t.current(1.05) is None


def test_auto_passes_the_policy_only_while_held_and_fresh():
    t = WebTeleop(speed=0.1, hold_timeout=0.35, auto=True, auto_timeout=0.3)
    t.press(1.0, 'auto')
    assert t.command(1.0) == (0.0, 0.0)                     # no policy command yet
    t.set_auto(1.0, 0.1, 0.2)
    assert t.command(1.1) == pytest.approx((0.1, 0.2))
    assert t.command(1.31) == (0.0, 0.0)                    # policy silent > auto_timeout
    t.set_auto(1.3, 0.1, 0.2)
    assert t.command(1.36) == (0.0, 0.0)                    # button released (hold_timeout)


@pytest.mark.parametrize('cmd, expected', [
    ((0.3, 0.0), (0.1, 0.0)),                               # never above max_speed
    ((-0.1, 0.0), (0.0, 0.0)),                              # never backwards
    ((0.1, 1.0), (0.1, 0.1 / 0.35)),                        # never tighter than full lock
    ((0.1, -1.0), (0.1, -0.1 / 0.35)),
    ((0.0, 0.5), (0.0, 0.0))])                              # never turning on the spot
def test_auto_commands_are_clamped(cmd, expected):
    t = WebTeleop(speed=0.1, auto=True)
    t.press(1.0, 'auto')
    t.set_auto(1.0, *cmd)
    assert t.command(1.05) == pytest.approx(expected)


@pytest.mark.parametrize('kwargs', [
    {'speed': 0.2}, {'speed': 0.0}, {'back_speed': 0.3}, {'turn_radius': 0.3},
    {'hold_timeout': 0.0}, {'max_speed': 0.2}, {'speed': 0.1, 'max_speed': 0.05}])
def test_refuses_unsafe_settings(kwargs):
    with pytest.raises(ValueError):
        WebTeleop(**kwargs)


@pytest.fixture
def server():
    clock = {'t': 10.0}
    teleop = WebTeleop(max_speed=0.15)
    srv = make_server(teleop, 'abc123', 0, lambda: clock['t'], lambda: {'battery': 11.8})
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield 'http://127.0.0.1:%d' % srv.server_address[1], teleop, clock
    srv.shutdown()
    srv.server_close()


def request(url, post=False):
    req = urllib.request.Request(url, data=b'' if post else None)
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, r.read().decode('utf-8')


def test_wrong_token_cannot_drive(server):
    base, teleop, _ = server
    for url in (base + '/cmd?a=straight', base + '/cmd?k=nope&a=straight'):
        with pytest.raises(urllib.error.HTTPError) as e:
            request(url, post=True)
        assert e.value.code == 403
    assert teleop.requests == 0


def test_page_and_driving_over_http(server):
    base, teleop, clock = server
    status, page = request(base + '/?k=abc123')
    assert status == 200 and 'pointerdown' in page and 'setInterval' in page
    status, body = request(base + '/cmd?k=abc123&a=left', post=True)
    s = json.loads(body)
    assert (s['action'], s['vx'], s['wz'], s['battery']) == ('left', 0.1, 0.2, 11.8)
    clock['t'] += 0.5                                   # phone stopped sending
    state = json.loads(request(base + '/state?k=abc123')[1])
    assert (state['action'], state['vx']) == ('stop', 0.0)


def test_speed_and_joystick_over_http(server):
    base, teleop, _ = server
    s = json.loads(request(base + '/speed?k=abc123&d=up', post=True)[1])
    assert s['speed'] == pytest.approx(0.15) and s['max_speed'] == pytest.approx(0.15)
    s = json.loads(request(base + '/cmd?k=abc123&a=full_right', post=True)[1])
    assert (s['action'], s['vx'], s['wz']) == ('full_right', 0.15, round(-0.15 / 0.35, 3))
    s = json.loads(request(base + '/speed?k=abc123&d=down', post=True)[1])
    assert s['speed'] == pytest.approx(0.1)
    with pytest.raises(urllib.error.HTTPError) as e:
        request(base + '/speed?d=up', post=True)
    assert e.value.code == 403 and teleop.speed == pytest.approx(0.1)
