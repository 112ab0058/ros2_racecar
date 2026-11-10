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
    ('back', (-0.08, 0.0)), ('stop', (0.0, 0.0)), ('spin', (0.0, 0.0))])
def test_actions(action, expected):
    t = WebTeleop(speed=0.1, turn_radius=0.5)
    t.press(1.0, action)
    assert t.command(1.1) == pytest.approx(expected)


def test_letting_go_stops_within_the_hold_timeout():
    t = WebTeleop(hold_timeout=0.35)
    t.press(1.0, 'straight')
    assert t.command(1.34)[0] > 0.0
    assert t.command(1.36) == (0.0, 0.0)     # no new request: phone let go or lost WiFi
    t.press(1.4, 'straight')
    t.press(1.45, 'stop')
    assert t.command(1.46) == (0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'speed': 0.2}, {'speed': 0.0}, {'back_speed': 0.3}, {'turn_radius': 0.3},
    {'hold_timeout': 0.0}])
def test_refuses_unsafe_settings(kwargs):
    with pytest.raises(ValueError):
        WebTeleop(**kwargs)


@pytest.fixture
def server():
    clock = {'t': 10.0}
    teleop = WebTeleop()
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
