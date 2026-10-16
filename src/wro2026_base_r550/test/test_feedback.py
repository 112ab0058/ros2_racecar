"""Tests for the read-only feedback path (no ROS, no serial hardware)."""
import ast
import math
import os
import re

import pytest

from wro2026_base_r550.feedback import (feedback_summary, normalize_angle,
                                        OdomIntegrator, quaternion_from_yaw, RateMeter)
from wro2026_base_r550.listen import listen
from wro2026_base_r550.protocol import bcc, decode_feedback

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# official document example: vx 0.155 m/s, wz -0.033 rad/s, 23.431 V
OFFICIAL_FEEDBACK = bytes.fromhex(
    '7B 00 00 9B 00 00 FF DF 00 60 00 0C 40 A8 FF FD 00 06 00 1E 5B 87 82 7D')


def make_frame(vx_mm=0, wz_milli=0, stop_flag=0, mv=12000):
    body = bytearray(22)
    body[0] = 0x7B
    body[1] = stop_flag
    body[2:4] = vx_mm.to_bytes(2, 'big', signed=True)
    body[6:8] = wz_milli.to_bytes(2, 'big', signed=True)
    body[20:22] = mv.to_bytes(2, 'big')
    return bytes(body) + bytes([bcc(body), 0x7D])


# ------------------------------------------------------------------ odometry
def test_odom_straight_line():
    odom = OdomIntegrator()
    for i in range(21):  # 20 Hz for 1 s at 0.2 m/s
        odom.update(i * 0.05, 0.2, 0.0)
    assert odom.x == pytest.approx(0.2)
    assert odom.y == pytest.approx(0.0)
    assert odom.yaw == pytest.approx(0.0)


def test_odom_quarter_circle_left():
    # v = 0.5 m/s, wz = 0.5 rad/s -> radius 1 m; pi seconds = quarter turn
    odom = OdomIntegrator()
    n = 2000
    for i in range(n + 1):
        odom.update(i * (math.pi / n), 0.5, 0.5)
    assert odom.yaw == pytest.approx(math.pi / 2, abs=1e-6)
    assert odom.x == pytest.approx(1.0, abs=1e-4)
    assert odom.y == pytest.approx(1.0, abs=1e-4)  # left turn -> +y (REP-103)


def test_odom_does_not_integrate_across_gap():
    odom = OdomIntegrator(max_gap=0.5)
    odom.update(0.0, 1.0, 0.0)
    odom.update(2.0, 1.0, 0.0)  # 2 s gap: lost frames, not 2 m of motion
    assert odom.x == 0.0 and odom.gaps == 1
    odom.update(2.05, 1.0, 0.0)  # resumes after the gap
    assert odom.x == pytest.approx(0.05)


def test_odom_first_frame_and_time_going_backwards():
    odom = OdomIntegrator()
    odom.update(5.0, 1.0, 1.0)
    assert (odom.x, odom.yaw) == (0.0, 0.0)
    odom.update(4.9, 1.0, 1.0)
    assert (odom.x, odom.yaw) == (0.0, 0.0)


def test_angle_helpers():
    assert normalize_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    x, y, z, w = quaternion_from_yaw(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))


def test_rate_meter():
    rate = RateMeter(window=1.0)
    for i in range(40):
        rate.tick(i * 0.05)  # 20 Hz for 2 s
    assert rate.rate(1.975) == pytest.approx(20.0)  # samples 1.00 .. 1.95
    assert rate.rate(10.0) == 0.0


def test_motor_flag_and_summary():
    assert 'flag_stop=1 ' in feedback_summary(decode_feedback(make_frame(stop_flag=1)))
    text = feedback_summary(decode_feedback(OFFICIAL_FEEDBACK))
    assert 'vx=+0.155' in text and 'wz=-0.033' in text and 'battery=23.43 V' in text


# -------------------------------------------------------------------- listen
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.01
        return self.t


def test_listen_decodes_split_stream_and_saves_raw(tmp_path):
    data = b'\x00\x11' + OFFICIAL_FEEDBACK + make_frame(100, 0) + OFFICIAL_FEEDBACK[:10]
    chunks = [data[i:i + 7] for i in range(0, len(data), 7)]
    lines = []
    raw_path = tmp_path / 'raw.bin'
    with open(raw_path, 'wb') as raw:
        stream = listen(lambda: chunks.pop(0) if chunks else b'', 2.0,
                        out=lines.append, raw_file=raw, clock=FakeClock())
    assert raw_path.read_bytes() == data
    assert stream.bad_frames == 0
    assert any('vx=+0.100' in line for line in lines)
    assert lines[-1].startswith(f'done: {len(data)} bytes')


def test_listen_reports_no_frames():
    lines = []
    listen(lambda: b'', 1.5, out=lines.append, clock=FakeClock())
    assert any('no valid frame yet' in line for line in lines)


# --------------------------------------------------------- read-only guarantees
READ_ONLY_FILES = ['feedback.py', 'feedback_node.py', 'listen.py']


@pytest.mark.parametrize('name', READ_ONLY_FILES)
def test_read_only_modules_never_send(name):
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_base_r550', name), encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            # the only write() allowed is listen's --raw-out file, never the serial port
            if node.func.attr == 'write':
                assert name == 'listen.py' and ast.unparse(node.func.value) == 'raw_file'
            assert node.func.attr != 'create_subscription'  # no /cmd_vel or anything else
        if isinstance(node, (ast.Name, ast.alias)):
            ident = node.id if isinstance(node, ast.Name) else node.name
            assert ident not in ('encode_twist_command', 'encode_ackermann_command',
                                 'stop_command', 'Twist'), ident


def test_parameters_match_yaml():
    src = open(os.path.join(PKG, 'wro2026_base_r550', 'feedback_node.py'),
               encoding='utf-8').read()
    tree = ast.parse(src)
    params = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'PARAMETERS':
            params = node.value
    names = {}
    for item in params.elts:
        name, default = item.elts[0].value, item.elts[1]
        names[name] = default.value if isinstance(default, ast.Constant) else 115200
    yaml_text = open(os.path.join(PKG, 'config', 'r550_feedback.yaml'), encoding='utf-8').read()
    yaml_vals = {}
    for line in yaml_text.splitlines():
        m = re.match(r'^\s{4}(\w+):\s*([^#]*?)\s*(#.*)?$', line)
        if m and m.group(1) != 'ros__parameters':
            yaml_vals[m.group(1)] = m.group(2)
    assert set(yaml_vals) == set(names)
    for key, default in names.items():
        raw = yaml_vals[key]
        if isinstance(default, bool):
            assert raw == str(default).lower(), key
        elif isinstance(default, str):
            assert raw.strip("'") == default, key
        else:
            assert float(raw) == pytest.approx(float(default)), key
