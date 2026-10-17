"""N10P protocol and scan assembly tests (synthetic packets, no hardware)."""
import ast
import math
import os
import re

import pytest

from wro2026_lidar_n10p.listen import listen
from wro2026_lidar_n10p.protocol import (checksum, decode_packet, encode_packet,
                                         lidar_deg_to_ros_rad, PACKET_SIZE, PacketError,
                                         PacketStream, ScanAssembler)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FULL = [1000 + 10 * n for n in range(16)]  # mm


def test_encode_decode_roundtrip():
    pkt = decode_packet(encode_packet(10.0, 25.0, FULL))
    assert len(encode_packet(10.0, 25.0, FULL)) == PACKET_SIZE
    assert (pkt.start_deg, pkt.end_deg) == (10.0, 25.0)
    assert len(pkt.points) == 16
    assert pkt.points[0][:2] == (10.0, 1.0)
    assert pkt.points[15][0] == pytest.approx(25.0)   # last slot at the end angle
    assert pkt.points[15][1] == pytest.approx(1.15)


# Recorded from our N10P on 2026-10-17 (~/n10p_first.bin): wall at ~2.6-2.8 m,
# two points with distance 0 (no return).
REAL_PACKET = bytes.fromhex(
    'a5 5a 6c 10 46 26 f8 0a 41 4d 00 00 00 0a 43 51 00 00 00 0a 4b 45 00 00 00 0a 4e 44'
    ' 00 00 00 0a 5e 40 00 00 00 0a 63 43 00 00 00 0a 76 44 00 00 00 0b 16 5d 00 00 00'
    ' 0b 1b 73 00 00 00 0b 21 41 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0a e7 39'
    ' 00 00 00 0a e6 18 00 00 00 0a cf 43 00 00 00 0a d4 20 00 00 00 00 02 2a f3 56')


def test_real_packet_from_our_lidar():
    pkt = decode_packet(REAL_PACKET)
    assert REAL_PACKET[2] == PACKET_SIZE and REAL_PACKET[3] == 16
    assert (pkt.start_deg, pkt.end_deg) == (99.76, 109.95)
    assert len(pkt.points) == 14                       # the two distance-0 slots are dropped
    assert pkt.points[0][1:] == (2.625, 0x4D)          # 2625 mm, intensity 77
    assert pkt.points[-1][1] == pytest.approx(2.772)
    assert pkt.points[-1][0] == pytest.approx(109.95)  # slot 15 at the end angle


def test_no_return_points_keep_their_slot_angle():
    d = list(FULL)
    d[1] = d[2] = None
    pkt = decode_packet(encode_packet(0.0, 15.0, d))
    assert len(pkt.points) == 14
    assert [p[0] for p in pkt.points][:2] == [0.0, 3.0]  # slots 0 and 3, 1 deg apart


def test_packet_across_zero():
    pkt = decode_packet(encode_packet(355.0, 7.5, FULL))
    assert pkt.points[0][0] == pytest.approx(355.0)
    assert pkt.points[-1][0] == pytest.approx(7.5)
    assert pkt.points[8][0] == pytest.approx((355.0 + 8 * 12.5 / 15) % 360)


@pytest.mark.parametrize('index, value, message', [
    (0, 0x00, 'header'), (107, 0x00, 'checksum'), (10, 0x00, 'checksum')])
def test_bad_packets(index, value, message):
    raw = bytearray(encode_packet(10.0, 25.0, FULL))
    raw[index] = value
    with pytest.raises(PacketError, match=message):
        decode_packet(bytes(raw))
    with pytest.raises(PacketError, match='length'):
        decode_packet(bytes(raw[:-1]))


def test_checksum_is_low_byte_of_sum():
    assert checksum(b'\xff\x02') == 0x01


def test_stream_resync_and_split_header():
    good = encode_packet(10.0, 25.0, FULL)
    data = b'\x00\xa5\x11' + good + good[:50]
    s = PacketStream()
    got = []
    for i in range(0, len(data), 7):
        got += s.feed(data[i:i + 7])
    got += s.feed(good[50:])
    assert len(got) == 2 and s.bad_packets == 0


def test_ros_angle_is_counter_clockwise():
    assert lidar_deg_to_ros_rad(0.0) == 0.0
    assert lidar_deg_to_ros_rad(90.0) == pytest.approx(math.radians(270.0))  # lidar CW 90 = right
    assert lidar_deg_to_ros_rad(270.0) == pytest.approx(math.radians(90.0))
    assert lidar_deg_to_ros_rad(10.0, offset_deg=10.0) == 0.0


def revolution(dist_mm):
    """22.5 packets of 16 points cover 360 deg; use 24 packets of 15 deg."""
    return [encode_packet(15.0 * k, 15.0 * k + 14.0, [dist_mm] * 16) for k in range(24)]


def test_scan_assembler_bins_one_revolution():
    asm = ScanAssembler(bins=360)
    outs = [asm.add(decode_packet(p)) for p in revolution(2000) * 3]
    done = [o for o in outs if o is not None]
    assert len(done) == 1   # first (partial) revolution dropped, third not finished yet
    ranges = done[0]
    assert len(ranges) == 360
    assert sum(math.isfinite(r) for r in ranges) > 300
    assert all(r == 2.0 for r in ranges if math.isfinite(r))


def test_scan_assembler_range_limits():
    asm = ScanAssembler(bins=360, range_min=0.1, range_max=1.0)
    outs = [asm.add(decode_packet(p)) for p in revolution(2000) * 3]
    ranges = [o for o in outs if o is not None][0]
    assert not any(math.isfinite(r) for r in ranges)   # 2 m > range_max


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.01
        return self.t


def test_listen_with_stream(tmp_path):
    data = b''.join(revolution(1500) * 3)
    chunks = [data[i:i + 200] for i in range(0, len(data), 200)]
    lines = []
    with open(tmp_path / 'raw.bin', 'wb') as raw:
        stream, asm = listen(lambda: chunks.pop(0) if chunks else b'', 3.0,
                             out=lines.append, raw_file=raw, clock=FakeClock(), bins=360)
    assert (tmp_path / 'raw.bin').read_bytes() == data
    assert stream.bad_packets == 0
    assert lines[0].startswith('first packet')
    assert any('min=1.50 m' in line for line in lines)


@pytest.mark.parametrize('name', ['protocol.py', 'listen.py', 'scan_node.py'])
def test_read_only(name):
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_lidar_n10p', name), encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr == 'write':
                assert name == 'listen.py' and ast.unparse(node.func.value) == 'raw_file'
            assert node.func.attr not in ('send', 'create_subscription')


def test_parameters_match_yaml():
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_lidar_n10p', 'scan_node.py'),
                          encoding='utf-8').read())
    params = next(n.value for n in tree.body
                  if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'PARAMETERS')
    names = {}
    for item in params.elts:
        default = item.elts[1]
        names[item.elts[0].value] = default.value if isinstance(default, ast.Constant) else 460800
    yaml_vals = {}
    for line in open(os.path.join(PKG, 'config', 'n10p.yaml'), encoding='utf-8'):
        m = re.match(r'^\s{4}(\w+):\s*([^#]*?)\s*(#.*)?$', line)
        if m and m.group(1) != 'ros__parameters':
            yaml_vals[m.group(1)] = m.group(2)
    assert set(yaml_vals) == set(names)
    for key, default in names.items():
        raw = yaml_vals[key]
        if isinstance(default, str):
            assert raw.strip("'") == default, key
        else:
            assert float(raw) == pytest.approx(float(default)), key
