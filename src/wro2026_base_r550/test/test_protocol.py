"""
Protocol tests.

The first group of frames was computed by hand from the encoding rules.
The OFFICIAL_* frames at the end are copied from WHEELTEC's documents
(see protocol.py). None of them is a recording from our R550.
"""
import math

import pytest

from wro2026_base_r550.protocol import (bcc, CMD_FRAME_SIZE, decode_feedback,
                                        encode_ackermann_command, encode_twist_command,
                                        FEEDBACK_FRAME_SIZE, FeedbackStream, FrameError,
                                        stop_command)

STOP = bytes.fromhex('7B 00 00 00 00 00 00 00 00 7B 7D')
FWD_0P5 = bytes.fromhex('7B 00 00 01 F4 00 00 00 00 8E 7D')        # vx 0.5 m/s
BACK_TURN = bytes.fromhex('7B 00 00 FF 38 00 00 03 E8 57 7D')      # vx -0.2, wz 1.0 rad/s
AKM_0P3_0P4 = bytes.fromhex('7B 00 00 01 2C 00 00 00 C8 9E 7D')    # 0.3 m/s, 0.4 rad
# vx 0.25 m/s, wz -0.1 rad/s, accel z = +1 g, gyro z raw 3754, 12.100 V
FEEDBACK = bytes.fromhex(
    '7B 00 00 FA 00 00 FF 9C 00 00 00 00 40 00 00 00 00 00 0E AA 2F 44 6D 7D')


def test_bcc_is_xor():
    assert bcc(b'') == 0
    assert bcc(bytes([0x7B])) == 0x7B
    assert bcc(bytes([0x7B, 0x01, 0xF4])) == 0x8E


def test_command_examples():
    assert stop_command() == STOP
    assert encode_twist_command(0.5, 0.0) == FWD_0P5
    assert encode_twist_command(-0.2, 1.0) == BACK_TURN
    assert encode_ackermann_command(0.3, 0.4) == AKM_0P3_0P4
    assert len(STOP) == CMD_FRAME_SIZE


def test_command_bytes_1_2_are_configurable():
    frame = encode_twist_command(0.0, 0.0, byte1=1)
    assert frame[1] == 1 and frame[9] == bcc(frame[:9]) and frame[-1] == 0x7D


def test_truncation_toward_zero_like_c():
    # 0.0019 m/s -> 1.9 mm/s -> 1 (C cast truncates); -0.0019 -> -1
    assert encode_twist_command(0.0019, 0.0)[3:5] == (1).to_bytes(2, 'big', signed=True)
    assert encode_twist_command(-0.0019, 0.0)[3:5] == (-1).to_bytes(2, 'big', signed=True)


@pytest.mark.parametrize('bad', [math.nan, math.inf, 40.0, -40.0])
def test_unsendable_values_raise(bad):
    with pytest.raises(ValueError):
        encode_twist_command(bad, 0.0)
    with pytest.raises(ValueError):
        encode_ackermann_command(0.0, bad * 10 if math.isfinite(bad) else bad)


def test_decode_feedback_example():
    fb = decode_feedback(FEEDBACK)
    assert len(FEEDBACK) == FEEDBACK_FRAME_SIZE
    assert fb.stop_flag == 0
    assert fb.vx == pytest.approx(0.25)
    assert fb.vy == 0.0
    assert fb.wz == pytest.approx(-0.1)
    assert (fb.accel_x, fb.accel_y) == (0.0, 0.0)
    assert fb.accel_z == pytest.approx(9.8)
    assert fb.gyro_z == pytest.approx(3754 * 0.00026644)
    assert fb.voltage == pytest.approx(12.1)


@pytest.mark.parametrize('index, value, message', [
    (0, 0x00, 'header'),
    (23, 0x00, 'tail'),
    (22, 0x00, 'BCC'),
    (3, 0xFB, 'BCC'),  # payload corrupted -> checksum no longer matches
])
def test_decode_rejects_bad_frames(index, value, message):
    frame = bytearray(FEEDBACK)
    frame[index] = value
    with pytest.raises(FrameError, match=message):
        decode_feedback(bytes(frame))
    with pytest.raises(FrameError, match='length'):
        decode_feedback(FEEDBACK[:-1])


def test_stream_resyncs_and_handles_split_chunks():
    s = FeedbackStream()
    noise = bytes.fromhex('00 7B 11 7D FF')
    data = noise + FEEDBACK + FEEDBACK
    frames = []
    for i in range(0, len(data), 5):  # arbitrary chunking
        frames += s.feed(data[i:i + 5])
    assert len(frames) == 2
    assert all(f.vx == pytest.approx(0.25) for f in frames)
    assert s.dropped_bytes >= len(noise) - 1
    assert s.bad_frames >= 1
    assert s.feed(b'') == []


def test_stream_drops_corrupted_frame_and_keeps_next():
    bad = bytearray(FEEDBACK)
    bad[5] ^= 0x01
    frames = FeedbackStream().feed(bytes(bad) + FEEDBACK)
    assert len(frames) == 1


# ---- Frames printed in WHEELTEC's official documents (2026-10-16) ----
# 串口通信控制与反馈 V1.0, section 3.1 (feedback example and 100 mm/s command)
OFFICIAL_FEEDBACK = bytes.fromhex(
    '7B 00 00 9B 00 00 FF DF 00 60 00 0C 40 A8 FF FD 00 06 00 1E 5B 87 82 7D')

# 阿克曼转向角度指令计算 V1.0, table: (frame, vx mm/s, wz rad/s x 1000).
# Only the forward rows: the "backward 80 mm/s" column has z values that do
# not match the document's own formula (about 10x too large), so it is not
# used. The "-300 mm/s" frame is FE D3 = -301 in the document (typo for
# FE D4), so -301 is used here.
OFFICIAL_COMMANDS = [
    ('7B 00 00 01 2C 00 00 00 00 56 7D', 300, 0),
    ('7B 00 00 FE D3 00 00 00 00 56 7D', -301, 0),
    ('7B 00 00 00 64 00 00 00 00 1F 7D', 100, 0),
    ('7B 00 00 FF 9C 00 00 00 00 18 7D', -100, 0),
    ('7B 00 00 00 F0 00 00 00 3E B5 7D', 240, 62),     # left 5 deg
    ('7B 00 00 00 F0 00 00 00 78 F3 7D', 240, 120),    # left 10 deg
    ('7B 00 00 00 F0 00 00 00 B0 3B 7D', 240, 176),    # left 15 deg
    ('7B 00 00 00 F0 00 00 00 E5 6E 7D', 240, 229),    # left 20 deg
    ('7B 00 00 00 F0 00 00 FF BC C8 7D', 240, -68),    # right 5 deg
    ('7B 00 00 00 F0 00 00 FF 70 04 7D', 240, -144),   # right 10 deg
    ('7B 00 00 00 F0 00 00 FF 1A 6E 7D', 240, -230),   # right 15 deg
    ('7B 00 00 00 F0 00 00 FE B5 C0 7D', 240, -331),   # right 20 deg
]


@pytest.mark.parametrize('frame_hex, vx_mm, wz_milli', OFFICIAL_COMMANDS)
def test_official_command_frames(frame_hex, vx_mm, wz_milli):
    assert encode_twist_command(vx_mm / 1000, wz_milli / 1000) == bytes.fromhex(frame_hex)


def test_official_feedback_example():
    fb = decode_feedback(OFFICIAL_FEEDBACK)
    assert fb.stop_flag == 0
    assert fb.vx == pytest.approx(0.155)
    assert fb.wz == pytest.approx(-0.033)
    # the document divides by 1672 (accel) and 3753 (gyro)
    assert fb.accel_x == pytest.approx(96 / 1672, rel=1e-3)
    assert fb.accel_z == pytest.approx(16552 / 1672, rel=1e-3)
    assert fb.gyro_z == pytest.approx(30 / 3753, rel=1e-3)
    assert fb.voltage == pytest.approx(23.431)


@pytest.mark.parametrize('deg, wz_milli', [(5, 62), (10, 120), (15, 176), (20, 229),
                                           (-5, -68), (-10, -144), (-15, -230),
                                           (-20, -331)])
def test_official_steering_formula(deg, wz_milli):
    # Document's formula: R = H / tan(angle) + W / 2, wz = vx / R
    # (H = wheelbase, W = track; 0.322 m for the example car), 240 mm/s
    radius = 0.322 / math.tan(math.radians(deg)) + 0.322 / 2
    assert int(0.24 / radius * 1000) == wz_milli
