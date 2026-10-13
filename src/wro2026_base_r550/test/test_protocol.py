"""
Protocol tests.

The sources contain no captured packets, so the example frames below were
computed by hand from the encoding rules in the sources' C++ code
(see protocol.py). They are NOT recordings from an R550.
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
