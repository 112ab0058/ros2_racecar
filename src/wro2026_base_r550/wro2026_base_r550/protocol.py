"""
WHEELTEC STM32 serial protocol: encode commands, decode feedback (no rclpy).

Source (2026-10-16): WHEELTEC's own document "串口通信控制与反馈" V1.0
(2024-06-04, file 串口通信控制与反馈_2026-8-12.pdf in the R550 vendor
package) and "阿克曼转向角度指令计算" V1.0 (2021-10-26). test_protocol.py
checks every example frame printed in those documents. Not yet checked
against packets recorded from our R550 (still UNVERIFIED on the car).

Earlier sources (third-party copies of WHEELTEC's ROS driver), kept for
reference: robotverseny/drivers commit 26a63c5, shupx/akm_driver 075e0f1.

Serial: serial port 3 (middle USB-C on the C30D), 115200 baud, 8N1.

Command frame, host -> STM32, 11 bytes:
  [0] 0x7B header
  [1] [2] reserved, no meaning (official doc); send 0
  [3:5] x target speed, int16 big-endian, mm/s
  [5:7] y target speed, int16 big-endian, mm/s (omni chassis only)
  [7:9] z target angular velocity, int16 big-endian, rad/s x 1000.
        Ackermann chassis also use angular velocity here: the STM32
        converts (vx, wz) into a steering angle itself and rejects radii
        smaller than the minimum turning radius (official doc 4.4).
  [9]  BCC = XOR of bytes 0..8
  [10] 0x7D tail

Feedback frame, STM32 -> host, 24 bytes (20 Hz by default, RATE_20_HZ):
  [0] 0x7B, [1] flag_stop (0 = motors enabled, other = disabled)
  [2:4] [4:6] [6:8] x / y speed (mm/s), z angular velocity (rad/s x 1000)
  [8:14]  accelerometer x/y/z raw int16 BE; official: raw / 1672 = m/s^2
  [14:20] gyroscope x/y/z raw int16 BE; official: raw / 3753 = rad/s
  [20:22] battery voltage, mV
  [22] BCC = XOR of bytes 0..21, [23] 0x7D

Sign convention: the official doc's text says z is clockwise-positive, but
its worked examples and command table use left turn = positive z, i.e.
counter-clockwise positive like ROS REP-103. Check on the car.
"""
from collections import namedtuple
import math

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
CMD_FRAME_SIZE = 11
FEEDBACK_FRAME_SIZE = 24
BAUD_RATE = 115200

SPEED_SCALE = 1000.0           # m/s -> mm/s, rad/s -> mrad/s
STEERING_SCALE = 1000.0 / 2.0  # third-party driver only, see encode_ackermann_command
ACCEL_LSB_PER_G = 16384.0      # +-2 g full scale
STANDARD_G = 9.8               # 16384 / 9.8 = 1671.8 (official doc: divide by 1672)
GYRO_RAD_PER_LSB = 0.00026644  # 1 / 65.5 / 57.30 = 1 / 3753.2 (official doc: divide by 3753)

Feedback = namedtuple(
    'Feedback',
    'stop_flag vx vy wz accel_x accel_y accel_z gyro_x gyro_y gyro_z voltage')


class FrameError(ValueError):
    """A byte sequence is not a valid frame."""


def bcc(data):
    """Return the XOR of all bytes (the protocol's "BCC" check byte)."""
    out = 0
    for b in data:
        out ^= b
    return out


def _int16_wire(value, scale, name):
    """Scale a float and convert it like the C source (truncate toward zero)."""
    if not math.isfinite(value):
        raise ValueError(f'{name} is not finite: {value!r}')
    raw = int(value * scale)  # C: short transition = value * 1000
    if not -32768 <= raw <= 32767:
        raise ValueError(f'{name}={value} does not fit in int16 after x{scale:g}')
    return raw.to_bytes(2, 'big', signed=True)


def _command(x_bytes, y_bytes, z_bytes, byte1, byte2):
    body = bytes([FRAME_HEADER, byte1 & 0xFF, byte2 & 0xFF]) + x_bytes + y_bytes + z_bytes
    return body + bytes([bcc(body), FRAME_TAIL])


def encode_twist_command(vx, wz, *, vy=0.0, byte1=0, byte2=0):
    """
    Encode a command: vx, vy in m/s, wz in rad/s.

    This is the official format (bytes 1-2 reserved, 0). For the Ackermann
    car, wz is the body angular velocity, not the steering angle.
    """
    return _command(_int16_wire(vx, SPEED_SCALE, 'vx'), _int16_wire(vy, SPEED_SCALE, 'vy'),
                    _int16_wire(wz, SPEED_SCALE, 'wz'), byte1, byte2)


def encode_ackermann_command(speed, steering_angle, byte1=0, byte2=0):
    """
    Do not use on the R550. Third-party drivers only.

    The two third-party ROS drivers write steering_angle * 1000 / 2 into
    bytes 7-8. The official WHEELTEC document says bytes 7-8 are the
    angular velocity, so the STM32 would read this value as rad/s x 1000.
    Kept only until the STM32 source confirms there is no such mode.
    """
    return _command(_int16_wire(speed, SPEED_SCALE, 'speed'), b'\x00\x00',
                    _int16_wire(steering_angle, STEERING_SCALE, 'steering_angle'),
                    byte1, byte2)


def stop_command(byte1=0, byte2=0):
    """Encode an all-zero command."""
    return encode_twist_command(0.0, 0.0, vy=0.0, byte1=byte1, byte2=byte2)


def _i16(frame, i):
    return int.from_bytes(frame[i:i + 2], 'big', signed=True)


def check_feedback_frame(frame):
    """Raise FrameError unless frame is a complete feedback frame with a valid BCC."""
    if len(frame) != FEEDBACK_FRAME_SIZE:
        raise FrameError(f'length {len(frame)} != {FEEDBACK_FRAME_SIZE}')
    if frame[0] != FRAME_HEADER:
        raise FrameError(f'header 0x{frame[0]:02X} != 0x{FRAME_HEADER:02X}')
    if frame[-1] != FRAME_TAIL:
        raise FrameError(f'tail 0x{frame[-1]:02X} != 0x{FRAME_TAIL:02X}')
    expected = bcc(frame[:22])
    if frame[22] != expected:
        raise FrameError(f'BCC 0x{frame[22]:02X} != 0x{expected:02X}')


def decode_feedback(frame):
    """Decode a 24-byte feedback frame into SI units (m/s, rad/s, m/s^2, V)."""
    frame = bytes(frame)
    check_feedback_frame(frame)
    accel_scale = STANDARD_G / ACCEL_LSB_PER_G
    return Feedback(
        stop_flag=frame[1],
        vx=_i16(frame, 2) / SPEED_SCALE,
        vy=_i16(frame, 4) / SPEED_SCALE,
        wz=_i16(frame, 6) / SPEED_SCALE,
        accel_x=_i16(frame, 8) * accel_scale,
        accel_y=_i16(frame, 10) * accel_scale,
        accel_z=_i16(frame, 12) * accel_scale,
        gyro_x=_i16(frame, 14) * GYRO_RAD_PER_LSB,
        gyro_y=_i16(frame, 16) * GYRO_RAD_PER_LSB,
        gyro_z=_i16(frame, 18) * GYRO_RAD_PER_LSB,
        voltage=int.from_bytes(frame[20:22], 'big') / 1000.0,
    )


class FeedbackStream:
    """
    Reassemble feedback frames from an arbitrary byte stream.

    feed(chunk) returns the decoded frames that became complete. Bytes that
    cannot start a valid frame are skipped (counted in self.dropped_bytes);
    frames with a bad tail or BCC are dropped (self.bad_frames).
    """

    def __init__(self):
        self.buf = bytearray()
        self.dropped_bytes = 0
        self.bad_frames = 0

    def feed(self, chunk):
        """Add received bytes; return a list of Feedback."""
        self.buf.extend(chunk)
        out = []
        while True:
            start = self.buf.find(FRAME_HEADER)
            if start < 0:
                self.dropped_bytes += len(self.buf)
                self.buf.clear()
                return out
            if start:
                self.dropped_bytes += start
                del self.buf[:start]
            if len(self.buf) < FEEDBACK_FRAME_SIZE:
                return out
            frame = bytes(self.buf[:FEEDBACK_FRAME_SIZE])
            try:
                out.append(decode_feedback(frame))
            except FrameError:
                self.bad_frames += 1
                self.dropped_bytes += 1
                del self.buf[:1]  # resync on the next 0x7B
                continue
            del self.buf[:FEEDBACK_FRAME_SIZE]
