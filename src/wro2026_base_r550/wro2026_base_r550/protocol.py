"""
WHEELTEC STM32 serial protocol: encode commands, decode feedback (no rclpy).

UNVERIFIED: written from two third-party copies of WHEELTEC's ROS driver,
not from WHEELTEC documentation and not tested on our R550:

* robotverseny/drivers, turn_on_wheeltec_robot (ROS 2 Jazzy port),
  commit 26a63c5c8cbcea5b9239e544a9298b18039560e7 (2025-02-04),
  include/turn_on_wheeltec_robot/wheeltec_robot.h, src/wheeltec_robot.cpp
* shupx/akm_driver, turn_on_wheeltec_robot (ROS 1, WHEELTEC Ackermann),
  commit 075e0f1c88147cac5318ba9e2d15f5a6a399cc66 (2021-03-26)

Both agree on everything below except bytes 1-2 of the command frame
(see encode_twist_command). Serial: 115200 baud, 8N1 (both sources).

Command frame, host -> STM32, 11 bytes:
  [0] 0x7B header
  [1] reserved (robotverseny: 0; shupx: 1 "product type")
  [2] reserved (shupx: "robot enable flag", 0)
  [3:5] x speed, int16 big-endian, mm/s
  [5:7] y speed, int16 big-endian, mm/s (omni chassis only)
  [7:9] z: angular velocity in mrad/s (Twist mode), or
        steering angle in rad x 1000 / 2 (Ackermann mode)
  [9]  BCC = XOR of bytes 0..8
  [10] 0x7D tail

Feedback frame, STM32 -> host, 24 bytes:
  [0] 0x7B, [1] stop flag
  [2:4] [4:6] [6:8] x / y speed (mm/s), z angular velocity (mrad/s), int16 BE
  [8:14]  accelerometer x/y/z raw int16 BE, +-2 g (16384 LSB per g)
  [14:20] gyroscope x/y/z raw int16 BE, +-500 deg/s (65.5 LSB per deg/s)
  [20:22] battery voltage, mV, 16 bit BE
  [22] BCC = XOR of bytes 0..21, [23] 0x7D
"""
from collections import namedtuple
import math

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
CMD_FRAME_SIZE = 11
FEEDBACK_FRAME_SIZE = 24
BAUD_RATE = 115200

SPEED_SCALE = 1000.0           # m/s -> mm/s, rad/s -> mrad/s
STEERING_SCALE = 1000.0 / 2.0  # Ackermann mode: rad -> wire units (source: x1000/2)
ACCEL_LSB_PER_G = 16384.0      # +-2 g full scale
STANDARD_G = 9.8               # the source divides by 1671.84 = 32768 / 19.6
GYRO_RAD_PER_LSB = 0.00026644  # 1 / 65.5 / 57.30 (+-500 deg/s full scale)

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


def encode_twist_command(vx, wz, vy=0.0, byte1=0, byte2=0):
    """
    Encode a Twist-mode command: vx, vy in m/s, wz in rad/s.

    byte1 / byte2 are 0 in the ROS 2 source; the ROS 1 Ackermann source sends
    byte1 = 1. Which one our R550 firmware expects is UNKNOWN.
    """
    return _command(_int16_wire(vx, SPEED_SCALE, 'vx'), _int16_wire(vy, SPEED_SCALE, 'vy'),
                    _int16_wire(wz, SPEED_SCALE, 'wz'), byte1, byte2)


def encode_ackermann_command(speed, steering_angle, byte1=0, byte2=0):
    """
    Encode an Ackermann-mode command: speed in m/s, steering angle in rad.

    Both sources write steering_angle * 1000 / 2 into bytes 7-8 and leave
    y = 0. Whether the R550 firmware reads (v, w) or (v, steering angle) is
    UNKNOWN until it is checked on the car.
    """
    return _command(_int16_wire(speed, SPEED_SCALE, 'speed'), b'\x00\x00',
                    _int16_wire(steering_angle, STEERING_SCALE, 'steering_angle'),
                    byte1, byte2)


def stop_command(byte1=0, byte2=0):
    """Encode an all-zero command."""
    return encode_twist_command(0.0, 0.0, 0.0, byte1, byte2)


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
