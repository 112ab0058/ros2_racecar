"""
LSLIDAR N10P serial protocol and scan assembly (no rclpy). READ ONLY.

Source: LSLIDAR's official ROS 2 driver, branch "M10P/N10P"
(github.com/Lslidar/Lslidar_ROS2_driver, commit 8bb760c, 2023-05-29),
lslidar_driver.cc, lidar_name == "N10_P". Not yet checked against bytes
recorded from our lidar (UNVERIFIED on hardware).

Serial: 460800 baud, 8N1. The N10P streams by itself after power-on; the
official driver sends nothing to an N10P at start-up, and neither do we.

Checked on our N10P (2026-10-17, 3324 packets recorded, 0 bad): 333
packets/s, 10 revolutions/s, about 10.2 deg per packet. Byte 2 is always
0x6C (108, the packet length) and byte 3 is 16 (points per packet).

Packet, 108 bytes:
  [0:2]     0xA5 0x5A header
  [2]       0x6C = 108 (packet length; seen on our lidar)
  [3]       16 (points per packet; seen on our lidar)
  [4]       varies 64-70 on our lidar, meaning UNKNOWN (not used)
  [5:7]     start angle, uint16 big-endian, degrees x 100
  [7:103]   16 points x 6 bytes: distance (uint16 BE, mm), intensity (uint8),
            then a second echo in the same layout. 0xFFFF = no return,
            and our lidar also sends distance 0 for no return (18 % of
            points in the 2026-10-17 recording). The vendor's ROS 2 driver
            (lslidar_x10_driver.cpp, model "N10Plus": 108-byte packets,
            460800 baud, angles at bytes 5 / 105, data from 7) reads every
            point as two echoes of 3 bytes, offset group*6 + echo*3, and
            publishes them as a MultiEchoLaserScan (checked 2026-11-27). /scan
            uses the first echo; the second echo and both intensities are kept
            (Packet.echo2, ScanAssembler.intensities / ranges2 / intensities2).
  [103:105] small counter (0-4 on our lidar), meaning UNKNOWN (not used)
  [105:107] end angle, uint16 big-endian, degrees x 100
  [107]     checksum = sum of bytes 0..106, low 8 bits

Angles are clockwise seen from above (lidar convention). ROS uses
counter-clockwise, so ROS angle = 360 deg - lidar angle (as the official
driver does when it fills LaserScan).

Points in a packet are spread evenly from the start to the end angle. The
official driver divides by the number of *valid* points; we use the slot
index (16 slots, 15 gaps), which keeps every point at its own angle when
some points have no return.
"""
from collections import namedtuple
import math

HEADER = b'\xa5\x5a'
PACKET_SIZE = 108
POINTS_PER_PACKET = 16
POINT_STRIDE = 6
DATA_START = 7
START_ANGLE_AT = 5
END_ANGLE_AT = 105
NO_RETURN = 0xFFFF  # distance 0 is also treated as no return
BAUD_RATE = 460800

# points: first echo [(deg, range_m, intensity)]; echo2: the second echo, same layout
Packet = namedtuple('Packet', 'start_deg end_deg points echo2', defaults=((),))


class PacketError(ValueError):
    """Bytes are not a valid N10P packet."""


def checksum(data):
    """Sum of all bytes, low 8 bits."""
    return sum(data) & 0xFF


def _u16(data, i):
    return (data[i] << 8) | data[i + 1]


def decode_packet(packet):
    """Decode one 108-byte packet; raise PacketError if it is not valid."""
    packet = bytes(packet)
    if len(packet) != PACKET_SIZE:
        raise PacketError(f'length {len(packet)} != {PACKET_SIZE}')
    if packet[:2] != HEADER:
        raise PacketError('header is not A5 5A')
    if packet[-1] != checksum(packet[:-1]):
        raise PacketError(f'checksum 0x{packet[-1]:02X} != 0x{checksum(packet[:-1]):02X}')
    start = _u16(packet, START_ANGLE_AT) / 100.0
    end = _u16(packet, END_ANGLE_AT) / 100.0
    if start > 360.0 or end > 360.0:
        raise PacketError(f'angle out of range: start {start}, end {end}')
    span = end - start if end >= start else end + 360.0 - start
    step = span / (POINTS_PER_PACKET - 1)
    points, echo2 = [], []
    for n in range(POINTS_PER_PACKET):
        i = DATA_START + n * POINT_STRIDE
        deg = (start + step * n) % 360.0
        dist = _u16(packet, i)
        if dist not in (NO_RETURN, 0):
            points.append((deg, dist / 1000.0, packet[i + 2]))
        dist2 = _u16(packet, i + 3)
        if dist2 not in (NO_RETURN, 0):
            echo2.append((deg, dist2 / 1000.0, packet[i + 5]))
    return Packet(start, end, points, echo2)


def encode_packet(start_deg, end_deg, distances_mm, intensity=100, echo2_mm=None,
                  intensity2=0):
    """
    Build a packet (for tests and simulators). distances_mm: 16 values, None = no return.

    echo2_mm: the second echo's 16 distances (default: the same as the first echo);
    intensity2: its intensity.
    """
    if len(distances_mm) != POINTS_PER_PACKET:
        raise ValueError('need 16 distances')
    body = bytearray(PACKET_SIZE - 1)
    body[0:2] = HEADER
    body[START_ANGLE_AT:START_ANGLE_AT + 2] = int(round(start_deg * 100)).to_bytes(2, 'big')
    body[END_ANGLE_AT:END_ANGLE_AT + 2] = int(round(end_deg * 100)).to_bytes(2, 'big')
    second = distances_mm if echo2_mm is None else echo2_mm
    for n, d in enumerate(distances_mm):
        i = DATA_START + n * POINT_STRIDE
        raw = NO_RETURN if d is None else int(d)
        body[i:i + 2] = raw.to_bytes(2, 'big')
        body[i + 2] = 0 if d is None else intensity
        d2 = second[n]
        raw2 = NO_RETURN if d2 is None else int(d2)
        body[i + 3:i + 5] = raw2.to_bytes(2, 'big')
        body[i + 5] = 0 if d2 is None else intensity2
    return bytes(body) + bytes([checksum(body)])


class PacketStream:
    """Reassemble packets from a byte stream (bad packets counted and skipped)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_packets = 0
        self.dropped_bytes = 0

    def feed(self, chunk):
        """Add bytes; return the list of decoded Packets that became complete."""
        self.buf.extend(chunk)
        out = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                keep = 1 if self.buf[-1:] == HEADER[:1] else 0  # a header may be split
                self.dropped_bytes += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if start:
                self.dropped_bytes += start
                del self.buf[:start]
            if len(self.buf) < PACKET_SIZE:
                return out
            try:
                out.append(decode_packet(self.buf[:PACKET_SIZE]))
            except PacketError:
                self.bad_packets += 1
                self.dropped_bytes += 1
                del self.buf[:1]
                continue
            del self.buf[:PACKET_SIZE]


def lidar_deg_to_ros_rad(deg, offset_deg=0.0):
    """Lidar clockwise degrees -> ROS counter-clockwise radians in [0, 2*pi)."""
    return math.radians((360.0 - deg + offset_deg) % 360.0)


class ScanAssembler:
    """
    Collect points into one revolution and bin them into a fixed-size scan.

    A revolution ends when the angle wraps (a packet whose start angle is
    smaller than the previous one). Fixed bins (default 720 = 0.5 deg) keep
    the number of beams constant, which slam_toolbox requires.
    add(packet) returns a list of ranges (inf = no return) when a revolution
    is complete, else None. Then also: intensities (of the point each bin
    kept, 0 = none), ranges2 / intensities2 (the second echo, binned the same
    way). Two points in one bin: the nearer one is kept, with its intensity.
    range_offset (m) is subtracted from every distance before the range limits:
    the lidar reads a constant amount too far (see config/n10p.yaml).
    """

    def __init__(self, bins=720, range_min=0.1, range_max=25.0, offset_deg=0.0,
                 range_offset=0.0):
        self.bins = bins
        self.range_min = range_min
        self.range_max = range_max
        self.offset_deg = offset_deg
        self.range_offset = range_offset
        self.points = []
        self.points2 = []
        self.last_start = None
        self.revolutions = 0
        self.intensities = self.ranges2 = self.intensities2 = None

    def add(self, packet):
        """Add one packet; return finished ranges or None."""
        done = None
        if self.last_start is not None and packet.start_deg < self.last_start:
            if self.revolutions > 0:  # the first revolution is partial: drop it
                done = self._bin()
            self.revolutions += 1
            self.points = []
            self.points2 = []
        self.last_start = packet.start_deg
        self.points.extend(packet.points)
        self.points2.extend(packet.echo2)
        return done

    def _bin(self):
        ranges, self.intensities = self._bin_points(self.points)
        self.ranges2, self.intensities2 = self._bin_points(self.points2)
        return ranges

    def _bin_points(self, points):
        ranges = [math.inf] * self.bins
        intensities = [0.0] * self.bins
        width = 2.0 * math.pi / self.bins
        for deg, rng, inten in points:
            rng -= self.range_offset
            if not self.range_min <= rng <= self.range_max:
                continue
            k = int(round(lidar_deg_to_ros_rad(deg, self.offset_deg) / width)) % self.bins
            if rng < ranges[k]:
                ranges[k], intensities[k] = rng, float(inten)
        return ranges, intensities


def parse_sectors(text):
    """
    Parse "start:end,start:end" (ROS degrees: 0 = front, counter-clockwise).

    A sector may wrap past 360 (e.g. "350:10"). Empty text means no sectors.
    """
    sectors = []
    for part in text.replace(' ', '').split(','):
        if not part:
            continue
        a, b = part.split(':')
        sectors.append((float(a) % 360.0, float(b) % 360.0))
    return sectors


def mask_ranges(ranges, sectors):
    """
    Return a copy of ranges with every beam inside a sector set to inf.

    Used to hide parts of our own car (Orin, bracket) that the lidar sees.
    Beam k points at k * 360 / len(ranges) degrees (ROS frame).
    """
    if not sectors:
        return list(ranges)
    n = len(ranges)
    out = list(ranges)
    for k in range(n):
        deg = k * 360.0 / n
        for a, b in sectors:
            inside = a <= deg <= b if a <= b else (deg >= a or deg <= b)
            if inside:
                out[k] = math.inf
                break
    return out
