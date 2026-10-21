"""
Listen to the N10P without ROS (read only), print one summary line per second.

    ros2 run wro2026_lidar_n10p listen --port /dev/ttyUSB0 --seconds 10 --raw-out n10p.bin

Shows packets per second, revolutions per second, bad packets, the number of
points in the last revolution and the closest / farthest return. --raw-out
saves every received byte so the recording can be decoded again later.
Nothing is ever written to the serial port.
"""
import argparse
import math
import sys
import time

from .protocol import BAUD_RATE, PacketStream, ScanAssembler


def listen(read, seconds, out=print, raw_file=None, clock=time.monotonic, bins=480):
    """Read with read() for `seconds`; return (stream, assembler)."""
    stream = PacketStream()
    scans = ScanAssembler(bins=bins)
    start = clock()
    next_print = start + 1.0
    packets = revs = total = 0
    last = None
    first = True
    while True:
        now = clock()
        if now - start >= seconds:
            break
        chunk = read()
        if chunk:
            total += len(chunk)
            if raw_file is not None:
                raw_file.write(chunk)
            for pkt in stream.feed(chunk):
                if first:
                    first = False
                    out(f'first packet: start {pkt.start_deg:.2f} deg, end {pkt.end_deg:.2f} deg, '
                        f'{len(pkt.points)} valid points')
                packets += 1
                ranges = scans.add(pkt)
                if ranges is not None:
                    revs += 1
                    last = ranges
        if now >= next_print:
            next_print += 1.0
            line = (f't={now - start:4.1f}s bytes={total} packets/s={packets} rev/s={revs} '
                    f'bad={stream.bad_packets} dropped_bytes={stream.dropped_bytes}')
            if last is not None:
                valid = [r for r in last if math.isfinite(r)]
                if valid:
                    line += f' | beams={len(valid)}/{len(last)} min={min(valid):.2f} m ' \
                            f'max={max(valid):.2f} m'
            out(line)
            packets = revs = 0
    out(f'done: {total} bytes, bad_packets={stream.bad_packets}, '
        f'dropped_bytes={stream.dropped_bytes}')
    return stream, scans


def main(argv=None):
    """Command-line entry point."""
    ap = argparse.ArgumentParser(description='Listen to an LSLIDAR N10P (read only).')
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=BAUD_RATE)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--raw-out')
    args = ap.parse_args(argv)
    try:
        import serial
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f'cannot open {args.port}: {e}', file=sys.stderr)
        return 1
    raw = open(args.raw_out, 'wb') if args.raw_out else None
    try:
        print(f'listening on {args.port} @ {args.baud} for {args.seconds:g} s (read only)')
        listen(lambda: ser.read(ser.in_waiting or 1), args.seconds, raw_file=raw)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if raw is not None:
            raw.close()
            print(f'raw bytes saved to {args.raw_out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
