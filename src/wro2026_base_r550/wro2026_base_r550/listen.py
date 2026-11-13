"""
Listen to the R550's STM32 without sending anything (no ROS needed).

    ros2 run wro2026_base_r550 listen --port /dev/ttyACM0
    python3 -m wro2026_base_r550.listen --port /dev/ttyACM0 --seconds 10 --raw-out r550.bin

Opens the serial port, decodes the 24-byte feedback frames and prints one
summary line per second. --raw-out saves every received byte so the
recording can be decoded again later (and used as a test sample).

This tool never writes to the serial port. Find the port name with
`ls -l /dev/serial/by-id/` after plugging in the middle USB-C (serial 3).
"""
import argparse
import sys
import time

from .feedback import feedback_summary, RateMeter
from .protocol import BAUD_RATE, FeedbackStream


def open_port(port, baud):
    """Open the serial port for reading (pyserial imported here so tests need no pyserial)."""
    import serial
    return serial.Serial(port, baud, timeout=0.05)


def listen(read, seconds, out=print, raw_file=None, clock=time.monotonic):
    """
    Read with read() for `seconds`; print a summary every second.

    read() returns the bytes available (possibly b''). Returns the
    FeedbackStream so callers and tests can inspect the counters.
    """
    stream = FeedbackStream()
    rate = RateMeter(window=1.0)
    start = clock()
    next_print = start + 1.0
    last = None
    first_hex_printed = False
    total = 0
    while True:
        now = clock()
        if now - start >= seconds:
            break
        chunk = read()
        if chunk:
            total += len(chunk)
            if raw_file is not None:
                raw_file.write(chunk)
            for fb in stream.feed(chunk):
                rate.tick(now)
                last = fb
                if not first_hex_printed:
                    first_hex_printed = True
                    out('first valid frame decoded')
        if now >= next_print:
            next_print += 1.0
            head = (f't={now - start:4.1f}s bytes={total} rate={rate.rate(now):4.1f} Hz '
                    f'bad_frames={stream.bad_frames} dropped_bytes={stream.dropped_bytes}')
            out(head + (' | ' + feedback_summary(last) if last else ' | no valid frame yet'))
    out(f'done: {total} bytes, bad_frames={stream.bad_frames}, '
        f'dropped_bytes={stream.dropped_bytes}')
    return stream


def main(argv=None):
    """Command-line entry point."""
    ap = argparse.ArgumentParser(description='Listen to the R550 STM32 (read only).')
    ap.add_argument('--port', required=True, help='serial device, e.g. /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=BAUD_RATE)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--raw-out', help='save every received byte to this file')
    args = ap.parse_args(argv)

    try:
        ser = open_port(args.port, args.baud)
    except Exception as e:  # serial.SerialException, PermissionError, ImportError
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
