"""
Replaceable transports between the driver loop and the ESP32 (no rclpy).

All transports implement:
  start(on_line)  begin delivering asynchronous reply lines (serial only)
  send(cmd)       send one command dict; return the reply text (HTTP),
                  None (serial: replies arrive through on_line) or raise
                  TransportError
  close()

``FakeTransport`` records every command for tests and never touches
hardware.
"""
import threading
import urllib.parse
import urllib.request

from .core import encode_command


class TransportError(Exception):
    """A command could not be delivered."""


class Transport:
    """Interface shared by all transports."""

    name = 'base'

    def start(self, on_line):
        """Begin delivering asynchronous reply lines to on_line(line)."""

    def send(self, cmd):
        """Send one command; see the module docstring for the return value."""
        raise NotImplementedError

    def close(self):
        """Release the connection."""


class HttpTransport(Transport):
    """ESP32 web API: GET http://<host>/js?json=<command>."""

    name = 'http'

    def __init__(self, host, timeout=0.3, urlopen=None):
        self.host = host
        self.timeout = timeout
        self.urlopen = urlopen or urllib.request.urlopen

    def url(self, cmd):
        """Return the request URL for cmd."""
        return f'http://{self.host}/js?json=' + urllib.parse.quote(encode_command(cmd))

    def send(self, cmd):
        """Send cmd and return the reply body."""
        try:
            with self.urlopen(self.url(cmd), timeout=self.timeout) as resp:
                return resp.read().decode(errors='ignore')
        except Exception as e:
            raise TransportError(str(e)) from e


class SerialTransport(Transport):
    """UART: one JSON object per line, replies read by a background thread."""

    name = 'serial'

    def __init__(self, port, baud=115200, read_timeout=0.1, serial_factory=None):
        self.port = port
        self.baud = baud
        self.read_timeout = read_timeout
        self.serial_factory = serial_factory
        self.ser = None
        self.running = False
        self.reader = None
        self.write_lock = threading.Lock()

    def start(self, on_line):
        """Open the port and start the reader thread."""
        factory = self.serial_factory
        if factory is None:
            import serial  # python3-serial
            factory = serial.Serial
        self.ser = factory(self.port, self.baud, timeout=self.read_timeout)
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, args=(on_line,), daemon=True)
        self.reader.start()

    def _read_loop(self, on_line):
        while self.running:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except Exception:
                if not self.running:
                    return
                continue
            if line:
                on_line(line)

    def send(self, cmd):
        """Write cmd followed by a newline."""
        if self.ser is None:
            raise TransportError('serial port not open')
        try:
            with self.write_lock:
                self.ser.write((encode_command(cmd) + '\n').encode())
        except Exception as e:
            raise TransportError(str(e)) from e
        return None

    def close(self):
        """Stop the reader and close the port."""
        self.running = False
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        if self.ser is not None:
            self.ser.close()


class FakeTransport(Transport):
    """
    Record commands instead of sending them (tests, dry runs).

    reply(cmd) -> str | None decides what send() returns; fail=True makes
    every send raise TransportError.
    """

    name = 'fake'

    def __init__(self, reply=None, fail=False):
        self.reply = reply
        self.fail = fail
        self.sent = []  # wire strings, in order
        self.on_line = None
        self.closed = False

    def start(self, on_line):
        """Remember on_line so tests can push asynchronous replies."""
        self.on_line = on_line

    def send(self, cmd):
        """Record cmd and return the scripted reply."""
        if self.fail:
            raise TransportError('fake transport failure')
        self.sent.append(encode_command(cmd))
        return self.reply(cmd) if self.reply else None

    def push_line(self, line):
        """Deliver an asynchronous reply line, as a serial port would."""
        if self.on_line:
            self.on_line(line)

    def close(self):
        """Mark the transport closed."""
        self.closed = True


def make_transport(kind, host='192.168.4.1', http_timeout=0.3, port='/dev/ttyTHS1',
                   baud=115200):
    """Build a transport from the driver parameters."""
    if kind == 'http':
        return HttpTransport(host, http_timeout)
    if kind == 'serial':
        return SerialTransport(port, baud)
    if kind == 'fake':
        return FakeTransport()
    raise ValueError("transport must be 'serial', 'http' or 'fake'")
