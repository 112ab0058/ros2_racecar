"""Transports, exercised with fakes instead of the network or a UART."""
import threading
import time

import pytest

from wro2026_base_waverover.core import STOP_CMD
from wro2026_base_waverover.transport import (FakeTransport, HttpTransport, make_transport,
                                              SerialTransport, TransportError)


class FakeResponse:
    def __init__(self, body):
        self.body = body

    def read(self):
        return self.body.encode()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def test_http_url_and_reply():
    calls = []

    def urlopen(url, timeout):
        calls.append((url, timeout))
        return FakeResponse('{"T":1001,"v":12.2}')

    t = HttpTransport('192.168.0.16', timeout=0.3, urlopen=urlopen)
    assert t.send(STOP_CMD) == '{"T":1001,"v":12.2}'
    url = 'http://192.168.0.16/js?json=%7B%22T%22%3A1%2C%22L%22%3A0%2C%22R%22%3A0%7D'
    assert calls == [(url, 0.3)]


def test_http_failure_raises_transport_error():
    def urlopen(url, timeout):
        raise OSError('timed out')

    with pytest.raises(TransportError):
        HttpTransport('h', urlopen=urlopen).send(STOP_CMD)


class FakeSerial:
    def __init__(self, port, baud, timeout):
        self.args = (port, baud, timeout)
        self.written = []
        self.lines = [b'{"T":1001,"v":12.1}\n', b'garbage\n']
        self.closed = False

    def write(self, data):
        self.written.append(data)

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        time.sleep(0.01)
        return b''

    def close(self):
        self.closed = True


def test_serial_writes_one_json_line_and_reads_replies():
    ports = []

    def factory(port, baud, timeout):
        ports.append(FakeSerial(port, baud, timeout))
        return ports[-1]

    got = []
    done = threading.Event()

    def on_line(line):
        got.append(line)
        if len(got) == 2:
            done.set()

    t = SerialTransport('/dev/ttyTHS1', 115200, serial_factory=factory)
    t.start(on_line)
    assert t.send(STOP_CMD) is None
    assert done.wait(2.0)
    t.close()
    ser = ports[0]
    assert ser.args == ('/dev/ttyTHS1', 115200, 0.1)
    assert ser.written == [b'{"T":1,"L":0,"R":0}\n']
    assert got == ['{"T":1001,"v":12.1}', 'garbage']
    assert ser.closed


def test_serial_send_before_start_fails():
    with pytest.raises(TransportError):
        SerialTransport('/dev/null').send(STOP_CMD)


def test_fake_records_and_can_fail():
    t = FakeTransport(reply=lambda cmd: 'ok')
    assert t.send({'T': 130}) == 'ok'
    assert t.sent == ['{"T":130}']
    t.fail = True
    with pytest.raises(TransportError):
        t.send(STOP_CMD)
    assert t.sent == ['{"T":130}']


def test_make_transport():
    assert make_transport('http', host='x').host == 'x'
    assert make_transport('serial', port='/dev/x').port == '/dev/x'
    assert isinstance(make_transport('fake'), FakeTransport)
    with pytest.raises(ValueError):
        make_transport('can')
