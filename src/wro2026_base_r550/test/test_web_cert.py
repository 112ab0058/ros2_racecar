"""HTTPS for the phone page: the Orin's own CA, the page certificate, the https server."""
import os
import shutil
import ssl
import subprocess
import threading
import urllib.error
import urllib.request

import pytest
from wro2026_base_r550.web_cert import ensure_ca, make_server_cert
from wro2026_base_r550.web_teleop import make_server, WebTeleop

pytestmark = pytest.mark.skipif(shutil.which('openssl') is None, reason='needs openssl')


def text_of(crt):
    return subprocess.run(['openssl', 'x509', '-in', crt, '-noout', '-text'],
                          check=True, capture_output=True, text=True).stdout


def test_ca_is_made_once_and_its_key_is_private(tmp_path):
    crt = ensure_ca(str(tmp_path / 'web'))
    key = tmp_path / 'web' / 'ca.key'
    assert oct(key.stat().st_mode & 0o777) == '0o600'
    first = open(crt).read()
    assert ensure_ca(str(tmp_path / 'web')) == crt and open(crt).read() == first   # kept
    assert 'CA:TRUE' in text_of(crt)


def test_page_certificate_follows_the_iphone_rules(tmp_path):
    d = str(tmp_path)
    ca = ensure_ca(d)
    crt, key = make_server_cert(d, ['172.20.10.2', '100.73.223.57'], ['112ab-jno.netbird.cloud'])
    subprocess.run(['openssl', 'verify', '-CAfile', ca, crt], check=True, capture_output=True)
    text = text_of(crt)
    for part in ('IP Address:172.20.10.2', 'IP Address:100.73.223.57',
                 'DNS:112ab-jno.netbird.cloud', 'TLS Web Server Authentication',
                 'sha256WithRSAEncryption', 'Public-Key: (2048 bit)', 'CA:FALSE'):
        assert part in text
    assert oct(os.stat(key).st_mode & 0o777) == '0o600'


def test_certificate_refuses_garbage_addresses(tmp_path):
    ensure_ca(str(tmp_path))
    with pytest.raises(ValueError):
        make_server_cert(str(tmp_path), ['not-an-ip'])
    with pytest.raises(ValueError):
        make_server_cert(str(tmp_path), [])


def test_https_server_end_to_end(tmp_path):
    d = str(tmp_path)
    ca = ensure_ca(d)
    crt, key = make_server_cert(d, ['127.0.0.1'])
    server_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    server_ctx.load_cert_chain(crt, key)
    srv = make_server(WebTeleop(), 'abc123', 0, lambda: 0.0, dict, server_ctx, ca)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    try:
        # a plain http request on the https port must not break the server
        with pytest.raises(Exception):
            urllib.request.urlopen('http://127.0.0.1:%d/?k=abc123' % port, timeout=3)
        client = ssl.create_default_context(cafile=ca)     # trusts only the Orin's CA
        base = 'https://127.0.0.1:%d' % port
        with urllib.request.urlopen(base + '/?k=abc123', context=client, timeout=5) as r:
            assert r.status == 200 and b'deadman' in r.read()
        with urllib.request.urlopen(base + '/ca.crt', context=client, timeout=5) as r:
            assert r.read() == open(ca, 'rb').read()      # public, no token needed
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + '/state', context=client, timeout=5)
        assert e.value.code == 403
        with pytest.raises(urllib.error.URLError):          # without the CA: not trusted
            urllib.request.urlopen(base + '/?k=abc123', context=ssl.create_default_context(),
                                   timeout=5)
    finally:
        srv.shutdown()
        srv.server_close()
