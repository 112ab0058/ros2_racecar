"""
HTTPS certificates for the phone page (no ROS). Uses the openssl command.

The page runs on addresses no public authority signs (hotspot 172.20.10.x,
NetBird 100.x), so the Orin keeps its own small certificate authority (CA):

  <cert_dir>/ca.key     CA private key, mode 600, NEVER leaves the Orin (not in git)
  <cert_dir>/ca.crt     CA certificate: install and trust it on the phone ONCE
  <cert_dir>/server.*   page certificate, signed by the CA for the addresses the
                        Orin has right now; made again at every start

After the phone trusts ca.crt (iPhone: Settings > General > About > Certificate
Trust Settings), the page opens without warnings on every address in the
server certificate. https also lets iPhone Safari give the page the motion
sensors (tilt mode).

Certificate rules that iPhones enforce (Apple, iOS 13): RSA >= 2048, SHA-256,
subjectAltName, extendedKeyUsage serverAuth, server validity <= 825 days.
"""
import ipaddress
import os
import subprocess
import tempfile

CA_DAYS = 3650
SERVER_DAYS = 397


def _openssl(*args):
    subprocess.run(('openssl',) + args, check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.PIPE)


def ensure_ca(cert_dir):
    """Create the CA in cert_dir if it is not there yet; return the path of ca.crt."""
    os.makedirs(cert_dir, mode=0o700, exist_ok=True)
    key, crt = os.path.join(cert_dir, 'ca.key'), os.path.join(cert_dir, 'ca.crt')
    if not (os.path.exists(key) and os.path.exists(crt)):
        _openssl('req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-sha256',
                 '-keyout', key, '-out', crt, '-days', str(CA_DAYS),
                 '-subj', '/CN=R550 phone page CA (this Orin only)',
                 '-addext', 'basicConstraints=critical,CA:TRUE',
                 '-addext', 'keyUsage=critical,keyCertSign,cRLSign')
        os.chmod(key, 0o600)
    return crt


def make_server_cert(cert_dir, addresses, names=()):
    """
    Sign a page certificate for these IP addresses and DNS names.

    Return (cert path, key path). The CA must exist (ensure_ca).
    """
    sans = []
    for a in addresses:
        ipaddress.ip_address(a)          # raises ValueError on garbage
        sans.append('IP:' + a)
    sans += ['DNS:' + n for n in names]
    if not sans:
        raise ValueError('no address for the certificate')
    key, crt = os.path.join(cert_dir, 'server.key'), os.path.join(cert_dir, 'server.crt')
    with tempfile.TemporaryDirectory() as tmp:
        csr, ext = os.path.join(tmp, 'server.csr'), os.path.join(tmp, 'ext.cnf')
        with open(ext, 'w') as f:
            f.write('subjectAltName=%s\nbasicConstraints=CA:FALSE\n'
                    'keyUsage=critical,digitalSignature,keyEncipherment\n'
                    'extendedKeyUsage=serverAuth\n' % ','.join(sans))
        _openssl('req', '-newkey', 'rsa:2048', '-nodes', '-sha256', '-keyout', key,
                 '-out', csr, '-subj', '/CN=R550 phone page')
        _openssl('x509', '-req', '-in', csr, '-sha256', '-days', str(SERVER_DAYS),
                 '-CA', os.path.join(cert_dir, 'ca.crt'),
                 '-CAkey', os.path.join(cert_dir, 'ca.key'), '-set_serial',
                 str(int.from_bytes(os.urandom(8), 'big')), '-extfile', ext, '-out', crt)
    os.chmod(key, 0o600)
    return crt, key


def local_addresses():
    """Return this computer's IPv4 addresses except loopback (hostname -I)."""
    out = subprocess.run(['hostname', '-I'], check=True, capture_output=True, text=True).stdout
    return [a for a in out.split() if ':' not in a and not a.startswith('127.')]
