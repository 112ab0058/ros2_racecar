"""
Read-only check of the Orin 40-pin UART before using transport:=serial.

  ros2 run wro2026_base_waverover check_serial [--port /dev/ttyTHS1]

Checks that the device exists, the current user may open it, and that no
getty / kernel console / other process is using it. Never writes to the port.
Exit code 0 = no FAIL.
"""
import argparse
import grp
import os
import pwd
import stat
import subprocess

results = []


def report(level, item, detail):
    results.append(level)
    print(f'[{level:4}] {item}: {detail}')


def systemctl(*args):
    try:
        r = subprocess.run(['systemctl', *args], capture_output=True, text=True, timeout=5)
        return r.stdout.strip() or r.stderr.strip()
    except Exception as e:
        return f'error: {e}'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyTHS1')
    args = ap.parse_args()
    port = args.port
    name = os.path.basename(port)
    user = pwd.getpwuid(os.getuid()).pw_name

    # 1. device node
    if not os.path.exists(port):
        report('FAIL', 'device', f'{port} does not exist')
        return 1
    st = os.stat(port)
    if not stat.S_ISCHR(st.st_mode):
        report('FAIL', 'device', f'{port} is not a character device')
        return 1
    owner = pwd.getpwuid(st.st_uid).pw_name
    group = grp.getgrgid(st.st_gid).gr_name
    report('OK', 'device', f'{port} {stat.filemode(st.st_mode)} {owner}:{group}')

    # 2. group membership (process vs. /etc/group: differs until re-login)
    proc_groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
    db_members = set(grp.getgrgid(st.st_gid).gr_mem)
    if group in proc_groups:
        report('OK', 'group', f'{user} is in {group} (this session)')
    elif user in db_members:
        report('WARN', 'group',
               f'{user} added to {group} but this session predates it: log out and back in')
    else:
        report('FAIL', 'group', f'{user} not in {group}: needs sudo usermod -aG {group} {user}')

    # 3. permission + open test (open/close only, nothing is written)
    if os.access(port, os.R_OK | os.W_OK):
        report('OK', 'access', 'read/write permitted')
    else:
        report('FAIL', 'access', 'no read/write permission')
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        report('OK', 'open', 'opened and closed without writing')
    except OSError as e:
        report('FAIL', 'open', str(e))

    # 4. getty services
    for unit in ('nvgetty.service', f'serial-getty@{name}.service'):
        active = systemctl('is-active', unit)
        enabled = systemctl('is-enabled', unit)
        level = 'FAIL' if active == 'active' else ('WARN' if enabled == 'enabled' else 'OK')
        report(level, unit, f'active={active} enabled={enabled}')

    # 5. kernel console
    try:
        cmdline = open('/proc/cmdline').read().split()
        consoles = [a.split('=', 1)[1] for a in cmdline if a.startswith('console=')]
        level = 'FAIL' if any(c.split(',')[0] == name for c in consoles) else 'OK'
        report(level, 'kernel cmdline console', ', '.join(consoles) or 'none')
    except OSError as e:
        report('WARN', 'kernel cmdline console', str(e))
    try:
        active = [line.split()[0] for line in open('/proc/consoles') if line.strip()]
        level = 'FAIL' if name in active else 'OK'
        report(level, '/proc/consoles', ', '.join(active) or 'none')
    except OSError as e:
        report('WARN', '/proc/consoles', str(e))

    # 6. other processes holding the port (only processes we may inspect)
    holders, hidden = [], 0
    for pid in filter(str.isdigit, os.listdir('/proc')):
        fd_dir = f'/proc/{pid}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            hidden += 1
            continue
        for fd in fds:
            try:
                if os.readlink(f'{fd_dir}/{fd}') == port:
                    comm = open(f'/proc/{pid}/comm').read().strip()
                    holders.append(f'{pid}({comm})')
                    break
            except OSError:
                pass
    if holders:
        report('FAIL', 'open by', ', '.join(holders))
    else:
        report('OK', 'open by', f'nobody among visible processes '
                                f'({hidden} processes not inspectable without sudo)')

    fails = results.count('FAIL')
    warns = results.count('WARN')
    print(f'\nsummary: {fails} FAIL, {warns} WARN')
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
