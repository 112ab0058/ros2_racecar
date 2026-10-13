"""Two drivers must not run at the same time."""
import os

from wro2026_base_waverover.process_guard import find_other_drivers


def make_proc(tmp_path, pid, argv):
    d = tmp_path / str(pid)
    d.mkdir()
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')


def test_finds_old_and_new_driver_but_not_self_or_others(tmp_path):
    old = '/home/user2/ros2_ws/install/wave_rover_driver/lib/wave_rover_driver/driver'
    new = '/x/install/wro2026_base_waverover/lib/wro2026_base_waverover/driver'
    make_proc(tmp_path, 10, ['/usr/bin/python3', old, '--ros-args'])
    make_proc(tmp_path, 11, ['/usr/bin/python3', new])
    make_proc(tmp_path, 12, ['/usr/bin/python3', new])
    make_proc(tmp_path, 13, ['bash', '-c', 'pgrep -f lib/wave_rover_driver/driver'])
    make_proc(tmp_path, 14, ['/usr/bin/python3', '/x/lib/wro2026_base_waverover/test_motion'])
    (tmp_path / 'self').mkdir()
    found = find_other_drivers(str(tmp_path), self_pid=12)
    assert [pid for pid, _ in found] == [10, 11]


def test_missing_proc_returns_empty(tmp_path):
    assert find_other_drivers(os.path.join(str(tmp_path), 'nope')) == []
