"""
Refuse to start a second WAVE ROVER driver (no rclpy).

Two drivers talking to the same ESP32 would fight over the wheel
commands, so the driver and test_motion look for the old Jetson-local
``wave_rover_driver`` and for another ``wro2026_base_waverover`` driver.
"""
import os

DRIVER_EXECUTABLES = (
    '/lib/wave_rover_driver/driver',         # old package on the Jetson
    '/lib/wro2026_base_waverover/driver',    # this package
)


def find_other_drivers(proc_root='/proc', self_pid=None):
    """
    Return [(pid, cmdline)] of running driver processes other than self_pid.

    A process matches when argv[0] or argv[1] (python3 <script>) ends with
    one of DRIVER_EXECUTABLES. Returns [] when proc_root does not exist
    (e.g. on Windows).
    """
    self_pid = os.getpid() if self_pid is None else self_pid
    found = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return found
    for name in entries:
        if not name.isdigit() or int(name) == self_pid:
            continue
        try:
            with open(os.path.join(proc_root, name, 'cmdline'), 'rb') as f:
                argv = [a.decode(errors='ignore') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if any(arg.endswith(DRIVER_EXECUTABLES) for arg in argv[:2]):
            found.append((int(name), ' '.join(argv)))
    return sorted(found)
