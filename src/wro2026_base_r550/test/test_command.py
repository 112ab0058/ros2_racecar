"""CommandGate: every safety rule, with a fake clock (no ROS, no serial)."""
import ast
import os
import re

import pytest

from wro2026_base_r550 import command as c
from wro2026_base_r550.command import CommandGate

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def armed(**kw):
    """A gate that is enabled, has fresh feedback and a good battery at t=0."""
    g = CommandGate(enabled=True, max_accel=100.0, **kw)
    g.set_voltage(0.0, 12.0)
    g.update(0.0, 0.0)  # first tick: starts the ramp clock
    return g


def test_disabled_never_moves():
    g = CommandGate(enabled=False)
    g.set_voltage(0.0, 12.0)
    g.set_cmd(0.0, 0.2, 0.5)
    assert g.update(0.05, 0.05) == (0.0, 0.0, c.DISABLED)


def test_ok_command_passes():
    g = armed()
    g.set_cmd(0.0, 0.2, 0.4)
    vx, wz, reason = g.update(0.05, 0.05)
    assert reason == c.OK and vx == pytest.approx(0.2) and wz == pytest.approx(0.4)


def test_cmd_timeout_stops():
    g = armed(cmd_timeout=0.3)
    g.set_cmd(0.0, 0.2, 0.0)
    assert g.update(0.29, 0.29)[2] == c.OK
    assert g.update(0.31, 0.31) == (0.0, 0.0, c.CMD_TIMEOUT)


def test_no_cmd_yet():
    assert armed().update(0.1, 0.1) == (0.0, 0.0, c.NO_CMD)


def test_feedback_lost_stops():
    g = armed(feedback_timeout=0.5)
    g.set_cmd(1.0, 0.2, 0.0)
    assert g.update(1.0, 0.4)[2] == c.FEEDBACK_LOST   # last frame 0.6 s ago
    assert g.update(1.0, None)[2] == c.FEEDBACK_LOST  # never had a frame


def test_speed_limits():
    g = armed(max_vx=0.3, max_reverse_vx=0.15)
    g.set_cmd(0.0, 5.0, 0.0)
    assert g.update(0.05, 0.05)[0] == pytest.approx(0.3)
    g.set_cmd(0.1, -5.0, 0.0)
    assert g.update(0.15, 0.15)[0] == pytest.approx(-0.15)


def test_wz_limited_and_curvature_kept():
    g = armed(max_vx=0.3, max_wz=1.0)
    g.set_cmd(0.0, 0.6, 1.2)  # curvature 2 /m; vx clamped to 0.3 -> wz 0.6
    vx, wz, _ = g.update(0.05, 0.05)
    assert vx == pytest.approx(0.3) and wz == pytest.approx(0.6)
    g.set_cmd(0.1, 0.3, 3.0)  # curvature 10 /m -> wz 3.0 -> clamped to 1.0
    assert g.update(0.15, 0.15)[1] == pytest.approx(1.0)


def test_no_turning_on_the_spot():
    g = armed()
    g.set_cmd(0.0, 0.0, 1.0)
    assert g.update(0.05, 0.05)[:2] == (0.0, 0.0)


def test_ramp_up_but_immediate_safety_stop():
    g = CommandGate(enabled=True, max_accel=0.5)
    g.set_voltage(0.0, 12.0)
    g.set_cmd(0.0, 0.3, 0.0)
    g.update(0.0, 0.0)
    vx = g.update(0.1, 0.1)[0]
    assert vx == pytest.approx(0.05)          # 0.5 m/s^2 * 0.1 s
    g.set_cmd(0.1, 0.3, 0.0)
    assert g.update(0.2, 0.2)[0] == pytest.approx(0.10)
    assert g.update(0.5, 0.5)[0] == 0.0       # cmd timeout: straight to 0


def test_battery_low_with_filter_and_hysteresis():
    g = CommandGate(enabled=True, min_battery_voltage=10.5, battery_hysteresis=0.2,
                    battery_filter_time=2.0, max_accel=100.0)
    g.set_voltage(0.0, 11.0)
    g.set_cmd(0.0, 0.1, 0.0)
    g.set_voltage(0.05, 9.0)                  # short sag under load: filtered
    assert g.update(0.05, 0.05)[2] == c.OK
    for i in range(1, 60):                    # stays low for 3 s
        g.set_voltage(0.05 + i * 0.05, 9.0)
    g.set_cmd(3.0, 0.1, 0.0)
    assert g.update(3.0, 3.0)[2] == c.BATTERY_LOW
    g2 = CommandGate(enabled=True, min_battery_voltage=10.5, battery_filter_time=0.0)
    g2.set_voltage(0.0, 10.4)
    g2.set_voltage(0.1, 10.6)                 # above the limit but not by 0.2 V
    assert g2.battery_low
    g2.set_voltage(0.2, 10.7)
    assert not g2.battery_low


def test_max_run_time_counts_from_first_command():
    g = armed(max_run_time=10.0)
    assert g.update(6.0, 6.0)[2] == c.NO_CMD     # idle time before the first command
    g.set_cmd(8.0, 0.1, 0.0)
    assert g.update(8.0, 8.0)[2] == c.OK
    g.set_cmd(17.9, 0.1, 0.0)
    assert g.update(17.9, 17.9)[2] == c.OK       # 9.9 s after the first command
    g.set_cmd(18.5, 0.1, 0.0)
    assert g.update(18.5, 18.5)[2] == c.RUN_TIME_OVER


def test_max_run_time_latches():
    g = armed(max_run_time=10.0)
    g.set_cmd(0.0, 0.1, 0.0)
    assert g.update(0.0, 0.0)[2] == c.OK
    g.set_cmd(10.5, 0.1, 0.0)
    assert g.update(10.5, 10.5)[2] == c.RUN_TIME_OVER
    g.set_cmd(11.0, 0.1, 0.0)
    assert g.update(11.0, 11.0)[2] == c.RUN_TIME_OVER


def test_nan_command_rejected():
    g = armed()
    assert not g.set_cmd(0.0, float('nan'), 0.0)
    assert g.rejected_cmds == 1
    assert g.update(0.05, 0.05)[2] == c.NO_CMD


def test_bad_limits_rejected():
    with pytest.raises(ValueError):
        CommandGate(cmd_timeout=0.0)


def test_base_parameters_match_yaml():
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_base_r550', 'base_node.py'),
                          encoding='utf-8').read())
    names = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == \
                'COMMAND_PARAMETERS':
            for item in node.value.elts:
                names[item.elts[0].value] = item.elts[1].value
    text = open(os.path.join(PKG, 'config', 'r550_base.yaml'), encoding='utf-8').read()
    yaml_vals = {}
    for line in text.splitlines():
        m = re.match(r'^\s{4}(\w+):\s*([^#]*?)\s*(#.*)?$', line)
        if m and m.group(1) != 'ros__parameters':
            yaml_vals[m.group(1)] = m.group(2)
    assert set(yaml_vals) == set(names)
    for key, default in names.items():
        raw = yaml_vals[key]
        if isinstance(default, bool):
            assert raw == str(default).lower(), key
        else:
            assert type(default)(raw) == default, key
    assert names['enable_commands'] is False      # never on by default
    assert names['cmd_timeout'] <= 0.3            # CLAUDE.md


def test_gate_defaults_match_claude_md():
    g = CommandGate()
    assert g.enabled is False and g.cmd_timeout == 0.3


def test_command_byte1_is_always_zero():
    """byte 1 = 1/2/3 switches the firmware to auto-charging modes (usartx.c)."""
    from wro2026_base_r550.protocol import encode_twist_command, stop_command
    assert encode_twist_command(0.1, 0.2)[1] == 0 and stop_command()[1] == 0
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_base_r550', 'base_node.py'),
                          encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') in (
                'encode_twist_command', 'stop_command'):
            assert not node.keywords, 'base_node must not set byte1/byte2'
