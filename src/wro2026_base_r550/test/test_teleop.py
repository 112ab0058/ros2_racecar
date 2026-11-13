"""Keyboard teleop logic with a fake clock (no ROS, no terminal)."""
import ast
import os

import pytest

from wro2026_base_r550.teleop import Teleop

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_nothing_pressed_is_zero():
    assert Teleop().command(0.0) == (0.0, 0.0)


def test_single_tap_drives_first_hold_then_stops():
    t = Teleop(speed=0.1, first_hold=0.6)
    t.key(0.0, 'w')
    assert t.command(0.5) == (pytest.approx(0.1), 0.0)
    assert t.command(0.61) == (0.0, 0.0)


def test_held_key_stops_soon_after_release():
    t = Teleop(speed=0.1, first_hold=0.6, release=0.25)
    t.key(0.0, 'w')
    k = 0.5                                  # first repeat after the repeat delay
    while k < 3.0:
        t.key(k, 'w')
        k += 0.033                           # ~30 repeats per second
    last = k - 0.033
    assert t.command(last + 0.2)[0] == pytest.approx(0.1)
    assert t.command(last + 0.26) == (0.0, 0.0)


def test_stop_keys_are_immediate():
    for stop in (' ', 's', 'S'):
        t = Teleop()
        t.key(0.0, 'w')
        assert t.key(0.1, stop) == 'STOP'
        assert t.command(0.1) == (0.0, 0.0)


def test_steering_signs_and_curvature():
    t = Teleop(speed=0.1, turn_radius=0.5)   # curvature 2 /m
    t.key(0.0, 'q')
    assert t.command(0.0) == (pytest.approx(0.1), pytest.approx(0.2))    # left = +wz
    t.key(0.0, 'e')
    assert t.command(0.0) == (pytest.approx(0.1), pytest.approx(-0.2))
    t.key(0.0, 'z')                          # reverse, wheels left: curvature kept
    vx, wz = t.command(0.0)
    assert vx == pytest.approx(-0.1) and wz / vx == pytest.approx(2.0)


def test_no_turning_on_the_spot():
    t = Teleop()
    for ch in 'adADjk\x1b':
        t.key(0.0, ch)
        assert t.command(0.0) == (0.0, 0.0)


def test_speed_steps_are_limited():
    t = Teleop(speed=0.1, max_speed=0.15, min_speed=0.04, speed_step=0.02)
    for _ in range(10):
        t.key(0.0, '+')
    assert t.speed == pytest.approx(0.15)
    for _ in range(10):
        t.key(0.0, '-')
    assert t.speed == pytest.approx(0.04)


def test_bad_settings_rejected():
    with pytest.raises(ValueError):
        Teleop(turn_radius=0.3)              # below the firmware minimum 0.35 m
    with pytest.raises(ValueError):
        Teleop(speed=0.2, max_speed=0.15)


def test_node_publishes_from_a_timer_not_from_keys():
    """Steady stream: publish() is called only from the timer and on exit."""
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_base_r550', 'teleop_node.py'),
                          encoding='utf-8').read())
    callers = set()
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef):
            for n in ast.walk(fn):
                if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'publish' \
                        and getattr(n.func.value, 'id', '') in ('self', 'node'):
                    callers.add(fn.name)
    assert callers == {'_tick', 'main'}
