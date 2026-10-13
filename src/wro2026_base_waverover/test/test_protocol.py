"""Waveshare JSON encoding and feedback parsing."""
import math

import pytest

from wro2026_base_waverover.core import (battery_voltage, encode_command, G, imu_values,
                                         parse_feedback, POLL_BASE_CMD, POLL_IMU_CMD,
                                         quaternion_from_rpy, STOP_CMD, wheel_command)


def test_stop_is_exactly_the_allowed_literal():
    assert encode_command(STOP_CMD) == '{"T":1,"L":0,"R":0}'
    assert encode_command(wheel_command(0, 0)) == '{"T":1,"L":0,"R":0}'
    assert encode_command(wheel_command(0.0, -0.0)) == '{"T":1,"L":0,"R":0}'
    assert encode_command(wheel_command(0.0001, -0.0004)) == '{"T":1,"L":0,"R":0}'
    assert encode_command(wheel_command(math.nan, math.inf)) == '{"T":1,"L":0,"R":0}'


def test_wheel_command_encoding_and_rounding():
    assert encode_command(wheel_command(0.05, -0.05)) == '{"T":1,"L":0.05,"R":-0.05}'
    assert encode_command(wheel_command(0.12345, 0.0306)) == '{"T":1,"L":0.123,"R":0.031}'


def test_poll_commands():
    assert encode_command(POLL_BASE_CMD) == '{"T":130}'
    assert encode_command(POLL_IMU_CMD) == '{"T":126}'


def test_parse_feedback():
    msg = parse_feedback('{"T":1001,"L":0,"R":0,"r":0,"p":0,"v":12.25}')
    assert msg['T'] == 1001
    assert battery_voltage(msg) == 12.25
    assert parse_feedback('not json') is None
    assert parse_feedback('[1, 2]') is None
    assert parse_feedback(None) is None
    assert battery_voltage({'T': 1001}) is None
    assert battery_voltage({'T': 1002, 'v': 12.0}) is None


def test_imu_units_are_converted_to_si():
    msg = {'T': 1002, 'r': 90, 'p': 0, 'y': -180, 'ax': 0, 'ay': 0, 'az': 980,
           'gx': 0, 'gy': 0, 'gz': 57.29577951308232}
    vals = imu_values(msg)
    assert vals['r'] == pytest.approx(math.pi / 2)
    assert vals['y'] == pytest.approx(-math.pi)
    assert vals['gz'] == pytest.approx(1.0)
    assert vals['az'] == pytest.approx(0.980 * G)
    assert imu_values({'T': 1002, 'r': 0}) is None
    assert imu_values({'T': 1001}) is None


def test_quaternion_from_yaw():
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))
