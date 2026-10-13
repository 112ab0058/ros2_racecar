"""Drive race_node.py through stub ROS modules in the 2D simulator.

Checks the message plumbing that the core tests do not touch: parameter
declaration, scan stamps, path length from /odom positions, sensor timeouts,
the start parameter, and /race_status.  No ROS installation is needed.
"""

import importlib
import json
import math
import os
import sys
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

try:
    import sim2d
    HAVE_SIM = True
except ImportError:
    HAVE_SIM = False


# ------------------------------------------------------------------ stubs
class _Clock:
    t = 0.0

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(round(_Clock.t * 1e9)))


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, *a, **k):
        pass

    warn = error = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Node:
    def __init__(self, name):
        self._stub_params = {}
        self.subs = {}
        self.pubs = {}
        self.timers = []

    def declare_parameter(self, name, value):
        self._stub_params[name] = _Param(value)

    def get_parameter(self, name):
        return self._stub_params[name]

    def set_param(self, name, value):
        self._stub_params[name] = _Param(value)

    def add_on_set_parameters_callback(self, cb):
        pass

    def create_publisher(self, msg_type, topic, qos):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = cb

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        pass


def _ns(**kw):
    return types.SimpleNamespace(**kw)


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _install_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.qos_profile_sensor_data = 10
    rclpy.node, rclpy.qos = node_mod, qos_mod
    mods = {"rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos_mod}
    for pkg, names in {"geometry_msgs.msg": ["Twist"], "nav_msgs.msg": ["Odometry"],
                       "sensor_msgs.msg": ["Imu", "LaserScan"],
                       "std_msgs.msg": ["Float32MultiArray", "String"],
                       "rcl_interfaces.msg": ["SetParametersResult"]}.items():
        top = pkg.split(".")[0]
        mods.setdefault(top, types.ModuleType(top))
        m = types.ModuleType(pkg)
        for n in names:
            if n == "Twist":
                setattr(m, n, lambda: _ns(linear=_ns(x=0.0), angular=_ns(z=0.0)))
            elif n == "String":
                setattr(m, n, lambda data="": _ns(data=data))
            else:
                setattr(m, n, _Msg)
        mods[pkg] = m
    sys.modules.update(mods)


def _odom(x, y, yaw):
    return _ns(pose=_ns(pose=_ns(position=_ns(x=x, y=y),
                                 orientation=_ns(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


def _scan(s, t):
    sec = int(t)
    return _ns(header=_ns(stamp=_ns(sec=sec, nanosec=int(round((t - sec) * 1e9)))),
               angle_min=s.angle_min, angle_increment=s.angle_increment,
               ranges=list(s.ranges), range_min=s.range_min, range_max=s.range_max)


@pytest.fixture(scope="module")
def race_node_module():
    saved = dict(sys.modules)
    _install_stubs()
    mod = importlib.import_module("wro2026_control.race_node")
    yield mod
    sys.modules.clear()
    sys.modules.update(saved)


@pytest.mark.skipif(not HAVE_SIM, reason="shapely not installed")
def test_node_drives_three_laps(race_node_module):
    node = race_node_module.RaceNode()
    tick = node.timers[0]
    world = sim2d.load_world(obstacles=sim2d.SCENARIOS["sdf"])
    cfg = sim2d.SimConfig()
    rng = np.random.default_rng(0)
    x, y, yaw = cfg.start
    v = w = 0.0
    dt = 0.01
    _Clock.t = 1.0
    status = None
    moved_before_start = False
    for step in range(int(220 / dt)):
        _Clock.t = 1.0 + step * dt
        t = _Clock.t
        if step % 4 == 0:                                   # 25 Hz odom
            node.subs["/odom"](_odom(x, y, yaw))
        if step % 12 == 0:                                  # ~8 Hz scan
            node.subs["/scan"](_scan(sim2d.lidar_scan(world, cfg, x, y, yaw, rng), t))
        if step % 10 == 0:                                  # 10 Hz lines
            o, b = sim2d.camera_lines(world, cfg, x, y, yaw)
            node.subs["/line_detection"](_ns(data=[0, 0, 0, float(o), 0, 0, 0, float(b), 0]))
        if step % 5 == 0:                                   # 20 Hz control
            if step == 300:
                node.set_param("start", True)
            tick()
            cmd = node.pubs["/cmd_vel"].msgs[-1]
            v, w = cmd.linear.x, cmd.angular.z
            if step < 300 and (v != 0.0 or w != 0.0):
                moved_before_start = True
            if node.pubs["/race_status"].msgs:
                status = json.loads(node.pubs["/race_status"].msgs[-1].data)
            if status and status["state"] == "DONE":
                break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        clear = min(sim2d.footprint(cfg, x, y, yaw).distance(p) for p in world.solids) if step % 20 == 0 else 1
        assert clear > 0.0, "collision"
    assert not moved_before_start
    assert status["state"] == "DONE" and status["reason"] == "laps complete"
    assert status["corners"] == 12
    assert math.hypot(x - cfg.start[0], y - cfg.start[1]) < 0.08


def test_node_stops_when_sensors_go_quiet(race_node_module):
    node = race_node_module.RaceNode()
    node.set_param("start", True)
    _Clock.t = 100.0
    node.timers[0]()
    cmd = node.pubs["/cmd_vel"].msgs[-1]
    assert (cmd.linear.x, cmd.angular.z) == (0.0, 0.0)
