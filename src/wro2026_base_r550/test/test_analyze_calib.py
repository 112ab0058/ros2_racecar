"""tools/real/analyze_calib.py on synthetic recordings."""
import importlib.util
import math
import os

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
TOOL = os.path.join(ROOT, 'tools', 'real', 'analyze_calib.py')
pytestmark = pytest.mark.skipif(not os.path.exists(TOOL), reason='tools/ not deployed here')


def tool():
    spec = importlib.util.spec_from_file_location('analyze_calib', TOOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def write(d, odom, imu):
    with open(os.path.join(d, 'odom.csv'), 'w') as f:
        f.write('t,x,y,yaw,vx,wz\n')
        for r in odom:
            f.write('%.4f,%.5f,%.5f,%.5f,%.5f,%.5f\n' % r)
    with open(os.path.join(d, 'imu.csv'), 'w') as f:
        f.write('t,gx,gy,gz,ax,ay,az\n')
        for r in imu:
            f.write('%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n' % r)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_static_bias_and_gravity(tmp_path):
    imu = [(i * 0.05, 0.0, 0.0, 0.002 + (0.001 if i % 2 else -0.001), 0.0, 0.0, 9.8)
           for i in range(1200)]
    odom = [(i * 0.05, 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(1200)]
    write(tmp_path, odom, imu)
    tool().main(['static', str(tmp_path)])
    text = (tmp_path / 'analysis_static.txt').read_text()
    assert '+0.00200' in text                 # gz bias
    assert 'largest on +z' in text
    assert '6.88 deg per minute' in text      # 0.002 rad/s * 60 s


def test_straight_gives_new_vx_scale(tmp_path):
    # odom says 2.2 m, tape says 2.0 m -> scale * 2.0 / 2.2
    odom = [(i * 0.05, 2.2 * i / 100, 0.0, 0.0, 0.1, 0.0) for i in range(101)]
    imu = [(i * 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 9.8) for i in range(101)]
    write(tmp_path, odom, imu)
    lines = tool().straight(tool().load(str(tmp_path / 'odom.csv')),
                            tool().load(str(tmp_path / 'imu.csv')), 2.0, 0.814, 3.0)
    assert any('= 0.7400' in s for s in lines)  # 0.814 * 2 / 2.2


def test_left_turn_signs_and_wz_scale(tmp_path):
    # 3 s still, then 10 s at +0.6283 rad/s (one full turn); gyro has bias 0.01;
    # odom under-reads by 10 %.
    imu, odom = [], []
    for i in range(261):
        t = i * 0.05
        w = 2 * math.pi / 10 if t > 3.0 else 0.0
        imu.append((t, 0.0, 0.0, w + 0.01, 0.0, 0.0, 9.8))
        yaw = 0.9 * 2 * math.pi / 10 * max(0.0, t - 3.0)
        odom.append((t, 0.0, 0.0, wrap(yaw), 0.0, 0.9 * w))
    write(tmp_path, odom, imu)
    m = tool()
    lines = m.turn(m.load(str(tmp_path / 'odom.csv')), m.load(str(tmp_path / 'imu.csv')),
                   1.0, 0.814, 3.0)
    text = '\n'.join(lines)
    assert 'OK: imu_link axes as assumed' in text and 'sign OK' in text
    new = float([s for s in lines if s.startswith('new wz_scale')][0].split('=')[-1])
    assert new == pytest.approx(0.814 / 0.9, rel=0.01)
