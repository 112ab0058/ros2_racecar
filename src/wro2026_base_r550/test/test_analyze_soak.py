"""tools/real/analyze_soak.py on synthetic soak recordings."""
import importlib.util
import math
import os
import random

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
TOOL = os.path.join(ROOT, 'tools', 'real', 'analyze_soak.py')
pytestmark = pytest.mark.skipif(not os.path.exists(TOOL), reason='tools/ not deployed here')


def tool():
    spec = importlib.util.spec_from_file_location('analyze_soak', TOOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_allan_of_white_noise_falls_with_sqrt_tau():
    rnd = random.Random(1)
    sigma, tau0 = 0.001, 0.05
    curve = tool().allan([rnd.gauss(0.0, sigma) for _ in range(20000)], tau0)
    a1 = tool().adev_at(curve, 1.0)
    assert a1 == pytest.approx(sigma * math.sqrt(tau0 / 1.0), rel=0.1)


def test_report_flags_drift_and_leak(tmp_path):
    d = tmp_path
    with open(d / 'ekf.csv', 'w') as f:
        f.write('t,x,y,yaw\n')
        for i in range(3601):                    # 1 h, yaw drifts 10 deg
            f.write('%d,0,0,%.6f\n' % (i, math.radians(10.0 * i / 3600)))
    with open(d / 'resources.csv', 'w') as f:
        f.write('t,name,cpu_s,rss_kb\n')
        for i in range(61):                      # 1 h, +60 MB
            f.write('%d,leaky,%.1f,%d\n' % (i * 60, i * 6.0, 50000 + i * 1024))
            f.write('%d,steady,%.1f,%d\n' % (i * 60, i * 6.0, 50000))
    lines, _ = tool().analyse(str(d))
    text = '\n'.join(lines)
    ekf = [s for s in lines if s.startswith('yaw')][0]
    assert '+10.000 deg' in ekf and ekf.endswith('WARN')
    leaky = [s for s in lines if s.startswith('leaky')][0]
    steady = [s for s in lines if s.startswith('steady')][0]
    assert leaky.endswith('WARN') and steady.endswith('PASS')
    assert 'CPU  10.0 %' in text


def test_unwrap_handles_pi_crossing():
    u = tool().unwrap([3.1, -3.1, -3.0])
    assert u[-1] == pytest.approx((2 * math.pi - 6.2) + 0.1, abs=1e-6)  # +0.083 then +0.1


def test_zupt_replay_matches_live_bias(tmp_path):
    from wro2026_base_r550.zupt import GyroBias
    rnd = random.Random(3)
    live = GyroBias()
    with open(tmp_path / 'imu_raw.csv', 'w') as f, open(tmp_path / 'bias.csv', 'w') as b:
        f.write('t,gx,gy,gz\n')
        b.write('t,gyro_bias_z,zupt_learning\n')
        for i in range(20 * 300):                # 5 min at 20 Hz
            t = 100.0 + i * 0.05
            gz = 0.0005 + rnd.gauss(0.0, 0.00037)
            live.update(t, 0.0, 0.0, 0.0, 0.0, gz)
            f.write('%.4f,0,0,%.6f\n' % (t, gz))
            if i % 20 == 0:
                b.write('%.2f,%+.6f,True\n' % (t, live.bias[2]))
    lines, _ = tool().analyse(str(tmp_path))
    rep = [s for s in lines if s.startswith('offline replay')][0]
    assert rep.endswith('PASS'), rep
