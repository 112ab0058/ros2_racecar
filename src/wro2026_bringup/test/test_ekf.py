"""ekf.yaml and localization.launch.py: what is fused, who publishes odom -> base_link."""
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIELDS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz',
          'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az']


def params():
    with open(os.path.join(PKG, 'config', 'ekf.yaml')) as f:
        return yaml.safe_load(f)['ekf_filter_node']['ros__parameters']


def fused(config):
    assert len(config) == 15
    return {name for name, on in zip(FIELDS, config) if on}


def test_2d_frames_and_rate():
    p = params()
    assert p['two_d_mode'] is True and p['publish_tf'] is True
    assert (p['odom_frame'], p['base_link_frame'], p['world_frame']) == \
        ('odom', 'base_link', 'odom')
    assert p['frequency'] >= 20.0


def test_fuse_velocities_only():
    """No absolute pose from two sources: odom gives vx, vy; the gyro gives vyaw."""
    p = params()
    assert fused(p['odom0_config']) == {'vx', 'vy'}
    assert fused(p['imu0_config']) == {'vyaw'}       # firmware zeroes accel at boot
    assert p['odom0'] == '/odom' and p['imu0'] == '/imu/data'


def test_only_the_ekf_publishes_odom_tf():
    with open(os.path.join(PKG, 'launch', 'localization.launch.py')) as f:
        text = f.read()
    assert "'publish_odom_tf': 'false'" in text
    assert 'ekf.yaml' in text
    assert 'cmd_vel' not in text.lower()             # read only, like sensors.launch.py


def test_sensors_launch_passes_publish_odom_tf():
    with open(os.path.join(PKG, 'launch', 'sensors.launch.py')) as f:
        text = f.read()
    assert "'publish_odom_tf': LaunchConfiguration('publish_odom_tf')" in text
    assert "DeclareLaunchArgument('publish_odom_tf', default_value='true')" in text
