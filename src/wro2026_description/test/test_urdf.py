"""The URDF must match the frame names the drivers publish and the measured offsets."""
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
URDF = os.path.join(os.path.dirname(HERE), 'urdf', 'r550.urdf')


def _robot():
    return ET.parse(URDF).getroot()


def _joints():
    out = {}
    for j in _robot().findall('joint'):
        origin = j.find('origin')
        out[j.find('child').get('link')] = (
            j.get('type'),
            j.find('parent').get('link'),
            [float(v) for v in origin.get('xyz').split()],
            [float(v) for v in origin.get('rpy').split()],
        )
    return out


def _params(pkg, cfg, node):
    path = os.path.join(SRC, pkg, 'config', cfg)
    with open(path) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def test_single_tree_rooted_at_base_link():
    links = {link.get('name') for link in _robot().findall('link')}
    joints = _joints()
    assert links == {'base_link', 'base_footprint', 'laser_link', 'imu_link'}
    roots = links - set(joints)
    assert roots == {'base_link'}  # /odom publishes odom -> base_link
    assert all(j[0] == 'fixed' and j[1] == 'base_link' for j in joints.values())


def test_frames_match_drivers():
    base = _params('wro2026_base_r550', 'r550_feedback.yaml', 'r550_feedback')
    lidar = _params('wro2026_lidar_n10p', 'n10p.yaml', 'n10p_scan')
    joints = _joints()
    assert base['base_frame_id'] == 'base_link'
    assert base['imu_frame_id'] in joints
    assert lidar['frame_id'] in joints


def test_measured_offsets():
    joints = _joints()
    assert joints['laser_link'][2] == [0.13, 0.0, 0.115]   # 13 cm ahead, 15 - 3.5 cm up
    assert joints['laser_link'][3] == [0.0, 0.0, 0.0]      # driver: lidar 0 deg -> +x
    assert joints['base_footprint'][2] == [0.0, 0.0, -0.035]  # wheel radius
