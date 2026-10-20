"""The simulated R550 must match the real car's numbers, and stay simulation only."""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(PKG)


def model():
    return ET.parse(os.path.join(PKG, 'models', 'r550', 'model.sdf')).getroot()


def plugin(root, name):
    for p in root.iter('plugin'):
        if p.get('name') == name:
            return {c.tag: c.text for c in p}
    raise KeyError(name)


def pose_of(root, tag, name):
    for el in root.iter(tag):
        if el.get('name') == name:
            return [float(v) for v in el.find('pose').text.split()]
    raise KeyError(name)


def test_ackermann_geometry_matches_firmware_and_measurements():
    root = model()
    ack = plugin(root, 'gz::sim::systems::AckermannSteering')
    assert float(ack['wheel_base']) == pytest.approx(0.144)          # Akm_axlespacing
    assert float(ack['wheel_separation']) == pytest.approx(0.162)    # Akm_wheelspacing
    assert float(ack['wheel_radius']) == pytest.approx(0.215 / (2 * math.pi), abs=2e-4)
    # min turn radius 0.35 m (MINI_AKM_MIN_TURN_RADIUS) -> centre steering limit
    assert float(ack['steering_limit']) == pytest.approx(math.atan(0.144 / 0.35), abs=0.01)
    assert pose_of(root, 'link', 'front_left_steer')[:2] == pytest.approx([0.144, 0.081])
    assert pose_of(root, 'link', 'rear_right_wheel')[:2] == pytest.approx([0.0, -0.081])
    for c in root.iter('cylinder'):
        if c.find('length').text == '0.028':
            assert float(c.find('radius').text) == pytest.approx(float(ack['wheel_radius']))


def test_rear_wheel_drive_and_limits_match_commandgate():
    ack = plugin(model(), 'gz::sim::systems::AckermannSteering')
    assert ack['left_joint'].startswith('rear_') and ack['right_joint'].startswith('rear_')
    with open(os.path.join(SRC, 'wro2026_base_r550', 'config', 'r550_base.yaml')) as f:
        base = yaml.safe_load(f)['r550_base']['ros__parameters']
    assert float(ack['max_velocity']) == pytest.approx(base['max_vx'])
    assert float(ack['min_velocity']) == pytest.approx(-base['max_reverse_vx'])
    assert ack['topic'] == '/sim/cmd_vel'                  # only sim_base_node drives it


def test_body_footprint_measured():
    p = pose_of(model(), 'collision', 'body')
    size = [float(v) for v in model().find('.//collision[@name="body"]/geometry/box/size')
            .text.split()]
    assert p[0] - size[0] / 2 == pytest.approx(-0.031)     # rear axle -> rear end
    assert p[0] + size[0] / 2 == pytest.approx(0.24)       # rear axle -> front end
    assert size[1] <= 0.18


def test_sensors_like_the_car():
    root = model()
    lidar = [s for s in root.iter('sensor') if s.get('type') == 'gpu_lidar'][0]
    assert lidar.find('topic').text == '/scan'
    assert lidar.find('gz_frame_id').text == 'laser_link'
    assert lidar.find('.//samples').text == '480'
    assert float(lidar.find('update_rate').text) == 10
    urdf = ET.parse(os.path.join(SRC, 'wro2026_description', 'urdf', 'r550.urdf')).getroot()
    xyz = [j.find('origin').get('xyz') for j in urdf.iter('joint')
           if j.get('name') == 'laser_joint'][0]
    assert [float(v) for v in lidar.find('pose').text.split()[:3]] == \
        pytest.approx([float(v) for v in xyz.split()])
    imu = [s for s in root.iter('sensor') if s.get('type') == 'imu'][0]
    assert imu.find('gz_frame_id').text == 'imu_link'
    assert float(imu.find('update_rate').text) == 20


def test_field_160cm_lanes_60cm_walls_25cm():
    world = ET.parse(os.path.join(PKG, 'worlds', 'r550_field.sdf')).getroot()
    walls = {c.get('name'): c for c in world.iter('collision')}
    north = walls['north']
    y, z = [float(v) for v in north.find('pose').text.split()][1:3]
    thick, height = [float(v) for v in north.find('.//size').text.split()][1:3]
    assert y - thick / 2 == pytest.approx(0.8) and height == pytest.approx(0.25)
    island = [float(v) for v in walls['island'].find('.//size').text.split()]
    assert island[:2] == pytest.approx([0.4, 0.4])


def test_only_sim_cmd_vel_goes_to_gazebo():
    with open(os.path.join(PKG, 'config', 'bridge.yaml')) as f:
        bridge = yaml.safe_load(f)
    to_gz = [b['ros_topic_name'] for b in bridge if b['direction'] == 'ROS_TO_GZ']
    assert to_gz == ['/sim/cmd_vel']                       # /cmd_vel always via the gate


def test_launch_refuses_real_car_and_uses_sim_time():
    with open(os.path.join(PKG, 'launch', 'sim.launch.py')) as f:
        text = f.read()
    assert 'base_node|feedback_node' in text and 'raise RuntimeError' in text
    assert text.count("'use_sim_time': True") >= 5
    for word in ('serial', '/dev/'):
        assert word not in text
