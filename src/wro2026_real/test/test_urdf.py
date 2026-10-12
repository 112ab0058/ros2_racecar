"""The URDF parses and has the frames the drivers use."""
import os
import xml.etree.ElementTree as ET

URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'urdf', 'wave_rover.urdf')


def test_frames_and_tree():
    root = ET.parse(URDF).getroot()
    links = {link.get('name') for link in root.findall('link')}
    assert {'base_link', 'imu_link', 'camera_link'} <= links
    joints = {j.get('name'): (j.find('parent').get('link'), j.find('child').get('link'))
              for j in root.findall('joint')}
    children = [child for _, child in joints.values()]
    # base_link must be the only root: the base driver publishes odom -> base_link
    assert 'base_link' not in children
    assert sorted(children) == sorted(links - {'base_link'})
    assert joints['imu_joint'] == ('base_link', 'imu_link')
    assert joints['camera_joint'] == ('base_link', 'camera_link')
