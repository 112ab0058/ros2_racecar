"""drive_map.launch.py and orin_map.sh: read only by default, limits, what is recorded."""
import os
import re

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG, 'launch', 'drive_map.launch.py')
SCRIPT = os.path.join(PKG, '..', '..', 'tools', 'real', 'orin_map.sh')


def _text(path=LAUNCH):
    with open(path) as f:
        return f.read()


def test_commands_are_off_by_default():
    assert "DeclareLaunchArgument('enable_commands', default_value='false')" in _text()


def test_only_the_ekf_publishes_odom_tf():
    assert "'publish_odom_tf': 'false'" in _text()


def test_launch_does_not_start_a_command_source():
    """Only the keyboard (teleop_node, started by the script) may publish /cmd_vel."""
    assert "executable='teleop_node'" not in _text()


def test_foxglove_is_read_only():
    text = _text()
    assert "'client_topic_whitelist': ['^$']" in text
    assert "'capabilities': ['connectionGraph', 'assets']" in text


def test_bag_has_the_training_data():
    topics = re.search(r'BAG_TOPICS = \[(.*?)\]', _text(), re.S).group(1)
    for name in ('/scan', '/cmd_vel_out', '/odometry/filtered', '/tf_static'):
        assert "'%s'" % name in topics


def test_script_asks_first_and_keeps_the_limits():
    if not os.path.exists(SCRIPT):
        pytest.skip('tools/ not deployed here')
    text = _text(SCRIPT)
    assert 'Type YES' in text and '[ "$a" = YES ] || exit 0' in text
    assert '$MAX_VX <= 0.15' in text
    assert '$RUN_TIME <= 600' in text
    assert '$RADIUS >= 0.35' in text
