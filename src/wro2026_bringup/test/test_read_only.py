"""sensors.launch.py and the SLAM files must stay read only (CLAUDE.md safety rules)."""
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG, 'launch', 'sensors.launch.py')
READ_ONLY = [LAUNCH, os.path.join(PKG, 'launch', 'slam.launch.py'),
             os.path.join(PKG, 'config', 'slam_toolbox.yaml')]


def _text(path=LAUNCH):
    with open(path) as f:
        return f.read()


def test_no_motion_topics():
    for path in READ_ONLY:
        text = _text(path)
        for word in ('cmd_vel', 'ackermann', 'command'):
            assert word not in text.lower(), (path, word)


def test_foxglove_bridge_is_read_only():
    text = _text()
    assert "'client_topic_whitelist': ['^$']" in text
    assert "'capabilities': ['connectionGraph', 'assets']" in text
    for cap in ('clientPublish', 'services', 'parameters', 'parametersSubscribe'):
        assert "'%s'" % cap not in text
