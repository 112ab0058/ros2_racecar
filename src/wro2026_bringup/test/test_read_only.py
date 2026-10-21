"""sensors.launch.py must stay read only (CLAUDE.md safety rules)."""
import os

LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'launch', 'sensors.launch.py')


def _text():
    with open(LAUNCH) as f:
        return f.read()


def test_no_motion_topics():
    text = _text()
    for word in ('cmd_vel', 'ackermann', 'command'):
        assert word not in text.lower()


def test_foxglove_bridge_is_read_only():
    text = _text()
    assert "'client_topic_whitelist': ['^$']" in text
    assert "'capabilities': ['connectionGraph', 'assets']" in text
    for cap in ('clientPublish', 'services', 'parameters', 'parametersSubscribe'):
        assert "'%s'" % cap not in text
