"""foxglove_bridge must be read-only: nobody may publish /cmd_vel from Foxglove."""
import re

from wro2026_real.foxglove_config import (foxglove_parameters, FOXGLOVE_PORT,
                                          WRITE_CAPABILITIES)


def test_no_write_capability():
    caps = foxglove_parameters()['capabilities']
    assert 'clientPublish' not in caps
    assert not set(caps) & set(WRITE_CAPABILITIES)
    assert caps  # still useful: connection graph, assets


def test_client_topics_match_nothing():
    params = foxglove_parameters()
    for pattern in params['client_topic_whitelist']:
        for topic in ('/cmd_vel', 'cmd_vel', '/anything', ''):
            assert re.search(pattern, topic) is None


def test_port():
    assert FOXGLOVE_PORT == 8765
    assert foxglove_parameters()['port'] == 8765
    assert foxglove_parameters(9000)['port'] == 9000
