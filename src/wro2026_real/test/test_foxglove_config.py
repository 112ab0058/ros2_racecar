"""foxglove_bridge must be read-only: nobody may publish /cmd_vel from Foxglove."""
import re

from wro2026_real.foxglove_config import (foxglove_parameters, FOXGLOVE_PORT,
                                          WRITE_CAPABILITIES)


def test_no_write_capability():
    caps = foxglove_parameters()['capabilities']
    assert 'clientPublish' not in caps
    assert not set(caps) & set(WRITE_CAPABILITIES)
    assert caps  # still useful: connection graph, assets


def test_whitelists_match_no_ros_name():
    params = foxglove_parameters()
    names = ('/cmd_vel', 'cmd_vel', '/anything', '/wave_rover_driver/max_speed',
             '/rosapi/get_param', 'max_speed')
    for key in ('client_topic_whitelist', 'service_whitelist', 'param_whitelist'):
        for pattern in params[key]:
            for name in names:  # full match (C++ regex_match) and search
                assert re.fullmatch(pattern, name) is None
                assert re.search(pattern, name) is None


def test_port():
    assert FOXGLOVE_PORT == 8765
    assert foxglove_parameters()['port'] == 8765
    assert foxglove_parameters(9000)['port'] == 9000
