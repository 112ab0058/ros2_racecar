"""
Read-only foxglove_bridge parameters for the real car (no rclpy).

Parameter names from the ROS 2 foxglove_bridge documentation
(foxglove/foxglove-sdk, ros/src/foxglove_bridge/README.md). The default
capabilities are [clientPublish, parameters, parametersSubscribe, services,
connectionGraph, assets]; here only the read-only ones are kept, so nobody
can publish /cmd_vel, change a parameter (e.g. max_speed) or call a service
from the Foxglove UI.
"""

FOXGLOVE_PORT = 8765

# capabilities that would let a Foxglove client change the robot
WRITE_CAPABILITIES = ('clientPublish', 'parameters', 'parametersSubscribe', 'services')

READ_ONLY_CAPABILITIES = ['connectionGraph', 'assets']

# Regex that matches no ROS name (names are never empty). '^$' is plain
# enough for every regex engine the bridge may use (std::regex, Rust regex);
# even if clientPublish were enabled, no topic would be accepted from clients.
MATCH_NOTHING = '^$'


def foxglove_parameters(port=FOXGLOVE_PORT):
    """Return the foxglove_bridge parameter dict (read-only)."""
    return {
        'port': int(port),
        'address': '0.0.0.0',
        'capabilities': list(READ_ONLY_CAPABILITIES),
        'client_topic_whitelist': [MATCH_NOTHING],
        'service_whitelist': [MATCH_NOTHING],
        'param_whitelist': [MATCH_NOTHING],
    }
