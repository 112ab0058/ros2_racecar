"""Static checks that race_node / race_monitor take every topic name from a parameter.

The offline tests and sim2d only exercise core.py, so these files are
checked by parsing them (no rclpy / OpenCV needed, runs on the car too).
"""

import ast
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "wro2026_control")

EXPECTED_DEFAULTS = {
    "scan_topic": "/scan",
    "odom_topic": "/odom",
    "imu_topic": "/imu/data",
    "line_detection_topic": "/line_detection",
    "cmd_vel_topic": "/cmd_vel",
    "status_topic": "/race_status",
}


def _tree(name):
    with open(os.path.join(PKG, name), encoding="utf-8") as stream:
        return ast.parse(stream.read())


def _pubsub_calls(tree):
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr in ("create_publisher", "create_subscription")]


def _topic_defaults(tree):
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and getattr(node.targets[0], "id", "") == "TOPIC_DEFAULTS"):
            return ast.literal_eval(node.value)
    raise AssertionError("TOPIC_DEFAULTS not found")


def test_race_node_topic_defaults():
    assert _topic_defaults(_tree("race_node.py")) == EXPECTED_DEFAULTS


def test_race_node_wires_every_topic_through_a_parameter():
    calls = _pubsub_calls(_tree("race_node.py"))
    keys = []
    for call in calls:
        arg = call.args[1]
        assert isinstance(arg, ast.Subscript), ast.dump(arg)
        assert isinstance(arg.value, ast.Name) and arg.value.id == "topic"
        keys.append(ast.literal_eval(arg.slice))
    assert sorted(keys) == sorted(EXPECTED_DEFAULTS)


def test_race_monitor_topics_are_parameters():
    tree = _tree("race_monitor.py")
    calls = _pubsub_calls(tree)
    assert calls
    for call in calls:
        arg = call.args[1]
        assert not isinstance(arg, ast.Constant), "hard-coded topic: %r" % arg.value
    declared = {ast.literal_eval(n.args[0]): ast.literal_eval(n.args[1])
                for n in ast.walk(tree)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "declare_parameter" and len(n.args) == 2
                and isinstance(n.args[0], ast.Constant)
                and str(n.args[0].value).endswith("_topic")}
    assert declared == {"status_topic": "/race_status", "odom_topic": "/odom",
                        "scan_topic": "/scan"}


def test_gazebo_launch_keeps_the_old_imu_topic():
    path = os.path.join(HERE, "..", "launch", "sim_race.launch.py")
    with open(path, encoding="utf-8") as stream:
        assert '{"imu_topic": "/imu"}' in stream.read()
