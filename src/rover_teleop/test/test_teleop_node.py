"""The terminal driver must stay quiet when idle, so it doesn't fight the browser HMI."""

import time

import pytest
import rclpy

from rover_teleop.teleop_node import WasdTeleop


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = WasdTeleop()
    n.sent = []
    n.publish = n.sent.append
    yield n
    n.destroy_node()


def test_idle_publishes_nothing(node):
    for _ in range(5):
        node.tick()
    assert node.sent == []


def test_moving_then_one_stop_then_quiet(node):
    node.handle_key("w")
    node.tick()
    node.tick()
    node.last_key_time = time.monotonic() - 5  # key released long ago
    node.tick()
    node.tick()
    node.tick()
    assert node.sent == [(0.2, 0.0, 0.0), (0.2, 0.0, 0.0), (0.0, 0.0, 0.0)]
