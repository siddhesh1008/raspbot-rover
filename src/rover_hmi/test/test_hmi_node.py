"""HMI node behaviour: instant stop, stale requests dropped, stop when the browser goes quiet."""

import pytest
import rclpy

from rover_hmi.hmi_node import ZERO, HmiNode

GO = (0.2, 0.0, 0.0)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = HmiNode(port=0)        # any free port, so tests never clash with a running HMI
    n.sent = []
    n.publish = n.sent.append  # record instead of publishing
    yield n
    n.server.stop()
    n.destroy_node()


def test_release_stops_immediately(node):
    node.on_browser_cmd(GO, 1)
    node.on_browser_cmd(ZERO, 2)
    assert node.sent == [GO, ZERO]   # both go out at once, no timer tick needed


def test_old_drive_command_cannot_undo_a_stop(node):
    node.on_browser_cmd(GO, 1)
    node.on_browser_cmd(ZERO, 3)     # the stop arrives first...
    node.on_browser_cmd(GO, 2)       # ...then an older drive command
    node.tick()
    assert node.sent == [GO, ZERO]


def test_reloaded_page_is_accepted_after_a_pause(node):
    node.on_browser_cmd(GO, 500)
    node.on_browser_cmd(ZERO, 501)
    node.last_cmd_time -= 2.0        # 2 s later the page is reloaded and counts from 1 again
    node.on_browser_cmd(GO, 1)
    assert node.sent[-1] == GO


def test_browser_going_quiet_stops_the_rover_once(node):
    node.on_browser_cmd(GO, 1)
    node.last_cmd_time -= 1.0        # no requests for 1 s (tab closed / Wi-Fi gone)
    node.tick()
    node.tick()
    node.tick()
    assert node.sent == [GO, ZERO]


def test_idle_page_publishes_nothing(node):
    node.on_browser_cmd(ZERO, 1)     # STOP pressed while already standing still
    node.tick()
    assert node.sent == []
