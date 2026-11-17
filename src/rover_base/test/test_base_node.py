"""Node tests with a fake I2C bus: checks /cmd_vel handling and the safety watchdog."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from rover_driver import Board, FakeBus
from rover_driver import protocol as p
from rover_base.base_node import RoverBase


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    bus = FakeBus()
    n = RoverBase(board=Board(bus, sleep=lambda s: None))
    n.bus = bus
    yield n
    n.destroy_node()


def twist(x=0.0, y=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


def last_motor_writes(bus):
    return bus.writes_to(p.REG_MOTOR)[-4:]


def test_forward_command_drives_all_motors_forward(node):
    node.on_cmd_vel(twist(x=0.2))
    writes = last_motor_writes(node.bus)
    assert [w[0] for w in writes] == [0, 1, 2, 3]
    assert all(w[1] == 0 and w[2] > 0 for w in writes)


def test_watchdog_stops_motors_after_timeout(node):
    node.on_cmd_vel(twist(x=0.2))
    node.last_cmd_time = time.monotonic() - 1.0  # pretend the last command is 1 s old
    node.on_watchdog()
    assert last_motor_writes(node.bus) == [[m, 0, 0] for m in range(4)]
    assert node.current_pwm == (0, 0, 0, 0)


def test_watchdog_keeps_driving_while_commands_arrive(node):
    node.on_cmd_vel(twist(x=0.2))
    before = len(node.bus.writes)
    node.on_watchdog()
    assert len(node.bus.writes) == before


def test_shutdown_stops_motors(node):
    node.on_cmd_vel(twist(wz=1.0))
    node.shutdown()
    assert last_motor_writes(node.bus) == [[m, 0, 0] for m in range(4)]
