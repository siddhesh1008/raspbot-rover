"""Node tests with a fake I2C bus: checks /cmd_vel handling and the safety watchdog."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from rover_driver import Board, FakeBus
from rover_driver import protocol as p
from rover_base.base_node import RoverBase

ALL_STOPPED = [[m, 0, 0] for m in range(4)]


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(start_watchdog=False):
    bus = FakeBus()
    n = RoverBase(board=Board(bus, sleep=lambda s: None), start_watchdog=start_watchdog)
    n.bus = bus
    return n


@pytest.fixture
def node():
    n = make_node()
    yield n
    n.running = False
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
    node.check_watchdog()
    assert last_motor_writes(node.bus) == ALL_STOPPED
    assert node.current_pwm == (0, 0, 0, 0)


def test_watchdog_keeps_driving_while_commands_arrive(node):
    node.on_cmd_vel(twist(x=0.2))
    before = len(node.bus.writes)
    node.check_watchdog()
    assert len(node.bus.writes) == before


def test_watchdog_thread_stops_motors_even_if_ros_is_stuck():
    """The ROS executor is never spun here (as if it were blocked); the thread must still act."""
    n = make_node(start_watchdog=True)
    try:
        n.on_cmd_vel(twist(x=0.2))
        assert last_motor_writes(n.bus) != ALL_STOPPED
        time.sleep(n.cmd_timeout + 0.3)
        assert last_motor_writes(n.bus) == ALL_STOPPED
    finally:
        n.running = False
        n.destroy_node()


class DroppingBus(FakeBus):
    """Like the real board: silently ignores some writes. Tracks what the board 'really' does."""

    def __init__(self):
        super().__init__()
        self.drop_next = 0
        self.motor_state = {}

    def write_i2c_block_data(self, address, register, data):
        if self.drop_next:
            self.drop_next -= 1
            return  # lost, and nobody is told
        super().write_i2c_block_data(address, register, data)
        if register == p.REG_MOTOR:
            motor, direction, speed = data
            self.motor_state[motor] = -speed if direction else speed


def test_lost_stop_is_corrected_by_refresh():
    bus = DroppingBus()
    n = RoverBase(board=Board(bus, sleep=lambda s: None), start_watchdog=False)
    try:
        n.on_cmd_vel(twist(x=0.2))
        assert all(v > 0 for v in bus.motor_state.values())
        bus.drop_next = 2                       # the stop for L1 and L2 gets lost
        n.on_cmd_vel(twist())
        assert bus.motor_state[0] > 0          # the board still drives L1 although we sent stop
        assert n.current_pwm == (0, 0, 0, 0)   # ...and the node believes it stopped
        n.last_write_time -= 0.2               # 0.2 s later the watchdog thread refreshes
        n.refresh_motors()
        assert bus.motor_state == {0: 0, 1: 0, 2: 0, 3: 0}
    finally:
        n.running = False
        n.destroy_node()


def test_refresh_keeps_running_thread_sending_state():
    n = make_node(start_watchdog=True)
    try:
        n.on_cmd_vel(twist(x=0.2))
        writes_before = len(n.bus.writes_to(p.REG_MOTOR))
        time.sleep(0.35)  # commands keep "arriving" (we refresh last_cmd_time below)
        n.last_cmd_time = time.monotonic()
        assert len(n.bus.writes_to(p.REG_MOTOR)) > writes_before  # state was re-sent
    finally:
        n.running = False
        n.destroy_node()


def test_shutdown_stops_motors(node):
    node.on_cmd_vel(twist(wz=1.0))
    node.shutdown()
    assert last_motor_writes(node.bus) == ALL_STOPPED


def test_start_is_ramped_but_stop_is_instant(node):
    node.on_cmd_vel(twist(x=0.2))                  # target 102 on all wheels
    assert node.current_pwm == (40, 40, 40, 40)    # first step only
    node.step_ramp(); node.step_ramp()
    assert node.current_pwm == (102, 102, 102, 102)
    node.on_cmd_vel(twist())                       # release
    assert node.current_pwm == (0, 0, 0, 0)        # instantly
